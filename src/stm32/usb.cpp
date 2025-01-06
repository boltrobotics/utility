// Copyright (C) 2019 Sergey Kapustin <kapucin@gmail.com>

/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

#if BTR_USB0_ENABLED > 0

// SYSTEM INCLUDES
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>
#include <libopencm3/cm3/scb.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

// PROJECT INCLUDES
#include "utility/stm32/usb.hpp"  // class implemented

extern "C" {

static volatile bool ready_ = false;
static volatile uint8_t rx_error_;
static QueueHandle_t tx_q_;
static QueueHandle_t rx_q_;
static uint8_t ctrl_buff_[BTR_USART_CR_BUFF_SIZE];

////////////////////////////////////////////////////////////////////////////////////////////////////

static const struct usb_device_descriptor usb_dev_info = {
  .bLength = USB_DT_DEVICE_SIZE,
  .bDescriptorType = USB_DT_DEVICE,
  .bcdUSB = 0x0200,
  .bDeviceClass = USB_CLASS_CDC,
  .bDeviceSubClass = 0,
  .bDeviceProtocol = 0,
  .bMaxPacketSize0 = 64,
  .idVendor = 0x0483,
  .idProduct = 0x5740,
  .bcdDevice = 0x0200,
  .iManufacturer = 1,
  .iProduct = 2,
  .iSerialNumber = 3,
  .bNumConfigurations = 1,
};

static const struct usb_endpoint_descriptor comm_endp[] = {
  {
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = 0x83,
    .bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
    .wMaxPacketSize = 16,
    .bInterval = 255,
    .extra = NULL,
    .extralen = 0,
  }
};

static const struct usb_endpoint_descriptor data_endp[] = {
  {
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = 0x01,
    .bmAttributes = USB_ENDPOINT_ATTR_BULK,
    .wMaxPacketSize = 64,
    .bInterval = 1,
    .extra = NULL,
    .extralen = 0,
  }, {
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = 0x82,
    .bmAttributes = USB_ENDPOINT_ATTR_BULK,
    .wMaxPacketSize = 64,
    .bInterval = 1,
    .extra = NULL,
    .extralen = 0,
  }
};

static const struct {
  struct usb_cdc_header_descriptor header;
  struct usb_cdc_call_management_descriptor call_mgmt;
  struct usb_cdc_acm_descriptor acm;
  struct usb_cdc_union_descriptor cdc_union;
} __attribute__((packed)) cdcacm_functional_descriptors = {
  .header = {
    .bFunctionLength = sizeof(struct usb_cdc_header_descriptor),
    .bDescriptorType = CS_INTERFACE,
    .bDescriptorSubtype = USB_CDC_TYPE_HEADER,
    .bcdCDC = 0x0110,
  },
  .call_mgmt = {
    .bFunctionLength =
      sizeof(struct usb_cdc_call_management_descriptor),
    .bDescriptorType = CS_INTERFACE,
    .bDescriptorSubtype = USB_CDC_TYPE_CALL_MANAGEMENT,
    .bmCapabilities = 0,
    .bDataInterface = 1,
  },
  .acm = {
    .bFunctionLength = sizeof(struct usb_cdc_acm_descriptor),
    .bDescriptorType = CS_INTERFACE,
    .bDescriptorSubtype = USB_CDC_TYPE_ACM,
    .bmCapabilities = 0,
  },
  .cdc_union = {
    .bFunctionLength = sizeof(struct usb_cdc_union_descriptor),
    .bDescriptorType = CS_INTERFACE,
    .bDescriptorSubtype = USB_CDC_TYPE_UNION,
    .bControlInterface = 0,
    .bSubordinateInterface0 = 1,
  }
};

static const struct usb_interface_descriptor comm_iface[] = {
  {
    .bLength = USB_DT_INTERFACE_SIZE,
    .bDescriptorType = USB_DT_INTERFACE,
    .bInterfaceNumber = 0,
    .bAlternateSetting = 0,
    .bNumEndpoints = 1,
    .bInterfaceClass = USB_CLASS_CDC,
    .bInterfaceSubClass = USB_CDC_SUBCLASS_ACM,
    .bInterfaceProtocol = USB_CDC_PROTOCOL_AT,
    .iInterface = 0,
    .endpoint = comm_endp,
    .extra = &cdcacm_functional_descriptors,
    .extralen = sizeof(cdcacm_functional_descriptors)
  }
};

static const struct usb_interface_descriptor data_iface[] = {
  {
    .bLength = USB_DT_INTERFACE_SIZE,
    .bDescriptorType = USB_DT_INTERFACE,
    .bInterfaceNumber = 1,
    .bAlternateSetting = 0,
    .bNumEndpoints = 2,
    .bInterfaceClass = USB_CLASS_DATA,
    .bInterfaceSubClass = 0,
    .bInterfaceProtocol = 0,
    .iInterface = 0,
    .endpoint = data_endp,
    .extra = NULL,
    .extralen = 0,
  }
};

static const struct usb_interface ifaces[] = {
  {
    .cur_altsetting = NULL,
    .num_altsetting = 1,
    .iface_assoc = NULL,
    .altsetting = comm_iface,
  }, {
    .cur_altsetting = NULL,
    .num_altsetting = 1,
    .iface_assoc = NULL,
    .altsetting = data_iface,
  }
};

static const struct usb_config_descriptor usb_cnf_info = {
  .bLength = USB_DT_CONFIGURATION_SIZE,
  .bDescriptorType = USB_DT_CONFIGURATION,
  .wTotalLength = 0,
  .bNumInterfaces = 2,
  .bConfigurationValue = 1,
  .iConfiguration = 0,
  .bmAttributes = 0x80,
  .bMaxPower = 0x32,
  .interface = ifaces,
};

static const char * usb_strings[] = {
  "Bolt Robotics",
  "btr::Usb",
  "utility",
};

////////////////////////////////////////////////////////////////////////////////////////////////////

static enum usbd_request_return_codes onCtrlRecv(
    usbd_device* usbd_dev,
    struct usb_setup_data* req,
    uint8_t** buff,
    uint16_t* bytes,
    void (**complete)(usbd_device* usbd_dev, struct usb_setup_data* req))
{
  (void) complete;
  (void) buff;
  (void) usbd_dev;

  switch (req->bRequest) {
    case USB_CDC_REQ_SET_CONTROL_LINE_STATE:
      return USBD_REQ_HANDLED;
    case USB_CDC_REQ_SET_LINE_CODING:
      if (*bytes < sizeof(struct usb_cdc_line_coding)) {
        return USBD_REQ_NOTSUPP;
      }
      return USBD_REQ_HANDLED;
  }
  return USBD_REQ_NOTSUPP;
}

static void onDataRecv(usbd_device* usbd_dev, uint8_t ep)
{
  (void) ep;
  uint32_t rx_q_avail = uxQueueSpacesAvailable(rx_q_);

  if (rx_q_avail > 0) {
    char buff[BTR_USART_RX_BUFF_SIZE];

    int bytes = (BTR_USART_RX_BUFF_SIZE < rx_q_avail ? BTR_USART_RX_BUFF_SIZE : rx_q_avail);
    bytes = usbd_ep_read_packet(usbd_dev, 0x01, buff, bytes);

    for (int i = 0; i < bytes; ++i) {
      xQueueSend(rx_q_, &buff[i], 0);
    }
  }
  gpio_toggle(BTR_BUILTIN_LED_PORT, BTR_BUILTIN_LED_PIN);
}

static void txTask(void* arg)
{
  usbd_device* usb_dev = (usbd_device *) arg;

  char buff[BTR_USART_TX_BUFF_SIZE];
  uint16_t bytes = 0;

  for (;;) {
    usbd_poll(usb_dev);

    if (ready_) {
      while (bytes < sizeof(buff) && xQueueReceive(tx_q_, &buff[bytes], 0) == pdPASS) {
        ++bytes;
      }

      if (bytes > 0) {
        if (usbd_ep_write_packet(usb_dev, 0x82, buff, bytes) != 0) {
          gpio_toggle(BTR_BUILTIN_LED_PORT, BTR_BUILTIN_LED_PIN);
          bytes = 0;
        }
      } else {
        taskYIELD();
      }
    } else  {
      taskYIELD();
    }
  }
}

static void setConfig(usbd_device* usbd_dev, uint16_t wval)
{
  (void) wval;

  usbd_ep_setup(usbd_dev, 0x01, USB_ENDPOINT_ATTR_BULK, BTR_USART_RX_BUFF_SIZE, onDataRecv);
  usbd_ep_setup(usbd_dev, 0x82, USB_ENDPOINT_ATTR_BULK, BTR_USART_TX_BUFF_SIZE, NULL);
  usbd_ep_setup(usbd_dev, 0x83, USB_ENDPOINT_ATTR_INTERRUPT, BTR_USART_IR_BUFF_SIZE, NULL);

  usbd_register_control_callback(
      usbd_dev,
      USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
      USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
      onCtrlRecv);

  ready_ = true;
}

} // extern "C"

namespace btr
{

static Usb usb_;

/////////////////////////////////////////////// PUBLIC /////////////////////////////////////////////

//============================================= LIFECYCLE ==========================================

//============================================= OPERATIONS =========================================

// static
Usb* Usb::instance(uint32_t id, bool open)
{
  (void) id;

  if (open) {
    Usb::open();
  }
  return &usb_;
}

bool Usb::isOpen()
{
  return ready_;
}

int Usb::open()
{
  if (false == isOpen()) {
    tx_q_ = xQueueCreate(BTR_USART_TX_BUFF_SIZE, sizeof(char));
    rx_q_ = xQueueCreate(BTR_USART_RX_BUFF_SIZE, sizeof(char));

    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_USB);

    usbd_device* usb_dev = usbd_init(
        &st_usbfs_v1_usb_driver, &usb_dev_info, &usb_cnf_info,
        usb_strings, 3,
        ctrl_buff_, sizeof(ctrl_buff_));

    usbd_register_set_config_callback(usb_dev, setConfig);

    xTaskCreate(txTask, "USB", configMINIMAL_STACK_SIZE, usb_dev, configMAX_PRIORITIES-1, NULL);
  }
  return 0;
}

void Usb::close()
{
  // There is no use case for closing USB device. Skip implementation until it's required.
#if 0
  if (true == isOpen()) {
    rcc_periph_clock_disable(RCC_USB);
    //rcc_periph_clock_disable(RCC_GPIOA);
  }
#endif
}

int Usb::available()
{
  return uxQueueMessagesWaiting(rx_q_);
}

int Usb::flush(DirectionType dir)
{
  int rc = 0;

  if (dir == DirectionType::OUT || dir == DirectionType::INOUT) {
    if ((rc = uxQueueMessagesWaiting(tx_q_)) > 0) {
      vTaskDelay(pdMS_TO_TICKS(BTR_USART_TX_DELAY_MS));
      rc = uxQueueMessagesWaiting(tx_q_);
    }
  }
  return rc;
}

// static
uint32_t Usb::send(const char* buff, uint16_t bytes, uint32_t timeout)
{
  uint32_t rc = 0;

  if (isOpen()) {
    while (bytes > 0) {
      if (pdPASS != xQueueSend(tx_q_, buff, pdMS_TO_TICKS(timeout))) {
        rc |= BTR_DEV_ETIMEOUT;
        break;
      }
      ++buff;
      ++rc;
      --bytes;
    }
  } else {
    rc = BTR_DEV_ENOTOPEN;
  }
  return rc;
}

// static
uint32_t Usb::recv(char* buff, uint16_t bytes, uint32_t timeout)
{
  uint32_t rc = 0;

  if (isOpen()) {
    while (bytes > 0) {
      if (pdPASS == xQueueReceive(rx_q_, buff, pdMS_TO_TICKS(timeout))) {
        ++buff;
        --bytes;
        ++rc;
      } else {
        // return can be errQUEUE_EMPTY as per PDF manual (not online)
        rc |= BTR_DEV_ETIMEOUT;
        break;
      }
    }

    rc |= (uint32_t(rx_error_) << 16);
    rx_error_ = 0;
  } else {
    rc = BTR_DEV_ENOTOPEN;
  }
  return rc;
}

} // namespace btr

#endif // BTR_USB0_ENABLED > 0
