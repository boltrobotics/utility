The project contains portable C++ code for
* encoding/decoding numeric values to/from raw bytes
* sharing pointer on embedded platforms
* storing data in AVL tree
* managing raw data in a buffer with read/write pointers, auto-resizing
* improving output from unit tests
* tracking number ranges for calculating delta, mean or median

## Table of Contents

* [Build](#Build)
* [Unit Tests](#Unit_Tests)
* [Utility Classes](#Utility_Classes)
  * [AvlTree](#avl_tree)
  * [Buff](#buff)
  * [Misc](#misc)
  * [SharedPtr](#shared_ptr)
  * [Sorters](#sorters)
  * [SpinLock](#spin_lock)
  * [TestHelpers](#test_helpers)
  * [ValueCodec](#value_codec)
  * [ValueTracker](#value_tracker)

<a name="Build" ></a>
## Build

Set up the build following an <a href="https://github.com/boltrobotics/cmake-helpers/#Example">example</a> from
cmake-helpers project.

<a name="Unit_Tests" ></a>
## Unit Tests

Build unit tests with
```
make.sh -x -t
```

Run the tests with
```
./build-x86/Release/bin/utility-tests
```
or
```
cd build-x86 && ninja test
```

<a name="Utility_Classes" ></a>
## Utility Classes

<a name="avl_tree" ></a>
### AvlTree

Implements an AVL tree. See usage examples in avl_tree_test.cpp

<a name="buff"></a> 
### Buff

Implements dynamicaly resizable buffer. Provides read/write pointer tracking. See usage examples in
buff_test.cpp

<a name="misc" ></a>
### <a href="include/utility/misc.hpp">Misc</a>

Implements various utility functions such as
* Translation between value ranges
* Calculation of angle delta squared
* Computing modulo with negative numbers
* Setting/clearing bits at given byte addresses
* Converting degrees to radians and vice versa
* Converting binary data to human-readable hex representation

See usage examples in <a href="test/misc_test.cpp">misc_test.cpp</a>

<a name="shared_ptr"></a>
### <a href="include/utility/shared_ptr.hpp">SharedPtr</a>

Implements basic shared pointer to be used on an embedded platform. See usage examples in
<a href="test/shared_ptr_test.cpp">shared_ptr_test.cpp</a>

<a name="sorters"></a>
### <a href="include/utility/sorters.hpp">Sorters</a>

Implements sorting algorithms for sorting a plain-old-data array. See usage examples in
<a href="test/sorters_test.cpp">sorters_test.cpp</a>

<a name="spin_lock" ></a>
### <a href="include/utility/spin_lock.hpp">SpinLock</a>

Implements a basic spin lock for x86.

<a name="test_helpers" ></a>
### <a href="include/utility/test_helpers.hpp">TestHelpers</a>

Implements useful facilities for when unit testing. See usage examples in unit tests, example: 
<a href="test/value_codec_test.cpp">value_codec_test.cpp</a>

<a name="value_codec" ></a>
### <a href="include/utility/value_codec.hpp">ValueCodec</a>

Implements number encoding and decoding facilities accounting for target host order. See usage
examples in <a href="test/value_codec_test.cpp">value_codec_test.cpp</a>

<a name="value_tracker"></a>
### <a href="include/utility/value_tracker.hpp">ValueTracker</a>

The class keeps track of numeric values so as to calculate the delta, mean or median between them.
See usage examples in
<a href="test/value_tracker_test.cpp">value_tracker_test.cpp</a>

<a name="x86"></a>
## x86

<a name="PseudoTTY"></a>
### <a href="include/devices/x86/pseudo_tty.hpp">PseudoTTY</a>

The class creates two serial devices using <a href="https://linux.die.net/man/1/socat">socat</a>
utility. PseudoTTY is useful for testing serial client or server. The unit tests for Usart class
use it for similating serial ports.

<a name="Usart"></a>
### <a href="include/devices/usart.hpp">Usart</a>

The class provides an interface for communication over serial connection. Each read/write is
configured to time out if the operation doesn't complete within a specified window. The code uses
Boost ASIO library.

<a name="usart_test" href="test/usart_test.cpp">usart_test.cpp</a>
contains unit tests for Usart class.

<a name="UsartTermios"></a> 
### <a href="include/devices/x86/usart_termios.hpp">Usart Termios</a>

Class functionality is similar to [Usart](#Usart) but using termios library.

<a name="usart_termios_test" href="test/usart_termios_test.cpp">usart_termios_test.cpp</a>
contains unit tests for Usart class.

<a name="stm32"></a>
## STM32

<a name="Usb"></a>
### <a href="include/devices/stm32/usb.hpp">Usb</a>

The class provides an interface to transfer data over USB connection.

<a name="stm32_Usart"></a>
### <a href="include/devices/stm32/usart.hpp">Usart</a>

The class provides an interface for communication over serial connection.

<a name="PwmMotor3Wire"></a>
### <a href="include/devices/stm32/pwm_motor_3wire.hpp">PwmMotor3Wire</a>

The class can drive a motor that uses three wires for direction and speed control.

<a href="include/devices/avr/pwm_motor_3wire.hpp">PwmMotor3Wire for AVR</a>

<a name="PwmMotor2Wire"></a>
### <a href="include/devices/stm32/pwm_motor_2wire.hpp">PwmMotor2Wire</a>

The class can drive a motor that uses two wires for direction and speed control.

<a name="avr"></a>
## AVR

## Common Code

<a name="MaxSonarLvEx"></a>
### <a href="include/devices/maxsonar_lvez.hpp">MaxSonarLvEx</a>

The class calculates range in millimeters from an ADC sample of MaxSonar ultrasonic range finder.

<a name="WheelEncoder"></a>
### <a href="include/devices/wheel_encoder.hpp">WheelEncoder</a>

The class counts the number of clicks that a virtual wheel moved forward and backward.
