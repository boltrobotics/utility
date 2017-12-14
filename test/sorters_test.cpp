/* Copyright (C) 2017 Bolt Robotics <info@boltrobotics.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>. */

// SYSTEM INCLUDES
#include <gtest/gtest.h>

// PROJECT INCLUDES
#include "sorters.hpp"

//================================ TEST FIXTURES ===============================

//=================================== TESTS ====================================

namespace utility {

template<typename T, size_t size>
::testing::AssertionResult ArraysMatch(
        const T (&expected)[size], const T (&actual)[size]){

	for (size_t i = 0; i < size; ++i) {
		if (expected[i] != actual[i]) {
			return ::testing::AssertionFailure()
                << "actual[" << i << "] (" << actual[i] << ") != expected["
                << i << "] (" << expected[i] << ")";
		}
	}
	return ::testing::AssertionSuccess();
}

TEST(SortersTest, testInsertionSort) {
    const uint16_t data_odd_sorted[] = { 2, 4, 5, 7, 16 };
    uint16_t data_odd[] = { 7, 5, 2, 16, 4 };
    Sorters::insertionSort(data_odd, sizeof(data_odd_sorted) / sizeof(uint16_t));
    ASSERT_TRUE(ArraysMatch(data_odd_sorted, data_odd));

    const uint16_t data_even_sorted[] = { 2, 4, 5, 5, 7, 16 };
    uint16_t data_even[] = { 7, 5, 2, 16, 4, 5 };
    Sorters::insertionSort(data_even, sizeof(data_even_sorted) / sizeof(uint16_t));
    ASSERT_TRUE(ArraysMatch(data_even_sorted, data_even));
}

} // namespace utility
