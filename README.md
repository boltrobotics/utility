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
### <a href="include/utility/avl_tree.hpp">AvlTree</a>

Implements an AVL tree. See usage examples in <a href="test/avl_tree_test.cpp">
avl_tree_test.cpp</a>

<a name="buff"></a> 
### <a href="include/utility/buff.hpp">Buff</a>

Implements dynamicaly resizable buffer. Provides read/write pointer tracking. See usage examples in
<a href="test/buff_test.cpp">buff_test.cpp</a>

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
