# Overview

The project contains portable C++ utility tools for

* Encoding/decoding numeric values to/from raw bytes
* Sharing pointer on embedded platforms
* Storing data in AVL tree
* Managing raw data in a buffer with read/write pointers, auto-resizing
* Improving output from unit tests
* Tracking number ranges for calculating delta, mean or median

# Table of Contents

* [Build](#Build)
* [Tools](#Tools)
  * [AvlTree](#avl_tree)
  * [Buff](#buff)
  * [Misc](#misc)
  * [SharedPtr](#shared_ptr)
  * [Sorters](#sorters)
  * [SpinLock](#spin_lock)
  * [TestHelpers](#test_helpers)
  * [ValueCodec](#value_codec)
  * [ValueTracker](#value_tracker)

 <a name="Build"></a>
# Build

Set up <a href="https://github.com/boltrobotics/cmake-helpers" target="_blank">cmake build</a> by
following an <a href="https://github.com/boltrobotics/cmake-helpers#Example">example</a>.

<a name="Tools"></a>
# Tools

<a name="avl_tree"></a>
<h3><a href="https://github.com/boltrobotics/utility/tree/master/include/utility/avl_tree.hpp" target="_blank">AvlTree</a></h3>

Implements an AVL tree. See usage examples in <a href="https://github.com/boltrobotics/utility/tree/master/test/avl_tree_test.cpp" target="_blank">avl_tree_test.cpp</a>

<a name="buff"></a> 
<h3><a href="https://github.com/boltrobotics/utility/tree/master/include/utility/buff.hpp" target="_blank">Buff</a></h3>

Implements dynamicaly resizable buffer. Provides read/write pointer tracking. See usage examples in
<a href="https://github.com/boltrobotics/utility/tree/master/test/buff_test.cpp" target="_blank">buff_test.cpp</a>

<a name="misc" ></a>
<h3><a href="https://github.com/boltrobotics/utility/tree/master/include/utility/misc.hpp" target="_blank">Misc</a></h3>

Implements various utility functions such as
* Translation between value ranges
* Calculation of angle delta squared
* Computing modulo with negative numbers
* Setting/clearing bits at given byte addresses
* Converting degrees to radians and vice versa
* Converting binary data to human-readable hex representation

See usage examples in
<a href="https://github.com/boltrobotics/utility/tree/master/test/misc_test.cpp" target="_blank">misc_test.cpp</a>

<a name="shared_ptr"></a>
<h3><a href="https://github.com/boltrobotics/utility/tree/master/include/utility/shared_ptr.hpp" target="_blank">SharedPtr</a></h3>

Implements basic shared pointer to be used on an embedded platform. See usage examples in
<a href="https://github.com/boltrobotics/utility/tree/master/test/shared_ptr_test.cpp" target="_blank">shared_ptr_test.cpp</a>

<a name="sorters"></a>
<h3><a href="https://github.com/boltrobotics/utility/tree/master/include/utility/sorters.hpp" target="_blank">Sorters</a></h3>

Implements sorting algorithms for sorting a plain-old-data array. See usage examples in
<a href="https://github.com/boltrobotics/utility/tree/master/test/sorters_test.cpp" target="_blank">sorters_test.cpp</a>

<a name="spin_lock" ></a>
<h3><a href="https://github.com/boltrobotics/utility/tree/master/include/utility/spin_lock.hpp" target="_blank">SpinLock</a></h3>

Implements a basic spin lock for x86.

<a name="test_helpers" ></a>
<h3><a href="https://github.com/boltrobotics/utility/tree/master/include/utility/test_helpers.hpp" target="_blank">TestHelpers</a></h3>

Implements useful facilities for when unit testing. See usage examples in unit tests, example: 
<a href="https://github.com/boltrobotics/utility/tree/master/test/value_codec_test.cpp" target="_blank">value_codec_test.cpp</a>

<a name="value_codec" ></a>
<h3><a href="https://github.com/boltrobotics/utility/tree/master/include/utility/value_codec.hpp" target="_blank">ValueCodec</a></h3>

Implements number encoding and decoding facilities accounting for target host order. See usage
examples in
<a href="https://github.com/boltrobotics/utility/tree/master/test/value_codec_test.cpp" target="_blank">value_codec_test.cpp</a>

<a name="value_tracker"></a>
<h3><a href="https://github.com/boltrobotics/utility/tree/master/include/utility/value_tracker.hpp" target="_blank">ValueTracker</a></h3>

The class keeps track of numeric values so as to calculate the delta, mean or median between them.
See usage examples in
<a href="https://github.com/boltrobotics/utility/tree/master/test/value_tracker_test.cpp" target="_blank">value_tracker_test.cpp</a>
