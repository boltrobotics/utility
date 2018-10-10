/* Copyright (C) 2018 Bolt Robotics <info@boltrobotics.com> */

// SYSTEM INCLUDES
#include <gtest/gtest.h>
#include <iostream>
#include <sstream>
#include <limits>

// PROJECT INCLUDES
#include "utility/avl_tree.hpp"
#include "utility/test_helpers.hpp"

namespace btr
{

//------------------------------------------------------------------------------

class Server : public NodeBase<Server>, public NodeObserver<Server>
{
public:

// LIFECYCLE

  Server(int key) :
    NodeBase(key)
  {}

// OPERATIONS

  /**
   * @override NodeObserver::onTraverse
   */
  virtual void onTraverse(Server* node)
  {
    keys_.push_back(node->key());
  }

// ATTRIBUTES

  std::vector<int> keys_;
};

//------------------------------------------------------------------------------

class AvlTreeTest : public testing::Test
{
public:

// LIFECYCLE

  AvlTreeTest() :
    tree_(),
    root_(nullptr),
    observer_(0)
  {
  }

// ATTRIBUTES

  AvlTree<Server> tree_;
  Server* root_;
  Server observer_;
};

//------------------------------------------------------------------------------

TEST_F(AvlTreeTest, traverseInOrder)
{
  int keys[] = { 1, 9, 3, 7 };

  for (uint8_t i = 0; i < sizeof(keys)/sizeof(int); i++) {
    root_ = tree_.insert(root_, keys[i]);
  }

  tree_.traverseInOrder(tree_.root(), &observer_);

  ASSERT_EQ(keys[0], observer_.keys_[0]);
  ASSERT_EQ(keys[2], observer_.keys_[1]);
  ASSERT_EQ(keys[3], observer_.keys_[2]);
  ASSERT_EQ(keys[1], observer_.keys_[3]);
}

TEST_F(AvlTreeTest, search)
{
  int keys[] = { 1, 9, 3, 7 };

  for (uint8_t i = 0; i < sizeof(keys)/sizeof(int); i++) {
    root_ = tree_.insert(root_, keys[i]);
  }

  ASSERT_EQ(3, root_->key());
  ASSERT_EQ(3, tree_.root()->key());
  ASSERT_TRUE(root_ == tree_.root());

  for (uint8_t i = 0; i < sizeof(keys)/sizeof(int); i++) {
    Server* n = tree_.search(tree_.root(), keys[i]);
    ASSERT_TRUE(nullptr != n) << "Key: " << keys[i] << std::endl;
  }
}

TEST_F(AvlTreeTest, erase)
{
  int keys[] = { 1, 9, 3, 7 };

  for (uint8_t i = 0; i < sizeof(keys)/sizeof(int); i++) {
    root_ = tree_.insert(root_, keys[i]);
  }

  Server* s_node = tree_.search(tree_.root(), 3);
  ASSERT_TRUE(nullptr != s_node);
  ASSERT_TRUE(tree_.root() == s_node);
  ASSERT_EQ(3, s_node->key());
  s_node->keys_.clear();
  s_node->keys_.push_back(91);
  s_node->keys_.push_back(88);

  // After deleting root node 3, node 7 becomes root. We expect data members to be copied over
  // from 3 to 7.
  Server* e_node = tree_.erase(tree_.root(), 3);
  s_node = tree_.search(tree_.root(), 3);
  ASSERT_TRUE(nullptr == s_node);

  e_node = tree_.erase(tree_.root(), 9);
  s_node = tree_.search(tree_.root(), 9);
  ASSERT_TRUE(nullptr == s_node);

  s_node = tree_.search(tree_.root(), 7);
  ASSERT_TRUE(tree_.root() == s_node);
  ASSERT_EQ(tree_.root()->key(), s_node->key());
  ASSERT_EQ(1, s_node->left()->key());
  ASSERT_TRUE(nullptr == s_node->right());
  ASSERT_EQ(91, s_node->keys_[0]);
  ASSERT_EQ(88, s_node->keys_[1]);
}

} // namespace btr
