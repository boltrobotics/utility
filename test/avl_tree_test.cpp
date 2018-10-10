/* Copyright (C) 2018 Bolt Robotics <info@boltrobotics.com> */

// SYSTEM INCLUDES
#include <gtest/gtest.h>
#include <iostream>
#include <sstream>
#include <limits>

// PROJECT INCLUDES
#include "utility/avl_tree.hpp"

namespace btr
{

//------------------------------------------------------------------------------

class Server
{
public:

// LIFECYCLE

  Server(int key)
    :
    key_(key),
    height_(1),
    left_(nullptr),
    right_(nullptr)
  {
  }

// OPERATIONS

  void onTraverse()
  {
    std::cout << key_ << " ";
  }

// ATTRIBUTES

  int key_;
  int height_;
  Server* left_;
  Server* right_;
};

//------------------------------------------------------------------------------

class AvlTest : public testing::Test
{
public:

// LIFECYCLE

  AvlTest() :
    tree_()
  {
  }

// ATTRIBUTES

  AvlTree<Server> tree_;
};

//------------------------------------------------------------------------------

TEST_F(AvlTest, insert)
{
  Server* root = nullptr;
  root = tree_.insert(root, 1);
  root = tree_.insert(root, 9);
  root = tree_.insert(root, 3);
  root = tree_.insert(root, 7);
  tree_.traverseInOrder(root);
  std::cout << std::endl;
  root = tree_.erase(root, 1);
  tree_.traverseInOrder(root);
  std::cout << std::endl;
  root = tree_.erase(root, 9);
  root = tree_.insert(root, 1);
  tree_.traverseInOrder(root);
  std::cout << std::endl;
}

} // namespace btr
