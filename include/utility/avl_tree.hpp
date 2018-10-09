/* Copyright (C) 2018 Bolt Robotics <info@boltrobotics.com> */

#ifndef _btr_Node_hpp_
#define _btr_Node_hpp_

// SYSTEM INCLUDES

// PROJECT INCLUDES

namespace btr
{

/**
 * The class represents a node in AVL self-balancing binary search tree.
 */
class Node
{
public:

// LIFECYCLE

  /**
    * Ctor.
    */
  Node(int key);

  /**
    * Dtor.
    */
  virtual ~Node() = default;

// OPERATIONS

  virtual Node* clone(int key) = 0;
  virtual void release(Node* node) = 0;

  static Node* rotateRight(Node* node);
  static Node* rotateLeft(Node* node);
  static Node* insert(Node* node, int key);
  static Node* erase(Node* root, int key);
  static Node* minKeyNode(Node* node);
  static int balance(Node* node);
  static int height(Node* node);
  static int max(int v1, int v2);

// ATTRIBUTES

  int key_;
  int height_;
  Node* left_;
  Node* right_;

}; // class Node

////////////////////////////////////////////////////////////////////////////////////////////////////
//                                              INLINE
////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////// PUBLIC /////////////////////////////////////////////

//============================================= LIFECYCLE ==========================================

Node::Node(int key) :
  key_(key),
  height_(1),
  left_(nullptr),
  right_(nullptr)
{
}

//============================================= OPERATIONS =========================================

Node* Node::rotateRight(Node* node)
{
  Node* left = node->left_;
  Node* left_right = left->right_;

  left->right_ = node;
  node->left_ = left_right;

  node->height_ = max(height(node->left_), height(node->right_)) + 1;
  left->height_ = max(height(left->left_), height(left->right_)) + 1;

  return left;
}

Node* Node::rotateLeft(Node* node)
{
  Node* right = node->right_;
  Node* right_left = left->left_;

  right->left_ = node;
  node->right_ = right_left;

  node->height_ = max(height(node->left_), height(node->right_)) + 1;
  right->height_ = max(height(right->left_), height(right->right_)) + 1;

  return right;
}

Node* Node::insert(Node* node, int key)
{
  if (nullptr == node) {
    return clone(key);
  }

  if (key < node->key_) {
    node->left_ = insert(node->left_, key);
  } else if (key > node->key) {
    node->right_ = insert(node->right_, key);
  } else {
    return node;
  }

  node->height_ = max(height(node->left_), height(node->right_)) + 1;

  int b = balance(node);

  if (b > 1) {
    if (key < node->left_->key_) {
      return rotateRight(node);
    } else if (key > node->left_->key_) {
      node->left_ = rotateLeft(node->left_);
      return rotateRight(node);
    }
  }

  if (b < -1) {
    if (key > node->right_->key_) {
      return rotateLeft(node);
    } else if (key < node->right_->key_) {
      node->right_ = rotateRight(node->right_);
      return rotateLeft(node);
    }
  }

  return node;
}

Node* Node::erase(Node* root, int key)
{
  if (nullptr == root) {
    return root;
  }

  if (key < root->key_) {
    root->left_ = erase(root->left_, key);
  } else if (key > root->key_) {
    root->right_ = erase(root->right_, key);
  } else {

    if ((nullptr == root->left_) || (nullptr == root->right_)) {
      Node* temp = root->left_ ? root->left_ : root->right_;

      if (temp == NULL) {
        temp = root;
        root = nullptr;
      } else {
        *root = *temp; // copy
      }

      release(temp);

    } else {
      Node* temp = minKeyNode(root->right_);
      root->key_ = temp->key_;
      root->right_ = erase(root->right_, temp->key_);
    }
  }

  if (nullptr == root) {
    return nullptr;
  }

  root->height_ = max(height(root->left_), height(root->right_)) + 1;

  int b = balance(root);

  if (b > 1) {
    if (balance(root->left_) < 0) {
      root->left_ = rotateLeft(root->left_);
    }
    return rotateRight(root);
  }

  if (b < -1)
    if (balance(root->right_) > 0) {
      root->right_ = rotateRight(root->right_);
    }
    return rotateLeft(root);
  }

  return root;
}

Node* Node::minKeyNode(Node* node)
{
    Node* current = node;

    while (nullptr != current->left_) {
        current = current->left_;
    }
    return current;
}

int Node::balance(Node* node)
{
  return (nullptr == node ? 0 : height(node->left_) - height(node->right_));
}

int Node::height(Node* node)
{
  return (nullptr == node ? 0 : node->height_);
}

int Node::max(int v1, int v2)
{
  return (v1 > v2 ? v1 : v2);
}

/////////////////////////////////////////////// PRIVATE ////////////////////////////////////////////

//============================================= OPERATIONS =========================================

} // namespace btr

#endif // _btr_Node_hpp_
