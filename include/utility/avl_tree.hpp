/* Copyright (C) 2018 Bolt Robotics <info@boltrobotics.com> */

#ifndef _btr_AvlTree_hpp_
#define _btr_AvlTree_hpp_

// SYSTEM INCLUDES

// PROJECT INCLUDES

namespace btr
{

/**
 * The class represents a node in AVL self-balancing binary search tree.
 */
template<typename NodeType>
class AvlTree
{
public:

// OPERATIONS

  static NodeType* rotateRight(NodeType* node);
  static NodeType* rotateLeft(NodeType* node);
  static NodeType* insert(NodeType* node, int key);
  static NodeType* erase(NodeType* root, int key);
  static int balance(NodeType* node);
  static int height(NodeType* node);
  static int max(int v1, int v2);
  static void traverseInOrder(NodeType* node);
  static NodeType* searchMin(NodeType* node);
  static NodeType* search(NodeType* root, int key);

}; // class AvlTree

////////////////////////////////////////////////////////////////////////////////////////////////////
//                                              INLINE
////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////// PUBLIC /////////////////////////////////////////////

//============================================= LIFECYCLE ==========================================
//============================================= OPERATIONS =========================================

template<typename NodeType>
NodeType* AvlTree<NodeType>::rotateRight(NodeType* node)
{
  NodeType* left = node->left_;
  NodeType* left_right = left->right_;

  left->right_ = node;
  node->left_ = left_right;

  node->height_ = max(height(node->left_), height(node->right_)) + 1;
  left->height_ = max(height(left->left_), height(left->right_)) + 1;

  return left;
}

template<typename NodeType>
NodeType* AvlTree<NodeType>::rotateLeft(NodeType* node)
{
  NodeType* right = node->right_;
  NodeType* right_left = right->left_;

  right->left_ = node;
  node->right_ = right_left;

  node->height_ = max(height(node->left_), height(node->right_)) + 1;
  right->height_ = max(height(right->left_), height(right->right_)) + 1;

  return right;
}

template<typename NodeType>
NodeType* AvlTree<NodeType>::insert(NodeType* node, int key)
{
  if (nullptr == node) {
    return new NodeType(key);
  }

  if (key < node->key_) {
    node->left_ = insert(node->left_, key);
  } else if (key > node->key_) {
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

template<typename NodeType>
NodeType* AvlTree<NodeType>::erase(NodeType* root, int key)
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
      NodeType* temp = root->left_ ? root->left_ : root->right_;

      if (temp == NULL) {
        temp = root;
        root = nullptr;
      } else {
        *root = *temp; // copy
      }

      delete temp;

    } else {
      NodeType* temp = searchMin(root->right_);
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

  if (b < -1) {
    if (balance(root->right_) > 0) {
      root->right_ = rotateRight(root->right_);
    }
    return rotateLeft(root);
  }

  return root;
}

template<typename NodeType>
int AvlTree<NodeType>::balance(NodeType* node)
{
  return (nullptr == node ? 0 : height(node->left_) - height(node->right_));
}

template<typename NodeType>
int AvlTree<NodeType>::height(NodeType* node)
{
  return (nullptr == node ? 0 : node->height_);
}

template<typename NodeType>
int AvlTree<NodeType>::max(int v1, int v2)
{
  return (v1 > v2 ? v1 : v2);
}

template<typename NodeType>
void AvlTree<NodeType>::traverseInOrder(NodeType* node)
{
  if (nullptr == node) {
    return;
  }

  traverseInOrder(node->left_);
  node->onTraverse();
  traverseInOrder(node->right_);
}

template<typename NodeType>
NodeType* AvlTree<NodeType>::searchMin(NodeType* node)
{
    NodeType* current = node;

    while (nullptr != current->left_) {
        current = current->left_;
    }
    return current;
}

template<typename NodeType>
NodeType* AvlTree<NodeType>::search(NodeType* root, int key)
{
  if (nullptr == root || root->key_ == key) {
    return root;
  }

  if (root->key_ < key) {
    return search(root->right_, key);
  }
  return search(root->left_, key);
}

/////////////////////////////////////////////// PRIVATE ////////////////////////////////////////////

//============================================= OPERATIONS =========================================

} // namespace btr

#endif // _btr_AvlTree_hpp_
