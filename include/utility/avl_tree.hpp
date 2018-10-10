/* Copyright (C) 2018 Bolt Robotics <info@boltrobotics.com> */

#ifndef _btr_AvlTree_hpp_
#define _btr_AvlTree_hpp_

namespace btr
{

////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 *
 */
template <typename NodeType>
class NodeObserver
{
public:

// LIFECYCLE

  NodeObserver() = default;
  virtual ~NodeObserver() = default;

// OPERATIONS

  virtual void onTraverse(NodeType* node) = 0;
};

////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Base class that aggregates common variables and encapsulates access to them.
 */
template <typename NodeType>
class NodeBase
{
public:

// LIFECYCLE

  /**
   * Ctor.
   *
   * @param key - node key
   */
  NodeBase(int key);

  /**
   * Dtor.
   */
  virtual ~NodeBase() = default;

// OPERATIONS

  int key() const;
  void key(int k);
  int height() const;
  void height(int height);
  NodeType* left();
  void left(NodeType* node);
  NodeType* right();
  void right(NodeType* node);

private:

// ATTRIBUTES

  int key_;
  int height_;
  NodeType* left_;
  NodeType* right_;
};

////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * The class represents an AVL self-balancing binary search tree.
 */
template<typename NodeType>
class AvlTree
{
public:

// LIFECYCLE

  /**
   * Ctor.
   */
  AvlTree();

// OPERATIONS

  NodeType* root();
  void root(NodeType* root);
  NodeType* rotateRight(NodeType* node);
  NodeType* rotateLeft(NodeType* node);
  NodeType* insert(NodeType* node, int key);
  NodeType* erase(NodeType* root, int key);

  static int balance(NodeType* node);
  static int height(NodeType* node);
  static int max(int v1, int v2);
  static void traverseInOrder(NodeType* node, NodeObserver<NodeType>* o);
  static NodeType* searchMin(NodeType* node);
  static NodeType* search(NodeType* root, int key);

private:

// ATTRIBUTES

  NodeType* root_;

}; // class AvlTree

////////////////////////////////////////////////////////////////////////////////////////////////////
//                                              INLINE
////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////// PUBLIC /////////////////////////////////////////////

//============================================= LIFECYCLE ==========================================

template<typename NodeType>
inline NodeBase<NodeType>::NodeBase(int key) :
  key_(key),
  height_(1),
  left_(nullptr),
  right_(nullptr)
{
}

template<typename NodeType>
inline AvlTree<NodeType>::AvlTree() :
  root_(nullptr)
{
}

//============================================= OPERATIONS =========================================

template<typename NodeType>
inline int NodeBase<NodeType>::key() const
{
  return key_;
}

template<typename NodeType>
inline void NodeBase<NodeType>::key(int key)
{
  key_ = key;
}

template<typename NodeType>
inline int NodeBase<NodeType>::height() const
{
  return height_;
}

template<typename NodeType>
inline void NodeBase<NodeType>::height(int height)
{
  height_ = height;
}

template<typename NodeType>
inline NodeType* NodeBase<NodeType>::left()
{
  return left_;
}

template<typename NodeType>
inline void NodeBase<NodeType>::left(NodeType* node)
{
  left_ = node;
}

template<typename NodeType>
inline NodeType* NodeBase<NodeType>::right()
{
  return right_;
}

template<typename NodeType>
inline void NodeBase<NodeType>::right(NodeType* node)
{
  right_ = node;
}

template<typename NodeType>
inline NodeType* AvlTree<NodeType>::root()
{
  return root_;
}

template<typename NodeType>
inline void AvlTree<NodeType>::root(NodeType* root)
{
  root_ = root;
}

template<typename NodeType>
inline NodeType* AvlTree<NodeType>::rotateRight(NodeType* node)
{
  NodeType* left = node->left();
  NodeType* left_right = left->right();

  left->right(node);
  node->left(left_right);

  node->height(max(height(node->left()), height(node->right())) + 1);
  left->height(max(height(left->left()), height(left->right())) + 1);

  root_ = left;
  return left;
}

template<typename NodeType>
inline NodeType* AvlTree<NodeType>::rotateLeft(NodeType* node)
{
  NodeType* right = node->right();
  NodeType* right_left = right->left();

  right->left(node);
  node->right(right_left);

  node->height(max(height(node->left()), height(node->right())) + 1);
  right->height(max(height(right->left()), height(right->right())) + 1);

  root_ = right;
  return right;
}

template<typename NodeType>
inline NodeType* AvlTree<NodeType>::insert(NodeType* node, int key)
{
  if (nullptr == node) {
    node = new NodeType(key);
    root_ = node;
    return node;
  }

  if (key < node->key()) {
    node->left(insert(node->left(), key));
  } else if (key > node->key()) {
    node->right(insert(node->right(), key));
  } else {
    root_ = node;
    return node;
  }

  node->height(max(height(node->left()), height(node->right())) + 1);

  int b = balance(node);

  if (b > 1) {
    if (key < node->left()->key()) {
      node = rotateRight(node);
    } else if (key > node->left()->key()) {
      node->left(rotateLeft(node->left()));
      node = rotateRight(node);
    }
  } else if (b < -1) {
    if (key > node->right()->key()) {
      node = rotateLeft(node);
    } else if (key < node->right()->key()) {
      node->right(rotateRight(node->right()));
      node = rotateLeft(node);
    }
  }

  root_ = node;
  return node;
}

template<typename NodeType>
inline NodeType* AvlTree<NodeType>::erase(NodeType* root, int key)
{
  if (nullptr == root) {
    return nullptr;
  }

  if (key < root->key()) {
    root->left(erase(root->left(), key));
  } else if (key > root->key()) {
    root->right(erase(root->right(), key));
  } else {
    // Erase root node
    //
    if ((nullptr == root->left()) || (nullptr == root->right())) {
      NodeType* temp = root->left() ? root->left() : root->right();

      if (nullptr == temp) {
        // No child nodes
        temp = root;
        root = nullptr;
      } else {
        // Shallow copy data members from the one child node
        *root = *temp;
      }

      delete temp;

    } else {
      // Root has both child nodes. Back up the data of root-to-delete so as to reuse its
      // allocated memory chunk.
      NodeType* root_left = root->left();
      NodeType* root_right = root->right();
      int height = root->height();

      NodeType* temp = searchMin(root->right());

      // Shallow-copy data and restore left/right/height.
      *root = *temp;
      root->left(root_left);
      root->right(root_right);
      root->height(height);

      root->right(erase(root->right(), temp->key()));
    }
  }

  if (nullptr == root) {
    return nullptr;
  }

  root->height(max(height(root->left()), height(root->right())) + 1);

  int b = balance(root);

  if (b > 1) {
    if (balance(root->left()) < 0) {
      root->left(rotateLeft(root->left()));
    }
    return rotateRight(root);
  }

  if (b < -1) {
    if (balance(root->right()) > 0) {
      root->right(rotateRight(root->right()));
    }
    return rotateLeft(root);
  }

  root_ = root;
  return root;
}

template<typename NodeType>
int AvlTree<NodeType>::balance(NodeType* node)
{
  return (nullptr == node ? 0 : height(node->left()) - height(node->right()));
}

template<typename NodeType>
int AvlTree<NodeType>::height(NodeType* node)
{
  return (nullptr == node ? 0 : node->height());
}

template<typename NodeType>
int AvlTree<NodeType>::max(int v1, int v2)
{
  return (v1 > v2 ? v1 : v2);
}

template<typename NodeType>
void AvlTree<NodeType>::traverseInOrder(NodeType* node, NodeObserver<NodeType>* o)
{
  if (nullptr == node) {
    return;
  }

  traverseInOrder(node->left(), o);
  //node->onTraverse();
  o->onTraverse(node);
  traverseInOrder(node->right(), o);
}

template<typename NodeType>
NodeType* AvlTree<NodeType>::searchMin(NodeType* node)
{
    NodeType* current = node;

    while (nullptr != current->left()) {
        current = current->left();
    }
    return current;
}

template<typename NodeType>
NodeType* AvlTree<NodeType>::search(NodeType* root, int key)
{
  if (nullptr == root || root->key() == key) {
    return root;
  }

  if (root->key() < key) {
    return search(root->right(), key);
  }
  return search(root->left(), key);
}

/////////////////////////////////////////////// PRIVATE ////////////////////////////////////////////

//============================================= OPERATIONS =========================================

} // namespace btr

#endif // _btr_AvlTree_hpp_
