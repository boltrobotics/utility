/* Copyright (C) 2018 Bolt Robotics <info@boltrobotics.com> */

#ifndef _btr_AvlTree_hpp_
#define _btr_AvlTree_hpp_

namespace btr
{

////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 *
 */
template <typename N>
class NodeObserver
{
public:

// LIFECYCLE

  NodeObserver() = default;
  virtual ~NodeObserver() = default;

// OPERATIONS

  virtual void onTraverse(N* node) = 0;
};

////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Base class that aggregates common variables and encapsulates access to them.
 *
 * WARNING: make sure H is SIGNED integer
 */
template <typename N, typename K = uint16_t, typename H = int16_t>
class NodeBase
{
public:

// LIFECYCLE

  /**
   * Ctor.
   *
   * @param key - node key
   */
  NodeBase(K key);

  /**
   * Dtor.
   */
  virtual ~NodeBase() = default;

// OPERATIONS

  K key() const;
  void key(K k);
  H height() const;
  void height(H height);
  N* left();
  void left(N* node);
  N* right();
  void right(N* node);

private:

// ATTRIBUTES

  K key_;
  H height_;
  N* left_;
  N* right_;
};

////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * The class represents an AVL self-balancing binary search tree.
 *
 * WARNING: make sure H is SIGNED integer
 */
template <typename N, typename K = uint16_t, typename H = int16_t>
class AvlTree
{
public:

// LIFECYCLE

  /**
   * Ctor.
   */
  AvlTree();

  /**
   * Erase all branches on destruction.
   */
  ~AvlTree();

// OPERATIONS

  N* root();
  void root(N* root);
  N* rotateRight(N* node);
  N* rotateLeft(N* node);
  N* insert(K key);
  N* search(K key);
  N* erase(K key);
  void eraseBranch(N* node);

  static H balance(N* node);
  static H height(N* node);
  static H max(H v1, H v2);
  static void traverseInOrder(N* node, NodeObserver<N>* o);
  static N* searchMin(N* node);
  static N* search(N* root, K key);

private:

// OPERATIONS

  /**
   * Insert a new node at the node position.
   *
   * @return new root
   */
  N* insert(N* node, K key);

  /**
   * Erase a node at the node position.
   *
   * @return new root
   */
  N* erase(N* root, K key);

// ATTRIBUTES

  N* root_;

}; // class AvlTree

////////////////////////////////////////////////////////////////////////////////////////////////////
//                                              INLINE
////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////// PUBLIC /////////////////////////////////////////////

//============================================= LIFECYCLE ==========================================

template<typename N, typename K, typename H>
inline NodeBase<N, K, H>::NodeBase(K key) :
  key_(key),
  height_(1),
  left_(nullptr),
  right_(nullptr)
{
}

template<typename N, typename K, typename H>
inline AvlTree<N, K, H>::AvlTree() :
  root_(nullptr)
{
}

template<typename N, typename K, typename H>
inline AvlTree<N, K, H>::~AvlTree()
{
  eraseBranch(root_);
}

//============================================= OPERATIONS =========================================

template<typename N, typename K, typename H>
inline K NodeBase<N, K, H>::key() const
{
  return key_;
}

template<typename N, typename K, typename H>
inline void NodeBase<N, K, H>::key(K key)
{
  key_ = key;
}

template<typename N, typename K, typename H>
inline H NodeBase<N, K, H>::height() const
{
  return height_;
}

template<typename N, typename K, typename H>
inline void NodeBase<N, K, H>::height(H height)
{
  height_ = height;
}

template<typename N, typename K, typename H>
inline N* NodeBase<N, K, H>::left()
{
  return left_;
}

template<typename N, typename K, typename H>
inline void NodeBase<N, K, H>::left(N* node)
{
  left_ = node;
}

template<typename N, typename K, typename H>
inline N* NodeBase<N, K, H>::right()
{
  return right_;
}

template<typename N, typename K, typename H>
inline void NodeBase<N, K, H>::right(N* node)
{
  right_ = node;
}

template<typename N, typename K, typename H>
inline N* AvlTree<N, K, H>::root()
{
  return root_;
}

template<typename N, typename K, typename H>
inline void AvlTree<N, K, H>::root(N* root)
{
  root_ = root;
}

template<typename N, typename K, typename H>
inline N* AvlTree<N, K, H>::rotateRight(N* node)
{
  N* left = node->left();
  N* left_right = left->right();

  left->right(node);
  node->left(left_right);

  node->height(max(height(node->left()), height(node->right())) + 1);
  left->height(max(height(left->left()), height(left->right())) + 1);

  root_ = left;
  return left;
}

template<typename N, typename K, typename H>
inline N* AvlTree<N, K, H>::rotateLeft(N* node)
{
  N* right = node->right();
  N* right_left = right->left();

  right->left(node);
  node->right(right_left);

  node->height(max(height(node->left()), height(node->right())) + 1);
  right->height(max(height(right->left()), height(right->right())) + 1);

  root_ = right;
  return right;
}

template<typename N, typename K, typename H>
inline N* AvlTree<N, K, H>::insert(K key)
{
  N* n = insert(root_, key);
  root_ = n;
  return n;
}

template<typename N, typename K, typename H>
N* AvlTree<N, K, H>::search(K key)
{
  return search(root_, key);
}

template<typename N, typename K, typename H>
inline N* AvlTree<N, K, H>::erase(K key)
{
  N* n = erase(root_, key);
  root_ = n;
  return n;
}

template<typename N, typename K, typename H>
void AvlTree<N, K, H>::eraseBranch(N* node)
{
  if (nullptr == node) {
    return;
  }

  eraseBranch(node->left());
  eraseBranch(node->right());
  delete node;

  if (root_ == node) {
    root_ = nullptr;
  }
}

template<typename N, typename K, typename H>
H AvlTree<N, K, H>::balance(N* node)
{
  return (nullptr == node ? 0 : height(node->left()) - height(node->right()));
}

template<typename N, typename K, typename H>
H AvlTree<N, K, H>::height(N* node)
{
  return (nullptr == node ? 0 : node->height());
}

template<typename N, typename K, typename H>
H AvlTree<N, K, H>::max(H v1, H v2)
{
  return (v1 > v2 ? v1 : v2);
}

template<typename N, typename K, typename H>
void AvlTree<N, K, H>::traverseInOrder(N* node, NodeObserver<N>* o)
{
  if (nullptr == node) {
    return;
  }

  traverseInOrder(node->left(), o);
  o->onTraverse(node);
  traverseInOrder(node->right(), o);
}

template<typename N, typename K, typename H>
N* AvlTree<N, K, H>::searchMin(N* node)
{
    N* current = node;

    while (nullptr != current->left()) {
        current = current->left();
    }
    return current;
}

template<typename N, typename K, typename H>
N* AvlTree<N, K, H>::search(N* root, K key)
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

template<typename N, typename K, typename H>
inline N* AvlTree<N, K, H>::insert(N* node, K key)
{
  if (nullptr == node) {
    node = new N(key);
    return node;
  }

  if (key < node->key()) {
    node->left(insert(node->left(), key));
  } else if (key > node->key()) {
    node->right(insert(node->right(), key));
  } else {
    // Invalid condition as keys are the same. No new node is inserted.
    return node;
  }

  node->height(max(height(node->left()), height(node->right())) + 1);

  H b = balance(node);

  if (b > 1) {
    if (nullptr != node->left()) {
      if (key < node->left()->key()) {
        node = rotateRight(node);
      } else if (key > node->left()->key()) {
        node->left(rotateLeft(node->left()));
        node = rotateRight(node);
      }
    }
  } else if (b < -1) {
    if (nullptr != node->right()) {
      if (key > node->right()->key()) {
        node = rotateLeft(node);
      } else if (key < node->right()->key()) {
        node->right(rotateRight(node->right()));
        node = rotateLeft(node);
      }
    }
  }

  return node;
}

template<typename N, typename K, typename H>
inline N* AvlTree<N, K, H>::erase(N* root, K key)
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
      N* temp = root->left() ? root->left() : root->right();

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
      N* root_left = root->left();
      N* root_right = root->right();
      H height = root->height();

      N* temp = searchMin(root->right());

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

  H b = balance(root);

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

  return root;
}

} // namespace btr

#endif // _btr_AvlTree_hpp_
