#ifndef MUTABLEPRIORITYQUEUE
#define MUTABLEPRIORITYQUEUE

#include <vector>

/**
 * class T must have: (i) accessible field int queueIndex; (ii) operator< defined.
 */

template <class T>
class mutable_priority_queue {
    std::vector<T *> H;
    void heapifyUp(unsigned i);
    void heapifyDown(unsigned i);
    inline void set(unsigned i, T * x);
public:
    mutable_priority_queue();
    void insert(T * x);
    T * extractMin();
    void decreaseKey(T * x);
    bool empty();
};

// Index calculations
#define parent(i) ((i) / 2)
#define leftChild(i) ((i) * 2)

template <class T>
mutable_priority_queue<T>::mutable_priority_queue() {
    H.push_back(nullptr);
    // indices will be used starting in 1
    // to facilitate parent/child calculations
}

template <class T>
bool mutable_priority_queue<T>::empty() {
    return H.size() == 1;
}

template <class T>
T* mutable_priority_queue<T>::extractMin() {
    auto x = H[1];
    H[1] = H.back();
    H.pop_back();
    if(H.size() > 1) heapifyDown(1);
    x->queueIndex = 0;
    return x;
}

template <class T>
void mutable_priority_queue<T>::insert(T *x) {
    H.push_back(x);
    heapifyUp(H.size()-1);
}

template <class T>
void mutable_priority_queue<T>::decreaseKey(T *x) {
    heapifyUp(x->queueIndex);
}

template <class T>
void mutable_priority_queue<T>::heapifyUp(unsigned i) {
    auto x = H[i];
    while (i > 1 && *x < *H[parent(i)]) {
        set(i, H[parent(i)]);
        i = parent(i);
    }
    set(i, x);
}

template <class T>
void mutable_priority_queue<T>::heapifyDown(unsigned i) {
    auto x = H[i];
    while (true) {
        unsigned k = leftChild(i);
        if (k >= H.size())
            break;
        if (k+1 < H.size() && *H[k+1] < *H[k])
            ++k; // right child of i
        if ( ! (*H[k] < *x) )
            break;
        set(i, H[k]);
        i = k;
    }
    set(i, x);
}

template <class T>
void mutable_priority_queue<T>::set(unsigned i, T * x) {
    H[i] = x;
    x->queueIndex = i;
}

#endif /* MUTABLEPRIORITYQUEUE */
