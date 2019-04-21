#ifndef BinaryHeap_h
#define BinaryHeap_h

#include <vector>
#include <iostream>
#include <cmath>

class BinaryHeap;

class HeapNode{
    public:
        HeapNode(): _element(0), _key(0) {}
        HeapNode(int element, int key): _element(element), _key(key) {}
        ~HeapNode() {}
        friend class BinaryHeap;
    private:
        int _element;
        int _key;
};

class BinaryHeap{
    public:
        BinaryHeap() {};
        BinaryHeap(int n) { _heap.resize(n + 1); }
        ~BinaryHeap() {};

        int Minimum() { return _heap[1]._element; }
        int ExtractMin();
        bool IsEmpty() { return _heap.size() == 1; } 
        void MinHeapify(int node, int length);
        void BuildMinHeap(std::vector<int> keys);
        void DecreaseKey(int node, int new_key);

    private:
        std::vector<HeapNode> _heap;

        int FindPosition(int node);
        int GetParentNode(int node) { return std::floor(node / 2); }
        void Swap(class HeapNode &n1, class HeapNode &n2);
};

#endif
