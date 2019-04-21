#include "BinaryHeap.h"

int BinaryHeap::ExtractMin()
{
    if(IsEmpty()) {
        std::cout << "heap is empty" << std::endl;
        exit(1);
    }

    int min = _heap[1]._element;
    _heap[1] = _heap[_heap.size() - 1];
    _heap.erase(_heap.begin() + _heap.size() - 1);
    MinHeapify(1, (int)_heap.size());

    return min;
}
void BinaryHeap::MinHeapify(int node, int length)
{
    int left = 2 * node;
    int right = 2 * node + 1;
    int smallest;

    if(left <= length && _heap[left]._key < _heap[node]._key) smallest = left;
    else smallest = node;

    if(right <= length && _heap[right]._key < _heap[smallest]._key) smallest = right;

    if(smallest != node) {
        Swap(_heap[smallest], _heap[node]);
        MinHeapify(smallest, length);
    }
}
void BinaryHeap::BuildMinHeap(std::vector<int> keys)
{
    for(int i = 0; i < (int)keys.size(); ++i) {
        _heap[i + 1]._element = i;
        _heap[i + 1]._key = keys[i];
    }

    for(int i = (int)_heap.size() / 2; i >= 1; --i) {
        MinHeapify(i, (int)_heap.size() - 1);
    }
}
void BinaryHeap::DecreaseKey(int node, int new_key)
{
    int idx = FindPosition(node);
    if(new_key > _heap[idx]._key) {
        // std::cout << "new key is large than current key" << std::endl;
        return;
    }

    _heap[idx]._key = new_key;
    while(idx > 1 && _heap[GetParentNode(idx)]._key > _heap[idx]._key) {
        Swap(_heap[idx], _heap[GetParentNode(idx)]);
        idx = GetParentNode(idx);
    }
}
int BinaryHeap::FindPosition(int node)
{
    for(int i = 1; i < (int)_heap.size(); ++i) {
        if(_heap[i]._element == node) return i;
    }
}
void BinaryHeap::Swap(class HeapNode &n1, class HeapNode &n2)
{
    class HeapNode temp = n1;
    n1 = n2;
    n2 = temp;
}
