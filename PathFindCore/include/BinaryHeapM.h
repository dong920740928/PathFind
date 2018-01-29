#pragma once
class NodeRun;
class BinaryHeapM
{
private:
	NodeRun** binaryHeap;
	int heapSize;
	static const float growthFactor;
public:
	int NumberOfItems;
private:
	void UpdateIner(NodeRun* node);
public:
	BinaryHeapM(int numberOfElements);
	~BinaryHeapM();
	void Clear();
	void Update(NodeRun* node);
	void Add(NodeRun* node);
	NodeRun* Remove();
};