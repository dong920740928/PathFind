#include "BinaryHeapM.h"
#include <cmath>
#include <cstring>
#include "PathFindException.h"
#include "NodeRunData.h"
#include "PathLog.h"
const float BinaryHeapM::growthFactor = 2;
BinaryHeapM::BinaryHeapM(int numberOfElements)
{
	binaryHeap = new NodeRun*[numberOfElements];
	heapSize = numberOfElements;
	NumberOfItems = 2;
}
BinaryHeapM::~BinaryHeapM()
{
	if (binaryHeap != NULL)
	{
		delete binaryHeap;
	}
}
void BinaryHeapM::Clear()
{
	NumberOfItems = 1;
}

void BinaryHeapM::UpdateIner(NodeRun* node)
{
	int bubbleIndex = node->BinaryHeapIndex;
	unsigned int nodeF = node->f();
	while (bubbleIndex != 1)
	{
		int parentIndex = bubbleIndex / 2;

		if (nodeF < binaryHeap[parentIndex]->f())
		{
			binaryHeap[bubbleIndex] = binaryHeap[parentIndex];
			binaryHeap[bubbleIndex]->BinaryHeapIndex = bubbleIndex;
			binaryHeap[parentIndex] = node;
			node->BinaryHeapIndex = parentIndex;
			bubbleIndex = parentIndex;
		}
		else
		{
			break;
		}
	}
}
void BinaryHeapM::Update(NodeRun* node)
{
	if (node->BinaryHeapIndex == 0)
	{
		Add(node);
		return;
	}
	if (binaryHeap[node->BinaryHeapIndex] != node)
	{
		PathLog::Error("BinaryHeapM.Update error");
	}
	UpdateIner(node);
}

void BinaryHeapM::Add(NodeRun* node)
{

	if (node == NULL) throw new PathFindException("Sending null node to BinaryHeap");

	if (NumberOfItems == heapSize)
	{
		int newSize = (int)ceil(heapSize * growthFactor);
		if (newSize > (1 << 18))
		{
			throw new PathFindException("Binary Heap Size really large (2^18). A heap size this large is probably the cause of pathfinding running in an infinite loop. \nRemove this check (in BinaryHeap.cs) if you are sure that it is not caused by a bug");
		}

		NodeRun** tmp = new NodeRun* [newSize];
		memcpy(tmp,binaryHeap,sizeof(NodeRun*)*heapSize);
		delete binaryHeap;
		binaryHeap = tmp;
		heapSize = newSize;
		//UXLog.Log ("Forced to discard nodes because of binary heap size limit, please consider increasing the size ("+numberOfItems +" "+binaryHeap.Length+")");
		//numberOfItems--;
	}

	binaryHeap[NumberOfItems] = node;
	node->BinaryHeapIndex = NumberOfItems;

	UpdateIner(node);

	NumberOfItems++;
}
NodeRun* BinaryHeapM::Remove()
{
	NumberOfItems--;
	NodeRun* returnItem = binaryHeap[1];
	returnItem->BinaryHeapIndex = 0;

	binaryHeap[1] = binaryHeap[NumberOfItems];
	binaryHeap[1]->BinaryHeapIndex = 1;

	int swapItem = 1, parent = 1;

	do
	{
		parent = swapItem;
		int p2 = parent * 2;
		if (p2 + 1 < NumberOfItems)
		{
			// Both children exist
			if (binaryHeap[parent]->f() >= binaryHeap[p2]->f())
			{
				swapItem = p2;//2 * parent;
			}
			if (binaryHeap[swapItem]->f() >= binaryHeap[p2 + 1]->f())
			{
				swapItem = p2 + 1;
			}
		}
		else if ((p2) < NumberOfItems)
		{
			// Only one child exists
			if (binaryHeap[parent]->f() >= binaryHeap[p2]->f())
			{
				swapItem = p2;
			}
		}

		// One if the parent's children are smaller or equal, swap them
		if (parent != swapItem)
		{
			NodeRun* tmpIndex = binaryHeap[parent];


			binaryHeap[parent] = binaryHeap[swapItem];
			binaryHeap[parent]->BinaryHeapIndex = parent;
			binaryHeap[swapItem] = tmpIndex;
			tmpIndex->BinaryHeapIndex = swapItem;
		}
	} while (parent != swapItem);

	return returnItem;
}
