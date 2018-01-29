#pragma once
#include "BinaryHeapM.h"
#include "NaviPath.h"
class NodeRun;
class NodeRunData;
class Node;
class NodeRun
{
public:
	int BinaryHeapIndex;
	unsigned int g;
	unsigned int h;
	Node* node;
	unsigned int cost;
	NodeRun* parent;
	unsigned short pathID;
	unsigned int f()
	{
		return g + h;
	}
};
class NodeRunData
{
public:
	NodeRunData(int nodeNum, Node* nodes);
	~NodeRunData();
	NaviPath* path;
	unsigned short pathID;
	NodeRun* runNodes;
	int nodesCount;
	BinaryHeapM open;
public:
	NodeRunData();
	void Initialize(NaviPath* p);
	void ClearPathIDs();

};