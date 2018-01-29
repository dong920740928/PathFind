#include "NodeRunData.h"
#include "Node.h"
#include <cstring>
void Reset();
void NodeRunData::Initialize(NaviPath* p)
{
	path = p;
	pathID = p->PathID;
	open.Clear();
}
void NodeRunData::ClearPathIDs()
{
	for (int i = 0; i < nodesCount; i++)
	{
		runNodes[i].pathID = 0;
	}
}

NodeRunData::NodeRunData(int nodeNum, Node* nodes)
	:open(512),pathID(0),nodesCount(nodeNum),path(NULL)
{
	void* mem = new char[sizeof(NodeRun) * nodeNum];
	memset(mem, 0, sizeof(NodeRun) * nodeNum);
	runNodes = (NodeRun*)mem;
	for (int i = 0; i < nodeNum; i++)
	{
		(runNodes + i)->node = nodes + i;
	}
}
NodeRunData::~NodeRunData()
{
	if (runNodes !=NULL)
	{
		delete runNodes;
		runNodes = NULL;
	}
}