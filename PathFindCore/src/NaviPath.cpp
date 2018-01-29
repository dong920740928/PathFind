#include "NaviPath.h"
#include "Node.h"
#include <exception>
#include <algorithm>
#include "NNConstraint.h"
unsigned short NaviPath::nextFreePathID = 1;
bool NaviPath::CanTraverse(Node* node)
{
	return node->IsWalkAble(EnabledTags);
}

NaviPath::NaviPath()
{
	VectorPath = NULL;
	SearchedNodes = 0;
	EnabledTags = 0xffffffff;
	RunData = NULL; 
	CompleteState = PathCompleteState_NotCalculated;
	CurrentR = NULL;
	NnConstraint = new PathNNConstraint();
	PathID = nextFreePathID++;
	if (PathID == 0)//pathIDÓÀÔ¶²»ÄÜÎª0
	{
		PathID = nextFreePathID++;
	}
	UseNavWeight = false;
}
NaviPath::~NaviPath()
{
	if (NnConstraint != NULL)
	{
		delete NnConstraint;
		NnConstraint = NULL;
	}
}
void NaviPath::Trace(NodeRun* from)
{
	NodeRun* c = from;
	while(c != NULL)
	{
		Path.push_back(c->node);
		VectorPath->push_back(c->node->position);
		c = c->parent;
	}
	std::reverse(Path.begin(),Path.end());
	VectorPath->reverse();
}
void NaviPath::PrepareBase(NodeRunData* runData)
{
	if (runData->pathID > PathID)
	{
		runData->ClearPathIDs();
	}
	this->RunData = runData;
	runData->Initialize(this);
}