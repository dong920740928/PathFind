#pragma once
#include <vector>
#include "BaseType.h"
class Node;
class NodeRunData;
class NodeRun;
class Int3;
class NNConstraint;
class Int3Vector;
enum PathCompleteState
{
	PathCompleteState_NotCalculated = 0,
	PathCompleteState_Error = 1,
	PathCompleteState_Complete = 2,
	PathCompleteState_Partial = 3
};
class NaviPath
{
public:
	NaviPath();
	~NaviPath();
	bool CanTraverse(Node* node);
	bool IsDone()
	{
		return CompleteState != PathCompleteState_NotCalculated;
	} 
	void Error()
	{
		CompleteState = PathCompleteState_Error;
	}
	void Trace(NodeRun* from);
	void PrepareBase(NodeRunData* runData);
	virtual void Prepare(int nearDis) = 0;
	virtual void Initialize() = 0;
	virtual void CalculateStep() = 0;
public:
	Int3Vector* VectorPath;
	int SearchedNodes;
	TagType EnabledTags;
	NodeRun* CurrentR;
	enum PathCompleteState CompleteState;
	NodeRunData* RunData;
	bool UseNavWeight;
	std::vector<Node*> Path;
	NNConstraint* NnConstraint;
	unsigned short PathID;
private:
	static unsigned short nextFreePathID;

};
