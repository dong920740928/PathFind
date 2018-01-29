#include "ABPath.h"
#include "NavGraph.h"
#include "NaviPath.h"
#include "NNConstraint.h"
#include "Node.h"
#include "NavGraphImmutable.h"
#include "PathLog.h"
#include "PathFinder.h"
#include "PathFindLoader.h"
#include <iostream>
std::unordered_set<Int3,Int3PHash,Int3Equal> ABPath::StartPointLogDict;
ABPath::ABPath(const Int3* start, const Int3* end, TagType tags, bool useNavWeight)
	:OriginalStartPoint(*start),
	OriginalEndPoint(*end)
{
	EnabledTags = tags;
	this->UseNavWeight = useNavWeight;
	hTarget = *end;
	startNode = NULL;
	endNode = NULL;
	Graph = NULL;
}
ABPath :: ~ABPath()
{

}
Node* ABPath::GetNode(const Int3& position, const NNConstraint* constraint)
{
	if (Graph == NULL)
	{
		return NULL;
	}
	return Graph->GetNode(position, constraint);
}
Node* ABPath::GetNearest(const Int3& position, const NNConstraint* constraint)
{
	if (Graph == NULL)
	{
		return NULL;
	}
	return Graph->GetNearest(position, constraint);
}
void ABPath::Prepare(int nearDis)
{
	NnConstraint->tags = EnabledTags;
	NnConstraint->useNavWeight = UseNavWeight;
	NnConstraint->checkDir = true;
	startNode = GetNode(OriginalStartPoint,NnConstraint);

	NnConstraint->constrainDistance = nearDis;
	endNode = GetNearest(OriginalEndPoint, NnConstraint);
	if (startNode == NULL && endNode == NULL)
	{
		Error();
		std::ostringstream stringStream;
		stringStream << *Graph->Immutable->Name << " ABPath.Prepare couldn't find close nodes to the start point " << OriginalStartPoint.ToString() <<" and the end point "<< OriginalEndPoint.ToString();
		PathLog::Warn(stringStream.str().c_str());
		return;
	}
	if (startNode == NULL)
	{
		if (StartPointLogDict.find(OriginalStartPoint) == StartPointLogDict.end())
		{
			StartPointLogDict.insert(OriginalStartPoint);
			std::ostringstream stringStream;
			const char* stack = PathFinder::GetStack();
			stringStream << *Graph->Immutable->Name << " ABPath.Prepare couldn't find close nodes to the start point " << OriginalStartPoint.ToString() <<" stack:\r\n"<< stack;
			PathLog::Warn(stringStream.str().c_str());
		}
		Error();
		return;
	}
	if (endNode == NULL)
	{
		Error();
		std::ostringstream stringStream;
		stringStream << *Graph->Immutable->Name << " ABPath.Prepare couldn't find a close node to the end point " << OriginalEndPoint.ToString()<<" nearDis "<<nearDis;
		PathLog::Warn(stringStream.str().c_str());
		return;
	}

	if (!startNode->IsWalkAble(EnabledTags))
	{
		Error();
		std::ostringstream stringStream;
		stringStream << *Graph->Immutable->Name << " ABPath.Prepare the node closest to the start point is not walkable, pos: " << OriginalStartPoint.ToString();
		PathLog::Warn(stringStream.str().c_str());
		return;
	}
	if (!endNode->IsWalkAble(EnabledTags))
	{
		Error();
		std::ostringstream stringStream;
		stringStream << *Graph->Immutable->Name << " ABPath.Prepare the node closest to the end point is not walkable, pos:  " << OriginalEndPoint.ToString();
		PathLog::Warn(stringStream.str().c_str());
		return;
	}
	if (!Graph->IgnoreDirect && !Graph->CanMove(startNode->area(),endNode->area(),EnabledTags))
	{
		Error();
		return;
	}
	
	PathNNConstraint* pathNNConstraint = (PathNNConstraint*)NnConstraint;
	pathNNConstraint->SetStart(startNode);


}
void ABPath::Initialize()
{
	if (startNode == endNode)
	{
		NodeRun* endNodeR = endNode->GetNodeRun(RunData);
		endNodeR->parent = NULL;
		endNodeR->h = 0;
		endNodeR->g = 0;
		Trace(endNodeR);
		CompleteState = PathCompleteState_Complete;
		return;
	}
	NodeRun* startRNode = startNode->GetNodeRun(RunData);
	startRNode->pathID = PathID;
	startRNode->BinaryHeapIndex = 0;
	startRNode->parent = NULL;
	startRNode->cost = 0;
	startRNode->g = 0;
	startNode->UpdateH(hTarget, startRNode);

	startNode->Open(RunData, startRNode, &hTarget, this);
	SearchedNodes++;

	if (RunData->open.NumberOfItems <= 1)
	{
		Error();
		std::ostringstream stringStream;
		stringStream << *Graph->Immutable->Name << " ABPath.Initialize no open points, the start node didn't open any nodes  " << startNode->ToString();
		PathLog::Warn(stringStream.str().c_str());
		return;
	}
	CurrentR = RunData->open.Remove();

}
void ABPath::CalculateStep()
{
	while (CompleteState == PathCompleteState_NotCalculated)
	{
		SearchedNodes++;
		//到达目标点
		if (CurrentR->node == endNode)
		{
			CompleteState = PathCompleteState_Complete;
			break;
		}
		CurrentR->node->Open(RunData,CurrentR,&hTarget,this);

		if (RunData->open.NumberOfItems <= 1)
		{
			Error();
			std::ostringstream stringStream;
			stringStream << *Graph->Immutable->Name << " No open points, whole area searched  start: " << OriginalStartPoint.ToString() <<"  end:  "<<OriginalEndPoint.ToString()<<" EnabledTags: "<< EnabledTags<<" UseNavWeight:"<<UseNavWeight;
			PathLog::Error(stringStream.str().c_str());
			return;
		}
		//Select the node with the lowest F score and remove it from the open list
		CurrentR = RunData->open.Remove();
	}

	if (CompleteState == PathCompleteState_Complete)
	{
		Trace(CurrentR);
	}
}