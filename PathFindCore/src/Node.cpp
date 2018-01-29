#include "Node.h"
#include "NavGraph.h"
#include "NavGraphImmutable.h"
#include <cmath>
#include <iostream>
std::string Node::ToString()
{
	using namespace std;
	ostringstream stringStream;
	stringStream << "node index : " << nodeIndex << ", vertices(" <<
		(vList[0]!=NULL? vList[0]->ToString():"null" )<< ", " <<
		(vList[1]!=NULL?  vList[1]->ToString():"null")<<", "<<
		(vList[2]!=NULL? vList[2]->ToString():"null" )<< ")";
	return stringStream.str();
}

int Node::GetWeightCost(Node* node1, Node* node2)
{
	return (int)round((node1->position - node2->position).XZMagnitude() * (pow(2.0, node1->weight() - 1L) + pow(2.0, node2->weight() - 1L)) / 2);
}

int Node::GetCost(Node* node1,Node* node2)
{
	return (node1->position - node2->position).XZMagnitude();
}
void Node::Open(NodeRunData* nodeRunData, NodeRun* nodeR, Int3* targetPosition, ABPath* path)
{
	NavGraph* graph = path->Graph;
	NavGraphImmutable* immutable = graph->Immutable;
	for (int i = 0; i < 3; i++)
	{
		Node* conNode = immutable->AllConnects[nodeIndex * 3 + i];
		if (conNode == NULL)
		{
			continue;
		}

		if (graph->GetNodeAllCloseConnects(this, i))
		{
			continue;
		}

		if (!path->CanTraverse(conNode))
		{
			continue;
		}

		NodeRun* nodeR2 = conNode->GetNodeRun(nodeRunData);



		if (nodeR2->pathID != nodeRunData->pathID)
		{

			nodeR2->parent = nodeR;
			nodeR2->pathID = nodeRunData->pathID;
			//nodeRunData.
			if (!path->UseNavWeight)
			{
				nodeR2->cost = immutable->AllConnectionCosts[nodeIndex * 3 + i];
			}
			else
			{
				nodeR2->cost = immutable->AllWeightConnectionCosts[nodeIndex * 3 + i];
			}


			conNode->UpdateH(*targetPosition, nodeR2);
			conNode->UpdateG(nodeR2, nodeRunData);

			nodeRunData->open.Add(nodeR2);
		}
		else
		{
			//If not we can test if the path from the current node to this one is a better one then the one already used
			unsigned int tmpCost;
			if (!path->UseNavWeight)
			{
				tmpCost = immutable->AllConnectionCosts[nodeIndex * 3 + i];
			}
			else
			{
				tmpCost = immutable->AllWeightConnectionCosts[nodeIndex * 3 + i];
			}

			if (nodeR->g + tmpCost < nodeR2->g)
			{

				nodeR2->cost = tmpCost;
				nodeR2->parent = nodeR;

				conNode->UpdateG(nodeR2, nodeRunData);
				nodeRunData->open.Update(nodeR2);
			}
		}
	}
}