#include "FunnelModifier.h"
#include "ABPath.h"
#include "NavGraph.h"
#include <vector>
#include "Node.h"
void FunnelModifier::Apply(ABPath* p, NavGraph* graph)
{
	vector<Node*>* path = &p->Path;
	Int3Vector* vectorPath = p->VectorPath;

	if (path == NULL || path->size() == 0 || vectorPath == NULL || vectorPath->size() !=  path->size())
	{
		return;
	}

	int currentGraphStart = 0;

	Int3Vector funnelPath;

	deque<Int3> left;
	deque<Int3> right;

	ConstructFunnel(graph, *vectorPath, *path, currentGraphStart, (int)path->size() - 1, funnelPath, left, right);

	p->VectorPath->swap(funnelPath);
}
void FunnelModifier::ConstructFunnel(NavGraph* graph, Int3Vector& vectorPath, vector<Node*>& path, int sIndex, int eIndex, Int3Vector& funnelPath, deque<Int3>& left, deque<Int3>& right)
{
	left.clear();
	right.clear();

	left.push_back(vectorPath[sIndex]);
	right.push_back(vectorPath[sIndex]);

	BuildFunnelCorridor(graph, path, sIndex, eIndex, left, right);

	left.push_back(vectorPath[eIndex]);
	right.push_back(vectorPath[eIndex]);

	if (!RunFunnel(&left, &right, funnelPath))
	{

		//Add the start and end positions to the path
		funnelPath.push_back(vectorPath[sIndex]);
		funnelPath.push_back(vectorPath[eIndex]);

	}
}
void FunnelModifier::BuildFunnelCorridor(NavGraph* graph, vector<Node*>& path, int startIndex, int endIndex, deque<Int3>& left, deque<Int3>& right)
{
	if (graph == NULL)
	{
		return;
	}

	Int3* lastLeftVertice = NULL;
	Int3* lastRightVertice = NULL;

	for (int i = startIndex; i < endIndex; i++)
	{
		//Find the connection between the nodes

		Node* n1 = path[i];
		Node* n2 = path[i + 1];

		bool foundFirst = false;

		Int3* first = NULL;
		Int3* second = NULL;

		for (int x = 0; x < 3; x++)
		{
			Int3* vertice1 = n1->vList[x];
			for (int y = 0; y < 3; y++)
			{
				Int3* vertice2 = n2->vList[y];
				if (*vertice1 == *vertice2)
				{
					if (foundFirst)
					{
						second = vertice2;
						break;
					}
					else
					{
						first = vertice2;
						foundFirst = true;
					}
				}
			}
		}

		if (first == NULL || second == NULL)
		{
			left.push_back(n1->position);
			right.push_back(n1->position);
			left.push_back(n2->position);
			right.push_back(n2->position);
			lastLeftVertice = first;
			lastRightVertice = second;

		}
		else
		{

		
			if (first == lastLeftVertice)
			{
				left.push_back(*first);
				right.push_back(*second);
				lastLeftVertice = first;
				lastRightVertice = second;

			}
			else if (first == lastRightVertice)
			{
				left.push_back(*second);
				right.push_back(*first);
				lastLeftVertice = second;
				lastRightVertice = first;

			}
			else if (second == lastLeftVertice)
			{
				left.push_back(*second);
				right.push_back(*first);
				lastLeftVertice = second;
				lastRightVertice = first;

			}
			else
			{
				left.push_back(*first);
				right.push_back(*second);
				lastLeftVertice = first;
				lastRightVertice = second;
			}
		}
	}
}
bool FunnelModifier::RunFunnel(deque<Int3>* left, deque<Int3>* right, Int3Vector& funnelPath)
{
	if (left->size() < 3)
	{
		return false;
	}
	while ((*left)[1] == (*left)[2] && right[1] == right[2])
	{
		left->erase(left->begin()+1);
		right->erase(right->begin()+1);

		if (left->size() < 3)
		{
			return false;
		}

	}

	Int3 swPoint = (*left)[2];
	if (swPoint == (*left)[1])
	{
		swPoint = (*right)[2];
	}
	//Test
	while (Polygon::IsColinear((*left)[0], (*left)[1], (*right)[1]) || Polygon::Left((*left)[1], (*right)[1], swPoint) == Polygon::Left((*left)[1], (*right)[1], (*left)[0]))
	{

		left->erase(left->begin()+1);
		right->erase(right->begin()+1);

		if (left->size() < 3)
		{
			return false;
		}

		swPoint = (*left)[2];
		if (swPoint == (*left)[1])
		{
			swPoint = (*right)[2];
		}
	}

	swPoint = (*left)[left->size() - 3];
	if (swPoint == (*left)[left->size() - 2])
	{
		swPoint = (*right)[right->size() - 3];
	}

	while (Polygon::IsColinear(left->back(), (*left)[left->size() - 2], (*right)[right->size() - 2]) || Polygon::Left((*left)[left->size() - 2], (*right)[right->size() - 2], swPoint) == Polygon::Left((*left)[left->size() - 2], (*right)[right->size() - 2], left->back()))
	{
		left->erase(left->end()-2);
		right->erase(right->end()-2);
		if (left->size() < 3)
		{
			return false;
		}
		swPoint = (*left)[left->size() - 3];
		if (swPoint == (*left)[left->size() - 2])
		{
			swPoint = (*right)[right->size() - 3];
		}
	}



	//Switch left and right to really be on the "left" and "right" sides
	if (!Polygon::IsClockwise((*left)[0], (*left)[1], (*right)[1]) && !Polygon::IsColinear((*left)[0], (*left)[1], (*right)[1]))
	{
		//System.Console.WriteLine ("Wrong Side 2");
		deque<Int3>* tmp = left;
		left = right;
		right = tmp;
	}

	funnelPath.push_back((*left)[0]);

	Int3 portalApex = (*left)[0];
	Int3 portalLeft = (*left)[1];
	Int3 portalRight = (*right)[1];

	int apexIndex = 0;
	int rightIndex = 1;
	int leftIndex = 1;

	for (unsigned int i = 2; i < left->size(); i++)
	{

		if (funnelPath.size() > 200)
		{
			break;
		}

		Int3 pLeft = (*left)[i];
		Int3 pRight = (*right)[i];

		if (Polygon::TriangleArea2(portalApex, portalRight, pRight) >= 0L)
		{

			if (portalApex == portalRight || Polygon::TriangleArea2(portalApex, portalLeft, pRight) <= 0L)
			{
				portalRight = pRight;
				rightIndex = i;
			}
			else
			{
				funnelPath.push_back(portalLeft);
				portalApex = portalLeft;
				apexIndex = leftIndex;

				portalLeft = (*left)[apexIndex];
				portalRight = (*right)[apexIndex];

				leftIndex = apexIndex;
				rightIndex = apexIndex;

				i = apexIndex;
				continue;
			}
		}

		if (Polygon::TriangleArea2(portalApex, portalLeft, pLeft) <= 0L)
		{

			if (portalApex == portalLeft || Polygon::TriangleArea2(portalApex, portalRight, pLeft) >= 0L)
			{
				portalLeft = pLeft;
				leftIndex = i;

			}
			else
			{

				funnelPath.push_back(portalRight);
				portalApex = portalRight;
				apexIndex = rightIndex;

				portalLeft = (*left)[apexIndex];
				portalRight = (*right)[apexIndex];

				leftIndex = apexIndex;
				rightIndex = apexIndex;

				i = apexIndex;
				continue;
			}
		}
	}
	funnelPath.push_back((*left)[left->size() - 1]);
	return true;
}