#include "NavGraph.h"
#include "PathFindLoader.h"
#include "Node.h"
#include "PathFinder.h"
#include <unordered_map>
#include <unordered_set>
#include "NavGraphImmutable.h"
#include <algorithm>    // std::find
#include <cmath>
#include <cstring>
#include "NNConstraint.h"
#include "PathLog.h"
NavGraph::~NavGraph()
{
	Dispose(false);
}
void NavGraph::Dispose(bool freeRes)
{
	if (Immutable!=NULL && freeRes)
	{
		delete Immutable;
		Immutable = NULL;
	}
	if (oneWayConnectOpenTagsDic != NULL)
	{
		delete oneWayConnectOpenTagsDic;
		oneWayConnectOpenTagsDic = NULL;
	}
	if (allCloseConnects != NULL)
	{
		delete allCloseConnects;
		allCloseConnects = NULL;
	}
	if (AreaIndirectConnectsCache != NULL)
	{
		for(unordered_map<TagType, unordered_map<int, vector<int>*>*>::iterator iter = AreaIndirectConnectsCache->begin(); iter != AreaIndirectConnectsCache->end(); ++iter)
		{
			unordered_map<int, vector<int>*>* dic2 = iter->second;
			for(unordered_map<int, vector<int>*>::iterator iter2 = dic2->begin();iter2!=dic2->end();++iter2)
			{
				delete iter2->second;
			}
			delete dic2;
		}
		delete AreaIndirectConnectsCache;
		AreaIndirectConnectsCache = NULL;
	}
	if (AreaCloseOneway != NULL)
	{
		for(unordered_map<int, unordered_set<int>*>::iterator iter = AreaCloseOneway->begin(); iter != AreaCloseOneway->end(); ++iter)
		{
			delete iter->second;
		}
		delete AreaCloseOneway;
		AreaCloseOneway = NULL;
	}
}
NavGraph::NavGraph(){
	memset(this, 0 , sizeof(NavGraph));
}
NavGraph::NavGraph(MeshStruct* ms, int areaSizeF)
{
	memset(this, 0, sizeof(NavGraph));
	if (ms->verticesNum == 0 || ms->trianglesNum == 0)
	{
		return;
	}
	Immutable = new NavGraphImmutable(ms->trianglesNum,ms->graphName);
	Immutable->Vertices = ms->vertices;
	Immutable->VerticesCount = ms->verticesNum;
	ms->vertices = NULL;
	Int3* vertices = Immutable->Vertices;
	Node* nodes = Immutable->Nodes;
	int* triangles = ms->triangles;
	int nodesCount = Immutable->NodesCount;
	{
		int index = 0;
		for (int i =0; i< nodesCount; i++)
		{
			Node* node = nodes + i;
			node->nodeIndex = i;
			node->flags = ms->flags[i];

			Int3* v0 = vertices+ triangles[index];
			Int3* v1 = vertices+ triangles[index + 1];
			Int3* v2 = vertices+ triangles[index + 2];
			index+=3;
			node->vList[0] = v0;
			node->vList[1] = v1;
			node->vList[2] = v2;
			node->vList[3] = v0;
			node->position = (*v0 + *v1 + *v2)/3;
			node->aabb = AABB(*v0,*v1,*v2, NavGraph::disError * 2);
		}
	}


	Immutable->AreaDirectConnectsDic = ms->areaConnectsDic;
	ms->areaConnectsDic = NULL;

	Immutable->AreaTagDic = ms->areaTagDic;
	ms->areaTagDic = NULL;

	oneWayConnectOpenTagsDic = new bool[TAGNUMBER];
	memset(oneWayConnectOpenTagsDic,0,sizeof(bool) * TAGNUMBER);
	allCloseConnects = new unordered_map<int, bool>();
	AreaIndirectConnectsCache = new unordered_map<TagType, unordered_map<int, vector<int>*>*>();
	AreaCloseOneway = new unordered_map<int, unordered_set<int>*>();

	int* connectIndexs = ms->connects;
	int* allDirectConnections = ms->directConnects;
	vector<int*>** oneWayConnectDic = Immutable->OneWayConnectDic;
	int* allConnectionCosts = Immutable->AllConnectionCosts;
	int* allWeightConnectionCosts = Immutable->AllWeightConnectionCosts;
	int* allConnectOtherEdgeIndexs = Immutable->AllConnectOtherEdgeIndexs;
	Node** allConnects = Immutable->AllConnects;
	{
		for(int nodeCount= 0; nodeCount<nodesCount; nodeCount++)
		{
			int i = nodeCount*3;
			Node* node = nodes + nodeCount;
			for(int j = 0;j<3;j++)
			{
				int x= connectIndexs[i + j];
				if (x >= 0)
				{
					Node* other = nodes + x;
					//connections.Add(other);
					allConnects[i + j] = other;

					//计算连接的Node边的Index
					int j1 = i + j;
					int j2 = i + (j + 1) % 3;
					for (int k = 0; k < 3; k++)
					{
						int k1 = x * 3 + k;
						int k2 = x * 3 + (k + 1) % 3;
						if ((triangles[j1] == triangles[k1] && triangles[j2] == triangles[k2])
							|| (triangles[j1] == triangles[k2] && triangles[j2] == triangles[k1]))
						{
							allConnectOtherEdgeIndexs[i + j] = k;
							break;
						}
					}
				}
			}


			if (allDirectConnections[i] > 0 || allDirectConnections[i + 1] > 0 || allDirectConnections[i + 2] > 0)
			{
				Immutable->ContainOneWayConnect = true;

				if (oneWayConnectDic[node->tags()] == NULL)
				{
					oneWayConnectDic[node->tags()] = new std::vector<int*>();
					//默认单向门处于关闭状态
					oneWayConnectOpenTagsDic[node->tags()] = false;
				}
				//记录单向门包含的节点和每条边的连接模式

				int* elem = new int[4];
				elem[0] = node->nodeIndex;
				elem[1] = allDirectConnections[i];
				elem[2] = allDirectConnections[i+1];
				elem[3] = allDirectConnections[i+2];
				oneWayConnectDic[node->tags()]->push_back(elem);
			}


			for (int j = 0; j < 3; j++)
			{
				if (allConnects[i + j] != NULL)
				{
					allConnectionCosts[i + j] = Node::GetCost(node, allConnects[i + j]);
					allWeightConnectionCosts[i + j] = Node::GetWeightCost(node, allConnects[i + j]);
				}

			}

		}
	}
	{


		for(int i=0;i<nodesCount;i++)
		{
			Node* node = nodes + i;
			for (int j=0;j<3;j++)
			{
				Node* otherNode = allConnects[node->nodeIndex * 3 + j];
				if (otherNode != NULL && node->tags() == OnlyOutLand && node->tags() != otherNode->tags())
				{
					Immutable->ContainOneWayConnect = true;
					for (int k = 0; k< 3;k ++)
					{
						if (node == allConnects[otherNode->nodeIndex*3 + k])
						{
							SetAreaCloseOneway(otherNode->area(), node->area(),true);
							SetNodeAllCloseConnects(otherNode, k, true);
							break;
						}
					}
				}
			}
		}
	}


	Immutable->CreateAreaNodeList(PathFinder::AreaSize * areaSizeF, PathFinder::AreaSize * areaSizeF);
	
	isValid = true;
}
void NavGraph::SetNodeAllCloseConnects(Node* n, int i, bool value)
{
	if (value)
	{
		(*allCloseConnects)[(n->nodeIndex << 2) + i] = true;
	}
	else
	{
		allCloseConnects->erase((n->nodeIndex <<2) + i);
	}
}
void NavGraph::ClearAreaIndirectConnectsCache()
{
	if(AreaIndirectConnectsCache->size() > 0)
	{
		for(unordered_map<TagType, unordered_map<int, vector<int>*>*>::iterator iter = AreaIndirectConnectsCache->begin(); iter != AreaIndirectConnectsCache->end(); ++iter)
		{
			unordered_map<int, vector<int>*>* dic2 = iter->second;
			for(unordered_map<int, vector<int>*>::iterator iter2 = dic2->begin();iter2!=dic2->end();++iter2)
			{
				delete iter2->second;
			}
			delete dic2;
		}
		AreaIndirectConnectsCache->clear();
	}
}
void NavGraph::SetAreaCloseOneway(int fromArea, int toArea, bool close)
{
	if (close)
	{
		if(AreaCloseOneway->find(fromArea) == AreaCloseOneway->end())
		{
			(*AreaCloseOneway)[fromArea] = new unordered_set<int>();
		}
		if((*AreaCloseOneway)[fromArea]->insert(toArea).second)
		{
			ClearAreaIndirectConnectsCache();
		}
	}
	else
	{
		if(AreaCloseOneway->find(fromArea) != AreaCloseOneway->end())
		{
			delete (*AreaCloseOneway)[fromArea];
			AreaCloseOneway->erase(fromArea);
			ClearAreaIndirectConnectsCache();
		}
	}
}
void NavGraph::CreateRunData()
{
	Immutable->CreateRunData();
}
NavGraph* NavGraph::Clone()
{
	NavGraph* ret = new NavGraph();
	ret->Immutable = Immutable;
	ret->allCloseConnects = new unordered_map<int, bool>();
	ret->oneWayConnectOpenTagsDic = new bool[TAGNUMBER];

	memcpy(ret->oneWayConnectOpenTagsDic,oneWayConnectOpenTagsDic,sizeof(bool)*TAGNUMBER);

	ret->isValid = isValid;

	for(unordered_map<int, bool>::iterator iter = allCloseConnects->begin(); iter!=allCloseConnects->end(); ++iter)
	{
		(*ret->allCloseConnects)[iter->first] = iter->second;
	}
	ret->AreaIndirectConnectsCache = new unordered_map<TagType, unordered_map<int, vector<int>*>*>();
	ret->AreaCloseOneway = new unordered_map<int, unordered_set<int>*>();
	for(unordered_map<int, unordered_set<int>*>::iterator iter = AreaCloseOneway->begin(); iter!=AreaCloseOneway->end(); ++iter)
	{
		(*ret->AreaCloseOneway)[iter->first] = new unordered_set<int>(*iter->second);
	}
	return ret;
}


bool NavGraph::GetNodeAllCloseConnects(Node* n, int i)
{
	if (IgnoreDirect)
	{
		return false;
	}
	return allCloseConnects->find((n->nodeIndex<<2) + i) != allCloseConnects->end();
}
Node* NavGraph::GetNode(const Int3& position, const NNConstraint* constraint)
{

	if (Immutable->Nodes == NULL)
	{
		return NULL;
	}
	Node* minNode = NULL;
	Node* xzRayNode = GetWalkableNode(position, constraint->tags, constraint->checkDir, false, false);
	if (xzRayNode != NULL && constraint->Suitable(xzRayNode))
	{
		minNode = xzRayNode;
	}
	return minNode;
}
Node* NavGraph::GetNearest(const Int3& position, const NNConstraint* constraint)
{
	if (Immutable->Nodes == NULL)
	{
		return NULL;
	}

	int maxDistSqr = constraint->constrainDistance;

	double minDist = PathfindMaxDouble;
	Node* minNode = NULL;


	//先拿脚底下的三角形;没有取最近的
	Node* xzRayNode = GetWalkableNode(position, constraint->tags, constraint->checkDir,false, false);
	if (xzRayNode != NULL && constraint->Suitable(xzRayNode))
	{
		minNode = xzRayNode;
	}
	else if(maxDistSqr > 0)
	{
		unordered_set<Node*> nodes;
		AABB area(position,maxDistSqr);
		Immutable->GetIntersectAreaNodes(area, nodes);
		for(unordered_set<Node*>::iterator iter = nodes.begin(); iter!=nodes.end(); ++iter)
		{
			Node* node = *iter;

			if (!constraint->Suitable(node))
			{
				continue;
			}
			double dist = (position - node->position).XZMagnitudeD();
			if (dist < minDist)
			{
				minDist = dist;
				minNode = node;
			}
		}

	}
	return minNode;
}
bool NavGraph::CanMove(int sArea, int tArea, TagType navTags)
{
	if (sArea == tArea)//区域往往是相同的，所以应该放到第一步判断
	{
		return true;
	}
	//当前区域不可过，就不用计算邻接区域
	if (Immutable->AreaTagDic == NULL)
	{
		return true;
	}

	int tags = Immutable->AreaTagDic[sArea];
	if (((1 << tags) & navTags) == 0)
	{
		return false;
	}
	if (AreaIndirectConnectsCache->find(navTags) == AreaIndirectConnectsCache->end())
	{
		(*AreaIndirectConnectsCache)[navTags] = GenerateAreaConnects(navTags);
	}
	unordered_map<int, vector<int>*>* areaDirectConnectsDic = (*AreaIndirectConnectsCache)[navTags];
	if (areaDirectConnectsDic->find(sArea) != areaDirectConnectsDic->end())
	{
		vector<int>* connects = (*areaDirectConnectsDic)[sArea];
		for(vector<int> ::iterator iter = connects->begin(); iter!=connects->end(); ++iter)
		{
			if (*iter == tArea)
			{
				return true;
			}
		}
	}
	return false;
}
unordered_map<int, vector<int>*>* NavGraph::GenerateAreaConnects(TagType navTags)
{
	unordered_map<int, vector<int>*>* areaIndirectConnectsDic = new unordered_map<int, vector<int>*>();
	//深搜找到所有邻接的area
	vector<int> areaStack;
	unordered_set<int> visitArea;
	for(unordered_map<int, AreaConnectList*>::iterator iter = Immutable->AreaDirectConnectsDic->begin(); iter != Immutable->AreaDirectConnectsDic->end(); ++iter)
	{
		vector<int>* areaIndirectConnects = new vector<int>();
		areaStack.clear();
		visitArea.clear();
		int area = iter->first;
		visitArea.insert(area);
		if (((1 << Immutable->AreaTagDic[area]) & navTags) > 0)
		{
			areaStack.push_back(area);
		}
		while(areaStack.size() > 0)
		{
			int currentArea = areaStack.back();
			areaStack.pop_back();

			AreaConnectList* connectAreaList = (*Immutable->AreaDirectConnectsDic)[currentArea];
			if (connectAreaList != NULL)
			{
				unordered_set<int>* closedWay= NULL;
				unordered_map<int, unordered_set<int>*>::iterator it = AreaCloseOneway->find(currentArea);
				if(it!= AreaCloseOneway->end())
				{
					closedWay = it->second;
				}
				for(int iter = 0; iter < connectAreaList->m_len; iter++ )
				{
					int connectArea = connectAreaList->m_areas[iter];
					if (visitArea.find(connectArea) != visitArea.end())
					{
						continue;
					}
					if (((1 << Immutable->AreaTagDic[connectArea]) & navTags) > 0)
					{
						if(closedWay != NULL && (closedWay->find(connectArea) != closedWay->end()))
						{
							continue;
						}
						visitArea.insert(connectArea);
						areaStack.push_back(connectArea);
						areaIndirectConnects->push_back(connectArea);
					}
				}
			}
		}
		(*areaIndirectConnectsDic)[area] = areaIndirectConnects;
	}
	return areaIndirectConnectsDic;
}
bool NavGraph::GetWalkableY(const Int3& point, TagType tags)
{
	Node* node = GetWalkableNode(point,tags);
	return node != NULL;
}
Int3 NavGraph::GetNearestPointOnNode(Node* n, const Int3& p)
{
	Int3* v0 = n->vList[0];
	Int3* v1 = n->vList[1];
	Int3* v2 = n->vList[2];

	if (Polygon::CheckPointInTriangle2D(*v0, *v1, *v2, p))
	{
		return p;
	}
	Int3 adjustPoint = p + upDiff;
	if (Polygon::CheckPointInTriangle2D(*v0, *v1, *v2, adjustPoint))
	{
		return adjustPoint;
	}
	adjustPoint = p + downDiff;
	if (Polygon::CheckPointInTriangle2D(*v0, *v1, *v2, adjustPoint))
	{
		return adjustPoint;
	}
	adjustPoint = p + rightDiff;
	if (Polygon::CheckPointInTriangle2D(*v0, *v1, *v2, adjustPoint))
	{
		return adjustPoint;
	}
	adjustPoint = p + leftDiff;
	if (Polygon::CheckPointInTriangle2D(*v0, *v1, *v2, adjustPoint))
	{
		return adjustPoint;
	}
	//发生极端情况了，就是那种在顶点附近，而这个顶点的夹角又特别小，且又在边界。。。。。。。。。。
	//todo
	//UXLog.Warn(string.Format("GetNearestPointOnNode  Node : {0} ;  pos: {1}", n.ToString(), p.ToString()));
	long long dis0 = (*v0 - p).XZSquareMagnitude();
	long long dis1 = (*v1 - p).XZSquareMagnitude();
	long long dis2 = (*v2 - p).XZSquareMagnitude();
	Int3* ret;
	long long min;
	if (dis0 < dis1)
	{
		if (dis0 < dis2)
		{
			min = dis0;
			ret = v0;
		}
		else
		{
			min = dis2;
			ret = v2;
		}
	}
	else
	{
		if (dis1 < dis2)
		{
			min = dis1;
			ret = v1;
		}
		else
		{
			min = dis2;
			ret = v2;
		}
	}
	if (min > 200)
	{
		std::ostringstream stringStream;
		stringStream << *Immutable->Name << " GetNearestPointOnNode  node=" << n->ToString() << "  pos=" << p.ToString();
		PathLog::Warn(stringStream.str().c_str());
	}
	return *ret;
}

bool NavGraph::IsWalkable(const Int3* point, TagType tags)
{
	NodeListType* allNodeList = Immutable->GetIntersectAreaNodes(point);
	if (allNodeList == NULL)
	{
		return false;
	}
	for(int iter = 0; iter!=allNodeList->m_last; ++iter)
	{
		Node* node = allNodeList->m_data[iter];
		if (!node->IsWalkAble(tags))
		{
			continue;
		}
		if (!node->aabb.Contains2D(*point))
		{
			continue;
		}
		if (Polygon::CheckPointInTriangle2D(*node->vList[0], *node->vList[1], *node->vList[2],*point,disError))
		{
			return true;
		}
	}
	return false;
}
Node* NavGraph::GetWalkableNode(const Int3& point, TagType navTags, bool checkDirect, bool isExact, bool isEnd)
{	


	NodeVector nodeList;
	NodeListType* allNodeList = Immutable->GetIntersectAreaNodes(&point);
	if (allNodeList == NULL)
	{
		return NULL;
	}
	if (isExact)
	{
		//精确检测，把包含point的nodes找出来，因为point可能在边上或者是顶点，所以会有多个
		for(int iter = 0; iter!=allNodeList->m_last; ++iter)
		{
			Node* node = allNodeList->m_data[iter];
			if (!node->aabb.Contains2D(point))//先通过包围盒检查
			{
				continue;
			}
			if (!Polygon::CheckPointInTriangle2D(*node->vList[0], *node->vList[1], *node->vList[2], point))//通过
			{
				continue;
			}
			//  containNode = true;
			if (!node->IsWalkAble(navTags))
			{
				continue;
			}
			if (!checkDirect)
			{
				return node;
			}
			/*if (find(nodeList.begin(),nodeList.end(),node) == nodeList.end())
			{*/
			nodeList.push_back(node);
			//}
		}
	}else 
	{
		for(int iter = 0; iter!=allNodeList->m_last; ++iter)
		{
			Node* node = allNodeList->m_data[iter];
			if (!node->IsWalkAble(navTags))
			{
				continue;
			}
			if (!node->aabb.Contains2D(point))
			{
				continue;
			}
			if (Polygon::CheckPointInTriangle2D(*node->vList[0], *node->vList[1], *node->vList[2], point, disError))
			{
				if (!checkDirect)
				{
					return node;
				}
				/*if (find(nodeList.begin(),nodeList.end(),node) == nodeList.end())
				{*/
				nodeList.push_back(node);
				//}
			}
		}
	}

	//检查方向时，如果有单向的节点，就只保留单向节点
	if (checkDirect && Immutable->ContainOneWayConnect && nodeList.size() > 1)
	{
		int minDis = PathfindMaxInt;
		NodeVector minDisNodeList(nodeList.size());
		Node** allConnects = Immutable->AllConnects;
		int* allConnectOtherEdgeIndexs = Immutable->AllConnectOtherEdgeIndexs;
		for (int i = nodeList.size() - 1; i >= 0; i--)
		{
			Node* node = nodeList[i];
			Int3** vList = node->vList;
			for (int j = 0; j < 3; j++)
			{
				//if (GetNodeChange(node).allCloseConnects[j] == true)
				bool cond = false;
				if (isEnd)
				{
					if (allConnects[node->nodeIndex * 3 + j] != NULL && GetNodeAllCloseConnects(allConnects[node->nodeIndex * 3 + j], allConnectOtherEdgeIndexs[node->nodeIndex * 3 + j]))
					{
						cond = true;
					}
				}
				else
				{ 
					cond = GetNodeAllCloseConnects(node, j);
				}
				if (cond)
				{
					int cDis = Polygon::PointToLineDis(*vList[j], *vList[j + 1], point);
					if (cDis <= minDis)
					{
						if (cDis < minDis)
						{
							minDisNodeList.clear();
							minDis = cDis;
						}
						minDisNodeList.push_back(node);
					}
				}
			}
		}
		if (minDisNodeList.size() > 0)
		{
			return minDisNodeList[0];
		}
	}
	if(nodeList.size()>1)
	{//有多个适合三角形的时候尽量选择精确包含的那一个
		for (int i = nodeList.size() - 1; i >= 0; i--)
		{
			Node* node = nodeList[i];
			if (Polygon::CheckPointInTriangle2D(*node->vList[0], *node->vList[1], *node->vList[2], point))
			{
				return node;
			}
		}
	}

	if (nodeList.size() > 0)
	{
		return nodeList[0];
	}
	else
	{
		return NULL;
	}
}

bool NavGraph::IsBorderLine(Node* currenttNode, int index, TagType tags)
{
	Node* otherNode = Immutable->AllConnects[currenttNode->nodeIndex * 3 + index];
	if (otherNode == NULL ||
		!otherNode->IsWalkAble(tags) ||
		GetNodeAllCloseConnects(currenttNode, index))
	{
		return true;
	}
	return false;
}
void NavGraph::DirectRaycast(const Int3& start, const Int3& end, TagType navTags, Int3LineList* farthestCrossBorderLines, Int3Vector* crossPoints, Int3& recastFarthestPoint, bool& canDirectMove, Int3& realStart, Int3& realEnd, TagType& pathTags, bool isOut)
{
	canDirectMove = false;
	realStart = start;
	realEnd = end;
	pathTags = 0;
	Node* nodeStart = NULL;
	Node* nodeEnd = NULL;
	//获取起点所在的Node
	{
		nodeStart = GetWalkableNode(start, navTags, true);
		if (nodeStart != NULL)
		{
			realStart = GetNearestPointOnNode(nodeStart, start);
		}
		else
		{
			//UXLog.Warn(string.Format("DirectRaycast start:{0} is not on mesh", start.ToString()));
			recastFarthestPoint = start;
			return;
		}

		//获取终点所在的Node
		nodeEnd = GetWalkableNode(end, navTags, true, false, !isOut);
		if (nodeEnd != NULL)
		{
			realEnd = GetNearestPointOnNode(nodeEnd, end);
		}

	}
	//距离为0

	recastFarthestPoint = realStart;
	//起点终点在同一个节点内
	if (nodeStart == nodeEnd)
	{
		recastFarthestPoint = realEnd;
		canDirectMove = true;
		pathTags = 1u << nodeStart->tags();
		return;
	}
	Node* currentNode = nodeStart;
	//Int3 lastCrossPoint = realStart;
	unordered_set<int> edgeHasCaculated;
	vector<Node*> UnProcessed;
	unsigned int lastUnProcessedIndex = 0;

	//从起点Start所在的Node开始搜索相交的节点
	while (currentNode != NULL)
	{
		//如果到了目标点所在的Node
		if (currentNode == nodeEnd)
		{
			canDirectMove = true;
			break;
		}
		Int3** vList = currentNode->vList;
		int baseEdgeIndex = currentNode->nodeIndex << 2;
		bool flag = true;
		for (int i = 0; i < 3 && flag; i++)
		{
			if (edgeHasCaculated.find(baseEdgeIndex + i) == edgeHasCaculated.end())//
			{
				Node* otherNode = Immutable->AllConnects[currentNode->nodeIndex * 3 + i];
				Int3* v1 = vList[i];
				Int3* v2 = vList[i + 1];
				//计算与三角形边的交点
				Int3 cp;
				bool hascp = LineCrossLinePoint2D2(realStart, realEnd, *v1, *v2, cp);
				edgeHasCaculated.insert(baseEdgeIndex + i);
				if (otherNode != NULL)
				{
					edgeHasCaculated.insert((otherNode->nodeIndex << 2) + Immutable->AllConnectOtherEdgeIndexs[currentNode->nodeIndex * 3 + i]);
				}


				if (hascp)
				{
					if (!cp.XZEquals(recastFarthestPoint))
					{
						
						if (crossPoints != NULL)
						{
							crossPoints->push_back(cp);
						}
						recastFarthestPoint = cp;//更新上一个交点
						pathTags = pathTags | (1u << currentNode->tags());
						if (farthestCrossBorderLines != NULL)
						{
							farthestCrossBorderLines->Clear();//交点更新了，那么以前的边界线也失去意义了
						}
						UnProcessed.clear();//往前走了，以前的未处理节点也没啥用了
						lastUnProcessedIndex = 0;
						flag = false;
					}
					if (IsBorderLine(currentNode, i, navTags))
					{
						if (farthestCrossBorderLines != NULL)
						{
							farthestCrossBorderLines->AddLine(*v1, *v2);
						}
					}
					else
					{
						if (find(UnProcessed.begin(),UnProcessed.end(),otherNode) == UnProcessed.end())
						{
							UnProcessed.push_back(otherNode);
						}
					}
				}
			}
		}
		if (UnProcessed.size() > lastUnProcessedIndex)
		{
			currentNode = UnProcessed[lastUnProcessedIndex++];
		}
		else
		{
			break;
		}
	}
	if (canDirectMove)
	{
		recastFarthestPoint = realEnd;
		pathTags = pathTags | (1u << nodeEnd->tags());
		if (farthestCrossBorderLines != NULL)
		{
			farthestCrossBorderLines->Clear();
		}
	}

	if (crossPoints != NULL)
	{
		crossPoints->eraseNear(10);
	}


}
vector<Int3*>* NavGraph::LineCrossFarLines(const Int3& from, const Int3& to, TagType navTags)
{
	return NULL;
}
Int3 NavGraph::LineCrossFarPoint(const Int3& from, const Int3& to, TagType navTags)
{
	Int3 point;
	bool canDirectMove;
	Int3 realStart;
	Int3 realEnd;
	TagType outTag;
	DirectRaycast(from, to, navTags, NULL, NULL, point, canDirectMove, realStart, realEnd,outTag);
	if (IsWalkable(&point, navTags))
	{
		return point;
	}
	std::ostringstream stringStream;
	stringStream << *Immutable->Name << " LineCrossFarPoint error from=" << from.ToString() << "  to=" << to.ToString() << " crossPoint=" << point.ToString() << " navTags=" << navTags;
	PathLog::Error(stringStream.str().c_str());
	return from;
}
bool NavGraph::LineCrossLinePoint2D2(const Int3& pointA, const Int3& pointB, const Int3& pointC, const Int3& pointD, Int3& cp)
{
	long long dxAB = pointB.X - pointA.X;
	long long dzAB = pointB.Z - pointA.Z;
	long long dxCD = pointD.X - pointC.X;
	long long dzCD = pointD.Z - pointC.Z;
	long long dxAC = pointC.X - pointA.X;
	long long dzAC = pointC.Z - pointA.Z;
	long long dd = dxAB * dzCD - dxCD * dzAB;//向量AB卷积CD，AB*CD
	if (dd == 0)
	{
		return false;
	}
	long long fac = dxAC * dzCD - dxCD * dzAC;

	//(fac != 0 && (fac ^ dd) < 0) 等价于(fac * dd < 0)，而后者会溢出。
	if ((fac != 0 && (fac ^ dd) < 0) || abs(fac) > abs(dd))
	{
		return false;
	}
	fac = dxAC * dzAB - dxAB * dzAC;
	if ((fac != 0 && (fac ^ dd) < 0) || abs(fac) > abs(dd))
	{
		return false;
	}
	long long temp = dxCD * fac;
	int offsetX = (int)((temp ^ dd) > 0 ? (temp + dd / 2) / dd : (temp - dd / 2) / dd);
	temp = dzCD * fac;
	int offsetZ = (int)((temp ^ dd) > 0 ? (temp + dd / 2) / dd : (temp - dd / 2) / dd);
	cp = Int3(offsetX + pointC.X, offsetZ + pointC.Z);
	return true;
}
bool NavGraph::CanDirectMove(const Int3& start, const Int3& end, Int3Vector* path, TagType navTags, TagType& pathTags, bool isOut)
{
	Int3 crossFarthestPoint;
	bool canDirectMove = false;
	Int3 realStart;
	Int3 realEnd;
	if(path != NULL)
	{
		path->push_back(realStart);
	}
	DirectRaycast(start, end, navTags,NULL, path, crossFarthestPoint, canDirectMove, realStart, realEnd, pathTags, isOut);
	if (canDirectMove)
	{
		if (path != NULL)
		{
			(*path)[0] = realStart;
			path->push_back(realEnd);
		}

		return true;
	}
	return false;
}
void NavGraph::GetMoveDisPath(const Int3* start, float angleY, int distance2Move, TagType navTags, vector<Int3>& route)
{

	route.push_back(*start);
	float dx = (float)sin(PI*angleY/180);
	float dz = (float)cos(PI*angleY/180);
	int remainedDis = distance2Move;
	Int3 currentStart = *start;
	Int3 currentEnd = currentStart + Int3((int)(dx * remainedDis), (int)(dz * remainedDis));
	Int3LineList crossLines;
	Int3 recastFarthestPoint;
	Int3 realStart;
	Int3 realEnd;
	bool canDirectMove;
	int count = 20;
	TagType pathTags;
	Int3 nextPoint;
	do 
	{


		{

			crossLines.Clear();
			DirectRaycast(currentStart, currentEnd, navTags, &crossLines,NULL,recastFarthestPoint,canDirectMove, realStart, realEnd, pathTags);

		}


		if (!recastFarthestPoint.XZEquals(route.back()))
		{
			route.push_back(recastFarthestPoint);
			remainedDis -= Int3::XZDistance(recastFarthestPoint, currentStart);
			currentStart = recastFarthestPoint;
			currentEnd = currentStart + Int3((int)(dx * remainedDis), (int)(dz * remainedDis));
		}
		if (remainedDis <= 0)
		{
			break;
		}
		//没有交到边界边，一般是出错了
		if (crossLines.List.size() == 0)
		{
			break;
		}

		GetMinDegLine(crossLines,currentStart,currentEnd,nextPoint);
		int borderLineDis = Int3::XZDistance(currentStart, nextPoint);
		if (borderLineDis < remainedDis)
		{
			if (!nextPoint.XZEquals(route[route.size() - 1]))
			{
				route.push_back(nextPoint);
			}
			remainedDis -= borderLineDis;
			currentStart = nextPoint;
			currentEnd = nextPoint + Int3((int)(dx * remainedDis),  (int)(dz * remainedDis));

		}
		else
		{
			Int3 point = currentStart + (nextPoint - currentStart) * remainedDis / borderLineDis;
			route.push_back(point);
			break;
		}
		//isFirst = false;
		count--;

	} 
	while (count > 0);
}


void NavGraph::GetMinDegLine(Int3LineList& lines, Int3& start, Int3& end, Int3& nextPoint)
{

	double minDeg = 0;
	Int3 start2End = end - start;//从start到end的方向向量 
	nextPoint = start;
	for (vector<Int3Line*>::iterator iter = lines.List.begin(); iter!=lines.List.end(); ++iter)
	{
		Int3Line* currentLine = (*iter);
		bool changeDir = false;
		Int3 dir = currentLine->EndPoint - start;
		if (dir == Int3::Zero)
		{
			dir = currentLine->BeginPoint - start;
			changeDir = true;
		}
		long long dot = Int3::XZDot(start2End, dir);

		double r = dot * dot / (double)(start2End.XZSquareMagnitude() * dir.XZSquareMagnitude());//单位投影的平方
		if (r > minDeg)
		{
			const Int3* endPoint;
			if ((dot > 0) ^ changeDir)
			{
				endPoint = &currentLine->EndPoint;
			}
			else
			{
				endPoint = &currentLine->BeginPoint;
			}
			if (*endPoint != start)
			{
				minDeg = r;
				nextPoint = *endPoint;
			}
		}
	}
}
void NavGraph::OpenOneWayConnect(TagType tags)
{
	if(tags >= TAGNUMBER)
	{
		std::ostringstream stringStream;
		stringStream << *Immutable->Name << " OpenOneWayConnect tag=" << tags <<  " overflow";
		PathLog::Error(stringStream.str().c_str());
		return;
	}
	if (Immutable->OneWayConnectDic[tags] == NULL)
	{
		return;
	}

	vector<int*>* directConnect = Immutable->OneWayConnectDic[tags];
	if (oneWayConnectOpenTagsDic[tags])
	{
		std::ostringstream stringStream;
		stringStream << *Immutable->Name << " OpenOneWayConnect  one way has opened tag=" << tags;
		PathLog::Debug(stringStream.str().c_str());
		return;
	}
	oneWayConnectOpenTagsDic[tags] = true;
	Node* nodes = Immutable->Nodes;
	Node** allConnects = Immutable->AllConnects;
	if (directConnect != NULL)
	{
		int len = (int)directConnect->size();
		for (int i = 0; i < len; i++)
		{
			Node* node = nodes + (*directConnect)[i][0];
			for (int j = 0; j < 3; j++)
			{
				int directConnectMode = (*directConnect)[i][j + 1];
				//关闭进来
				if (directConnectMode == 1)
				{
					Node* otherNode = allConnects[node->nodeIndex *3 + j];
					if (otherNode == NULL)
					{
						std::ostringstream stringStream;
						stringStream << *Immutable->Name << " OpenOneWayConnect  otherNode not exist, Node=" << node->ToString() << " j=" << j;
						PathLog::Error(stringStream.str().c_str());
						continue;
					}
					for (int k = 0; k < 3; k++)
					{
						if (node == allConnects[otherNode->nodeIndex * 3 + k])
						{
							SetAreaCloseOneway(otherNode->area(), node->area(), true);
							SetNodeAllCloseConnects(otherNode, k, true);
							break;
						}
					}

				} //关闭出去
				else if (directConnectMode == 2)
				{
					Node* otherNode = allConnects[node->nodeIndex *3 + j];
					if (otherNode == NULL)
					{
						std::ostringstream stringStream;
						stringStream << *Immutable->Name << " OpenOneWayConnect  otherNode not exist, Node=" << node->ToString() << " j=" << j;
						PathLog::Error(stringStream.str().c_str());
						continue;
					}
					for (int k = 0; k < 3; k++)
					{
						if (otherNode == allConnects[node->nodeIndex * 3 + k])
						{
							SetAreaCloseOneway(node->area(), otherNode->area(), true);
							SetNodeAllCloseConnects(node, k, true);
							break;
						}
					}
				}
			}
		}
	}

}
void NavGraph::CloseOneWayConnect(TagType tags)
{
	if(tags >= TAGNUMBER)
	{
		std::ostringstream stringStream;
		stringStream << *Immutable->Name << " CloseOneWayConnect tag=" << tags <<  " overflow";
		PathLog::Error(stringStream.str().c_str());
		return;
	}
	if (Immutable->OneWayConnectDic[tags] == NULL)
	{
		return;
	}
	vector<int*>* directConnect = Immutable->OneWayConnectDic[tags];
	if (!oneWayConnectOpenTagsDic[tags])
	{
		std::ostringstream stringStream;
		stringStream << *Immutable->Name << " CloseOneWayConnect  one way has closed tag=" << tags <<  " Had Close";
		PathLog::Debug(stringStream.str().c_str());
		return;
	}
	oneWayConnectOpenTagsDic[tags] = false;
	Node* nodes = Immutable->Nodes;
	Node** allConnects = Immutable->AllConnects;

	if (directConnect != NULL)
	{
		int len = (int)directConnect->size();
		for (int i = 0; i < len; i++)
		{
			Node* node = nodes+(*directConnect)[i][0];
			for (int j = 0; j < 3; j++)
			{
				int directConnectMode = (*directConnect)[i][j + 1];
				//开启进来
				if (directConnectMode == 1)
				{
					Node* otherNode = allConnects[node->nodeIndex * 3 + j];
					if (otherNode == NULL)
					{
						std::ostringstream stringStream;
						stringStream << *Immutable->Name << " CloseOneWayConnect  otherNode not exist, Node=" << node->ToString() << " j=" << j;
						PathLog::Error(stringStream.str().c_str());
						continue;
					}

					for (int k = 0; k < 3; k++)
					{
						if (node == allConnects[otherNode->nodeIndex * 3 + k])
						{
							SetAreaCloseOneway(otherNode->area(), node->area(), false);
							SetNodeAllCloseConnects(otherNode, k, false);
							break;
						}
					}

				} //开启出去
				else if (directConnectMode == 2)
				{
					Node* otherNode = allConnects[node->nodeIndex *3 + j];
					if (otherNode == NULL)
					{
						std::ostringstream stringStream;
						stringStream << *Immutable->Name << " CloseOneWayConnect  otherNode not exist, Node=" << node->ToString() << " j=" << j;
						PathLog::Error(stringStream.str().c_str());
						continue;
					}
					for (int k = 0; k < 3; k++)
					{
						if (otherNode == allConnects[node->nodeIndex *3 + k])
						{
							SetAreaCloseOneway(node->area(), otherNode->area(), false);
							SetNodeAllCloseConnects(node, k, false);
							break;
						}
					}
				}
			}
		}
	}
}
int NavGraph::OpenOneWays(int ways[], int arrSize)
{
	int ret = 0;
	for (int iter = 0; iter < TAGNUMBER; iter++)
	{
		if (oneWayConnectOpenTagsDic[iter])
		{
			ways[ret] = iter;
		}
		ret++;
		if (ret >= arrSize)
		{
			break;
		}
	}
	return ret;
}
void NavGraph::GetNavInfoLen(int* verticesLen, int* trianglesLen)
{
	*verticesLen = Immutable->VerticesCount;
	*trianglesLen = Immutable->NodesCount * 3;
}
void NavGraph::GetNavInfo(Vector3 vertices[], int verticesLen, int triangles[], int trianglesLen, int tags[], int areas[])
{
	int minverticesLen = verticesLen < Immutable->VerticesCount ? verticesLen : Immutable->VerticesCount;
	Int3* fromV = Immutable->Vertices;
	for(int i =0; i<minverticesLen; i++)
	{
		vertices[i].X = Int3::ToFloat(fromV[i].X);
		vertices[i].Z = Int3::ToFloat(fromV[i].Z);
	}

	int mintrianglesLen = (trianglesLen/3) < Immutable->NodesCount? (trianglesLen/3) : Immutable->NodesCount;
	Node* nodes = Immutable->Nodes;
	for(int i=0; i<mintrianglesLen; i++)
	{
		triangles[i*3] = (int)(nodes[i].vList[0] - fromV);
		triangles[i*3 + 1] = (int)(nodes[i].vList[1] - fromV);
		triangles[i*3 + 2] = (int)(nodes[i].vList[2] - fromV);
		tags[i] = nodes[i].tags();
		areas[i] = nodes[i].area();
	}
}
const int NavGraph::disError = 2;
Int3 NavGraph::upDiff(-disError, 0);
Int3 NavGraph::downDiff(disError, 0);
Int3 NavGraph::rightDiff(0, disError);
Int3 NavGraph::leftDiff(0, -disError);
