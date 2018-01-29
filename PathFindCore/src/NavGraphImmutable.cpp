#include "NavGraphImmutable.h"
#include "Node.h"
#include "NodeRunData.h"
#include "PathFindUtil.h"
#include "PathFindLoader.h"
#include "PathLog.h"
#include <string>
#include <iostream>
NavGraphImmutable::NavGraphImmutable(int nodeNum, const char* name)
{
	memset(this, 0, sizeof(NavGraphImmutable));
	this->Name = new string(name);

	NodesCount = nodeNum;

	void* mem = new char[sizeof(Node) * NodesCount];
	if (mem == NULL)
	{
		isError= true;
		return;
	}
	memset(mem,0,sizeof(Node) * NodesCount);
	Nodes = (Node*)mem;

	AllConnectionCosts = new int[NodesCount * 3];
	if (AllConnectionCosts == NULL)
	{
		isError= true;
		return;
	}
	memset(AllConnectionCosts,0,sizeof(int)* NodesCount*3);

	AllWeightConnectionCosts = new int[NodesCount * 3];
	if (AllWeightConnectionCosts == NULL)
	{
		isError= true;
		return;
	}
	memset(AllWeightConnectionCosts,0,sizeof(int)* NodesCount*3);

	AllConnectOtherEdgeIndexs = new int[NodesCount * 3];
	if (AllConnectOtherEdgeIndexs == NULL)
	{
		isError= true;
		return;
	}
	memset(AllConnectOtherEdgeIndexs,0,sizeof(int)* NodesCount*3);

	AllConnects = new Node*[NodesCount * 3];
	if (AllConnects == NULL)
	{
		isError= true;
		return;
	}
	memset(AllConnects,0,sizeof(Node*)* NodesCount*3);

	OneWayConnectDic = new vector<int*>*[TAGNUMBER];
	memset(OneWayConnectDic, 0 , sizeof(vector<int*>*) * TAGNUMBER);

	
}
NavGraphImmutable::~NavGraphImmutable()
{
	if (Name != NULL)
	{
		delete Name;
		Name = NULL;
	}
	if (Vertices != NULL)
	{
		delete Vertices;
		Vertices = NULL;
	}
	if (Nodes != NULL)
	{
		delete Nodes;
		Nodes = NULL;
	}
	if (AllConnectionCosts !=NULL)
	{
		delete AllConnectionCosts;
		AllConnectionCosts = NULL;
	}
	if (AllWeightConnectionCosts !=NULL)
	{
		delete AllWeightConnectionCosts;
		AllWeightConnectionCosts = NULL;
	}
	if (AllConnectOtherEdgeIndexs !=NULL)
	{
		delete AllConnectOtherEdgeIndexs;
		AllConnectOtherEdgeIndexs = NULL;
	}
	if (AllConnects !=NULL)
	{
		delete AllConnects;
		AllConnects = NULL;
	}
	if (RunDataPtr != NULL)
	{
		delete RunDataPtr;
		RunDataPtr = NULL;
	}
	if (AreaNodeList != NULL)
	{
		for (int i = 0; i < AreaXNum; i++)
		{
			NodeListType** col =AreaNodeList[i];
			for (int j = 0; j < AreaZNum; j++)
			{
				if (col[j]!= NULL)
				{
					delete col[j];
				}
			}
			delete col;
		}
		delete AreaNodeList;
		AreaNodeList = NULL;
	}
	
	if (AreaDirectConnectsDic !=NULL)
	{
		for(unordered_map<int, AreaConnectList*>::iterator iter = AreaDirectConnectsDic->begin(); iter != AreaDirectConnectsDic->end(); ++iter)
		{
			//(*iter).
			AreaConnectList* v = iter->second;
			delete v;
		}
		delete AreaDirectConnectsDic;
		AreaDirectConnectsDic = NULL;
	}
	if (AreaTagDic !=NULL)
	{
		delete AreaTagDic;
		AreaTagDic = NULL;
	}

	if (OneWayConnectDic != NULL)
	{
		for(int i =0; i< TAGNUMBER; i++)
		{
			vector<int*>* v = OneWayConnectDic[i];
			if (v == NULL)
			{
				continue;
			}
			for(vector<int*>::iterator iter2 = v->begin();iter2!=v->end();++iter2)
			{	
				int* intarr = *iter2;
				delete intarr;
			}
			delete v;
		}
		delete OneWayConnectDic;
		OneWayConnectDic = NULL;
	}
}

void NavGraphImmutable::CreateRunData()
{
	if (RunDataPtr != NULL)
	{
		return;
	}
	RunDataPtr = new NodeRunData(NodesCount, Nodes);
}
void NavGraphImmutable::CreateAreaNodeList(float xSize, float zSize)
{
	AreaXSize = (int)round(xSize*Int3::FloatPrecision);
	AreaZSize = (int)round(zSize*Int3::FloatPrecision);
	int minX = PathfindMaxInt;
	int minZ = PathfindMaxInt;
	int maxX = PathfindMinInt;
	int maxZ = PathfindMinInt;
	{


		for (int i= 0; i<NodesCount; i++)
		{
			Node* node = Nodes + i;
			Int3& v1 = *node->vList[0];
			Int3& v2 = *node->vList[1];
			Int3& v3 = *node->vList[2];

			minX = minX < v1.X ? minX : v1.X;
			minZ = minZ < v1.Z ? minZ : v1.Z;

			maxX = maxX > v1.X ? maxX : v1.X;
			maxZ = maxZ > v1.Z ? maxZ : v1.Z;

			minX = minX < v2.X ? minX : v2.X;
			minZ = minZ < v2.Z ? minZ : v2.Z;

			maxX = maxX > v2.X ? maxX : v2.X;
			maxZ = maxZ > v2.Z ? maxZ : v2.Z;

			minX = minX < v3.X ? minX : v3.X;
			minZ = minZ < v3.Z ? minZ : v3.Z;

			maxX = maxX > v3.X ? maxX : v3.X;
			maxZ = maxZ > v3.Z ? maxZ : v3.Z;
		}
	}
	AreaMinX = minX - 2 * AreaXSize;
	AreaMinZ = minZ - 2 * AreaZSize;
	AreaMaxX = maxX + 2 * AreaXSize;
	AreaMaxZ = maxZ + 2 * AreaZSize;

	AreaXNum = (AreaMaxX - AreaMinX + AreaXSize) / AreaXSize;
	AreaZNum = (AreaMaxZ - AreaMinZ + AreaXSize) / AreaZSize;
	AreaNodeList =new NodeListType**[AreaXNum];
	{
		for (int i = 0; i < AreaXNum; i++)
		{
			AreaNodeList[i] = new NodeListType*[AreaZNum];
			memset(AreaNodeList[i], 0, sizeof(NodeListType*) * AreaZNum);
		}
	}
	BresenhamLineType indexs(1024);
	{
		int* ZRangMin = new int[AreaXNum];
		int* ZRangMax = new int[AreaXNum];
		bool* ZRangValid = new bool[AreaXNum];
		ParseAllNode(ZRangMin, ZRangMax, ZRangValid, indexs, true);
		//ParseAllNode(ZRangMin, ZRangMax, ZRangValid, indexs, false);
		delete ZRangValid;
		delete ZRangMin;
		delete ZRangMax;
	}
}
void NavGraphImmutable::ParseAllNode(int* ZRangMin, int* ZRangMax, bool* ZRangValid, BresenhamLineType& indexs, bool first)
{
	int XRangMin;
	int XRangMax;
	for (int i= 0; i<NodesCount; i++)
	{
		Node& node = Nodes[i];
		//xMap.
		memset(ZRangValid, 0, sizeof(bool) * AreaXNum);
		XRangMin = AreaXNum;
		XRangMax = -1;

		AABB& aabb = node.aabb;
		int minXIndex = (aabb.MinX - AreaMinX) / AreaXSize;
		int minZIndex = (aabb.MinZ - AreaMinZ) / AreaZSize;

		int maxXIndex = (aabb.MaxX - AreaMinX + AreaXSize - 1) / AreaXSize;
		int maxZIndex = (aabb.MaxZ - AreaMinZ + AreaXSize - 1) / AreaZSize;
		//计算三角形所占的格子
		for (int i = 0; i < 3; i++)
		{
			Int3* start = node.vList[i];
			Int3* end = node.vList[i + 1];
			indexs.clear();
			BresenhamLine(start->X - AreaMinX, start->Z - AreaMinZ, end->X - AreaMinX, end->Z - AreaMinZ, AreaXSize, AreaZSize, indexs);

			for (int j = 0,count = indexs.size(); j < count; j += 2)
			{
				int x = indexs.m_data[j];
				int z = indexs.m_data[j + 1];
				//直线探测算法会丢掉一些格子,比如经过下面的格子的中间点时，左上和右下的格子不认为相交
				//_______
				//|__|__|
				//|__|__|

				for (int x1 = x - 1; x1 <= x + 1; x1++)
				{
					if (x1 >= 0 && x1 < AreaXNum)
					{
						if (x1 > XRangMax)
						{
							XRangMax = x1;
						}
						if (x1 < XRangMin)
						{
							XRangMin = x1;
						}
						for (int z1 = z - 1; z1 <= z + 1; z1++)
						{
							if (ZRangValid[x1])
							{
								if (z1 < ZRangMin[x1])
								{
									ZRangMin[x1] = z1;
								}
								if (z1 > ZRangMax[x1])
								{
									ZRangMax[x1] = z1;
								}
							}
							else
							{
								ZRangValid[x1] = true;
								ZRangMin[x1] = z1;
								ZRangMax[x1] = z1;
							}
						}


					}
				}
			}


		}
		for(int x = XRangMin; x <= XRangMax; x++)
		{
			NodeListType** areaNodeListX = AreaNodeList[x];
			for (int z = ZRangMin[x]; z <= ZRangMax[x]; z++)
			{
				if (z < AreaZNum && z>=0)
				{
					NodeListType* areaNodeListXZ= areaNodeListX[z];
					if (areaNodeListXZ == NULL)
					{
						areaNodeListXZ = areaNodeListX[z] = new NodeListType();
					}
					areaNodeListXZ->push_back(&node);
				}
			}
		}
	}

}
int NavGraphImmutable::Div(int x, int y)
{
	if (x >= 0)
	{
		return x / y;
	}
	else
	{
		return (x - (y - 1)) / y;
	}
}
void NavGraphImmutable::BresenhamLine(int startX, int startZ, int endX, int endZ, int stepX, int stepZ, BresenhamLineType& arr)
{

	int realX, realY;
	int x, z, dx, dz, temp;
	int realAddX, realStepX, realStepZ;
	long long realAddZ, offsetZ, stepDeleteZ;
	//调整XY
	bool swapXZ = false;
	dx = endX > startX ? endX - startX : startX - endX;
	dz = endZ > startZ ? endZ - startZ : startZ - endZ;
	if (dx < dz)
	{
		swapXZ = true;
		temp = startX;
		startX = startZ;
		startZ = temp;

		temp = endX;
		endX = endZ;
		endZ = temp;

		temp = dx;
		dx = dz;
		dz = temp;
	}

	realStepX = endX - startX >= 0 ? stepX : -stepX;
	realStepZ = endZ - startZ >= 0 ? stepZ : -stepZ;

	x = startX;
	if (realStepZ < 0)
	{
		z = Div(startZ + stepZ - 1, stepZ) * stepZ;
		offsetZ = (long long)(z - startZ) * dx;
	}
	else
	{
		z = Div(startZ, stepZ) * stepZ;
		offsetZ = (long long)(startZ - z) * dx;
	}
	int count = 0;

	//x每步进一格，Z的偏移量，按道理应该要除以dx，但为防止误差，给stepDeleteZ乘了dx
	stepDeleteZ = (long long)stepZ * dx;
	do
	{
		if (realStepX >= 0)
		{
			realX = Div(x, stepX);
		}
		else
		{
			realX = Div(x - 1, stepX);
		}

		if (realStepZ >= 0)
		{
			realY = Div(z, stepZ);
		}
		else
		{
			realY = Div(z - 1, stepZ);
		}

		if (swapXZ)
		{

			arr.push_back(realY, realX);
		}
		else
		{
			arr.push_back(realX, realY);
		}

		if (realStepX > 0)
		{
			int nextX = Div(x + realStepX, stepX) * stepX;
			nextX = nextX < endX ? nextX : endX;
			realAddX = nextX - x;
			realAddZ = (long long)realAddX * dz;
		}
		else
		{
			int nextX = Div(x + realStepX + stepX - 1, stepX) * stepX;
			nextX = nextX > endX ? nextX : endX;
			realAddX = nextX - x;
			realAddZ = -(long long)realAddX * dz;
		}


		offsetZ += realAddZ;

		if (offsetZ >= stepDeleteZ)
		{
			z += realStepZ;
			if (offsetZ > stepDeleteZ)
			{
				if (realStepX >= 0)
				{
					realX = Div(x, stepX);
				}
				else
				{
					realX = Div(x - 1, stepX);
				}

				if (realStepZ >= 0)
				{
					realY = Div(z, stepZ);
				}
				else
				{
					realY = Div(z - 1, stepZ);
				}

				if (swapXZ)
				{
					arr.push_back(realY,realX);
				}
				else
				{
					arr.push_back(realX,realY);
				}
			}
			offsetZ -= stepDeleteZ;
		}
		x += realAddX;
		if ((realStepX > 0 && x >= endX) || (realStepX < 0 && x <= endX))
		{
			break;
		}
		count++;
		if (count > 1000)
		{
			std::ostringstream stringStream;
			stringStream << " BresenhamLine error  " << count << " " << x << " " << z << " " << realAddX + " " << realAddZ << " " << startX << " " << startZ << " " << endX << " " << endZ;
			PathLog::Error(stringStream.str().c_str());
			break;
		}
	} while (true);

}
void NavGraphImmutable::GetIntersectAreaNodes(AABB& aabb, unordered_set<Node*>& nodeList)
{

	int minX = (aabb.MinX - AreaMinX) / AreaXSize;
	minX = minX < 0 ? 0 : minX;
	int maxX = ((aabb.MaxX - AreaMinX) + AreaXSize - 1) / AreaXSize;
	maxX = maxX > AreaXNum ? AreaXNum : maxX;
	int minZ = (aabb.MinZ - AreaMinZ) / AreaZSize;
	minZ = minZ < 0 ? 0 : minZ;
	int maxZ = ((aabb.MaxZ - AreaMinZ) + AreaZSize - 1) / AreaZSize;
	maxZ = maxZ > AreaZNum ? AreaZNum : maxZ;
	for (int i = minX; i < maxX; i++)
	{
		for (int j = minZ; j < maxZ; j++)
		{
			NodeListType* nodeList2 = AreaNodeList[i][j];
			if (nodeList2 == NULL)
			{
				continue;
			}
			for(int iter = 0; iter<nodeList2->m_last;iter++)
			{
				nodeList.insert(nodeList2->m_data[iter]);
			}
		}
	}
}

NodeListType* NavGraphImmutable::GetIntersectAreaNodes(const Int3* pos)
{
	int p1 = (pos->X - AreaMinX) / AreaXSize;
	int p2 = (pos->Z - AreaMinZ) / AreaZSize;
	if (p1 >= 0 && p2 >= 0 && p1 < AreaXNum && p2 < AreaZNum)
	{
		return AreaNodeList[p1][p2];
	}
	else
	{
		return NULL;
	}
}