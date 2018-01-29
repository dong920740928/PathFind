#pragma once
using namespace std;
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include "BaseType.h"
#include <deque>
class NodeRunData;
class Node;
class Int3;
class AABB;
class NodeListType;
class BresenhamLineType;
class AreaConnectList;
class NavGraphImmutable
{
public:
	NavGraphImmutable(int nodeNum, const char* name);
	~NavGraphImmutable();
	void CreateRunData();
	void CreateAreaNodeList(float xSize, float zSize);

	//调用者不能释放返回值
	NodeListType* GetIntersectAreaNodes(const Int3* pos);
	void GetIntersectAreaNodes(AABB& aabb, unordered_set<Node*>& nodeList);
private:
	
	static void BresenhamLine(int startX, int startZ, int endX, int endZ, int stepX, int stepZ, BresenhamLineType& arr);
	static int Div(int x, int y);
	void ParseAllNode(int* ZRangMin, int* ZRangMax, bool* ZRangValid, BresenhamLineType& indexs, bool first);
public:
	string* Name;
	Node* Nodes;
	int NodesCount;
	bool ContainOneWayConnect;
	Int3* Vertices;
	int VerticesCount;
	vector<int*>** OneWayConnectDic;
	unordered_map<int, AreaConnectList*>* AreaDirectConnectsDic;
	
	int* AreaTagDic;
	NodeRunData* RunDataPtr;
	NodeListType*** AreaNodeList;

	int* AllConnectionCosts;
	int* AllWeightConnectionCosts;
	int* AllConnectOtherEdgeIndexs;
	Node** AllConnects;

	int AreaXNum;
	int AreaZNum;
	int AreaMinX;
	int AreaMinZ;
	int AreaMaxX;
	int AreaMaxZ;
	int AreaXSize;
	int AreaZSize;
private:
	bool isError;
};