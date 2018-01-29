/**
寻路系统不保证线程安全
**/
#pragma once
#include <vector>
#include <string>
#include "PathFindUtil.h"
#include "BaseType.h"
using namespace std;
class NavGraph;
class Node;
class PathFinder
{
private:
	PathFinder(PathFinder* another);

public:
	PathFinder(const char* filePath);
	PathFinder(const char* filePath, const char* bytes, int bytesCount);
	void* FindPath(const Vector3* start, const Vector3* end, TagType tags, TagType * pathTags, bool useNavWeight, int* count);
	void* FindPathNear(const Vector3* start, const Vector3* end,  float nearDis, TagType tags, TagType * pathTags, bool useNavWeight, int* count);
	void* FindPathTolerant(const Vector3* start, const Vector3* end, float range, TagType tags, TagType * pathTags, bool useNavWeight, int* count);
	
	int CanMove(const Vector3* start, const Vector3* end, TagType tags);
	int CheckXzPointInTriangle(float x, float z, TagType tags);
	int IsWalkable(float x, float z, TagType tags);
	void OpenOneWayConnect(TagType tags);
	void CloseOneWayConnect(TagType tags);
	int OpenOneWays(int ways[], int arrSize);
	int CanDirectMove(const Vector3* start, const Vector3* end, TagType tags);
	int LineCrossFarPoint(const Vector3* start, const Vector3* end, TagType tags, Vector3* ret);
	PathFinder* Clone();
	void* GetMoveDisPath(const Vector3* start, float angleY, float distance, TagType tags, int* count);
	void Dispose(bool freeRes);
	~PathFinder();
	bool GetAreaIfWalkable(Vector3* pos, TagType tag, int* area);
	static const char* GetStack();
	void SetIgnoreDirect(bool value);
	bool IsWalkableWithtag(const Vector3* pos, TagType tags, bool checkDirect, int* posTag);
	void GetNavInfoLen(int mod, int* verticesLen, int* trianglesLen);
	void GetNavInfo(int mod, Vector3 vertices[], int verticesLen, int triangles[], int trianglesLen, int tags[], int areas[]);
	void GetNavSize(int mod, int* minX, int* minZ, int* maxX, int* maxZ);
	void GetNodeInfo(int mod, int X, int Z, TagType tags, int* v0X, int* v0Z, int* v1X, int* v1Z, int* v2X, int* v2Z, int* adj1, int* adj2, int* adj3, int* index, int* tag, int* area);
private:
	bool FindPath(Int3& start, Int3& end, int nearDis, TagType tags, TagType * pathTags, bool useNavWeight, Int3Vector& rout);
	bool FindNode(NavGraph* navGragh, Int3& iStart, Int3& iEnd, TagType tags, Node** sNode, Node** tNode);
	void Init();
public: 
	string fileName;
	NavGraph* smallNavGragh;
	NavGraph* bigNavGragh;
	vector<NavGraph*>* graphList;
	bool IsValid;
	
public:
	static float AreaSize;
	static int DirectMoveTestDisS;
	static int directPathTime;
	static int astarPathTime;

	static GetStackCallback GetStackCb;
	//static/

};