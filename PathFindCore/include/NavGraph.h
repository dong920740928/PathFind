#pragma once
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include "BaseType.h"
class MeshStruct;
class Int3;
class Node;
class NNConstraint;
class NavGraphImmutable;
class Int3LineList;
class Int3Line;
class Int3Vector;
using namespace std;
class NavGraph
{
public:
	~NavGraph();
	NavGraph(MeshStruct* ms, int areaSizeF);
	void CreateRunData();
	bool GetNodeAllCloseConnects(Node* n, int i);
	void SetNodeAllCloseConnects(Node* n, int i, bool value);
	void SetAreaCloseOneway(int fromArea, int toArea, bool close);
	bool CanDirectMove(const Int3& start, const Int3& end, Int3Vector* path, TagType navTags, TagType& pathTags, bool isOut = true);
	bool CanMove(int sArea, int tArea, TagType navTags);
	Node* GetNode(const Int3& position, const NNConstraint* constraint);
	Node* GetNearest(const Int3& position, const NNConstraint* constraint);
	bool GetWalkableY(const Int3& point, TagType tags);
	bool IsWalkable(const Int3* point, TagType tags);
	void GetMoveDisPath(const Int3* start, float angleY, int distance2Move, TagType navTags, vector<Int3>& rout);
	Node* GetWalkableNode(const Int3& point, TagType navTags, bool checkDirect = false, bool isExact = false, bool isEnd = false);
	void OpenOneWayConnect(TagType tags);
	void CloseOneWayConnect(TagType tags);
	int OpenOneWays(int ways[], int arrSize);
	Int3 LineCrossFarPoint(const Int3& from, const Int3& to, TagType navTags);
	void GetNavInfoLen(int* verticesLen, int* trianglesLen);
	void GetNavInfo(Vector3 vertices[], int verticesLen, int triangles[], int trianglesLen, int tags[], int areas[]);
	NavGraph* Clone();
	void Dispose(bool freeRes);
private:
	NavGraph();
	
	Int3 GetNearestPointOnNode(Node* n, const Int3& p);
	vector<Int3*>* LineCrossFarLines(const Int3& from, const Int3& to, TagType navTags);
	bool IsBorderLine(Node* currenttNode, int index, TagType tags);
	void DirectRaycast(const Int3& start, const Int3& end, TagType navTags, Int3LineList* farthestCrossBorderLines, Int3Vector* crossPoints, Int3& recastFarthestPoint, bool& canDirectMove, Int3& realStart, Int3& realEnd, TagType& pathTags, bool isOut = true);
	bool LineCrossLinePoint2D2(const Int3& pointA, const Int3& pointB, const Int3& pointC, const Int3& pointD, Int3& cp);
	//从多条线中找出一条角度最小的线段（可能有多条）
	void GetMinDegLine(Int3LineList& lines, Int3& start, Int3& end, Int3& nextPoint);
	unordered_map<int, vector<int>*>* GenerateAreaConnects(TagType navTags);
	void ClearAreaIndirectConnectsCache();
private:
	unordered_map<int, bool>* allCloseConnects;
	bool* oneWayConnectOpenTagsDic;
	unordered_map<TagType, unordered_map<int, vector<int>*>*>* AreaIndirectConnectsCache;
	unordered_map<int, unordered_set<int>*>* AreaCloseOneway;
	bool isValid;

	static const int disError;
	static Int3 upDiff;
	static Int3 downDiff;
	static Int3 rightDiff;
	static Int3 leftDiff;
public:
	NavGraphImmutable* Immutable;
	bool IgnoreDirect;
};
