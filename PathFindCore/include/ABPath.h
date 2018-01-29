#pragma once
#include "NaviPath.h"
#include <unordered_set>
#include "PathFindUtil.h"
#include "BaseType.h"
struct Int3Equal
{
	bool operator()(const Int3& a, const Int3& b)const
	{
		return a == b;
	}
};
struct  Int3PHash
{
	size_t operator()(const Int3& a)const
	{
		std::hash<int> hasher;
		return hasher(a.X) ^ hasher(a.Z);
	}
};
class NavGraph;
struct NNInfo;
class ABPath :public NaviPath
{
public:
	ABPath(const Int3* start, const Int3* end, TagType tags, bool useNavWeight);
	~ABPath();

	void Prepare(int nearDis);
	void Initialize();
	void CalculateStep();
private:
	Node* GetNode(const Int3& position, const NNConstraint* constraint);
	Node* GetNearest(const Int3& position, const NNConstraint* constraint);
public:
	TagType PathTags;
	const Int3 OriginalStartPoint;
	const Int3 OriginalEndPoint;
	NavGraph* Graph;
private:
	Node* startNode;
	Node* endNode;
	Int3 hTarget;

private:
	static std::unordered_set<Int3,Int3PHash,Int3Equal> StartPointLogDict;
};