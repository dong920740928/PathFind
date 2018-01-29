#pragma once
#include <vector>
#include <deque>
class ABPath;
class NavGraph;
class Node;
class Int3;
class Int3Vector;
using namespace std;
class FunnelModifier
{
public:
	static void Apply(ABPath* p, NavGraph* graph);
private:
	static void ConstructFunnel(NavGraph* graph, Int3Vector& vectorPath, vector<Node*>& path, int sIndex, int eIndex, Int3Vector& funnelPath,deque<Int3>& left, deque<Int3>& right);
	static void BuildFunnelCorridor(NavGraph* graph, vector<Node*>& path, int startIndex, int endIndex, deque<Int3>& left, deque<Int3>& right);
	static bool RunFunnel(deque<Int3>* left, deque<Int3>* right, Int3Vector& funnelPath);
};