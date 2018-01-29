#pragma once
class ABPath;
class NavGraph;

class DirectMoveModifier
{
public:
	static void Apply(ABPath* p, NavGraph* graph);
private:
	static void CheckDirectMove(ABPath* p, NavGraph* graph, bool direct);
};