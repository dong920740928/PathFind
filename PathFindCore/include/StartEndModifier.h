#pragma once
class ABPath;
class NavGraph;
class StartEndModifier
{
public:
	static void Apply(ABPath* p, NavGraph* graph);
};