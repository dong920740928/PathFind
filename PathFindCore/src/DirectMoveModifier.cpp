#include "DirectMoveModifier.h"
#include "ABPath.h"
#include "NavGraph.h"
void DirectMoveModifier::Apply(ABPath* p, NavGraph* graph)
{
	p->VectorPath->reverse();
	CheckDirectMove(p,graph,false);
	p->VectorPath->reverse();
	CheckDirectMove(p,graph,true);
}
void DirectMoveModifier::CheckDirectMove(ABPath* p, NavGraph* graph, bool direct)
{
	if (p->VectorPath->size() <= 2)
	{
		return;
	}
	int i = 0;
	while (i + 2 <(int) p->VectorPath->size())
	{

		int left = i + 1;
		int right = p->VectorPath->size() - 1;
		while (left <= right)
		{
			int middle = (left + right) / 2;
			TagType tag;
			if (direct ? graph->CanDirectMove((*p->VectorPath)[i],(*p->VectorPath)[middle], NULL, p->EnabledTags, tag, false) : graph->CanDirectMove((*p->VectorPath)[middle], (*p->VectorPath)[i], NULL, p->EnabledTags, tag, false))
			{
				left = middle + 1;
				p->PathTags |= tag;
			}
			else
			{
				right = middle -1;
			}
		}
		p->VectorPath->eraseRang(i + 1, left - 1);
		i++;
	}
}