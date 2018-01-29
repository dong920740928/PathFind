#include "StartEndModifier.h"
#include "ABPath.h"
#include "NavGraph.h"
 void StartEndModifier::Apply(ABPath* p, NavGraph* graph)
 {
	 if (p->VectorPath->size() == 0)
	 {
		 return;
	 }
	 if (p->VectorPath->size() == 1)
	 {
		 p->VectorPath->push_back(p->VectorPath->back());
	 }
	 (*p->VectorPath)[0] = p->OriginalStartPoint;
	 (*p->VectorPath)[p->VectorPath->size()-1] = p->OriginalEndPoint;

	 if (!graph->IsWalkable(&p->OriginalEndPoint, p->EnabledTags))
	 {
		 Int3 crossPt = graph->LineCrossFarPoint((*p->VectorPath)[p->VectorPath->size()-2], p->OriginalEndPoint,p->EnabledTags);
		 (*p->VectorPath)[p->VectorPath->size()-1] = crossPt;
	 }
 }