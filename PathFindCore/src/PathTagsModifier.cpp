#include "PathTagsModifier.h"
#include "ABPath.h"
#include <vector>
#include "Node.h"
#include "NaviPath.h"
void PathTagsModifier::Apply(ABPath* p)
{
	TagType tag = 0;
	for(std::vector<Node*>::iterator iter = p->Path.begin(); iter != p->Path.end(); ++iter)
	{
		tag |= (1u << ((*iter)->tags()));
	}
	p->PathTags = tag;
}