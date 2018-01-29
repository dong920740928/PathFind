#include "NNConstraint.h"
#include "Node.h"
bool NNConstraint::Suitable(Node* node)const
{
	return node->IsWalkAble(tags);
}
NNConstraint::NNConstraint()
{
	area = -1;
	tags = 0xffffffff;
	useNavWeight = false;
	constrainDistance = 0;
	//isEnd = false;
	checkDir = true;
}
void PathNNConstraint::SetStart(Node* node)
{
	if (node != NULL)
	{
		area = node->area();
	}
}