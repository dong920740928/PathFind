#pragma once
#include "BaseType.h"
class Node;
class NNConstraint
{
public:
	NNConstraint();
	bool Suitable(Node* node)const;
public:
	int area;
	TagType tags;
	bool useNavWeight;
	int constrainDistance;
	//bool isEnd;
	bool checkDir;
};

class PathNNConstraint : public NNConstraint
{
public: 
	void SetStart(Node* node);
};