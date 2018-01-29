#pragma once
#include "PathFindUtil.h"
#include "NodeRunData.h"
#include "ABPath.h"
#include "BaseType.h"
const int WalkableBitNumber = 23; /**< Bit number for the #walkable bool */
const int WalkableBit = 1 << WalkableBitNumber; /** 1 \<\< #WalkableBitNumber */

const int AreaBitNumber = 24; /**< Bit number at which #area starts */
const int AreaBitsSize = 0xFF; /**< Size of the #area bits */
const int NotAreaBits = ~(AreaBitsSize << AreaBitNumber); /**< The bits in #flags which are NOT #area bits */

const int GraphIndexBitNumber = 18;
const int GraphIndexBitsSize = 0x1F;
const int NotGraphIndexBits = ~(GraphIndexBitsSize << GraphIndexBitNumber); /**< Bits which are NOT #graphIndex bits */

class NodeRun;
class NodeRunData;
 class Node
 {
 public:
	 int tags()const
	 {
		return (flags >> 9) & 0x1F;
	 }
	 int weight()const
	 {
		 return (flags >> 16) & 0x3;
	 }
	 bool walkable()const
	 {
		 return (flags & WalkableBit) == WalkableBit;
	 }
	 bool IsWalkAble(TagType navTags)const
	 {
		 return walkable() && (((navTags >> tags()) & 0x1) != 0);
	 }
	 NodeRun* GetNodeRun(NodeRunData* data)
	 {
		return data->runNodes + nodeIndex;
	 }
	 int area()const
	 {
		return (flags >> AreaBitNumber) & AreaBitsSize;
	 }
	 void UpdateH(Int3& targetPosition, NodeRun* nodeR)
	 {
		nodeR->h = (position - targetPosition).XZMagnitude();
	 }
	 void UpdateG(NodeRun* nodeR, NodeRunData* nodeRunData)
	 {
		 nodeR->g = nodeR->parent->g + nodeR->cost;
	 }
	 void Open(NodeRunData* nodeRunData, NodeRun* nodeR, Int3* targetPosition, ABPath* path);
	 std::string ToString();
 public:
	 static int GetWeightCost(Node* node1, Node* node2);
	 static int GetCost(Node* node1, Node* node2);
 public:
	 int nodeIndex;
	 Int3* vList[4];
	 Int3 position;
	 int flags;
	 AABB aabb;
 };