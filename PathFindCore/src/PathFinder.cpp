#include "PathFinder.h"
#include "PathFindLoader.h"
#include <iostream>
#include <cmath>
#include "Node.h"
#include "NavGraphImmutable.h"
#include <unordered_map>
#include "ABPath.h"
#include "NaviPath.h"
#include "DirectMoveModifier.h"
#include "FunnelModifier.h"
#include "PathTagsModifier.h"
#include "StartEndModifier.h"
#include "PathLog.h"
#include <fstream>
#include <sstream>
float PathFinder::AreaSize = 1.0f;
int PathFinder::DirectMoveTestDisS = Int3::FloatPrecisionInt * 30 * Int3::FloatPrecisionInt * 30;
int PathFinder::directPathTime = 0;
int PathFinder::astarPathTime = 0;
GetStackCallback PathFinder::GetStackCb = NULL;

PathFinder::PathFinder(const char* filePath):
fileName(filePath),
	smallNavGragh(NULL),
	bigNavGragh(NULL),
	IsValid(false),
	graphList(NULL)
{
	ifstream fileStream;
	fileStream.open(fileName, std::ios::binary | std::ios::in);
	if (!fileStream || fileStream.eof())
	{
		std::ostringstream stringStream;
		stringStream << "PathFinder no nav file, FileName=" << filePath;
		PathLog::Error(stringStream.str().c_str());
		
	}
	else
	{
		graphList = PathFindLoader::Load(filePath, fileStream);
		Init();
	}
	fileStream.close();
}
PathFinder::PathFinder(const char* filePath, const char* bytes, int bytesCount):
fileName(filePath),
	smallNavGragh(NULL),
	bigNavGragh(NULL),
	IsValid(false),
	graphList(NULL)
{
	string str(bytes, bytesCount);
	istringstream fileStream(str,std::ios::binary);
	//fileStream.
	//fileStream.open(fileName, std::ios::binary | std::ios::in);
	if (!fileStream || fileStream.eof())
	{
		std::ostringstream stringStream;
		stringStream << "PathFinder no nav file, FileName=" << filePath;
		PathLog::Error(stringStream.str().c_str());

	}
	else
	{
		graphList = PathFindLoader::Load(filePath, fileStream);
		Init();
	}
	//fileStream.close();
}
void PathFinder::Init()
{
	if (graphList->size() >= 1)
	{
		smallNavGragh = (*graphList)[0];
		smallNavGragh->CreateRunData();
	}
	if (graphList->size() >= 2)
	{
		bigNavGragh = (*graphList)[1];
		IsValid = true;
	}
}
PathFinder::PathFinder(PathFinder* another):
fileName(another->fileName),
	IsValid(another->IsValid),
	graphList(NULL),
	smallNavGragh(NULL),
	bigNavGragh(NULL)
{
	graphList = new vector<NavGraph*>();
	if (another->smallNavGragh != NULL)
	{
		smallNavGragh = another->smallNavGragh->Clone();
		graphList->push_back(smallNavGragh);
	}
	if (another->bigNavGragh != NULL)
	{
		bigNavGragh = another->bigNavGragh->Clone();
		graphList->push_back(bigNavGragh);
	}
}
PathFinder* PathFinder::Clone()
{
	return new PathFinder(this);
}
void PathFinder::Dispose(bool freeRes)
{
	if (graphList !=NULL)
	{
		delete graphList;
		graphList = NULL;
	}
	if (smallNavGragh != NULL)
	{
		smallNavGragh->Dispose(freeRes);
		delete smallNavGragh;
		smallNavGragh = NULL;
	}
	if (bigNavGragh != NULL)
	{
		bigNavGragh->Dispose(freeRes);
		delete bigNavGragh;
		bigNavGragh = NULL;
	}
}
PathFinder::~PathFinder()
{
	Dispose(false);
}

bool PathFinder::GetAreaIfWalkable(Vector3* pos, TagType tag, int* area)
{
	Int3 iPos(*pos);
	Node* node = smallNavGragh->GetWalkableNode(iPos, tag, false, true);
	if (node == NULL)
	{
		return false;
	}
	*area = node->area();
	return true;
}

void* PathFinder::GetMoveDisPath(const Vector3* start, float angleY, float distance, TagType tags, int* count)
{
	Int3 intStart(*start);
	vector<Int3> iPath;
	if (bigNavGragh->IsWalkable(&intStart,tags))
	{
		bigNavGragh->GetMoveDisPath(&intStart,angleY,Int3::ToInt(distance),tags,iPath);
	}
	else
	{
		smallNavGragh->GetMoveDisPath(&intStart,angleY,Int3::ToInt(distance),tags,iPath);
	}
	if (iPath.size()>0)
	{
		int retcount = *count = (int)iPath.size();
		Vector3* ret = new Vector3[retcount];
		ret[0].X = start->X;
		ret[0].Z = start->Z;
		for(int i =1;i<retcount;i++)
		{
			Int3* n = &iPath[i];
			ret[i].X = Int3::ToFloat(n->X);
			ret[i].Z = Int3::ToFloat(n->Z);
		}
		return ret;
	}
	*count = 0;
	return NULL;
}
void* PathFinder::FindPath(const Vector3* start, const Vector3* end, TagType tags, TagType * pathTags, bool useNavWeight, int* count)
{
	return FindPathTolerant(start, end, 0,  tags, pathTags, useNavWeight, count);
}
void* PathFinder::FindPathNear(const Vector3* start, const Vector3* end,  float nearDis, TagType tags, TagType * pathTags, bool useNavWeight, int* count)
{
	Int3 iStart(*start);
	Int3 iEnd(*end);
	Int3Vector iPath;
	int iNearDis = Int3::ToInt(nearDis);
	FindPath(iStart, iEnd, iNearDis, tags, pathTags, useNavWeight, iPath);

	if (iPath.size()>1)
	{
		iNearDis -= (iPath.back()-iEnd).XZMagnitude();
		if(iNearDis > 0)
		{
			iPath.eraseLastDis(iNearDis);
		}
		int retcount = *count = iPath.size();
		Vector3* ret = new Vector3[retcount];
		for(int i =0;i<retcount;i++)
		{
			Int3* n = &iPath[i];
			ret[i].X = Int3::ToFloat(n->X);
			ret[i].Z = Int3::ToFloat(n->Z);
		}
		if (iPath[0].XZEquals(iStart))
		{
			ret[0].X = start->X;
			ret[0].Z = start->Z;
		}
		if (iPath[retcount -1].XZEquals(iEnd))
		{
			ret[retcount -1].X = end->X;
			ret[retcount -1].Z = end->Z;
		}
		return ret;
	}
	*count = 0;
	return NULL;
}
void * PathFinder::FindPathTolerant(const Vector3 * start, const Vector3 * end, float range, TagType tags, TagType * pathTags, bool useNavWeight, int * count)
{
	Int3 iStart(*start);
	Int3 iEnd(*end);
	Int3Vector iPath;
	FindPath(iStart, iEnd, Int3::ToInt(range), tags, pathTags, useNavWeight, iPath);

	if (iPath.size()>0)
	{
		int retcount = *count = iPath.size();
		Vector3* ret = new Vector3[retcount];
		for (int i = 0; i<retcount; i++)
		{
			Int3* n = &iPath[i];
			ret[i].X = Int3::ToFloat(n->X);
			ret[i].Z = Int3::ToFloat(n->Z);
		}
		if (iPath[0].XZEquals(iStart))
		{
			ret[0].X = start->X;
			ret[0].Z = start->Z;
		}
		if (iPath[retcount - 1].XZEquals(iEnd))
		{
			ret[retcount - 1].X = end->X;
			ret[retcount - 1].Z = end->Z;
		}
		return ret;
	}
	*count = 0;
	return NULL;
}

bool PathFinder::FindPath(Int3& start, Int3& end, int range, TagType tags, TagType * pathTags, bool useNavWeight, Int3Vector& rout)
{

	if (Int3::XZSquareDistance(start, end) < PathFinder::DirectMoveTestDisS)
	{
		PathFinder::directPathTime++;

		if (bigNavGragh != NULL && bigNavGragh->IsWalkable(&start, tags) && bigNavGragh->IsWalkable(&end, tags))
		{
			if (bigNavGragh->CanDirectMove(start,end, NULL, tags, *pathTags))
			{
				rout.push_back(start);
				rout.push_back(end);
				return true;
			}

		}
		else
		{
			if (smallNavGragh->CanDirectMove(start,end, NULL, tags, *pathTags))
			{
				rout.push_back(start);
				rout.push_back(end);
				return true;
			}
		}
	}
	rout.clear();
	PathFinder::astarPathTime++;

	ABPath p(&start, &end, tags, useNavWeight);
	p.Graph = smallNavGragh;
	p.VectorPath = &rout;

	p.PrepareBase(smallNavGragh->Immutable->RunDataPtr);

	p.Prepare(range);
	{
		if (!p.IsDone())
		{
			p.Initialize();
			while(!p.IsDone())
			{
				p.CalculateStep(); 
			}
		}
	}
	{
		StartEndModifier::Apply(&p,smallNavGragh);
	}
	{
		PathTagsModifier::Apply(&p);
	}
	{
		FunnelModifier::Apply(&p,smallNavGragh);
	}
	{
		DirectMoveModifier::Apply(&p,bigNavGragh);
	}
	
	*pathTags = p.PathTags;
	return true;
}
bool PathFinder::FindNode(NavGraph* navGragh, Int3& iStart, Int3& iEnd, TagType tags, Node** sNode, Node** tNode)
{
	*sNode = navGragh->GetWalkableNode(iStart, tags, true);
	if (*sNode == NULL)
	{
		return false;
	}
	*tNode = navGragh->GetWalkableNode(iEnd, tags);
	if (*tNode == NULL)
	{
		return false;
	}
	return true;
}
int PathFinder::CanMove(const Vector3* start, const Vector3* end, TagType tags)
{ 
	Int3 iStart(*start);
	Int3 iEnd(*end);
	NavGraph* navGragh = bigNavGragh;
	Node* sNode;
	Node* tNode;
	if (!FindNode(navGragh, iStart, iEnd, tags, &sNode, &tNode))
	{
		navGragh = smallNavGragh;
		if (!FindNode(navGragh, iStart, iEnd, tags, &sNode, &tNode))
		{
			return false;
		}
	}
	return navGragh->CanMove(sNode->area(),tNode->area(),tags);
}
int PathFinder::CheckXzPointInTriangle(float x, float z, TagType tags)
{
	
	Int3 pos(x, z);
	if (bigNavGragh->GetWalkableY(pos, tags) ||
		smallNavGragh->GetWalkableY(pos, tags))
	{
		return true;
	}
	return false;
}
int PathFinder::IsWalkable(float x, float z, TagType tags)
{
	Int3 pos(x, z);
	//下面的比较各有优点
	//return smallNavGragh->IsWalkable(&pos,tags);
	return bigNavGragh->IsWalkable(&pos,tags) || smallNavGragh->IsWalkable(&pos,tags);
}
void PathFinder::OpenOneWayConnect(TagType tags)
{
	for (unsigned int p = 0; p < graphList->size(); p++)
	{
		NavGraph* g = (*graphList)[p];
		g->OpenOneWayConnect(tags);
	}

}
void PathFinder::CloseOneWayConnect(TagType tags)
{
	for (unsigned int p = 0; p < graphList->size(); p++)
	{
		NavGraph* g = (*graphList)[p];
		g->CloseOneWayConnect(tags);
	}
}
int PathFinder::OpenOneWays(int ways[], int arrSize)
{
	return smallNavGragh->OpenOneWays(ways,arrSize);
}
int PathFinder::CanDirectMove(const Vector3* start, const Vector3* end, TagType navTags)
{
	TagType pathTags;
	Int3 iStart(*start);
	Int3 iEnd(*end);
	if (bigNavGragh->CanDirectMove(iStart, iEnd, NULL, navTags, pathTags))
	{
		return true;
	}
	else
	{
		return smallNavGragh->CanDirectMove(iStart, iEnd, NULL, navTags, pathTags);
	}
}
int PathFinder::LineCrossFarPoint(const Vector3* start, const Vector3* end, TagType tags, Vector3* ret)
{
	Int3 from(*start);
	Int3 to(*end);

	Int3 cp = smallNavGragh->LineCrossFarPoint(from,to,tags);
	if (ret != NULL)
	{
		ret->X = Int3::ToFloat(cp.X);
		ret->Z = Int3::ToFloat(cp.Z);
	}
	
	return from != cp;
}

const char* PathFinder::GetStack()
{
	if (GetStackCb != NULL)
	{
		return GetStackCb();
	}

	return NULL;
}
void PathFinder::SetIgnoreDirect(bool value)
{
	if (smallNavGragh != NULL)
	{
		smallNavGragh->IgnoreDirect = value;
	}
	if (bigNavGragh != NULL)
	{
		bigNavGragh->IgnoreDirect = value;
	}
}
bool PathFinder::IsWalkableWithtag(const Vector3* pos, TagType tags, bool checkDirect, int* posTag)
{
	Int3 iStart(*pos);
	Node* node = smallNavGragh->GetWalkableNode(iStart,tags,checkDirect);
	if (node == NULL)
	{
		*posTag = 0;
		return false;
	}
	*posTag = node->tags();
	return true;
}
void PathFinder::GetNavInfoLen(int mod, int* verticesLen, int* trianglesLen)
{
	NavGraph* g = smallNavGragh;
	if (mod ==1 && bigNavGragh != NULL)
	{
		g = bigNavGragh;
	}
	g->GetNavInfoLen(verticesLen, trianglesLen);

}
void PathFinder::GetNavInfo(int mod, Vector3 vertices[], int verticesLen, int triangles[], int trianglesLen, int tags[], int areas[])
{
	NavGraph* g = smallNavGragh;
	if (mod ==1 && bigNavGragh != NULL)
	{
		g = bigNavGragh;
	}
	g->GetNavInfo(vertices,verticesLen,triangles,trianglesLen, tags, areas);
}
void PathFinder::GetNavSize(int mod, int* minX, int* minZ, int* maxX, int* maxZ)
{
	NavGraph* g = smallNavGragh;
	if (mod ==1 && bigNavGragh != NULL)
	{
		g = bigNavGragh;
	}
	*minX = g->Immutable->AreaMinX;
	*minZ = g->Immutable->AreaMinZ;
	*maxX = g->Immutable->AreaMaxX;
	*maxZ = g->Immutable->AreaMaxZ;
}
void PathFinder::GetNodeInfo(int mod, int X, int Z, TagType tags, int* v0X, int* v0Z, int* v1X, int* v1Z, int* v2X, int* v2Z, int* adj1, int* adj2, int* adj3, int* index, int* tag, int* area)
{
	NavGraph* g = smallNavGragh;
	if (mod ==1 && bigNavGragh != NULL)
	{
		g = bigNavGragh;
	}
	Node* node = g->GetWalkableNode(Int3(X, Z),tags,false,true);
	if (node == NULL)
	{
		*v0X = 0;
		*v0Z = 0;
		*v1X = 0;
		*v1Z = 0;
		*v2X = 0;
		*v2Z = 0;
		*adj1 = -1;
		*adj2 = -1;
		*adj3 = -1;
		*index =-1;
		*tag = 0;
		*area = 0;
		return;
	}
	*v0X = node->vList[0]->X;
	*v0Z = node->vList[0]->Z;
	*v1X = node->vList[1]->X;
	*v1Z = node->vList[1]->Z;
	*v2X = node->vList[2]->X;
	*v2Z = node->vList[2]->Z;
	*adj1 = g->Immutable->AllConnects[node->nodeIndex * 3 + 0] == NULL ? -1:  g->Immutable->AllConnects[node->nodeIndex * 3 + 0]->nodeIndex;
	*adj2 = g->Immutable->AllConnects[node->nodeIndex * 3 + 1] == NULL ? -1:  g->Immutable->AllConnects[node->nodeIndex * 3 + 1]->nodeIndex;
	*adj3 = g->Immutable->AllConnects[node->nodeIndex * 3 + 2] == NULL ? -1:  g->Immutable->AllConnects[node->nodeIndex * 3 + 2]->nodeIndex;
	*index = node->nodeIndex;
	*tag = node->tags();
	*area = node->area();
}