#include "PathFindCore.h"
#include "PathFinder.h"
#include "PathLog.h"
#include <iostream>
#ifdef _DEBUG
//#include "vld.h"
#endif // _DEBUG

PATHFINDCOREDLL_API int STDCALL Add(int a, int b)
{
	return a + b;
}
PATHFINDCOREDLL_API void STDCALL FreeMemory(void* mem)
{
	if (NULL != mem)
	{
		delete (char*)mem;
		mem = NULL;
	}
}
PATHFINDCOREDLL_API void STDCALL SetLogCallback(LogCallback debug, LogCallback warn, LogCallback error)
{
	PathLog::DebugCb = debug;
	PathLog::WarnCb = warn;
	PathLog::ErrorCb = error;
	debug("C++ Pathfind System: initialization is complete");
}
PATHFINDCOREDLL_API void STDCALL SetGetStackCallback(GetStackCallback cb)
{
	PathFinder::GetStackCb = cb;
}

PATHFINDCOREDLL_API void* STDCALL CreatePathFinder(const char* filePath)
{
	PathFinder* ret = new PathFinder(filePath);
	if (ret->IsValid)
	{
		return ret;
	}
	delete ret;
	return NULL;
}
PATHFINDCOREDLL_API void* STDCALL CreatePathFinderWithBytes(const char* filePath, const char* bytes, int bytesCount)
{
	PathFinder* ret = new PathFinder(filePath, bytes, bytesCount);
	if (ret->IsValid)
	{
		return ret;
	}
	delete ret;
	return NULL;
}

PATHFINDCOREDLL_API void* STDCALL ClonePathFinder(void* anthor)
{
	if (anthor != NULL)
	{
		PathFinder *path = (PathFinder*)anthor;
		return path->Clone();
	}
	return NULL;
}

PATHFINDCOREDLL_API void STDCALL FreePathFinder(void *p, int freeRes)
{
	if (p!=NULL)
	{
		PathFinder *path = (PathFinder*)p;
		path->Dispose(freeRes != 0);
		delete path;
		p = NULL;
	}
}

PATHFINDCOREDLL_API void STDCALL GetVector3Array(Vector3 v[], int count, void* path)
{
	memcpy(v,path,sizeof(Vector3)*count);
	delete (char*)path;
}

PATHFINDCOREDLL_API int STDCALL GetAreaIfWalkable(void * pathfinder, Vector3 pos, TagType tag, int* area)
{
	*area = 0;
	if (pathfinder != NULL)
	{
		PathFinder *path = (PathFinder*)pathfinder;
		return path->GetAreaIfWalkable(&pos, tag, area);
	}
	return 0;
}
PATHFINDCOREDLL_API void* STDCALL FindPath(void * pathfinder, Vector3 start, Vector3 end, TagType tags, TagType* pathTags, int* count, int useNavWeight)
{
	*count = 0;
	if (pathfinder!=NULL)
	{
		PathFinder *path = (PathFinder*)pathfinder;
		return path->FindPath(&start,&end,tags,pathTags,useNavWeight != 0,count);
	}
	return NULL;
}
PATHFINDCOREDLL_API void* STDCALL FindPathNear(void * pathfinder, Vector3 start, Vector3 end, float nearDis, TagType tags, TagType* pathTags, int* count, int useNavWeight)
{
	*count = 0;
	if (pathfinder!=NULL)
	{
		PathFinder *path = (PathFinder*)pathfinder;
		return path->FindPathNear(&start,&end,nearDis, tags,pathTags,useNavWeight != 0,count);
	}
	return NULL;
}
PATHFINDCOREDLL_API void* STDCALL FindPathTolerant(void * pathfinder, Vector3 start, Vector3 end, float range, TagType tags, TagType* pathTags, int* count, int useNavWeight)
{
	*count = 0;
	if (pathfinder != NULL)
	{
		PathFinder *path = (PathFinder*)pathfinder;
		return path->FindPathTolerant(&start, &end, range, tags, pathTags, useNavWeight != 0, count);
	}
	return NULL;
}
PATHFINDCOREDLL_API void* STDCALL GetMoveDisPath(void * pathfinder, Vector3 start, float angleY, float distance, TagType tags, int* count)
{
	if (pathfinder!=NULL)
	{
		PathFinder *path = (PathFinder*)pathfinder;
		return path->GetMoveDisPath(&start,angleY,distance,tags, count);
	}
	return NULL;
}

PATHFINDCOREDLL_API int STDCALL CanMove(void * pathfinder, Vector3 start, Vector3 end, TagType tags)
{
	if (pathfinder!=NULL)
	{
		PathFinder *path = (PathFinder*)pathfinder;
		return path->CanMove(&start,&end,tags);
	}

	return 0;

}

PATHFINDCOREDLL_API int STDCALL CheckXzPointInTriangle(void * pathfinder, float x, float z, TagType tags, float* y)
{
	if (pathfinder!=NULL)
	{
		PathFinder *path = (PathFinder*)pathfinder;
		return path->CheckXzPointInTriangle(x,z,tags);
	}

	return 0;
}

PATHFINDCOREDLL_API int STDCALL IsWalkable(void * pathfinder, float x, float z, TagType tags)
{
	if (pathfinder!=NULL)
	{
		PathFinder *path = (PathFinder*)pathfinder;
		return path->IsWalkable(x,z,tags);
	}

	return 0;
}

PATHFINDCOREDLL_API void STDCALL OpenOneWayConnect(void * pathfinder,TagType tags)
{
	if (pathfinder!=NULL)
	{
		PathFinder *path = (PathFinder*)pathfinder;
		path->OpenOneWayConnect(tags);
	}
}

PATHFINDCOREDLL_API void STDCALL CloseOneWayConnect(void * pathfinder,TagType tags)
{
	if (pathfinder!=NULL)
	{
		PathFinder *path = (PathFinder*)pathfinder;
		path->CloseOneWayConnect(tags);
	}
}

PATHFINDCOREDLL_API int STDCALL OpenOneWays(void * pathfinder,int ways[], int arrSize)
{
	if (pathfinder!=NULL)
	{
		PathFinder *path = (PathFinder*)pathfinder;
		return path->OpenOneWays(ways,arrSize);
	}
	return 0;
}
PATHFINDCOREDLL_API int STDCALL CanDirectMove(void * pathfinder, Vector3 start, Vector3 end, TagType tags)
{
	if (pathfinder!=NULL)
	{
		PathFinder *path = (PathFinder*)pathfinder;
		return path->CanDirectMove(&start,&end,tags);
	}
	return 0;
}

PATHFINDCOREDLL_API int STDCALL LineCrossFarPoint(void * pathfinder, Vector3 start, Vector3 end, TagType tags, Vector3* ret)
{
	if (pathfinder!=NULL)
	{
		PathFinder *path = (PathFinder*)pathfinder;
		return path->LineCrossFarPoint(&start, &end, tags, ret);
	}
	return 0;
}
PATHFINDCOREDLL_API void STDCALL SetIgnoreDirect(void * pathfinder, int value)
{
	if (pathfinder!=NULL)
	{
		PathFinder *path = (PathFinder*)pathfinder;
		return path->SetIgnoreDirect(value  != 0);
	}
}
PATHFINDCOREDLL_API void STDCALL SetAreaSize(float value)
{
	PathFinder::AreaSize = value;
}

PATHFINDCOREDLL_API int STDCALL IsWalkableWithtag(void * pathfinder, Vector3 pos, TagType tags, int checkDirect, int* posTag)
{
	if (pathfinder!=NULL)
	{
		PathFinder *path = (PathFinder*)pathfinder;
		return path->IsWalkableWithtag(&pos, tags, checkDirect != 0, posTag);
	}
	
	return 0;
}
PATHFINDCOREDLL_API void STDCALL GetNavInfoLen(void * pathfinder, int mod, int* verticesLen, int* trianglesLen)
{
	if (pathfinder!=NULL)
	{
		PathFinder *path = (PathFinder*)pathfinder;
		return path->GetNavInfoLen(mod,verticesLen,trianglesLen);
	}
	*verticesLen = 0;
	*trianglesLen = 0;
}

PATHFINDCOREDLL_API void STDCALL GetNavInfo(void * pathfinder, int mod, Vector3 vertices[], int verticesLen, int triangles[], int trianglesLen, int tags[], int areas[])
{
	if (pathfinder!=NULL)
	{
		PathFinder *path = (PathFinder*)pathfinder;
		return path->GetNavInfo(mod, vertices, verticesLen, triangles, trianglesLen, tags, areas);
	}
}
PATHFINDCOREDLL_API void STDCALL GetNavSize(void * pathfinder, int mod, int* minX, int* minZ, int* maxX, int* maxZ)
{
	if (pathfinder!=NULL)
	{
		PathFinder *path = (PathFinder*)pathfinder;
		return path->GetNavSize(mod, minX, minZ, maxX, maxZ);
	}
}
extern "C" PATHFINDCOREDLL_API void STDCALL GetNodeInfo(void * pathfinder, int mod, int X, int Z, TagType tags, int* v0X, int* v0Z, int* v1X, int* v1Z, int* v2X, int* v2Z, int* adj1, int* adj2, int* adj3, int* index, int* tag, int* area)
{
	if (pathfinder!=NULL)
	{
		PathFinder *path = (PathFinder*)pathfinder;
		return path->GetNodeInfo(mod, X, Z, tags, v0X, v0Z, v1X, v1Z, v2X, v2Z, adj1, adj2, adj3, index, tag, area);
	}
}