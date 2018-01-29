#pragma once
#include "BaseType.h"

extern "C" PATHFINDCOREDLL_API int STDCALL Add(int a, int b);

extern "C" PATHFINDCOREDLL_API void STDCALL FreeMemory(void* mem); 

extern "C" PATHFINDCOREDLL_API void STDCALL SetLogCallback(LogCallback debug, LogCallback warn, LogCallback error);

extern "C" PATHFINDCOREDLL_API void STDCALL SetGetStackCallback(GetStackCallback cb);

extern "C" PATHFINDCOREDLL_API void STDCALL LoadNavs(const char** navPathList, long num);

extern "C" PATHFINDCOREDLL_API void STDCALL ClearCache();

extern "C" PATHFINDCOREDLL_API int STDCALL GetCacheNum();

extern "C" PATHFINDCOREDLL_API void* STDCALL CreatePathFinder(const char* filePath);

extern "C" PATHFINDCOREDLL_API void* STDCALL CreatePathFinderWithBytes(const char* filePath, const char* bytes, int bytesCount);

extern "C" PATHFINDCOREDLL_API void* STDCALL ClonePathFinder(void* anthor);

extern "C" PATHFINDCOREDLL_API void STDCALL FreePathFinder(void *p, int freeRes);

extern "C" PATHFINDCOREDLL_API void STDCALL GetVector3Array(Vector3 v[], int count, void* path);

extern "C" PATHFINDCOREDLL_API int STDCALL GetAreaIfWalkable(void * pathfinder, Vector3 pos, TagType tag, int* area);

extern "C" PATHFINDCOREDLL_API void* STDCALL FindPath(void * pathfinder, Vector3 start, Vector3 end, TagType tags, TagType* pathTags, int* count, int useNavWeight);

extern "C" PATHFINDCOREDLL_API void* STDCALL FindPathNear(void * pathfinder, Vector3 start, Vector3 end, float nearDis, TagType tags, TagType* pathTags, int* count, int useNavWeight);

extern "C" PATHFINDCOREDLL_API void* STDCALL FindPathTolerant(void * pathfinder, Vector3 start, Vector3 end, float range, TagType tags, TagType* pathTags, int* count, int useNavWeight);

extern "C" PATHFINDCOREDLL_API void* STDCALL GetMoveDisPath(void * pathfinder, Vector3 start, float angleY, float distance, TagType tags, int* count);

extern "C" PATHFINDCOREDLL_API int STDCALL CanMove(void * pathfinder, Vector3 start, Vector3 end, TagType tags);

extern "C" PATHFINDCOREDLL_API int STDCALL CheckXzPointInTriangle(void * pathfinder, float x, float z, TagType tags, float* y);

extern "C" PATHFINDCOREDLL_API int STDCALL IsWalkable(void * pathfinder, float x, float z, TagType tags);

extern "C" PATHFINDCOREDLL_API void STDCALL OpenOneWayConnect(void * pathfinder,TagType tags);

extern "C" PATHFINDCOREDLL_API void STDCALL CloseOneWayConnect(void * pathfinder,TagType tags);

extern "C" PATHFINDCOREDLL_API int STDCALL OpenOneWays(void * pathfinder,int ways[], int arrSize);

extern "C" PATHFINDCOREDLL_API int STDCALL CanDirectMove(void * pathfinder, Vector3 start, Vector3 end, TagType tags);

extern "C" PATHFINDCOREDLL_API int STDCALL LineCrossFarPoint(void * pathfinder, Vector3 start, Vector3 end, TagType tags, Vector3* ret);

extern "C" PATHFINDCOREDLL_API void STDCALL SetIgnoreDirect(void * pathfinder, int value);

extern "C" PATHFINDCOREDLL_API void STDCALL SetAreaSize(float value);

extern "C" PATHFINDCOREDLL_API int STDCALL IsWalkableWithtag(void * pathfinder, Vector3 pos, TagType tags, int checkDirect, int* posTag);

extern "C" PATHFINDCOREDLL_API void STDCALL GetNavInfoLen(void * pathfinder, int mod, int* verticesLen, int* trianglesLen);

extern "C" PATHFINDCOREDLL_API void STDCALL GetNavInfo(void * pathfinder, int mod, Vector3 vertices[], int verticesLen, int triangles[], int trianglesLen, int tags[], int areas[]);

extern "C" PATHFINDCOREDLL_API void STDCALL GetNavSize(void * pathfinder, int mod, int* minX, int* minZ, int* maxX, int* maxZ);

extern "C" PATHFINDCOREDLL_API void STDCALL GetNodeInfo(void * pathfinder, int mod, int X, int Z, TagType tags, int* v0X, int* v0Z, int* v1X, int* v1Z, int* v2X, int* v2Z, int* adj1, int* adj2, int* adj3, int* index, int* tag, int* area);