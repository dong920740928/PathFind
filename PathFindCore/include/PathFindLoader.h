#pragma once
#include <vector>
#include <unordered_map>
#include <queue>
#include <string>
#include "PathFindUtil.h"
#include "NavGraph.h"
#include <iostream>
class MeshStruct
{
public:
	char* graphName;
	int nameLen;
	Int3* vertices; //顶点
	int verticesNum;
	int* triangles; //三角形
	int trianglesNum;
	int* flags; //地形标记
	int* connects; //连接信息
	int* directConnects;//有向连接数据，可以不用数组，而是用map等等
	std::unordered_map<int, AreaConnectList*>* areaConnectsDic;
	int* areaTagDic;
	
	~MeshStruct();
};
enum NavTag
{
	Land = 0,
	OnlyOutLand = 6,
	BossLand = 7,
	Trigger1 = 8,
	Trigger2 = 9,
	Trigger3 = 10,
	Trigger4 = 11,
	Trigger5 = 12,
	Trigger6 = 13,
	Trigger7 = 14,
	Trigger8 = 15,
	Trigger9 = 16,
	Trigger10 = 17,
	Trigger11 = 18,
	Trigger12 = 19,
	Trigger13 = 20,
	Trigger14 = 21,
	Trigger15 = 22,
	Trigger16 = 23,
	Trigger17 = 24,
	Trigger18 = 25,
	Trigger19 = 26,
	Trigger20 = 27,
	Trigger21 = 28,
	Trigger22 = 29,
	Trigger23 = 30,
	Trigger24 = 31,
};

class PathFindLoader
{
public:
	static bool IsLittleEnd;
	static bool IsInit;
	static void Init();
	static int readInt(istream& reader);
	static void readInts(istream& reader, int* arr, int count);
	static int SwapInt(int v);
	static int toNativeInt(int v);
	static vector<NavGraph*>* Load(const char* fileName, istream& fileStream);
	static MeshStruct* ImportMesh(istream& reader);
};
inline int PathFindLoader::SwapInt(int value)
{
	return ((value & 0x000000FF) << 24) |  
		((value & 0x0000FF00) << 8) |  
		((value & 0x00FF0000) >> 8) |  
		((value & 0xFF000000) >> 24) ;  
}
inline int PathFindLoader::toNativeInt(int value)
{
	if(IsLittleEnd)
	{
		return value;
	}
	return ((value & 0x000000FF) << 24) |  
		((value & 0x0000FF00) << 8) |  
		((value & 0x00FF0000) >> 8) |  
		((value & 0xFF000000) >> 24) ;  
}
