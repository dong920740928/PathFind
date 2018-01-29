#include "PathFindLoader.h"
#include "PathLog.h"
#include <cmath>
#include <cstring>
MeshStruct :: ~MeshStruct()
{
	if(graphName != NULL)
	{
		delete graphName;
		graphName = NULL;
	}
	if (vertices !=NULL)
	{
		delete vertices;
	}
	if (triangles != NULL)
	{
		delete triangles;
	}
	if (flags != NULL)
	{
		delete flags;
	}
	if (connects != NULL)
	{
		delete connects;
	}
	if (directConnects != NULL)
	{
		delete directConnects;
	}
	if (areaConnectsDic != NULL)
	{
		for(std::unordered_map<int, AreaConnectList*>::iterator iter = areaConnectsDic->begin(); iter != areaConnectsDic->end(); ++iter)
		{
			delete iter->second;
		}
		delete areaConnectsDic;
	}
}

bool PathFindLoader::IsLittleEnd = true;
bool PathFindLoader::IsInit = false;
void PathFindLoader::Init()
{
	if(IsInit)
	{
		return;
	}
	//判断系统大小端
	unsigned short test = 0x1234;
	IsLittleEnd = *( (unsigned char*) &test ) == 0x34;
	IsInit = true;

}
int PathFindLoader::readInt(istream& reader)
{//文件必须是小端的
	int value;
	reader.read((char*)&value, sizeof(int));
	if(IsLittleEnd)
	{
		return value;
	}
	return ((value & 0x000000FF) << 24) |  
		((value & 0x0000FF00) << 8) |  
		((value & 0x00FF0000) >> 8) |  
		((value & 0xFF000000) >> 24) ;  
}
void PathFindLoader::readInts(istream& reader, int* arr, int count)
{
	reader.read((char*)arr,  sizeof(int) * count);
	if(!IsLittleEnd)
	{
		for (int i =0; i<count;i++)
		{
			int value = arr[i];
			arr[i] = ((value & 0x000000FF) << 24) |  
				((value & 0x0000FF00) << 8) |  
				((value & 0x00FF0000) >> 8) |  
				((value & 0xFF000000) >> 24) ;
		}
	}
}
vector<NavGraph*>* PathFindLoader::Load(const char* fileName, istream& fileStream)
{
	Init();
	vector<NavGraph*>* graphList = new vector<NavGraph*>();
	
	for(int i = 0; i < 2; i++)
	{
		MeshStruct* ms;
		ms = ImportMesh(fileStream);
		if (ms->verticesNum > 0 && ms->triangles != NULL)
		{
			NavGraph* graph = new NavGraph(ms, i+1);
			graphList->push_back(graph);
		}
		else
		{
			std::ostringstream stringStream;
			stringStream << " PathFindLoader.Load generateNodes nav data empty in file=" << fileName;
			PathLog::Error(stringStream.str().c_str());
		}
		delete ms;
	}
	
	return graphList;
}

MeshStruct* PathFindLoader::ImportMesh(istream& reader)
{
	MeshStruct* ms = new MeshStruct();
	std::memset(ms,0,sizeof(MeshStruct));

	char len;
	char currentText[100];
	while(!reader.eof())
	{
		//reader >> len;
		reader.read(&len,1);
		reader.read(currentText,len);
		currentText[len] = 0;
		if (std::strcmp(currentText,"g") == 0)
		{
			char nameLen;
			reader.read(&nameLen,1);
			ms->nameLen = nameLen;
			ms->graphName = new char[ms->nameLen + 1];
			reader.read(ms->graphName,nameLen);
			ms->graphName[nameLen] = 0;
		}
		else if (strcmp(currentText,"vertice") == 0)
		{
			int verticeNum = readInt(reader);
			if (verticeNum <= 0)
			{
				continue;
			}
			ms->vertices = (Int3*)new char[sizeof(Int3) * verticeNum];
			reader.read((char*)ms->vertices, sizeof(Int3)*verticeNum);
			ms->verticesNum = verticeNum;
			int factor = Int3::FloatPrecisionInt / 10;
			for (int i = 0; i < verticeNum; i++)
			{
				//读取顶点信息
				Int3* v = ms->vertices + i;
				if (!IsLittleEnd)
				{
					v->X = SwapInt(v->X) * factor;
					v->Z = SwapInt(v->Z) * factor;
				}
				else
				{
					v->X *= factor;
					v->Z *= factor;
				}
			}
			/*std::ostringstream stringStream;
			stringStream << "verticles: "<< verticeNum;
			PathLog::Debug(stringStream.str().c_str());*/
		}
		else if (strcmp(currentText,"triangle") == 0)
		{
			int triangleNum = readInt(reader);
			if (triangleNum <= 0)
			{
				continue;
			}
			ms->triangles = new int[triangleNum * 3];
			ms->trianglesNum = triangleNum;
			ms->flags = new int[triangleNum];
			ms->connects = new int[triangleNum * 3];
			ms->directConnects = new int[triangleNum * 3];//这个数据可能不会用到，是否的时候再生成 todo
			std::memset(ms->directConnects, 0 , sizeof(int) * triangleNum * 3);
			int *readData = new int[sizeof(int) * 7 * triangleNum];
			reader.read((char*)readData, sizeof(int) * 7 * triangleNum);
			int startIndex = 0;
			int readIndex = 0;
			for (int i = 0; i < triangleNum; i++)
			{
				//读取三角形的顶点
				for (int j = 0; j < 3; j++)
				{
					ms->triangles[startIndex + j] = toNativeInt(readData[readIndex++]) - 1;
				}
				//读取三角形的flag
				ms->flags[i] = toNativeInt(readData[readIndex++]); 

				//读取连接
				for (int j = 0; j < 3; j++)
				{
					ms->connects[startIndex + j] = toNativeInt(readData[readIndex++]);
				}
				startIndex += 3;
			}
			delete readData;
			/*std::ostringstream stringStream;
			stringStream << "triangles: "<< triangleNum;
			PathLog::Debug(stringStream.str().c_str());*/
		}
		else if (strcmp(currentText,"edc") == 0)
		{
			int edcNum = readInt(reader);
			int* readData = new int[edcNum * 4];
			readInts(reader, readData, edcNum * 4);
			int readIndex = 0;
			for (int i = 0; i < edcNum; i++)
			{
				int nodeIndex = readData[readIndex++];
				ms->directConnects[nodeIndex*3] = readData[readIndex++];
				ms->directConnects[nodeIndex * 3 + 1] = readData[readIndex++];
				ms->directConnects[nodeIndex * 3 + 2] = readData[readIndex++];
			}
			delete readData;
			/*std::ostringstream stringStream;
			stringStream << "edc: "<< edcNum;
			PathLog::Debug(stringStream.str().c_str());*/
		}
		else if (strcmp(currentText,"areaConnect") == 0)
		{
			int areaNum = readInt(reader);
			ms->areaConnectsDic = new std::unordered_map<int, AreaConnectList*>();
			for (int i = 0; i < areaNum; i++)
			{
				int area =readInt(reader);
				int areaConnectNum = readInt(reader);
				AreaConnectList* list = new AreaConnectList(areaConnectNum);
				readInts(reader, list->m_areas, areaConnectNum);
				(*ms->areaConnectsDic)[area] = list;
			}
			/*std::ostringstream stringStream;
			stringStream << "areaConnect: "<< areaNum;
			PathLog::Debug(stringStream.str().c_str());*/
		}
		else if (strcmp(currentText,"areaTags") == 0)
		{
			int areaTagsNum = readInt(reader);
			ms->areaTagDic = new int[AREANUMBER];
			for (int i = 0; i < areaTagsNum; i++)
			{

				int tags = readInt(reader);
				int areaNum = readInt(reader);
				for (int j = 0; j < areaNum; j++)
				{
					int area = readInt(reader);
					ms->areaTagDic[area] = tags;
				}
			}
			/*std::ostringstream stringStream;
			stringStream << "areaTags: "<< areaTagsNum;
			PathLog::Debug(stringStream.str().c_str());*/
		}
		else if (strcmp(currentText,"graphEnd") == 0)
		{
			/*std::ostringstream stringStream;
			stringStream << "graphEnd: ";
			PathLog::Debug(stringStream.str().c_str());*/
			return ms;
		}
		else
		{
			break;
		}
	}

	delete ms;
	return NULL;
}