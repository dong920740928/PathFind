#pragma once
#include <limits>
#include <cmath>
#ifdef  _MSC_VER
#define PATHFINDCOREDLL_API __declspec(dllexport)
#define STDCALL __stdcall
inline double round(double v)
{
	return floor(v+0.5);
}
#else
#define PATHFINDCOREDLL_API
#define STDCALL
#endif
typedef void  (STDCALL * LogCallback)(const char* msg);
typedef const char* (STDCALL * GetStackCallback)();

struct Vector3
{
	float X;
	float Y;
	float Z;
};

typedef unsigned int TagType;

const double PI = 3.14159265;
#define VECTORDIMENSION 3

const int PathfindMaxInt =  std::numeric_limits<int>::max();
const int PathfindMinInt =  std::numeric_limits<int>::min();
const double PathfindMaxDouble = std::numeric_limits<double>::max();
#define AREANUMBER 256
#define TAGNUMBER 32