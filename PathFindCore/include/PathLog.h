#pragma once
#include "BaseType.h"
class PathLog
{
public:
	static void Debug(const char* msg);
	static void Warn(const char* msg);
	static void Error(const char* msg);
public:
	static LogCallback DebugCb;
	static LogCallback WarnCb;
	static LogCallback ErrorCb;
};