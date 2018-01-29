#include "PathLog.h"
#include <cstddef>
void PathLog::Debug(const char* msg)
{
	if (DebugCb != NULL)
	{
		DebugCb(msg);
	}
}
void PathLog::Warn(const char* msg)
{
	if (WarnCb != NULL)
	{
		WarnCb(msg);
	}
}
void PathLog::Error(const char* msg)
{
	if (ErrorCb != NULL)
	{
		ErrorCb(msg);
	}
}

LogCallback PathLog::DebugCb = NULL;
LogCallback PathLog::WarnCb = NULL;
LogCallback PathLog::ErrorCb = NULL;