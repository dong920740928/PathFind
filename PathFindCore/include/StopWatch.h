#pragma once
#include <time.h>
#ifdef _DEBUG
#include "Windows.h"
#endif // _DEBUG
class StopWatch
{

public:
	StopWatch(const char* name, bool begin = true);
	~StopWatch(void);
	void start();
	void stop();
private:
	
#ifdef _DEBUG
	const char* name;
	LARGE_INTEGER m_liPerfStart;
	bool isStart;
	long long total;
#endif// _DEBUG
};