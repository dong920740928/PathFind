#include "StopWatch.h"
#include <iostream>

StopWatch::StopWatch(const char* name, bool begin)
#ifdef _DEBUG
:name(name)
#endif // _DEBUG
{
#ifdef _DEBUG
	isStart = false;
	total = 0;
	if(begin)
	{
		start();
	}
#endif // _DEBUG
}
StopWatch::~StopWatch(void)
{
#ifdef _DEBUG
	stop();
	LARGE_INTEGER m_liPerfFreq;
	QueryPerformanceFrequency(&m_liPerfFreq); 
	std::cout<<name<<"  toatal: "<< (total*1000000/m_liPerfFreq.QuadPart) <<"us"<<std::endl;
#endif // _DEBUG
}
void StopWatch::start()
{
#ifdef _DEBUG
	if(isStart)
	{
		return;
	}
	isStart = true;
	QueryPerformanceCounter(&m_liPerfStart);
#endif // _DEBUG
}
void StopWatch::stop()
{
#ifdef _DEBUG
	if(!isStart)
	{
		return;
	}
	isStart = false;
	LARGE_INTEGER liPerfNow;
	QueryPerformanceCounter(&liPerfNow);
	total += liPerfNow.QuadPart - m_liPerfStart.QuadPart;
#endif // _DEBUG
}