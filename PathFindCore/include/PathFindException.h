#pragma once
#include <exception>
#include <string>
class PathFindException: public std::exception
{
public:
	std::string Msg;
	PathFindException(std::string msg)
	{
		Msg  = msg;
	}
	virtual const char* what() const throw()
	{
		return Msg.c_str();;
	}
};