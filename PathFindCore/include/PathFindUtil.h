#pragma once
#include <cmath>
#include <string>
#include <sstream>
#include "BaseType.h"
#include <vector>
#include <string>
#include <cstring>
//寻路系统计算的最基本单元，以后可以去掉Y分量
class Int3
{
public:
	int X;
	//int Y;
	int Z;
public:
	Int3()
	{
		X = 0;
		Z = 0;
	}
	Int3(int x, int z);
	Int3(float x, float z);
	Int3(const Vector3& p);
public://
	static const double FloatPrecision;
	static const int FloatPrecisionInt;
	static const double PrecisionFactor;
	static const double PrecisionFactorSquare;
	static const Int3 Zero;

public:
	bool operator==(const Int3& another)const;
	bool operator!=(const Int3& another)const;
	Int3 operator-(const Int3& another)const;
	Int3 operator +(const Int3& another)const;
	Int3 operator *(int factor)const;
	Int3 operator *(double factor)const;
	Int3 operator /(double factor)const;

	
	int XZMagnitude()const;
	double XZMagnitudeD()const;
	long long XZSquareMagnitude()const;
	std::string ToString()const;
	static int ToInt(float v);
	static float ToFloat(int v);
	static long long XZDot(const Int3& lhs, const Int3& rhs);
	static long long XZCross(const Int3& lhs, const Int3& rhs);
	static int XZDistance(const Int3& lhs, const Int3& rhs);
	static long long XZSquareDistance(const Int3& lhs, const Int3& rhs);
	bool XZEquals(const Int3& rhs)const;
};
inline bool Int3::operator==(const Int3& another)const
{
	return X == another.X &&
		Z == another.Z;
}
inline bool Int3::operator!=(const Int3& another)const
{
	return X != another.X ||
		Z != another.Z;
}
inline Int3 Int3::operator-(const Int3& another)const
{
	return Int3(X - another.X, Z - another.Z);
}
inline Int3 Int3::operator +(const Int3& another)const
{
	return Int3(X + another.X, Z + another.Z);
}
inline Int3 Int3::operator *(int factor)const
{
	return Int3(X * factor, Z * factor);
}

inline Int3 Int3::operator *(double factor)const
{
	return Int3((int)round((double)X * factor), (int)round((double)Z * factor));
}
inline Int3 Int3::operator /(double factor)const
{
	return Int3((int)round((double)X / factor), (int)round((double)Z / factor));
}


inline int Int3::XZMagnitude()const
{
	return (int)sqrt((double)X * X + (double)Z * Z);
}
inline double Int3::XZMagnitudeD()const
{
	return sqrt((double)X * X + (double)Z * Z);
}
inline long long Int3::XZSquareMagnitude()const
{
	return (long long)X * X  + (long long)Z * Z;
}
inline std::string Int3::ToString()const
{
	std::ostringstream stringStream;
	stringStream << "(" << X << ", " << Z << ")";
	return stringStream.str();
}

inline int Int3::ToInt(float v)
{
	return (int)round(v * FloatPrecision);
}
inline float Int3::ToFloat(int v)
{
	return (float)(v * PrecisionFactor);
}

inline long long Int3::XZDot(const Int3& lhs, const Int3& rhs)
{
	return (long long)lhs.X * rhs.X + (long long)lhs.Z * rhs.Z;
}
inline long long Int3::XZCross(const Int3& lhs, const Int3& rhs)
{
	return (long long)lhs.Z * rhs.X - (long long)lhs.X * rhs.Z;
}
inline int Int3::XZDistance(const Int3& lhs, const Int3& rhs)
{
	double dx = (double)(lhs.X - rhs.X);
	double dz = (double)(lhs.Z - rhs.Z);
	return (int)sqrt(dx * dx + dz * dz);
}
inline long long Int3::XZSquareDistance(const Int3& lhs, const Int3& rhs)
{
	long long dx = (long long)(lhs.X - rhs.X);
	long long dz = (long long)(lhs.Z - rhs.Z);
	return dx * dx + dz * dz;
}
inline bool Int3::XZEquals(const Int3& rhs)const
{
	return X == rhs.X && Z == rhs.Z;
}

class AABB
{
public:
	int MinX;
	int MinZ;
	int MaxX;
	int MaxZ;

	AABB(const Int3& from, const Int3& to);
	AABB(const Int3& v1, const Int3& v2, const Int3& v3, int extends = 0);
	AABB(const Int3& v1, int dis, int extends = 0);
	bool Contains2D(const Int3& point)const;
	bool Intersect2D(const AABB& other)const;
};

inline bool AABB::Contains2D(const Int3& point)const
{
	return
		(point.X >= MinX) && (point.X <= MaxX) &&
		(point.Z >= MinZ) && (point.Z <= MaxZ);
}
inline bool AABB::Intersect2D(const AABB& other)const
{
	return MaxX >= other.MinX && other.MaxX >= MinX &&
		MaxZ >= other.MinZ && other.MaxZ >= MinZ;
}
class Polygon
{
public:

	static long long TriangleArea(const Int3& a, const Int3& b, const Int3& c);
	static long long TriangleArea2(const Int3& a, const Int3& b, const Int3& c);

	/** Returns if \a p lies on the left side of the line \a a - \a b. Uses XZ space. Also returns true if the points are colinear */
	static bool Left(const Int3& a, const Int3& b, const Int3& c);

	/** Returns if the points a in a clockwise order */
	static bool IsClockwise(const Int3& a, const Int3& b, const Int3& c);

	static bool IsColinear(const Int3& a, const Int3& b, const Int3& c);
	static bool CheckPointInTriangle2D(const Int3& A, const Int3& B, const Int3& C, const Int3& P, int error = 0);

	static int PointToLineDis(const Int3& start, const Int3& end, const Int3& point);
};
inline long long Polygon::TriangleArea(const Int3& a, const Int3& b, const Int3& c)
{
	return ((long long)(b.X - a.X) * (c.Z - a.Z) - (long long)(c.X - a.X) * (b.Z - a.Z)) / 2;
}
inline long long Polygon::TriangleArea2(const Int3& a, const Int3& b, const Int3& c)
{
	return (long long)(b.X - a.X) * (c.Z - a.Z) - (long long)(c.X - a.X) * (b.Z - a.Z);
}
inline bool Polygon::Left(const Int3& a, const Int3& b, const Int3& c)
{
	return TriangleArea2(a, b, c) <= 0;
}
inline bool Polygon::IsClockwise(const Int3& a, const Int3& b, const Int3& c)
{
	return TriangleArea2(a, b, c) < 0;
}
inline bool Polygon::IsColinear(const Int3& a, const Int3& b, const Int3& c)
{
	return TriangleArea2(a, b, c) == 0;
}

inline int Polygon::PointToLineDis(const Int3& start, const Int3& end, const Int3& point)
{
	return (int)(std::abs(Polygon::TriangleArea2(start, end, point)) / (start - end).XZMagnitude());
}
inline bool Polygon::CheckPointInTriangle2D(const Int3& A, const Int3& B, const Int3& C, const Int3& P, int error)
{
	long long t1 = TriangleArea2(A, P, B);
	long long t2 = TriangleArea2(A, C, P);
	long long t3 = TriangleArea2(B, P, C);

	if (error > 0)//相当于把边加粗到error*sqrt(2)
	{
		long long d1 = (A - B).XZSquareMagnitude();
		long long d2 = (A - C).XZSquareMagnitude();
		long long d3 = (B - C).XZSquareMagnitude();
		t1 = t1 * t1 * 2 <= error * d1 ? 0 : t1;// |t1| < sqrt(error/2)*sqrt(d1)
		t2 = t2 * t2 * 2 <= error * d2 ? 0 : t2;
		t3 = t3 * t3 * 2 <= error * d3 ? 0 : t3;
	}
	return (t1 >= 0 && t2 >= 0 && t3 >= 0) || (t1 <= 0 && t2 <= 0 && t3 <= 0);
}

class Int3Line
{
public:
	Int3Line(const Int3& begin, const Int3& end);
public:
	const Int3 BeginPoint;
	const Int3 EndPoint;

};
class Int3LineList
{
public:
	void Clear();
	void AddLine(Int3& v1, Int3& v2);
	~Int3LineList();
private:
	void Add(Int3& b, Int3& e);
public:
	std::vector<Int3Line*> List;

};

class BresenhamLineType
{
public:
	BresenhamLineType(int count);
	~BresenhamLineType();
	void push_back(int x, int y);
	void clear()
	{
		m_last = 0;
	}
	int size()const
	{
		return m_last;
	};
private:
public:
	int* m_data;
private:
	int m_last;
	int m_datalen;
};

#define NODELISTINITSIZE 16
class Node;
class NodeListType
{
public:
	NodeListType()
	{
		m_data = new Node*[NODELISTINITSIZE];
		m_datalen = NODELISTINITSIZE;
		m_last = 0;
	}
	~NodeListType()
	{
		if (m_data != NULL)
		{
			delete m_data;
		}
	}
	void push_back(Node* node);
public:
	Node** m_data;
	int m_datalen;
	int m_last;
};
class AreaConnectList
{
public:
	AreaConnectList(int len)
		:m_len(len),
		m_areas(NULL)
	{
		if (m_len > 0)
		{
			m_areas = new int[len];
		}

	}
	~AreaConnectList()
	{
		if (m_areas != NULL)
		{
			delete m_areas;
		}
	}
public:
	int m_len;
	int* m_areas;
};

class Int3Vector 
{
public:
	Int3Vector(int count = 4);
	~Int3Vector();
	void push_back(Int3& v);
	void clear();
	void eraseNear(int dist);
	void eraseRang(int begin, int last);
	void eraseLastDis(int dis);
	void reverse();
	void push_back(Int3Vector& other, int from ,int to);
	void swap(Int3Vector& other);
	Int3& back()const
	{
		return m_data[m_last - 1];
	}
	Int3& front()const
	{
		return m_data[0];
	}
	int size()
	{
		return m_last;
	}
	Int3& operator[](int nIndex)const
	{
		return m_data[nIndex];
	}
private:

public:
	Int3* m_data;
	int m_datalen;
	int m_last;

};
class NodeVector 
{
public:
	NodeVector(int count = 4);
	~NodeVector();
	void push_back(Node* v);
	void clear();

	int size()
	{
		return m_last;
	}
	Node* operator[](int nIndex)const
	{
		return m_data[nIndex];
	}
private:

public:
	Node** m_data;
	int m_datalen;
	int m_last;

};