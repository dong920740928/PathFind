#include "PathFindUtil.h"
#include <cmath>
#include <iostream>

const double Int3::FloatPrecision = 1000.0;
const int Int3::FloatPrecisionInt = 1000;
const double Int3::PrecisionFactor = 0.001;
const double Int3::PrecisionFactorSquare = 0.000001; 

Int3::Int3(int x, int z):X(x),Z(z)
{
}
Int3::Int3(float x, float z):
X((int)round(x * Int3::FloatPrecision)),
	Z((int)round(z * Int3::FloatPrecision))
{

}
Int3::Int3(const Vector3& p):
X((int)round(p.X * Int3::FloatPrecision)),
	Z((int)round(p.Z * Int3::FloatPrecision))
{

}
const Int3 Int3::Zero = Int3(0,0);

AABB::AABB(const Int3& from, const Int3& to)
{
	MinX = from.X <= to.X ? from.X : to.X;
	MinZ = from.Z <= to.Z ? from.Z : to.Z;
	MaxX = from.X >= to.X ? from.X : to.X;
	MaxZ = from.Z >= to.Z ? from.Z : to.Z;
}
AABB::AABB(const Int3& v1, const Int3& v2, const Int3& v3, int extends)
{
	MinX = v1.X <= v2.X ? v1.X : v2.X;
	MinX = (MinX <= v3.X ? MinX : v3.X) - extends;

	MinZ = v1.Z <= v2.Z ? v1.Z : v2.Z;
	MinZ = (MinZ <= v3.Z ? MinZ : v3.Z) - extends;

	MaxX = v1.X >= v2.X ? v1.X : v2.X;
	MaxX = (MaxX >= v3.X ? MaxX : v3.X) + extends;


	MaxZ = v1.Z >= v2.Z ? v1.Z : v2.Z;
	MaxZ = (MaxZ >= v3.Z ? MaxZ : v3.Z) + extends;
}
AABB::AABB(const Int3& v1, int dis, int extends)
{
	dis = std::abs(dis) + std::abs(extends);
	MinX = v1.X - dis;
	MinZ = v1.Z - dis;
	MaxX = v1.X + dis;
	MaxZ = v1.Z + dis;
}
Int3Line::Int3Line(const Int3& b, const Int3& e)
	:BeginPoint(b),
	EndPoint(e)
{

}

using namespace std;
void Int3LineList::Clear()
{
	for(vector<Int3Line*>::iterator iter = List.begin();iter!=List.end();++iter)
	{
		Int3Line* line = *iter;
		delete line;
	}
	List.clear();
}
void Int3LineList::Add(Int3& b, Int3& e)
{
	List.push_back(new Int3Line(b,e));
}
void Int3LineList::AddLine(Int3& v1, Int3& v2)
{
	for(vector<Int3Line*>::iterator iter = List.begin();iter!=List.begin();++iter)
	{
		Int3Line* line = *iter;
		if ((line->BeginPoint.XZEquals(v1) && line->EndPoint.XZEquals(v2)) ||
			(line->BeginPoint.XZEquals(v2) && line->EndPoint.XZEquals(v1)))
		{
			return;
		}
	}
	Add(v1,v2);
}
Int3LineList::~Int3LineList()
{
	Clear();
}

BresenhamLineType::BresenhamLineType(int count)
{

	if (count > 0)
	{
		m_datalen = count * 2;
	}
	else
	{
		m_datalen = 64;
	}
	m_data = new int[m_datalen];
	m_last = 0;
}
BresenhamLineType::~BresenhamLineType()
{
	if (m_data != NULL)
	{
		delete m_data;
	}
}
void BresenhamLineType::push_back(int x, int y)
{
	if (m_last >= m_datalen)
	{
		int* newData = new int[m_datalen * 2];
		memcpy(newData,m_data,sizeof(int)*m_datalen);
		delete m_data;
		m_data = newData;
		m_datalen *= 2;
	}
	m_data[m_last++] = x;
	m_data[m_last++] = y;
}

void NodeListType::push_back(Node* node)
{
	if (m_last >=  m_datalen)
	{
		Node** newData = new Node*[m_datalen*2];
		std::memcpy(newData,m_data,sizeof(Node*) * m_datalen);
		m_datalen *= 2;
		delete m_data;
		m_data = newData;
	} 
	m_data[m_last++] = node;
}

Int3Vector::Int3Vector(int count)
{
	if (count < 2)
	{
		count = 2;
	}
	m_datalen = count;
	m_last = 0;
	m_data = (Int3*)new char[sizeof(Int3)*count];

}
Int3Vector::~Int3Vector()
{
	delete (char*)m_data;
}
void Int3Vector::push_back(Int3& v)
{
	if (m_last >= m_datalen)
	{
		void* newData = new char[sizeof(Int3)* m_datalen * 2];
		memcpy(newData,m_data,sizeof(Int3)*m_datalen);
		delete  (char*)m_data;
		m_data = (Int3*)newData;
		m_datalen *= 2;
	}
	m_data[m_last++] = v;
}
void Int3Vector::clear()
{
	m_last = 0;
}
void Int3Vector::eraseNear(int dist)
{
	int eraseNum  = 0;
	for(int i = 0, j = 1; j < m_last; j++)
	{
		if (Int3::XZDistance(m_data[i], m_data[j]) > 10)
		{
			i++;
			if (i != j)
			{
				m_data[i] = m_data[j];
			}
		}
		else
		{
			eraseNum++;
		}
	}
	m_last -= eraseNum;
}
void Int3Vector::eraseLastDis(int dis)
{
	int eraseNum  = 0;
	
	for(int i = m_last - 1; i>0; i--)
	{
		
		if(dis<=0)
		{
			break;
		}
		int cudis = Int3::XZDistance(m_data[i], m_data[i-1]);
		if(cudis <= dis)
		{
			dis -= cudis;
			eraseNum++;
			continue;
		}
		m_data[i] = m_data[i] + (m_data[i - 1] - m_data[i])*((double)dis / (double)cudis);
		break;
	}
	m_last -= eraseNum;
}

void Int3Vector::eraseRang(int begin, int last)
{
	if(begin >= last)
	{
		return;
	}
	if(last < m_last)
	{
		memmove(m_data + begin, m_data + last, sizeof(Int3)* (m_last - last));
	}
	m_last -= (last - begin);
}
void Int3Vector::reverse()
{
	Int3 temp;
	for (int first = 0, last = m_last; first != last && first != --last; ++first)
	{
		temp = m_data[first];
		m_data[first] = m_data[last];
		m_data[last] = temp;
	}
}
void Int3Vector::push_back(Int3Vector& other, int from ,int to)
{
	int copynumber = to - from;
	if(copynumber <= 0)
	{
		return;
	}
	if(m_last + copynumber > m_datalen)
	{
		int targetLen  = (m_last + copynumber) * 2;
		void* newData = new char[sizeof(Int3)* targetLen];
		memcpy(newData,m_data,sizeof(Int3)*m_datalen);
		delete  (char*)m_data;
		m_data = (Int3*)newData;
		m_datalen  = targetLen;
	}

	memcpy(m_data + m_last , other.m_data + from, copynumber * sizeof(Int3));
	m_last += copynumber;
}

void Int3Vector::swap(Int3Vector& other)
{
	Int3* temp = m_data;
	m_data = other.m_data;
	other.m_data = temp;

	int inttemp = m_datalen;
	m_datalen = other.m_datalen;
	other.m_datalen = inttemp;

	inttemp = m_last;
	m_last = other.m_last;
	other.m_last = inttemp;
}

NodeVector::NodeVector(int count)
{
	if (count < 2)
	{
		count = 2;
	}
	m_datalen = count;
	m_last = 0;
	m_data = (Node**)new char[sizeof(Node*)*count];

}
NodeVector::~NodeVector()
{
	delete (char*)m_data;
}
void NodeVector::push_back(Node* v)
{
	if (m_last >= m_datalen)
	{
		void* newData = new char[sizeof(Node*)* m_datalen * 2];
		memcpy(newData,m_data,sizeof(Node*)*m_datalen);
		delete  (char*)m_data;
		m_data = (Node**)newData;
		m_datalen *= 2;
	}
	m_data[m_last++] = v;
}
void NodeVector::clear()
{
	m_last = 0;
}