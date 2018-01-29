#include <iostream>
#include "PathFindCore.h"

void STDCALL loooog(const char* msg)
{
	std::cout<<msg;
}
int main(int argc, char* argv[])
{
	////using namespace std;
	///*float s =ceil(4.55);
	//float d = ceil(-4.55);*/
	///*cout<<" " << Int3::FloatPrecision<<" "<<Int3::PrecisionFactorSquare<<endl; 
	//cout<< Int3::Zero.X;*/
	//char* fileName = "Z:\\trunk\\Server\\UXGameServer\\ServerRes\\Scene_Activity_006.nav";
	//PathFinder* pathFinder = new PathFinder(fileName);
	////PathFindLoader::Load(fileName);
	///*Int3* x = new Int3(0.945f , -0.4552f, 3.8847f);
	//Int3 X(845,-455,3886);
	//Int3 xxx = *x+X;
	//bool e = X == (*x);
	//Int3 fff = Int3(xxx);
	//std::cout<<fff.ToString();*/
	//delete pathFinder;
	
	/*char* fileName = "Z:\\trunk\\Server\\UXGameServer\\ServerRes\\Scene_Activity_006.nav";
	if (argc>0)
	{
	fileName = argv[0];
	}
	void* p = CreatePathFinder(fileName);
	FreePathFinder(p,true);*/
	SetLogCallback(loooog, loooog, loooog);
	return 0;
}
