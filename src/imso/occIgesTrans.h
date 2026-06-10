#pragma once
#include<string>
#include<RWGeometric.h>
using namespace std;
class OccIgesTrans {
public:
	OccIgesTrans();
	void outputIges(const string &path,const string &outputPath);
	void outputIges(varray<SplineVolume>& svs,const string &outputPath);

};
