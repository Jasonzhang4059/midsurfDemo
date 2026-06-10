#pragma once
#include "stdafx.h"

using namespace Lib3MF;
class My3MF
{
public:
	My3MF();
	~My3MF();
	void SetMeshObject(std::vector<float> vertices, std::vector<int> indices);
	void WriteToFile(std::string fileName);
protected:
	sLib3MFPosition fnCreateVertex(float x, float y, float z);
	sLib3MFTriangle fnCreateTriangle(int v0, int v1, int v2);
};

