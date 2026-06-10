#pragma once

#include"occShapeManager.h"
#include<vector>
#include<set>
#include <limits>
#include <iostream>

#define OCC_SHAPE_MANAGER (occShapeManager::Instance())

typedef std::vector<TopoDS_Shape> TopoShapeArr;
typedef std::vector<TopoDS_Face> TopoFaceArr;
typedef std::vector<gp_Pnt> GpPntArr;
typedef std::vector<UINT> FaceLabArr;
typedef std::set<UINT> FaceLabSet;

struct MidPointKey
{
	gp_Pnt PA;
	gp_Pnt PB;
};
