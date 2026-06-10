
// stdafx.h : 标准系统包含文件的包含文件，
// 或是经常使用但不常更改的
// 特定于项目的包含文件

#pragma once

#ifndef VC_EXTRALEAN
#define VC_EXTRALEAN            // 从 Windows 头中排除极少使用的资料
#endif


#include "lib3mf_implicit.hpp"
#include <thread>
#include <future>
#include "varray.h"
#include "XVec.h"
#include "pointXd.h"
#include <Windows.h>
#include <iostream>
#include <vector>
using namespace base;

struct threadParamSpline {
	varray<varray<Vec3>> l2;
	varray<varray<Vec3>> p3d2;
};



struct threadParamSplineVOL {
	varray<varray<varray<Vec3>>> q3;
	varray<varray<varray<Vec3>>> l3;
};
struct threadParam {
	varray<varray<point3d>> l2;
	varray<varray<point3d>> p3d2;
};

struct threadParamVOL {
	varray<varray<varray<point3d>>> q3;
	varray<varray<varray<point3d>>> l3;
};



