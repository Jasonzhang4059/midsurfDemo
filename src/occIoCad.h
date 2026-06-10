#pragma once
#include"occTypeDefine.h"

bool ReadStepFile(const std::string& filePath, TopoDS_Shape& shape);
void ExportShapesToSTL(const std::vector<TopoDS_Shape>& shapes, const std::string& filename, double linearDeflection = 0.1);
