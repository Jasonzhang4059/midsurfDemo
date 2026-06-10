#include "occIoCad.h"

#include"occShapeManager.h"
#include<qDebug>

#include <StlAPI_Writer.hxx>
#include <STEPControl_Reader.hxx>  
#include <TopExp_Explorer.hxx>     
#include <TopoDS_Compound.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepTools.hxx>           
#include <BRep_Builder.hxx>

bool ReadStepFile(const std::string & filePath, TopoDS_Shape & shape)
{
	// 创建 STEP 读取器
	STEPControl_Reader reader;
	// 读取文件
	IFSelect_ReturnStatus status = reader.ReadFile(filePath.c_str());
	if (status != IFSelect_RetDone) {
		std::cerr << "Error: Failed to read STEP file." << std::endl;
		return false;
	}

	// 转换文件内容为 OCC 对象
	reader.TransferRoots();  // 转换所有根实体
	shape = reader.OneShape(); // 获取合并后的形状

	if (shape.IsNull()) {
		std::cerr << "Error: No valid shape found in STEP file." << std::endl;
		return false;
	}
	OCC_SHAPE_MANAGER.AddShape(shape);

	return true;
}

void ExportShapesToSTL(const std::vector<TopoDS_Shape>& shapes, const std::string& filename, double linearDeflection) 
{
    if (shapes.empty()) {
        std::cerr << "❌ 形状列表为空，无法导出。" << std::endl;
        return;
    }

    // 合并所有形状为一个 Compound
    TopoDS_Compound compound;
    BRep_Builder builder;
    builder.MakeCompound(compound);

    for (const auto& shape : shapes) {
        // 先对每个 shape 进行三角化
        BRepMesh_IncrementalMesh mesher(shape, linearDeflection);
        builder.Add(compound, shape);
    }

    // 导出 STL
    StlAPI_Writer writer;
    // writer.ASCIIMode() = Standard_True; // 如需 ASCII STL
    writer.Write(compound, filename.c_str());

    std::cout << "✅ 成功导出 STL: " << filename << std::endl;
}