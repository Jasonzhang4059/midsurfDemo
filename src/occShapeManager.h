#pragma once
#include <TopoDS_Shape.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Wire.hxx>
#include <TopoDS_Face.hxx>
#include <NCollection_DataMap.hxx>
#include <Standard_Mutex.hxx>
#include <string>

class occShapeManager {
public:
	// 获取单例实例（线程安全）
	static occShapeManager& Instance();

	// 禁止拷贝和赋值
	occShapeManager(const occShapeManager&) = delete;
	void operator=(const occShapeManager&) = delete;

	// 添加形状并绑定 ID（ID 每次自增）
	void AddShape(const TopoDS_Shape& shape);

	// 通过 ID 获取形状
	TopoDS_Shape GetShape(int id) const;

	// 删除形状
	bool RemoveShape(int id);

private:
	occShapeManager();  // 私有构造函数

	// 获取下一个 ID
	int GetNextId();

	// 线程安全锁
	mutable Standard_Mutex m_mutex;

	// 存储形状映射
	NCollection_DataMap<int, TopoDS_Shape> m_shapeMapInt;

	// 下一个形状 ID
	int m_nextId;
};
