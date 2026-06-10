#include "occShapeManager.h"

occShapeManager& occShapeManager::Instance() {
	static occShapeManager instance;
	return instance;
}

occShapeManager::occShapeManager() : m_nextId(0) {}

void occShapeManager::AddShape(const TopoDS_Shape& shape) {
	Standard_Mutex::Sentry lock(m_mutex);  // 自动加锁
	int newId = GetNextId();  // 获取下一个 ID
	m_shapeMapInt.Bind(newId, shape);  // 绑定形状
}

TopoDS_Shape occShapeManager::GetShape(int id) const {
	Standard_Mutex::Sentry lock(m_mutex);
	return m_shapeMapInt.IsBound(id) ? m_shapeMapInt.Find(id) : TopoDS_Shape();
}

bool occShapeManager::RemoveShape(int id) {
	Standard_Mutex::Sentry lock(m_mutex);
	return m_shapeMapInt.UnBind(id);
}

int occShapeManager::GetNextId() {
	return m_nextId++;  // 返回当前 ID，然后自增
}

#include "moc_OccMidsurfWidget.cpp"