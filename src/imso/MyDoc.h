#pragma once

#include "CNurbs.h"
#include "Fragment.h"
#include <qstring.h>
#include <qvector4d.h>
#include <qvector3d.h>

class MyDoc
{
public:
	//清理数组
	void cleanUp()
	{
		m_ShowData.clear();
		m_ShowLines.clear();
	}
	

	//清理显示函数
	void clearScrn()
	{
		VAO_L_ARRAY.clear();
		VAO_D_ARRAY.clear();
		VAO_CP_ARRAY.clear();
		VAO_FRAGMENT_ARRAY.clear();
		VAO_SUPPORT_ARRAY.clear();
		EACH_VAO_L_POINT_NUMBER.clear();
		EACH_VAO_CP_POINT_NUMBER.clear();
		EACH_VAO_D_POINT_NUMBER.clear();
		EACH_VAO_F_NUMBER.clear();
		EACH_VAO_SUPPORT_NUMBER.clear();
	}

	//获取单例模式指针
	typedef std::shared_ptr<MyDoc> Ptr;
	MyDoc(const MyDoc&) = delete;
	MyDoc& operator=(const MyDoc&) = delete;
	static Ptr getInstance()
	{
		if (m_instance_ptr == nullptr)
		{
			std::lock_guard<std::mutex> lk(mutex);
			if (m_instance_ptr == nullptr)
			{
				m_instance_ptr = std::shared_ptr<MyDoc>(new MyDoc);
			}
		}
		return m_instance_ptr;
	}
	~MyDoc() {}
private:
	static Ptr m_instance_ptr;
	static std::mutex mutex;
	MyDoc() {}
public:
	
	QString path;//模型路径
	int modelType;//模型的类型（nurbsVOL，nurbsSurface等）

	//初始细分度
	int m_Unum = 20;
	int m_Vnum = 20;
	int m_Wnum = 20;

	int m_tolNum;//单元总量
	int m_NUMPic = 54;//单个片，面片数目
	int m_CellIdx = -1;//分片模型的编号
	float m_dotSZ = 10.0;//显示时，点的大小
	float m_CPTSdotSZ = 10.0;//显示时，控制点的大小
	float _lineWidth = 3.0;//线宽
	bool m_showCtrlPts=false;//是否显示控制点
	bool m_showCoordinates=true;//是否显示坐标系
	bool m_restView = false;//是否重置视角
	bool m_Fragment = false;//是否需要分片
	int mode = 7;

	int THREADNUMBERS = 8;//线程大小的数量

	varray<varray<varray<point3d>>> m_ShowData;//3D显示数据
	varray<varray<varray<point3d>>> m_ShowLines;//线条显示数据
	varray<varray<point4d>> m_ShowData4D;//4D控制点显示数据

	varray<varray<varray<Vec3>>> m_DisplayData;//3D显示数据
	varray<varray<varray<Vec3>>> m_DisplayLines;//线条显示数据
	varray<varray<Vec4>> m_DisplayData4D;//4D控制点显示数据
	
	std::vector<float> showDta_vertices;//把上面画体的数组综合为一个一维数组
	std::vector<float> showLines_vertices;//把上面画线的数组综合为一个一维数组
	std::vector<float> showCp_vertices;//把上面控制点综合为一个一维数组
	std::vector<float> showFp_vertices;//分片点
	
	std::vector<unsigned int> VAO_L_ARRAY, VAO_D_ARRAY, VAO_CP_ARRAY, VAO_FRAGMENT_ARRAY, VAO_SUPPORT_ARRAY;//保存顶点数组缓冲对象的句柄
	std::vector<int> EACH_VAO_L_POINT_NUMBER, EACH_VAO_CP_POINT_NUMBER, EACH_VAO_D_POINT_NUMBER, EACH_VAO_F_NUMBER, EACH_VAO_SUPPORT_NUMBER;//保存每一条线的个数

	std::vector<unsigned> VAO_D;//渲染非均质模型的句柄
	std::vector<int> VAO_D_SIZE;
	int m_GLMode = 7;//0=POINTS,3=LINE,7=FILL

	float m_fragmentA, m_fragmentB, m_fragmentC;//分片切面
	int m_fragmentDegree;//多少个切面
	float m_fragmentBegin, m_fragmentEnd;//开始和结束的位置
	
	float _mouseSenstive=0.01;//鼠标灵敏度
	float _keyBordSensitive=10;//键盘灵敏度

	QVector3D _bordLineColor = { 0.1f,0.1f,0.1f };//边界线颜色 默认黑色
	QVector3D _surfaceColor = { 0.0f,1.0f,0.0f };//面片颜色 默认绿色
	QVector3D _controlPoinrColor = { 1.0f,0.0f,0.0f };//控制点颜色 默认红色
	QVector3D _sliceSurfaceColor = { 1.0f,0.0f,0.0f};//切片面颜色 默认红色
	QVector3D _sliceSuppotrColor = { 0.0f,0.0f,1.0f };//切片支撑色颜色 默认蓝色
	QVector4D _lightColor = { 1.0f,1.0f,1.0f,1.0f };

	//摄像机的初始位置
	float cameraX = 0;
	float cameraY = 0;
	float cameraZ = 80;
};
