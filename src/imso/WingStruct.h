#pragma once

#include "Nurbs.h"
#include <list>
#include <stack>
#include <string>
#include <fstream>

using std::vector;
using std::unique_ptr;
using std::string;
using std::ifstream;
using std::ofstream;

/*整个数据结构通过面之间的关系连接，具有增删查改等若干操作*/
namespace YN {


	//枚举数组，识别方向
	enum Direct { u0v0, u1v1, v0w0, v1w1, u0w0, u1w1 };
	struct Cube;

	//翼面数据结构：线
	struct CurveLine
	{
		YN::NurbsLine _lineData;
	};

	//翼面数据结构：面
	struct Surface
	{
		//几何信息，参数域信息
		YN::NurbsSurface _surfaceData;//面的数据
		vector<int> _surfaceGloblControlPointID;//面的控制点全局编号
		//拓扑信息
		Surface * _nextSurface = NULL;//邻接面
		bool _haveNextSuf;//是否有邻接面
		Cube * _pointOther = NULL;//邻接片
		Cube * _pointThis = NULL;//本片
		int _pointThisNumber;//面的识别序号
	};

	//翼面数据结构：体
	struct Cube
	{
		YN::NurbsVol _Vol_Data;
		vector<int> _volGloblControlPointID;//片内控制点全局编号
		Surface * _u0v0 = NULL;//0  数字为面的识别序号,对应枚举数组Direct
		Surface * _u1v1 = NULL;//1
		Surface * _v0w0 = NULL;//2
		Surface * _v1w1 = NULL;//3
		Surface * _u0w0 = NULL;//4
		Surface * _u1w1 = NULL;//5
		bool _isOrdered = false;//是否经过排序
		bool _bordFirstSearch = true;
	};

	//翼面数据结构
	class WingStruct
	{
	public:
		WingStruct();
		//无权重tag:false 有权重tag:true
		WingStruct(string path, bool tag);
		WingStruct(vector<NurbsVol> vols);
		~WingStruct();

		//增
		bool addVolToSurface(point4d target, YN::NurbsLine line, YN::NurbsSurface surface);
		//删
		bool removeVol(point4d target);
		//查，快速查询，输入目标点，起始片，输出查询路径片
		vector<Cube *> quickSearch(point4d, Cube *);
		//改
		void Modify(Cube * target, int direct, int u, int v, int w);
		//计算物体的表面
		vector<YN::NurbsSurface> getModelSurface();
		//文件的读取,其中tag如果为true表示文件中有权重，tag为false表示文件没有权重
		vector<Cube *> ReadVolumeTxt(string path,bool tag);
		//文件输出，返回值是施加在模型上的力和约束示意点
		vector<point4d>  OutputParaVolumeDataTxt(string filename, string Cp);
		//文件输出为VTK
		void  OutputParaVolumeDataVTK(string filename);
	public:
		Cube * _root;//根指针
	private:
		//数据结构不能复制
		WingStruct(const WingStruct&) = delete;
		WingStruct& operator=(const WingStruct&) = delete;
		//提取信息
		void abstractInfo(vector<Cube *>&);
		//输入cube，更新面信息
		vector<Surface *> upDateSurface(vector<Cube *>&);
		vector<Surface *> upDateCubeSurface(Cube *);
		//NurbsVol转换为Cube
		vector<Cube *> transNurbsVoltoCube(vector<YN::NurbsVol>);
		//设置拓扑信息
		bool setTwinPatch(vector<Cube *>&, vector<Surface *>&);
		//设置参数域信息
		bool Order(vector<Cube *>&);
		//广度遍历
		vector<Cube *> bordFirstSearch();
		//深度遍历,给出起始片、方向
		vector<Cube *> deepFirstSearch(Cube *, int);
		//两个片设置为相同UVW
		bool setNearCubeWithSameUVW(Cube *, Cube *, int index);
		//获得全局控制点的id
		void setGloblControlPointId();
		//判断点是否在片内部,在内部返回编号，不在内部返回-1
		int isPointInSurface(point4d, Cube *);
	};

};