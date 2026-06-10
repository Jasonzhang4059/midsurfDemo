#pragma once

#include "CNurbs.h"
#include "globalFunc.h"
#include <iostream>
#include <sstream>

#include "RWGeometric.h"

#include <vector>
#include "sisl.h"
#include "NurbsTrans.h"

#include <list>
#include <queue>
#include <stack>
#include <map>
#include <algorithm>

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Surface_sweep_2_algorithms.h>
#include <CGAL/intersections.h>
//SISL
#include<ctime>


typedef CGAL::Exact_predicates_exact_constructions_kernel       Kernel;
typedef Kernel::Point_2                                         Point_2;
typedef Kernel::Segment_2										Segment_22;
typedef CGAL::Arr_segment_traits_2<Kernel>                      Traits_2;
typedef Traits_2::Curve_2                                       Segment_2;
typedef Kernel::Intersect_2										Intersect_2;

using namespace std;

const bool largeAngle = true;//是否进行大角判定
const double lag = (double)(160 / 180.0)*PI;//大角度的定义>=160°
const bool byW = false;//若true，所有连线方式(尖点情况除外)，都统一在一起按照w值大小进行排序
const int sharpMode = 0;//若1，去除尖点，否则考虑尖点情况
const bool preProcessQuadPol = true;//默认true，false表示提前把四边形提取至四边的集合
const bool priorityQuadPol = false;//默认false，true则将四边形放到最后进行剖分
const bool cgalJudge = true;//true采用第一种判定，否则采用第二种判定方法

//曲线表达
struct PartNurbsLine :public Spline
{
public:
	//从Spline创建PartNurbsLine
	void CreatPartNurbsLine(Spline inputLine, int ifSeg);

	//从PartNurbsLine重新创建NurnsLine
	void CreatNurbsLine(Spline &newLine);

	//在u处分割曲线
	void PartSegmentation(const double u, varray<PartNurbsLine>& lines);

	//计算首顶点方向向量
	Vec3 CalBeginDirecVec();
	Vec3 CalBeginDirecVec()const;

	//计算尾顶点方向向量
	Vec3 CalEndDirecVec();
	Vec3 CalEndDirecVec()const;

public:
	int ifSegLine;                          //1为可分割线，0为不可分割线
};


//顶点
class SubPoint
{
public:
	SubPoint() {}
	SubPoint(int isoutp, Vec3 p) :ifOutPoint(isoutp), vetx(p) {}

public:
	int ifOutPoint;                     //1为外轮廓顶点，0为内轮廓顶点,-1表示新增截断处顶点
	Vec3 vetx;                          //顶点坐标值	dads
};


//排序使用(后续可将相关代码简化——使用stl及自带的sort函数进行排序)
class DandI
{
public:
	double funcVar;
	int order;
};

//边表节点
struct EdgeNode
{
	int adjvex;						//邻接点域，存储该顶点的下标
	PartNurbsLine EdgeLine;			//对应的Nurbs边曲线
	EdgeNode* next;					//链域，指向下一个邻接点
};

//顶点表节点
struct VertexNode
{
	SubPoint vexdata;				//顶点域，存储顶点信息
	EdgeNode* firstedge;			//边表头指针
};

//有向图
struct GraphAdjList
{
	varray<VertexNode> adjlist;		//顶点集合
	int numNodes, numEdges;			//图中当前顶点和边数
};

//可视点
struct VisiblePoints
{
	int firstPoint;					//第一点
	int secondPoint;				//第二点
	double w;						//两点连线方式的权值
	int ifConvex;					//是否存在凹点,1表示存在凹点
	int ifSharp;					//是否存在尖点,0表示不存在， 1表示存在
};

//多边形中的一点(pt)及对应的可分割线
struct VisibleLine
{
	int pt;							//顶点
	int beginP;						//被截断线头点
	int endP;						//被截断线尾点
	double u;						//分割的节点矢量
	double w;						//权值
};

//多边形轮廓
class MyPolygon
{
public:
	//获得一点的前后点编号
	void GetFBpNumber(int pointNumber, int &frontP, int &behindP);

	//获得某点的前序点编号
	int GetFrontNumber(int pointNumber);

public:
	varray<SubPoint> p_Points;				//顶点
	varray<PartNurbsLine> p_NurbsLines;		//边曲线
	varray<int> objectInAllLines;			//对应曲线在all_NurbsLines数组中的位置
	varray<int> convexHull;					//0代表凸点，1代表凹点，180°共线的也算凹点
	//varray<VisiblePoints> visiblePointPair;
};


//剖分类
class QuadPart 
{
public:
	QuadPart() {}

	QuadPart(varray<Spline> outLines, varray<Spline> inLines, varray<Spline> addLines, varray<int> outQuality, const varray<varray<Vec3>> removePol);

	QuadPart(varray<Spline> outLines, varray<Spline> inLines, varray<Spline> addLines, varray<int> outQuality);

public:
	varray<SubPoint> in_Points;					//内轮廓曲线顶点
	varray<SubPoint> out_Points;				//外轮廓曲线顶点
	varray<SubPoint> all_Points;				//所有顶点
	varray<PartNurbsLine> in_NurbsLines;		//内轮廓曲线
	varray<PartNurbsLine> out_NurbsLines;		//外轮廓曲线
	varray<PartNurbsLine> all_NurbsLines;		//所有曲线

	varray<int> out_Number;						//外轮廓曲线对应序号
	varray<varray<Vec3>> removepoints_Pol;		//不计算多边形的点坐标
	varray<bool> remove_genus;					//不计算多边形的亏格性
	varray<varray<int>> remove_seg;				//不计算多边形的曲线分割性

	GraphAdjList g_Adjlist;						//有向图

	varray<MyPolygon> quad_Polygon;				//四边形集合
	varray<MyPolygon> large_Polygon;			//大于四边的多边形集合
	varray<MyPolygon> remove_Polygon;			//去除不进行计算的多边形集合


public:
	//初始化
	void Init();

	//执行所有剖分流程
	void ExecuteQuadPart();

	//移除不需剖分的多边形
	void RemovePolygon();

	//剖分后处理
	void ResetData();

	bool existPoint(Vec4 point, varray<SubPoint> allPoints);

	//给定外轮廓曲线(面)与内轮廓曲线(面)，创建内轮廓点集和外轮廓点集，以及内轮廓曲线集和外轮廓曲线集
	//outQuality:与outLine数目相等，若为1该外轮廓边可分割，若为0，不可分割
	void CreateAllPointAndLine(varray<Spline> outLines, varray<Spline> inLines, varray<Spline> addLines, varray<int> &outQuality);
	void CreateAllPointAndLine0(varray<Spline> outLines, varray<Spline> inLines, varray<Spline> addLines, varray<int> &outQuality);
	//内外曲线重合判定及操作
	/*
	IL:内部曲线集合
	OL:外曲线集合
	*/
	void OperateOutandInLines(varray<Spline> &IL, varray<Spline> &OL, varray<int>& outQuality);

	//将外轮廓顶点按逆时针进行排序
	void OrderOutPointsAntioclock();

	//建立图的邻接表结构
	void CreateALGraph();

	//给定所有点集，边集，进行闭合多边形轮廓创建（逆时针方向)
	void CreatePolygon();

	//多边形重心计算
	Vec3 CalGravity(varray<SubPoint> polygenPoints);

	//多边形并排序(按照远离外轮廓重心的程度排序)，同时将四边形区分出来
	void OrderPolygens();
	
	//重新设定多边形数据结构(减少数据量)
	void ResetPolygens();

	//四边形和多边形，可分割与不可分割边重设定(暂时不需要这一步)

	//多边形可视点对寻找，五边形放到后面的工作中再处理
	void FindVisiblePoint(MyPolygon &pendingPolygen,varray<VisiblePoints> &visiblePointPair);

	//寻找第二类可视点对(两点可形成三角形，用于分割曲线的预处理)
	void FindAnotherVp(const MyPolygon &inputPolygen, varray<VisiblePoints> &anotherVp);

	//多边形的凹点判定
	void JudgeConcavePoint(MyPolygon &pendingPolygen);

	//可视点对的凹点判定
	void VisiblePointConcaveJudge(MyPolygon &inputPolygen, varray<VisiblePoints> &visiblePointPair);

	//可视点对的尖点判定
	void VisiblePointSharpJudeg(MyPolygon &inputPolygen, varray<VisiblePoints>&visiblePointPair, const varray<PartNurbsLine> &allLines);

	//点与线段之间u值点连线情况下的尖点判定
	bool LinkLineSharpJudge(const Vec3 &frontVec, const Vec3 &lastVec, const Vec3 &pt, const Vec3 &pu);

	//获得一点的前后点编号
	void GetFBpNumber(const MyPolygon &inputPolygen, int pointNumber, int &frontP, int &behindP);

	//获得一点的前后坐标
	void GetFBcoord(const MyPolygon &inputPolygen, int pointNumber, Vec3 &frontCoord, Vec3 &behindCoord);

	//获得一点的前与后向量
	void GetVecs(const MyPolygon &inputPolygen, int pointNumber, Vec3 &frontVec, Vec3 &behindVec);

	//求一点的权函数值(存在凹点的情况)
	double GetFuncSita(const MyPolygon &inputPolygen, int pointNumber, Vec3 vecAB);

	//求一点权函数值(凹点)，重载
	/*mid:该点
	begin:前一点
	end:后一点
	pt:外面一点*/
	double GetFuncSita(const Vec3 &begin, const Vec3 &end, const Vec3 &mid, const Vec3 &pt);

	//求两凸点的权函数(只存在凸点,暂时不用写，写在OrderVisiblePoints里面了)
	double GetConvexFunc();

	//对可视点对进行排序
	void OrderVisiblePoints(const MyPolygon &inputPolygen, varray<VisiblePoints> &visiblePointPair);

	//(没写完)可视点对的分类(内置排序)，存在尖点&不存在尖点，其次内部分类，按照凹凸性以及权函数进行排序
	/*最后visiblePointPair只含非尖点的集合
	sharpVisiblePointPair只含尖点集合
	*/
	void SortVisiblePoints(MyPolygon &inputPolygen, varray<VisiblePoints> &visiblePointPair, varray<VisiblePoints> &sharpVisiblePointPair);

	//获得多边形某点的下一点编号
	int GetNextPoint(const MyPolygon &inputPolygen, const int &pNumber);

	//求解分割点的u值
	/*begin:前一点;
	end:后一点
	crossp:两点连线中的某一点*/
	double GetPartPointU(const MyPolygon &inputPolygen, const varray<PartNurbsLine> &allLines, const Vec3 &begin, const Vec3 &end, const Vec3 &crossp);

	//寻找多边形中的可分割线，并找到存在的可视点(未经过尖点分类排序的)，计算所有情况的u(按照先垂直，再角平分，再中点)，
	//计算所有情况的权值并排序
	void CalPartLines(const MyPolygon &inputPolygen, const varray<PartNurbsLine> &allLines, varray<VisibleLine> &partLine, const int &mode );

	//通过两点创建2次Nurbs曲线(后续需要可增加曲线情况)
	void CreatPartNurbsLine(const Vec3 &p1, const Vec3 &p2, PartNurbsLine &line);

	//两点连线操作(默认第一个点比第二个点位置靠前)
	bool ConncetTwoPoints(MyPolygon &pendingPol, varray<MyPolygon> &quadPol, varray<MyPolygon> &morePol, varray<PartNurbsLine> &allLines, VisiblePoints &visibleP);
	
	//连接点和直线u处点
	bool ConnectLineWithPoint(MyPolygon &pendingPol, varray<MyPolygon> &quadPol, varray<MyPolygon> &morePol, varray<PartNurbsLine> &allLines, VisibleLine &visibleL);



	////////////////////////////////////////////////////////////////////////
	//剖分操作
	/*
	mode:运行模式，0为宽松模式，1为严格模式(针对尖点存在情况)*/
	bool QuadOperation(const varray<MyPolygon> &quadPolygen, const varray<MyPolygon> &morePolygen, const varray<PartNurbsLine> &allLines, varray<MyPolygon> &allPolygen, varray<PartNurbsLine> &endLines, int mode);
};

//曲面包含树节点
class SfCtainTreeNode
{
public:
	SfCtainTreeNode();

	//outlines:输入的外围轮廓线
	SfCtainTreeNode(const varray<Spline> &outlines);

	//conlines:输入的内外连接线
	SfCtainTreeNode(const varray<Spline> &outlines, const varray<Spline> &conlines);

	SfCtainTreeNode(const varray<Spline> &outlines, const varray<Spline> &conlines, varray<int> seg);

	//genus:是否为亏格曲面,false表示不亏格
	SfCtainTreeNode(const varray<Spline> &outlines, const varray<Spline> &conlines, varray<int> seg, bool genus);

	//析构函数
	~SfCtainTreeNode();


	//提取当前节点外轮廓顶点
	void GetOutPoints(varray<Vec3>& ps);

	//提取当前节点外轮廓曲线
	void GetOutLines(varray<Spline>& outline);

	//提取当前节点曲面
	void GetSurfs(varray<SplineSurface> &surfs);


public:
	varray<Spline> outLines;			//外围轮廓线
	varray<Spline> conLines;			//内外连接线
	varray<Vec3> vertex;				//外轮廓顶点坐标
	varray<int> isSeg;					//外轮廓曲线的可分割性
	bool isGenus;						//是否亏格

	varray<Spline> allLines;			//所有曲线
	varray<int> outNumber;				//外围轮廓线序号(已经完成剖分后的)
	varray<varray<int>> quadPolNumber;	//所有四边形对应曲线序号
	//varray<int> conNumber;			//内外连接线序号
	//SfCtainTreeNode* child;			//子节点
	//SfCtainTreeNode* brother;			//兄弟节点
	list<SfCtainTreeNode*> childs;		//所包含的曲面子节点
	bool isSort;						//顶点是否经过排序
	int num;							//曲面序号，测试时暂时使用
	bool quaded;						//是否完成剖分
};


//曲线去重
void DecoincideNurbsLine(varray<Spline>& nl);

bool existed(const varray<int> &erase_Index, int index);
void connectAddLine(const int &index, const varray<Spline> &addLine, varray<int> &erase_Index);

//处理每个包含树节点(作为父节点时)
void ReSetTreeNode(SfCtainTreeNode* father);

//创建曲面包含树
SfCtainTreeNode* CreateSurfContainTree(const varray<varray<Spline>> &surf, const varray<Spline> conlines, varray<varray<int>> seg, varray<bool> genus);
//创建曲面包含树(特征框架请使用该函数)
//surf:轮廓曲线，[0]位置为外轮廓曲线
//conlines:辅助内外连接线
//seg:曲线可截断性
//genus:亏格性(是否为空)true为孔 false为实心
//sfnum:曲面序号
SfCtainTreeNode* CreateSurfContainTree(const varray<varray<Spline>> &surf, const varray<Spline> conlines, varray<varray<int>> seg, varray<bool> genus, varray<int> sfnum);

/////////////////////////////////////////
//
//
//剖分当前节点曲面多边形，每一步的剖分函数
void QuadTreeNodeSurf(SfCtainTreeNode* root);

//根据曲面包含树进行剖分操作
void QuadWithContainTree(SfCtainTreeNode* root);

//从root节点开始计算所有曲面信息
void GetQuadSurfaces(SfCtainTreeNode* root, varray<SplineSurface> &surf);

//执行所有包含树操作
void StartTreeOperation(const varray<varray<Spline>>& surf, const varray<Spline>& conlines, const varray<varray<int>> seg, varray<bool> genus, varray<Spline> &allLines, varray<SplineSurface> &allSurf);

//通过两点创建2次Nurbs曲线(后续需要可增加曲线情况)
Spline CreatNurbsLine(const Vec3 &p1, const Vec3 &p2);


//获得一多边形某点前后曲线
void GetLinesByPoint(const MyPolygon &inputPolygen, int pointNumber, const varray<PartNurbsLine> &allLine, Spline &frLine, Spline &backLine);

//获取一多边形某点前后曲线对应切向量
void GetVecsByPoint(const MyPolygon &inputPolygen, int pointNumber, const varray<PartNurbsLine> &allLine, Vec3 &frVec, Vec3 &backVec);

//获得一多边形某点的前后坐标
void GetFBcoord(const MyPolygon &inputPolygen, int pointNumber, Vec3 &frontCoord, Vec3 &behindCoord);


//通过两点创建2次Nurbs曲线,针对两个端点情况(正在增加曲线情况)
//pol:排序完成的多边形
PartNurbsLine GetPartNurbsLine(const Vec3 &p1, const Vec3 &p2, const varray<PartNurbsLine> &allLine, MyPolygon pol);

//通过两点创建2次Nurbs曲线,针对一个端点,一条被截断曲线情况
//p1:连线端点坐标
//frontp:被截断曲线所对应在多边形上前一点坐标
//u:被截断点的参数
PartNurbsLine GetPartNurbsLine(const Vec3 &p1, const Vec3 &frontp, double u, const varray<PartNurbsLine> &allLine, MyPolygon pol);


//求解分割点的u值
//begin:前一点;
//end:后一点
//crossp:两点连线中的某一点
double GetPartU (const Spline &lines, const Vec3 &begin, const Vec3 &end, const Vec3 &crossp);

//给定前后两点(逆时针)以及圆心坐标，计算该NURBS圆弧(在xy平面上)
void CreatArc(const Vec3 &p1, const Vec3 &p2, const Vec3 &centre, Spline &line);

//对任一四边形四边进行排序(可用于Coons插值的序列)
void QuadOrder(const MyPolygon &quad,varray<PartNurbsLine> &allLine ,varray<Spline> &coonsLine);

//对任一四边形四边进行排序(可用于Coons插值的序列)
void QuadOrder(const varray<int> &quadNums, const varray<Spline> &allLine, varray<Spline> &coonsLine);

//判定两点是否重合
bool JudgeTwoPointsCoincide(const Vec3 &p1, const Vec3 &p2);

//判定两Nurbs曲线是否重合(只看头尾顶点)
bool JudgeTwoLinesCoincide(const Spline &L1, const Spline &L2);

//判定两Nurbs曲线是否重合(只看头尾顶点)
bool JudgeTwoLinesIntersect(const Spline &L1, const Spline &L2);

//求两向量间的角度
double GetAngle(Vec3 vec1, Vec3 vec2);

//求两直线的交点(二维，即在同一平面，z值相等),未讨论两直线重合情况
Vec3 GetTwoLinesCrossPoint(const Vec3 &p1, const Vec3 &p2, const Vec3 &p3, const Vec3 &p4);

//三维空间点到直线的垂足(可能在线段外面)
Vec3 GetFootPerpendicular(const Vec3 &begin, const Vec3 &end, const Vec3 &pt);

//判断是否存在该交点,且不为两定点
bool JudegeFootPerpendicular(const Vec3 &begin, const Vec3 &end, const Vec3 &crossPoint);

//求两向量角平分线向量
Vec3 GetAngularBisectorVec(const Vec3 &leftVec, const Vec3 &rigthtVec);

//求解角平分线向量所在直线和原直线的交点(之后再交给其他函数判断是否存在该交点)
Vec3 GetAngularBisectorPoint(const Vec3 &begin, const Vec3 &end,const Vec3 &pt, const Vec3 &vec);

//曲面(以曲线表达)空间变换(后续可以再修改)
void SurfaceConverse(varray<Spline> &lines, Vec3 targetPts, Vec3 targetVec, Matrix4d &conMx, Matrix4d &reconMx);

//曲线空间变换
void LineConverse(varray<Spline> &lines, Matrix4d &conMx);

//求解一点参数u是否为曲线端点
int CalU(double u);

//将组成闭合多边形的若干条边按逆时针排序,返回逆时针排序点坐标
//polLines:待排序多边形边曲线集合
varray<Vec3> OrderLinesAntioclock(varray<Spline> &polLines);


//求解正确u
bool CalRightu(double u, double y, Spline Nline, vector<double> &us);

bool CalRightu(vector<double> u, double y, Spline Nline, vector<double> &us);

bool CalRightu(vector<float> u, float y, Spline Nline, vector<float> &us);

//线段与NURBS(SISL)曲线求交点
//返回交点数
int LineIntersectNurbs(SISLCurve *& curve, const varray<Vec3>& strLine, const Spline * nurbsCurve = nullptr);

//求解一点在若干NURBS曲线围成的多边形(内/外/上)
//返回值为-1:error
//返回值为0:在多边形上(且为端点处)
//返回值为1:在多边形内部
//返回值为2:在多边形外
//返回值为3:在多边形上(且不为端点处)
int PointRelatePolygon(Vec3 p, varray<Spline> &pol);

//两NURBS曲面所围多边形包含关系,pola是否在polb内部
bool PolygonsRelationship(varray<Spline> pola, varray<Spline> polb);



