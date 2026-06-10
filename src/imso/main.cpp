#pragma once
#include "QtGuiApplication1.h"
#include <QtWidgets/QApplication>
#include "MyDoc.h"
#include<string>
#include "FeatureNetwork.h"
#include "PolyIGA.h"
#include "quadPart.h"
#include "MeshQuality.h"
#include<algorithm>
#include "PublicModels.h"
#include"Lee.h"
#include<fstream>
//#include"kuang.h"
//Delaunay测试
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Simple_cartesian.h> //笛卡尔坐标相关头文件
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Constrained_Delaunay_triangulation_2<K> CDT;
typedef CGAL::Simple_cartesian<double> Kernel0; // 内核使用双精度浮点数作为该点的笛卡尔坐标
typedef Kernel0::Point_2 Point_2D;               // 二维点
typedef CDT::Point Point_CDT;
typedef Kernel0::Segment_2 Line_2D;           // 二维线段
typedef CDT::Vertex_handle Vertex_handle;
//Delaunay测试

//两个静态成员的初始化
MyDoc::Ptr MyDoc::m_instance_ptr = nullptr;
std::mutex MyDoc::mutex;

//剖分函数
void quadPlane(varray<Spline> outer, varray<varray<Spline>> inner, varray<bool> &genus)
{
	RWGeometric rwg;
	varray<SplineSurface> SS;
	PublicSolution MS;

	MS.quad(outer, inner, genus, SS);

	rwg.WriteSplineSurface("E:\\Model\\PlaneQuad\\BoundarySurface.txt", SS);
}
void quadModel() {
	//构建内轮廓、外轮廓
	varray<Spline> outer;
	varray<varray<Spline>> inner;
	RWGeometric rwg;
	Spline SL;
	Spline0 sl1;
	double w = cos(PI / 4);
	Vec4 v1 = { -5,5,0,1 };
	Vec4 v2 = { 5,5,0,1 };
	Vec4 v3 = { 5,-5,0,1 };
	Vec4 v4 = { -5,-5,0,1 };

	Vec4 v5 = { -2,2,0,1 };
	Vec4 v6 = { 2,2,0,1 };
	Vec4 v7 = { 3,0,0,1 };
	Vec4 v8 = { 0,-2,0,1 };
	Vec4 v9 = { -3,0,0,1 };

	Vec4 v10 = { -1,1,0,1 };
	Vec4 v11 = { 0,2,0,1 };
	Vec4 v12 = { 1,1,0,1 };
	Vec4 v13 = { 0,0,0,1 };

	Vec4 v14 = { -1,2,0,w };
	Vec4 v15 = { 1,2,0,w };
	Vec4 v16 = { 1,0,0,w };
	Vec4 v17 = { -1,0,0,w };

	////创建外轮廓线
	SL = sl1.getSpline(v1, v2);
	outer.push_back(SL);
	SL = sl1.getSpline(v2, v3);
	outer.push_back(SL);
	SL = sl1.getSpline(v3, v4);
	outer.push_back(SL);
	SL = sl1.getSpline(v4, v1);
	outer.push_back(SL);

	inner.resize(2);
	////创建内轮廓线1
	//SL = sl1.getSpline(v5, v6);
	//inner[0].push_back(SL);
	//SL = sl1.getSpline(v6, v7);
	//inner[0].push_back(SL);
	//SL = sl1.getSpline(v7, v8);
	//inner[0].push_back(SL);
	//SL = sl1.getSpline(v8, v9);
	//inner[0].push_back(SL);
	//SL = sl1.getSpline(v9, v5);
	//inner[0].push_back(SL);

	inner.resize(2);
	rwg.ReadSpline("E:\\Model\\PlaneQuad\\OuterBoundry.txt", outer);

	varray<Spline> temp;
	rwg.ReadSpline("E:\\Model\\PlaneQuad\\InnerBoundry1.txt", temp);
	inner[0] = temp;
	rwg.ReadSpline("E:\\Model\\PlaneQuad\\InnerBoundry2.txt", temp);
	inner[1] = temp;

	//亏格设置
	varray<bool> genus;
	genus.resize(3);
	genus[0] = false;
	/*genus[1] = true;
	genus[2] = true;*/
	genus[1] = false;
	genus[2] = false;

	varray<Spline> OriginSpline;
	OriginSpline.clear();
	for (auto &i : outer) {
		OriginSpline.push_back(i);
	}

	for (auto &i : inner) {
		for (auto &j : i) {
			OriginSpline.push_back(j);
		}
	}

	rwg.WriteSpline("E:\\Model\\PlaneQuad\\OriginBoundry.txt", OriginSpline);

	quadPlane(outer, inner, genus);
	//平面剖分
}

varray<varray<SplineSurface>> quadPlane(varray<Spline> outer, varray<varray<Spline>> inner);

varray<varray<Spline>> readInnerBoundry(int num);
void writeInnerBoundry(varray<varray<Spline>> inner);
int ShowPlatform(int argc, char *argv[]);
void putOutVTK();
void putOutVTK(varray<SplineVolume>SV);
void Square_Circle();
void Model_chong();
void modelQualityTest();
void test(varray<SplineSurface> SS);
void test01();
void test02();
void test03();
void test04();
void test05();
void test06();
void test07();
void test08();
void test09();
void test10();
void test11();
void test12();
void test13();
void test14();
void test15();
void test16();
void test17();
void test18();
void test19();
void test20();
void test21();
void test22();
void test23();
void test24();
void test25();
void test26();
void test27();
void test28();
void test_s1955(Spline s1, Spline s2, double &u1, double &u2);
void test_BumpPiont();
void test_CDT();
void test_COONS();
void test_NurbsLine();
void create_Model();
void testProjectPointToMesh();
void PaPerModel_BSplineSurface();
void PaPerModel_nurbslineSurface();
void example1();
void example2();
void example3();
void example4();
void example5();
void example6();
void example7();
//轴承座支架
void example8();
void example9();
void threeTube();
void fourTube();
void T_Tube();
void checkModelQuality6();

//外正方行，内部多个圆孔，天罡用
void createModel();
//测试nurbs线

void bodyInterModel()
{
	//常用工具
	RWGeometric rwg;
	Boundary bo;
	Spline0 s0;
	Model_Solution ms;
	PublicSolution ps;
	Spline s;
	varray<Spline> S;
	varray< varray<Spline>> inner;
	varray<Spline> outer;
	varray<SplineSurface>SS;
	varray<SplineSurface>SS1;
	varray<SplineSurface>SS2;
	varray<SplineVolume> SV;
	varray<Spline> temp_S;
	varray<SplineSurface> temp_SS;
	varray<SplineVolume> temp_SV;
	SplineSurface ss;

	S = bo.getSquare(10,10);
	ps.sortEdg(S);
	ss.CoonsInterpolate(S);
	SS1.push_back(ss);
	

	S = bo.getSquare(5, 5);
	ps.sortEdg(S);
	ss.CoonsInterpolate(S);
	SS2.push_back(ss);
	ms.Trans(SS2, 10, 3);


	SV = ps.loft(SS1, SS2);

	rwg.ReadSplineSurface("F:\\Learn\\大论文\\Model\\特征操作\\BodyInterFace.txt", SS);

	int len = 2;
	ms.Trans(SS[0], len, -3);
	ms.Trans(SS[1], len, 1);
	ms.Trans(SS[2], len, -1);
	ms.Trans(SS[3], len, 2);
	ms.Trans(SS[4], len, -2);
	ms.Trans(SS[5], len, 3);
	rwg.WriteSplineSurface("F:\\Learn\\大论文\\Model\\特征操作\\BodyInterFace1.txt", SS);
}


void sweepModel()
{
	//常用工具
	RWGeometric rwg;
	Boundary bo;
	Spline0 s0;
	Model_Solution ms;
	PublicSolution ps;
	Spline s;
	varray<Spline> S;
	varray< varray<Spline>> inner;
	varray<Spline> outer;
	varray<SplineSurface>SS;
	varray<SplineVolume> SV;
	varray<Spline> temp_S;
	varray<SplineSurface> temp_SS;
	varray<SplineVolume> temp_SV;
	//常用工具

	//拉伸体
	outer = bo.getCircle(5);
	inner.push_back(bo.getCircle(3));

	CDT_Operate cdto(outer, inner);
	// 亏格设置
	varray<bool> genus;
	genus.resize(2, true);
	genus[0] = false;
	varray<Spline> addLines;
	ps.quad(outer, inner, cdto.addLine, genus, SS);

	rwg.WriteSplineSurface("F:\\Learn\\大论文\\Model\\特征操作\\SweepFace.txt", SS);
	temp_SV = ms.CreatSweepVol(SS, 10, 3);

	rwg.WriteSplineVolume("F:\\Learn\\大论文\\Model\\特征操作\\SweepVolume.txt", temp_SV);



	//放样体
	varray<SplineSurface>SS2;
	outer = bo.getCircle(3);

	inner.clear();
	inner.push_back(bo.getCircle(1));
	CDT_Operate cdto2(outer, inner);
	// 亏格设置
	genus.resize(2, true);
	genus[0] = false;
	ps.quad(outer, inner, cdto2.addLine, genus, SS2);

	ms.Trans(SS2, 10, 3);
	SV = ps.loft(SS, SS2);

	temp_SS = SS;
	for (auto&i : SS2)
	{
		temp_SS.push_back(i);
	}
	rwg.WriteSplineSurface("F:\\Learn\\大论文\\Model\\特征操作\\LoftFace.txt", temp_SS);

	rwg.WriteSplineVolume("F:\\Learn\\大论文\\Model\\特征操作\\LoftVolume.txt", SV);


	//扫掠体生成

	SS2.clear();
	for (auto&i : SS)
	{
		SS2.push_back(i);
	}
	ms.Trans(SS2, 20, 3);
	ms.Trans(SS2, 10, -1);

	SV = ps.loft(SS, SS2);

	temp_SS = SS;
	for (auto&i : SS2)
	{
		temp_SS.push_back(i);
	}
	rwg.WriteSplineSurface("F:\\Learn\\大论文\\Model\\特征操作\\SweepFace2.txt", temp_SS);

	rwg.WriteSplineVolume("F:\\Learn\\大论文\\Model\\特征操作\\SweepVolume2.txt", SV);
	
	//旋转扫掠
	temp_SS.clear();
	for (auto&i : SS)
	{
		temp_SS.push_back(i);
	}


	ms.Trans(temp_SS, 20, -1);
	SS2.clear();
	for (auto&i : temp_SS)
	{
		SS2.push_back(i);
	}
	varray<SplineSurface>SS3;

	ms.Rolate(temp_SS, PI / 2, 2);
	for (auto&i : temp_SS)
	{
		SS3.push_back(i);
	}
	SV = ps.loft(SS2, SS3);

	temp_SS = SS2;
	for (auto&i : SS3)
	{
		temp_SS.push_back(i);
	}
	rwg.WriteSplineSurface("F:\\Learn\\大论文\\Model\\特征操作\\SweepFace3.txt", temp_SS);

	rwg.WriteSplineVolume("F:\\Learn\\大论文\\Model\\特征操作\\SweepVolume3.txt", SV);


	//旋转
	SV.clear();
	rwg.ReadSplineVolume("F:\\Learn\\大论文\\Model\\特征操作\\旋转拉伸体.txt", temp_SV);
	for (auto &i : temp_SV)
	{
		SV.push_back(i);
	}
	

	ms.Rolate(temp_SV, PI / 2, 2);
	for (auto &i : temp_SV)
	{
		SV.push_back(i);
	}

	ms.Rolate(temp_SV, PI / 2, 2);
	for (auto &i : temp_SV)
	{
		SV.push_back(i);
	}

	ms.Rolate(temp_SV, PI / 2, 2);
	for (auto &i : temp_SV)
	{
		SV.push_back(i);
	}
	rwg.WriteSplineVolume("F:\\Learn\\大论文\\Model\\特征操作\\SweepVolume4.txt", SV);

	
}

//拼接轴承座
void basement()
{
	//常用工具
	RWGeometric rwg;
	Boundary bo;
	Spline0 s0;
	Model_Solution ms;
	PublicSolution ps;
	Spline s;
	varray<Spline> S;
	varray< varray<Spline>> inner;
	varray<Spline> outer;
	varray<SplineSurface>SS;
	varray<SplineSurface>allSurface;
	varray<varray<SplineSurface>> partSurface;
	varray<SplineVolume> SV;
	varray<Spline> temp_S;
	varray<SplineSurface> temp_SS;
	varray<SplineVolume> temp_SV;
	//常用工具
	
	outer = bo.getSquare(90, 90);
	
	inner.push_back(bo.getCircle(30));
	ms.Rolate(inner[0], PI / 4, 3);

	for (auto&i : outer)
	{
		temp_S.push_back(i);
	}


	for (auto&i : inner)
	{
		for (auto&j : i)
		{
			temp_S.push_back(j);
		}
	}

	rwg.WriteSpline("F:\\Learn\\大论文\\Model\\轴承座支架\\partBoudary.txt", temp_S);
	CDT_Operate cdto(outer, inner);
	// 亏格设置
	varray<bool> genus;
	genus.resize(2, true);
	genus[0] = false;
	varray<Spline> addLines;
	ps.quad(outer, inner, cdto.addLine, genus, SS);
	
	rwg.WriteSplineSurface("F:\\Learn\\大论文\\Model\\轴承座支架\\part2.txt", SS);
	ms.Rolate(SS, PI / 2, 1);
	ms.Trans(SS, 45, 3);
	
	ms.Trans(SS, 17, 2);
	for (auto&i : SS)
	{
		allSurface.push_back(i);
	}

	temp_SV = ms.CreatSweepVol(SS, 18, 2);
	rwg.WriteSplineSurface("F:\\Learn\\大论文\\Model\\轴承座支架\\topSurface.txt", SS);
	//ms.Trans(temp_SV, 36, 3);

	for (auto&i : temp_SV)
	{
		SV.push_back(i);
	}

	ms.Trans(temp_SV, 34 + 18, -2);
	for (auto&i : temp_SV)
	{
		SV.push_back(i);
	}
	/*ms.Trans(SS, 34, -2);
	for (auto&i : SS)
	{
		allSurface.push_back(i);
	}
	temp_SV = ms.CreatSweepVol(SS, 18, -2);

	ms.Trans(temp_SV, 36, 3);
	for (int i = temp_SV.size(); i < temp_SV.size(); ++i)
	{
		temp_SV[i].OrderCtrlPts(temp_SV[i]);
	}
	for (auto&i : temp_SV)
	{
		SV.push_back(i);
	}*/

	rwg.ReadSplineSurface("F:\\Learn\\大论文\\Model\\轴承座支架\\completeSurface.txt", SS);


	S = bo.getSquare(90, 18);
	ms.Trans(S, 26, 2);
	SplineSurface ss;
	ps.sortEdg(S);
	ss.CoonsInterpolate(S);
	SS.push_back(ss);

	S = bo.getSquare(90, 18);
	ms.Trans(S, -26, 2);
	ps.sortEdg(S);
	ss.CoonsInterpolate(S);
	SS.push_back(ss);
	for (auto&i : SS)
	{
		allSurface.push_back(i);
	}
	temp_SV = ms.CreatSweepVol(SS, 18, -3);
	rwg.WriteSplineSurface("F:\\Learn\\大论文\\Model\\轴承座支架\\bottomSurface.txt", SS);
	rwg.WriteSplineVolume("F:\\Learn\\大论文\\Model\\轴承座支架\\bottom.txt", temp_SV);
	//for (auto&i : temp_SV)
	//{
	//	SV.push_back(i);
	//}

	for (int i = 0; i < temp_SV.size(); ++i)
	{
		if (i == 28 || i == 29 || i == 32 || i == 34)
		{
			temp_SV[i].OrderCtrlPts(temp_SV[i]);
			temp_SV[i].OrderCtrlPts(temp_SV[i]);
			continue;
		}

		SV.push_back(temp_SV[i]);
	}
	rwg.WriteSplineSurface("F:\\Learn\\大论文\\Model\\轴承座支架\\allPart.txt", allSurface);
	rwg.WriteSplineVolume("F:\\Learn\\大论文\\Model\\轴承座支架\\expVolume.txt", SV);

	putOutVTK(SV);
}

void ChackModelQuality(varray<SplineSurface> SS)
{
	Model_Solution m;
	RWGeometric rwg;
	varray<SplineVolume> SV = m.CreatSweepVol(SS, 5, 3);
	putOutVTK(SV);
}

int main(int argc, char *argv[])
{
	Model_Solution m;
	RWGeometric rwg;
	PublicSolution ps;
	varray<SplineSurface>SS;
	varray<SplineVolume> SV;
	varray<Spline> S;
	varray<Spline> temp;
	varray<SplineVolume> temp_V;
	varray<SplineSurface> temp_S;
	YuYan yy;
	//sweepModel();
	//bodyInterModel();
	//fourTube();
	//example8();
	//basement();
	//createModel();
	//ps.quadSurface();
	//checkModelQuality6();
	//putOutVTK();
	//testProjectPointToMesh();
	//putOutVTK();
	//test_CDT();
	////查看模型质量
	//putOutVTK();
	ShowPlatform(argc, argv);
}

varray<varray<SplineSurface>> quadPlane(varray<Spline> outer, varray<varray<Spline>> inner)
{
	varray<Spline> S;
	varray<SplineSurface>CompleteSurface;
	varray<varray<SplineSurface>>partSurface;
	// 亏格设置
	varray<bool> genus;
	RWGeometric rwg;

	//所有轮廓线
	varray<varray<Spline>> surf;
	//初始化surf
	surf.push_back(outer);
	for (auto &i : inner)
	{
		surf.push_back(i);
	}

	//所有边界线
	varray<Spline> allSpline;
	for (auto &i : surf)
	{
		for (auto &j : i)
		{
			allSpline.push_back(j);
		}
	}

	//辅助查看的数组
	varray<Spline> chack;

	//临时变量
	ContourData cd;

	//层次遍历所用的队列
	queue<SfCtainTreeNode *> Qnodes;

	//层次遍历后根遍历所用队列
	stack<SfCtainTreeNode *>Snode;
	PublicSolution ps;
	//初略构建几何域包含树
	SfCtainTreeNode * root = ps.CreateTree(surf);

	//层次遍历指针
	SfCtainTreeNode* cur = nullptr;

	//辅助工具
	varray<Spline> tempChackLine;

	Qnodes.push(root);
	//cur节点的子节点
	varray<SfCtainTreeNode *> allNode;
	varray<SfCtainTreeNode *> childnodes;

	//层次遍历几何域包含树，
	while (!Qnodes.empty())
	{
		cur = Qnodes.front();
		Qnodes.pop();

		//用栈记录后根遍历的顺序
		Snode.push(cur);

		//实现层次遍历
		for (auto it = cur->childs.begin(); it != cur->childs.end(); ++it)
		{
			Qnodes.push(*it);
		}

	}
	varray<Spline> addLine;
	//根据后根遍历顺序，实现由内至外创建连接线
	while (!Snode.empty())
	{
		cur = Snode.top();
		Snode.pop();
		if (!cur->childs.empty())
		{
			varray<varray<Spline>>curInner;
			//以cur为外轮廓，子节点为内轮廓，创建连接线
			for (auto it = cur->childs.begin(); it != cur->childs.end(); ++it)
			{
				curInner.push_back((*it)->outLines);
			}

			//生成连接线
			CDT_Operate cdto(cur->outLines, curInner);

			varray<Spline> allLine;
			allLine = cur->outLines;
			for (auto&i : curInner)
			{
				for (auto&j : i)
				{
					allLine.push_back(j);
				}
			}
			rwg.WriteSpline("F:\\Learn\\大论文\\PlaneQuadModel\\boundary.txt", allLine);
			for (auto&i : cdto.addLine)
			{
				allLine.push_back(i);
			}
			rwg.WriteSpline("F:\\Learn\\大论文\\PlaneQuadModel\\addLine.txt", cdto.addLine);

			rwg.WriteSpline("F:\\Learn\\大论文\\PlaneQuadModel\\allLines.txt", allLine);
			//亏格设置
			genus.resize(curInner.size() + 1);
			genus[0] = false;
			for (int i = 1; i < genus.size(); ++i)
			{
				genus[i] = true;
			}
			varray<SplineSurface>allSurface;
			//平面剖分
			ps.quad(cur->outLines, curInner, cdto.addLine, genus, allSurface);
			for (auto&i : allSurface)
			{
				CompleteSurface.push_back(i);
			}
			partSurface.push_back(allSurface);
		}
	}
	return partSurface;
}

void createModel()
{
	RWGeometric rwg;
	varray<Spline>outer;
	varray< varray<Spline>>inner;
	varray<Spline> S;
	varray<Spline> addLine;
	Boundary bo;
	Model_Solution m;
	varray<Spline> checkLine;
	Spline0 s0;
	vector<string> path;
	int n = 0;
	varray<SplineSurface>CompleteSurface;
	varray<varray<SplineSurface>>partSurface;
	PublicSolution ps;

	outer = bo.getSquare(100, 100);

	double l = 15;
	vector<int> moveX = { 0,1,-1,2,-2 };
	vector<int> moveY = { 0,1,-1,2,-2 };
	for (int i = 0; i < 5; i++)
	{
		for (int j = 0; j < 5; j++)
		{
			S = bo.getCircle(5);
			m.Rolate(S, PI / 4, 3);
			m.Trans(S, moveX[i] * l, 1);
			m.Trans(S, moveY[j] * l, 2);
			inner.push_back(S);
		}
	}
	S.clear();
	for (auto& i : outer)
	{
		S.push_back(i);
	}

	for (auto& i : inner)
	{
		for (auto& j : i)
		{
			S.push_back(j);
		}
	}


	rwg.ReadSpline("F:\\Learn\\大论文\\PlaneQuadModel\\baseLine.txt", S);

	Spline rolateLine = S[0];
	varray<Spline> moveLine;
	varray<Spline> temp;

	addLine.push_back(rolateLine);

	m.Rolate(rolateLine, PI / 2, 3);
	addLine.push_back(rolateLine);

	m.Rolate(rolateLine, PI / 2, 3);
	addLine.push_back(rolateLine);

	m.Rolate(rolateLine, PI / 2, 3);
	addLine.push_back(rolateLine);

	temp.push_back(S[1]);
	temp.push_back(S[2]);
	

	for (int i = 0; i < 4; ++i)
	{
		for (int j = 0; j < 5; ++j)
		{
			moveLine.clear();
			for (auto&k : temp)
			{
				moveLine.push_back(k);
			}
			m.Trans(moveLine, i*l, 1);
			m.Trans(moveLine, j*l, -2);
			for (auto&k : moveLine)
			{
				addLine.push_back(k);
			}
		}
	}

	moveLine.clear();
	moveLine.push_back(S[3]);
	varray<Spline> toRolateLine;
	for (int j = 0; j < 4; ++j)
	{
		for (auto&k : moveLine)
		{
			toRolateLine.push_back(k);
		}
		m.Trans(moveLine, l, -2);
	}

	for (auto&k : toRolateLine)
	{
		addLine.push_back(k);
	}
	m.Rolate(toRolateLine, PI, 3);
	for (auto&k : toRolateLine)
	{
		addLine.push_back(k);
	}

	rwg.WriteSpline("F:\\Learn\\大论文\\PlaneQuadModel\\allAddLine.txt", addLine);


	rwg.WriteSpline("F:\\Learn\\大论文\\PlaneQuadModel\\TianGangModelBoundaryLine.txt", S);
	varray<bool> genus;
	genus.resize(26);
	genus[0] = false;
	ps.quad(outer, inner, addLine, genus, CompleteSurface);

}

//三通管
void threeTube()
{
	//常用工具
	RWGeometric rwg;
	Boundary bo;
	Spline0 s0;
	Model_Solution ms;
	PublicSolution ps;
	Spline s;
	varray<Spline> S;
	varray< varray<Spline>> inner;
	varray<Spline> outer;
	varray<SplineSurface>SS;
	varray<SplineVolume> SV;
	varray<Spline> temp_S;
	varray<SplineSurface> temp_SS;
	varray<SplineVolume> temp_SV;
	//常用工具

	{
		rwg.ReadSplineSurface("F:\\Learn\\大论文\\Model\\复杂模型\\三通管\\3TubeSurface.txt", SS);
		for (auto&i : SS)
		{
			temp_SS.push_back(i);
		}
		ps.littler(temp_SS, 1.1, 2);
		SV = ps.loft(SS, temp_SS);

		rwg.WriteSplineVolume("F:\\Learn\\大论文\\Model\\复杂模型\\三通管\\SingleTubeVolume.txt", SV);

		rwg.WriteSplineSurface("F:\\Learn\\大论文\\Model\\复杂模型\\三通管\\3TubeSurface_little.txt", SS);

		rwg.WriteSplineVolume("F:\\Learn\\大论文\\Model\\复杂模型\\三通管\\3TubeVolume.txt", SV);

		/*ms.MirrorVols(SV, temp_SV, 3);

		for (auto&i : SV)
		{
			temp_SV.push_back(i);
		}*/
		rwg.WriteSplineVolume("F:\\Learn\\大论文\\Model\\复杂模型\\三通管\\SingleTubeVolume.txt", SV);

		temp_SV.clear();
		for (auto&i : SV)
		{
			temp_SV.push_back(i);
		}

		ms.Rolate(temp_SV, PI / 3 * 2, 3);
		for (auto&i : temp_SV)
		{
			SV.push_back(i);
		}

		ms.Rolate(temp_SV, PI / 3 * 2, 3);
		for (auto&i : temp_SV)
		{
			SV.push_back(i);
		}

		ms.MirrorVols(SV, temp_SV, 3);

		for (auto&i : temp_SV)
		{
			SV.push_back(i);
		}
		rwg.WriteSplineVolume("F:\\Learn\\大论文\\Model\\复杂模型\\三通管\\3TubeVolume.txt", SV);
	}

	{
		//调整雅可比值
		rwg.ReadSplineVolume("F:\\Learn\\大论文\\Model\\复杂模型\\三通管\\3TubeVolume.txt", SV);

		for (int i = SV.size() / 2; i < SV.size(); ++i)
		{
			SV[i].OrderCtrlPts(SV[i]);
		}
		putOutVTK(SV);
	}
}

//四通管
void fourTube()
{
	//常用工具
	RWGeometric rwg;
	Boundary bo;
	Spline0 s0;
	Model_Solution ms;
	PublicSolution ps;
	Spline s;
	varray<Spline> S;
	varray< varray<Spline>> inner;
	varray<Spline> outer;
	varray<SplineSurface>SS;
	varray<SplineVolume> SV;
	varray<Spline> temp_S;
	varray<SplineSurface> temp_SS;
	varray<SplineVolume> temp_SV;
	//常用工具

	
	rwg.ReadSplineSurface("F:\\Learn\\大论文\\Model\\复杂模型\\四通管\\FourTubeCutSingleSurfacce.txt", SS);
	for (auto&i : SS)
	{
		temp_SS.push_back(i);
	}
	ps.littler(temp_SS, 1.1, 2);
	SV = ps.loft(SS, temp_SS);
	for (auto&i : temp_SS)
	{
		SS.push_back(i);
	}

	rwg.WriteSplineSurface("F:\\Learn\\大论文\\Model\\复杂模型\\四通管\\FourTubeCut_little.txt", SS);

	rwg.WriteSplineVolume("F:\\Learn\\大论文\\Model\\复杂模型\\四通管\\FourTubeCutVolume.txt", SV);

	ms.MirrorVols(SV, temp_SV, 1);

	for (auto&i : temp_SV)
	{
		SV.push_back(i);
	}

	temp_SV.clear();
	for (auto&i : SV)
	{
		temp_SV.push_back(i);
	}
	
	ms.Rolate(temp_SV, PI / 2, 3);
	for (auto&i : temp_SV)
	{
		SV.push_back(i);
	}

	ms.Rolate(temp_SV, PI / 2, 3);
	for (auto&i : temp_SV)
	{
		SV.push_back(i);
	}

	ms.Rolate(temp_SV, PI / 2, 3);
	for (auto&i : temp_SV)
	{
		SV.push_back(i);
	}

	ms.MirrorVols(SV, temp_SV, 3);
	for (auto&i : temp_SV)
	{
		SV.push_back(i);
	}
	rwg.WriteSplineVolume("F:\\Learn\\大论文\\Model\\复杂模型\\四通管\\FourTubeCutVolume.txt", SV);

	/*ms.MirrorSufaces(SS, temp_SS, 2);
	for (auto&i : temp_SS)
	{
		SS.push_back(i);
	}
	rwg.WriteSplineSurface("F:\\Learn\\大论文\\Model\\复杂模型\\四通管\\4Tubesingleface3.txt", SS);


	varray<SplineSurface> allFace;
	for (auto&i : SS)
	{
		allFace.push_back(i);
	}

	ms.Rolate(SS, PI / 2, 3);
	for (auto&i : SS)
	{
		allFace.push_back(i);
	}
	rwg.WriteSplineSurface("F:\\Learn\\大论文\\Model\\复杂模型\\四通管\\4Tubesingleface4.txt", allFace);
	ms.MirrorSufaces(allFace, temp_SS, 3);

	for (auto&i : temp_SS)
	{
		allFace.push_back(i);
	}

	rwg.WriteSplineSurface("F:\\Learn\\大论文\\Model\\复杂模型\\四通管\\4Tubesingleface5.txt", allFace);

	temp_SS.clear();
	for (auto&i : allFace)
	{
		temp_SS.push_back(i);
	}

	rwg.WriteSplineSurface("F:\\Learn\\大论文\\Model\\复杂模型\\四通管\\4Tubesingleface_little.txt", allFace);
	for (auto&i : temp_SS)
	{
		allFace.push_back(i);
	}
	rwg.WriteSplineSurface("F:\\Learn\\大论文\\Model\\复杂模型\\四通管\\4Tubesingleface_conbination.txt", allFace);*/
	{
		temp_SV.clear();
		for (int i = 0; i < SV.size() / 9; ++i)
		{
			if (i < SV.size() / 18)
			{
				if (i % 2 == 0)
				{
					for (int j = 0; j < 9; ++j)
					{
						SV[i * 9 + j].OrderCtrlPts(SV[i * 9 + j]);
					}
				}
			}
			else
			{
				if (i % 2 == 1)
				{
					for (int j = 0; j < 9; ++j)
					{
						SV[i * 9 + j].OrderCtrlPts(SV[i * 9 + j]);
					}
				}
			}

			
			
		}
		putOutVTK(SV);
	}

	{

		varray<varray<SplineVolume>>Tube_explosion;
		
		for (int i = 0; i < 2; i++)
		{
			temp_SV.clear();
			for (int j = 0; j < 9; ++j)
			{
				temp_SV.push_back(SV[i * 9 + j]);
			}
			Tube_explosion.push_back(temp_SV);
		}
		ms.Trans(Tube_explosion[0], 3, -2);
		ms.Trans(Tube_explosion[0], 3, -1);
		ms.Trans(Tube_explosion[0], 3, 3);

		ms.Trans(Tube_explosion[1], 3, -2);
		ms.Trans(Tube_explosion[1], 3, 1);
		ms.Trans(Tube_explosion[1], 3, 3);

		SV.clear();
		for (auto &i : Tube_explosion)
		{
			for (auto &j : i)
			{
				SV.push_back(j);
				temp_SV.push_back(j);
			}
		}

		ms.Rolate(temp_SV, PI / 2, 3);
		for (auto&i : temp_SV)
		{
			SV.push_back(i);
		}



		/*Tube_explosion.resize(4);
		for (int i = 0; i < 2; ++i)
		{
			for (int j = 0; j < 4; j++)
			{
				for (int k = 0; k < 18; k++)
				{
					Tube_explosion[j].push_back(SV[i*SV.size() / 2 + j * 18 + k]);
				}
			}
		}

		ms.Trans(Tube_explosion[0], 3, -2);
		ms.Trans(Tube_explosion[1], 3, 1);
		ms.Trans(Tube_explosion[2], 3, 2);
		ms.Trans(Tube_explosion[3], 3, -1);
		temp_SV.clear();
		for (auto &i : Tube_explosion)
		{
			for (auto &j : i)
			{
				temp_SV.push_back(j);
			}
		}*/

		rwg.WriteSplineVolume("F:\\Learn\\大论文\\Model\\复杂模型\\四通管\\Tube_explosion.txt", SV);
	}
}

void T_Tube()
{

}
void example1()
{
	RWGeometric rwg;
	varray<Spline> S;
	varray<Spline> temp;
	varray<Spline>outer;
	varray<varray<Spline>>inner;
	rwg.ReadSpline("F:\\Learn\\大论文\\step\\示例\\resSpline.txt",S);
	for (int i = 0; i < 4; i++)
	{
		outer.push_back(S[i]);
	}
	for (int i = 4; i < S.size(); i++)
	{
		temp.push_back(S[i]);
	}
	inner.push_back(temp);
	for (int i = 0; i < outer.size(); i++)
	{
		temp.push_back(outer[i]);
	}
	CDT_Operate cdto(outer, inner);
	// 亏格设置
	varray<bool> genus;
	genus.resize(2);
	genus[0] = false;
	genus[1] = true;
	varray<SplineSurface> SS;
	PublicSolution ps;
	varray<Spline> addLines;
	for (int i = 0; i < cdto.addLine.size(); i++)
	{
		if (i == 1)
		{
			continue;
		}
		addLines.push_back(cdto.addLine[i]);
		temp.push_back(cdto.addLine[i]);
		rwg.WriteSpline("E:\\Model\\PlaneQuad\\Delaunay\\addLines.txt", temp);
	}
	ps.quad(outer, inner, addLines, genus, SS);
	//.quad_Lee(outer, inner, genus, SS);
	//ps.quad(outer, inner, genus, SS);
	rwg.WriteSplineSurface("F:\\Learn\\大论文\\step\\示例\\Cut1.txt", SS);
}
void example3()
{
	RWGeometric rwg;
	varray<SplineVolume> SV1;
	varray<SplineVolume> temp;
	rwg.ReadSplineVolume("E:\\Model\\YuYanModel\\GearShaftVolumeMidPart.txt", SV1);

}
void example4()
{
	RWGeometric rwg;
	varray<SplineSurface>SS1;
	varray<SplineSurface>SS2;
	varray<SplineVolume>SV;
	varray<Spline>S;
	varray<Spline>S1;
	SplineSurface surface;
	varray<SplineSurface>temp;
	PublicSolution ps;
	varray<SplineVolume>allVolume;
	varray<varray<SplineSurface>>SS3;
	//rwg.ReadSpline("F:\\Learn\\大论文\\Model\\bottle\\bottleLine1.txt", S1);
	//for (auto&i : S1)
	//{
	//	S.push_back(i);
	//}
	//rwg.ReadSpline("F:\\Learn\\大论文\\Model\\bottle\\bottleLine2.txt", S1);
	//for (auto&i : S1)
	//{
	//	S.push_back(i);
	//}
	//rwg.ReadSpline("F:\\Learn\\大论文\\Model\\bottle\\bottleLine3.txt", S1);
	//for (auto&i : S1)
	//{
	//	S.push_back(i);
	//}
	//rwg.WriteSpline("F:\\Learn\\大论文\\Model\\bottle\\bottleLine.txt", S);
	//rwg.ReadSplineSurface("F:\\Learn\\大论文\\Model\\bottle\\Surface.txt", SS1);
	//rwg.ReadSplineSurface("F:\\Learn\\大论文\\Model\\bottle\\littleSurface.txt", SS2);
	//SV = ps.loft(SS1, SS2);
	//rwg.WriteSplineVolume("F:\\Learn\\大论文\\Model\\bottle\\bottlePart.txt", SV);

	
	Model_Solution ms;
	//for (int i = 0; i < 4; ++i)
	//{
	//	ms.Rolate(SV, PI / 2, 1);
	//	for (auto &j : SV)
	//	{
	//		allVolume.push_back(j);
	//	}
	//}
	//
	//

	rwg.ReadSpline("F:\\Learn\\大论文\\Model\\bottle\\xyBottomLine.txt", S);
	varray<Spline>outer;
	
	varray<varray<Spline>>inner;
	inner.resize(2);
	for (int i = 0; i < 4; ++i)
	{
		inner[0].push_back(S[i]);
	}

	for (int i = 0; i < 4; ++i)
	{
		outer.push_back(S[i+4]);
	}

	for (int i = 0; i < 4; ++i)
	{
		inner[1].push_back(S[i + 8]);
	}
//	CDT_Operate cdto(outer, inner);
	// 亏格设置
	varray<bool> genus;
	genus.resize(2);
	genus[0] = false;
	genus[1] = false;
	varray<Spline> addLines;
	SS1.clear();
	//ps.quad(outer, inner, cdto.addLine, genus, SS1);
	SS3 = quadPlane(outer, inner);
	for (int i = 0; i < SS3.size(); ++i)
	{
		for (auto &j : SS3[i])
		{
			SS1.push_back(j);
		}
	}
	ms.Rolate(SS1, PI / 2, 2);
	ms.Trans(SS1, 5, 1);
	rwg.WriteSplineSurface("F:\\Learn\\大论文\\Model\\bottle\\bottomFace.txt", SS1);

	//SV = ms.CreatSweepVol(SS1, 0.1, 1);
	//for (auto&i : SV)
	//{
	//	allVolume.push_back(i);
	//}

	/*rwg.ReadSplineVolume("F:\\Learn\\大论文\\Model\\bottle\\allVolume.txt", allVolume);
	vector<int>index = { 2,5,8,11,12,13,14,15,16 };
	

	for (int i = 0;i<allVolume.size();++i)
	{
		allVolume[i].OrderCtrlPts(allVolume[i]);
	}
	for (auto &i : index)
	{
		allVolume[i].OrderCtrlPts(allVolume[i]);
	}
	rwg.WriteSplineVolume("E:\\Model\\Jacobian\\bottle.txt", allVolume);*/
}

void example5()
{
	RWGeometric rwg;
	Model_Solution ms;
	PublicSolution ps;
	
	varray<SplineSurface>SS1;
	varray<SplineSurface>SS2;
	varray<SplineSurface>allFace;
	varray<SplineVolume>SV;
	allFace = SS1;
	rwg.ReadSplineSurface("F:\\Learn\\大论文\\Model\\Revolve.txt", SS1);
	rwg.ReadSplineSurface("F:\\Learn\\大论文\\Model\\Revolve.txt", SS2);
	allFace = SS1;
	ps.littler(SS2, 1.2, 3);
	for (auto&i : SS2)
	{
		allFace.push_back(i);
	}
	rwg.WriteSplineSurface("F:\\Learn\\大论文\\Model\\allRevolve.txt", allFace);
	
	SV = ps.loft2(SS1, SS2);
	rwg.WriteSplineVolume("F:\\Learn\\大论文\\Model\\RevolveVolume.txt", SV);
}

//盘盖模型
void example6()
{
	vector<string> path;
	int n = 0;
	Boundary bo;
	varray<Spline> outer;
	varray<Spline> S;
	varray<varray<Spline>> inner;
	varray<SplineSurface>CompleteSurface;
	varray<varray<SplineSurface>>partSurface;
	// 亏格设置
	varray<bool> genus;
	outer = bo.getSquare(80, 80);
	path.push_back("F:\\Learn\\大论文\\PlaneQuadModel\\part0.txt");
	path.push_back("F:\\Learn\\大论文\\PlaneQuadModel\\part1.txt");
	path.push_back("F:\\Learn\\大论文\\PlaneQuadModel\\part2.txt");
	path.push_back("F:\\Learn\\大论文\\PlaneQuadModel\\part3.txt");
	path.push_back("F:\\Learn\\大论文\\PlaneQuadModel\\part4.txt");
	path.push_back("F:\\Learn\\大论文\\PlaneQuadModel\\part5.txt");
	path.push_back("F:\\Learn\\大论文\\PlaneQuadModel\\part6.txt");
	path.push_back("F:\\Learn\\大论文\\PlaneQuadModel\\part7.txt");
	path.push_back("F:\\Learn\\大论文\\PlaneQuadModel\\part8.txt");
	path.push_back("F:\\Learn\\大论文\\PlaneQuadModel\\part9.txt");
	path.push_back("F:\\Learn\\大论文\\PlaneQuadModel\\part10.txt");
	path.push_back("F:\\Learn\\大论文\\PlaneQuadModel\\part11.txt");
	path.push_back("F:\\Learn\\大论文\\PlaneQuadModel\\part12.txt");
	path.push_back("F:\\Learn\\大论文\\PlaneQuadModel\\part13.txt");
	path.push_back("F:\\Learn\\大论文\\PlaneQuadModel\\part14.txt");
	S = bo.getCircle(8);
	Model_Solution m;
	for (auto &i : S)
	{
		m.Rolate(i, PI / 4, 3);
	}

	m.Trans(S, 28, -1);
	m.Trans(S, 28, -2);
	inner.push_back(S);

	S = bo.getCircle(8);
	for (auto &i : S)
	{
		m.Rolate(i, PI / 4, 3);
	}

	m.Trans(S, 28, 1);
	m.Trans(S, 28, -2);
	inner.push_back(S);
	S = bo.getCircle(8);
	for (auto &i : S)
	{
		m.Rolate(i, PI / 4, 3);
	}

	m.Trans(S, 28, -1);
	m.Trans(S, 28, 2);
	inner.push_back(S);
	S = bo.getCircle(8);
	for (auto &i : S)
	{
		m.Rolate(i, PI / 4, 3);
	}

	m.Trans(S, 28, 1);
	m.Trans(S, 28, 2);
	inner.push_back(S);

	S = bo.getCircle(12);
	for (auto &i : S)
	{
		m.Rolate(i, PI / 4, 3);
	}
	inner.push_back(S);

	S = bo.getCircle(10);
	for (auto &i : S)
	{
		m.Rolate(i, PI / 4, 3);
	}
	inner.push_back(S);

	S = bo.getCircle(13);
	for (auto &i : S)
	{
		m.Rolate(i, PI / 4, 3);
	}
	inner.push_back(S);

	S = bo.getCircle(14);
	for (auto &i : S)
	{
		m.Rolate(i, PI / 4, 3);
	}
	inner.push_back(S);

	S = bo.getCircle(15);
	for (auto &i : S)
	{
		m.Rolate(i, PI / 4, 3);
	}
	inner.push_back(S);

	S = bo.getCircle(20);
	for (auto &i : S)
	{
		m.Rolate(i, PI / 4, 3);
	}
	inner.push_back(S);

	S = bo.getCircle(21);
	for (auto &i : S)
	{
		m.Rolate(i, PI / 4, 3);
	}
	inner.push_back(S);

	S = bo.getCircle(24);
	for (auto &i : S)
	{
		m.Rolate(i, PI / 4, 3);
	}
	//inner.push_back(S);

	varray<Spline> temp;
	for (const auto &i : outer)
	{
		temp.push_back(i);
	}

	for (const auto &i : inner)
	{
		for (const auto &j : i)
		{
			temp.push_back(j);
		}
	}


	RWGeometric rwg;
	rwg.WriteSpline("F:\\Learn\\大论文\\PlaneQuadModel\\boundary.txt", temp);
	//所有轮廓线
	varray<varray<Spline>> surf;

	//初始化surf
	surf.push_back(outer);
	for (auto &i : inner)
	{
		surf.push_back(i);
	}

	//所有边界线
	varray<Spline> allSpline;
	for (auto &i : surf)
	{
		for (auto &j : i)
		{
			allSpline.push_back(j);
		}
	}

	//辅助查看的数组
	varray<Spline> chack;

	//临时变量
	ContourData cd;

	//层次遍历所用的队列
	queue<SfCtainTreeNode *> Qnodes;

	//层次遍历后根遍历所用队列
	stack<SfCtainTreeNode *>Snode;
	PublicSolution ps;
	//初略构建几何域包含树
	SfCtainTreeNode * root = ps.CreateTree(surf);

	//层次遍历指针
	SfCtainTreeNode* cur = nullptr;

	//辅助工具
	varray<Spline> tempChackLine;

	Qnodes.push(root);
	//cur节点的子节点
	varray<SfCtainTreeNode *> allNode;
	varray<SfCtainTreeNode *> childnodes;

	//层次遍历几何域包含树，
	while (!Qnodes.empty())
	{
		cur = Qnodes.front();
		Qnodes.pop();

		//用栈记录后根遍历的顺序
		Snode.push(cur);

		//实现层次遍历
		for (auto it = cur->childs.begin(); it != cur->childs.end(); ++it)
		{
			Qnodes.push(*it);
		}

	}
	varray<Spline> addLine;
	//根据后根遍历顺序，实现由内至外创建连接线
	while (!Snode.empty())
	{
		cur = Snode.top();
		Snode.pop();
		if (cur->childs.empty())
		{
			continue;
		}
		else
		{
			varray<varray<Spline>>curInner;
			//以cur为外轮廓，子节点为内轮廓，创建连接线
			for (auto it = cur->childs.begin(); it != cur->childs.end(); ++it)
			{
				curInner.push_back((*it)->outLines);
			}

			//生成连接线
			CDT_Operate cdto(cur->outLines, curInner);

			varray<Spline> allLine;
			allLine = cur->outLines;
			for (auto&i : curInner)
			{
				for (auto&j : i)
				{
					allLine.push_back(j);
				}
			}
			rwg.WriteSpline("F:\\Learn\\大论文\\PlaneQuadModel\\boundary.txt", allLine);
			for (auto&i : cdto.addLine)
			{
				allLine.push_back(i);
			}
			rwg.WriteSpline("F:\\Learn\\大论文\\PlaneQuadModel\\addLine.txt", cdto.addLine);
			rwg.WriteSpline("F:\\Learn\\大论文\\PlaneQuadModel\\allLines.txt", allLine);
			//亏格设置
			genus.resize(curInner.size() + 1);
			genus[0] = false;
			for (int i = 1; i < genus.size(); ++i)
			{
				genus[i] = true;
			}
			varray<SplineSurface>allSurface;


			//平面剖分
			ps.quad(cur->outLines, curInner, cdto.addLine, genus, allSurface);
			for (auto&i : allSurface)
			{
				CompleteSurface.push_back(i);
			}
			partSurface.push_back(allSurface);
			rwg.WriteSplineSurface(path[n], allSurface);
			n++;
		}
	}
	rwg.WriteSplineSurface("F:\\Learn\\大论文\\PlaneQuadModel\\completeSurface.txt", CompleteSurface);

	vector<vector<int>>index = { {1,2} ,{0,1,2} ,{0,1}, {0,1,2,3,4,5,6}, {3,4,5}, {3,4} ,{3} };
	vector<int>move = { -13,-11,-6,0,10,11,14 };
	vector<int>thickness = { 2,5,6,10,1,3,3 };
	varray<varray<SplineSurface>>faceGroup;
	faceGroup.resize(index.size());
	varray<SplineVolume>allVolume;
	varray<SplineVolume> partVolume;
	for (int i = 0; i < index.size(); ++i)
	{
		for (auto&j : index[i])
		{
			for (auto&m : partSurface[j])
			{
				faceGroup[i].push_back(m);
			}
		}
		m.Trans(faceGroup[i], move[i], 3);
		partVolume = m.CreatSweepVol(faceGroup[i], thickness[i], 3);
		for (auto&k : partVolume)
		{
			allVolume.push_back(k);
		}
	}
	rwg.WriteSplineVolume("F:\\Learn\\大论文\\PlaneQuadModel\\allVolume.txt", allVolume);
}

//多亏格球
void example7()
{
	//常用工具
	RWGeometric rwg;
	Boundary bo;
	Spline0 s0;
	Model_Solution ms;
	PublicSolution ps;
	Spline s;
	varray<Spline> S;
	varray< varray<Spline>> inner;
	varray<Spline> outer;
	varray<SplineSurface>SS;
	varray<SplineVolume> SV;
	varray<Spline> temp_S;
	varray<SplineSurface> temp_SS;
	varray<SplineVolume> temp_SV;
	//常用工具
	varray<SplineSurface>SS1;
	varray<SplineSurface>SS2;
	rwg.ReadSplineSurface("F:\\Learn\\大论文\\Model\\多亏格球\\ballSurface.txt", SS1);
	rwg.ReadSplineSurface("F:\\Learn\\大论文\\Model\\多亏格球\\ballSurface.txt", SS2);
	
	ps.littler(SS2, 1.1,0);
	for (auto&i : SS1)
	{
		temp_SS.push_back(i);
	}
	for (auto&i : SS2)
	{
		temp_SS.push_back(i);
	}

	rwg.WriteSplineSurface("F:\\Learn\\大论文\\Model\\多亏格球\\combinationFace.txt", temp_SS);

	SV = ps.loft(SS1, SS2);
	rwg.WriteSplineVolume("F:\\Learn\\大论文\\Model\\多亏格球\\ballVolume.txt", SV);
	
	/*for (int i = 0; i < SV.size(); ++i)
	{
		if (i == 4 || i == 13)
		{
			continue;
		}
		temp_SV.push_back(SV[i]);
	}*/

	//SV = temp_SV;
	ms.MirrorVols(SV, temp_SV,3);

	for (auto&i : temp_SV)
	{
		SV.push_back(i);
	}

	for (int i = SV.size() / 2; i < SV.size(); ++i)
	{
		SV[i].OrderCtrlPts(SV[i]);
	}
	rwg.WriteSplineVolume("F:\\Learn\\大论文\\Model\\多亏格球\\ballVolume.txt", SV);

	putOutVTK(SV);
}

//U型管
void example9()
{
	//常用工具
	RWGeometric rwg;
	Boundary bo;
	Spline0 s0;
	Model_Solution ms;
	PublicSolution ps;
	Spline s;
	varray<Spline> S;
	varray<SplineSurface>SS;
	varray<SplineVolume> SV;
	varray<Spline> temp_S;
	varray<SplineSurface> temp_SS;
	varray<SplineVolume> temp_SV;
	//常用工具
	varray<Spline> outer;
	varray<varray<Spline>> inner;
	varray<varray<SplineSurface>>partSurface;
	double d = 10;
	double t = 1;
	double r = 10;
	outer = bo.getCircle(r);
	ms.Trans(outer, d + r, -1);

	S = bo.getCircle(r - t);
	ms.Trans(S, d + r, -1);
	inner.push_back(S);
	partSurface = quadPlane(outer, inner);

	rwg.WriteSplineSurface("F:\\Learn\\大论文\\Model\\管状模型\\截面.txt", partSurface[0]);
	/*for (auto&i : partSurface)
	{
		for (auto &j : i)
		{
			SS.push_back(j);
		}
	}
	rwg.ReadSplineVolume("F:\\Learn\\大论文\\Model\\管状模型\\Volume.txt", SV);

	temp_SV = ms.CreatSweepVol(SS, 4 * r, -3);

	for (auto&i : temp_SV)
	{
		SV.push_back(i);
	}
	temp_SV.clear();
	for (auto&i : SV)
	{
		temp_SV.push_back(i);
	}

	ms.Rolate(temp_SV, PI, 3);

	for (auto&i : temp_SV)
	{
		SV.push_back(i);
	}
	rwg.WriteSplineVolume("F:\\Learn\\大论文\\Model\\管状模型\\allVolume.txt", SV);

	{
		temp_SS = partSurface[0];

		ms.Rolate(temp_SS, PI / 4, 2);
		partSurface.push_back(temp_SS);



		ms.Rolate(temp_SS, PI / 4, 2);
		partSurface.push_back(temp_SS);
		temp_SS.clear();
	}*/
	
	{
		//调整雅可比值
		rwg.ReadSplineVolume("F:\\Learn\\大论文\\Model\\管状模型\\U_tube.txt", SV);

		vector<int> vec = { 0,1,2,3,8,9,10,11 };
		for (auto &i : vec)
		{
			SV[i].OrderCtrlPts(SV[i]);
		}
		putOutVTK(SV);
	}

	temp_SV.clear();
	for (int i = 0; i < 4; ++i)
	{
		temp_SV.push_back(SV[i]);
	}
	rwg.WriteSplineVolume("F:\\Learn\\大论文\\Model\\管状模型\\straightPart.txt", temp_SV);

	temp_SV.clear();
	for (int i = 4; i < 8; ++i)
	{
		temp_SV.push_back(SV[i]);
	}
	rwg.WriteSplineVolume("F:\\Learn\\大论文\\Model\\管状模型\\circlePart.txt", temp_SV);
}

//轴承座支架
void example8()
{
	RWGeometric rwg;
	varray<Spline>outer;
	varray< varray<Spline>>inner;
	varray<Spline> S;
	Boundary bo;
	Model_Solution m;
	varray<Spline> checkLine;
	Spline0 s0;
	vector<string> path;
	int n = 0;
	varray<SplineSurface>CompleteSurface;
	varray<varray<SplineSurface>>partSurface;
	varray<SplineVolume> SV;
	// 亏格设置
	varray<bool> genus;

	//Vec4 p1 = { -90,45,0,1 };
	//Vec4 p2 = { 90,45,0,1 };
	//outer.push_back(s0.getSpline(p1, p2));
	//Vec4 p3 = { 100,35,0,1 };
	//Vec4 p4 = { 100,-35,0,1 };
	//outer.push_back(s0.getSpline(p3, p4));
	//Vec4 p5 = { 90,-45,0,1 };
	//Vec4 p6 = { -90,-45,0,1 };
	//outer.push_back(s0.getSpline(p5, p6));
	//Vec4 p7 = { -100,-35,0,1 };
	//Vec4 p8 = { -100,35,0,1 };
	//outer.push_back(s0.getSpline(p7, p8));
	//
	//S = bo.getCircle(10);
	//m.Trans(S[0], 90, -1);
	//m.Trans(S[0], 35, 2);

	//m.Trans(S[1], 90, 1);
	//m.Trans(S[1], 35, 2);

	//m.Trans(S[2], 90, 1);
	//m.Trans(S[2], 35, -2);

	//m.Trans(S[3], 90, -1);
	//m.Trans(S[3], 35, -2);

	//for (auto&i : S)
	//{
	//	outer.push_back(i);
	//}

	//S = bo.getSquare(90, 18);
	//m.Trans(S, 26, -2);
	//inner.push_back(S);

	///*S = bo.getSquare(88, 16);
	//m.Trans(S, 25, -2);
	//inner.push_back(S);*/

	//S = bo.getSquare(90, 18);
	//m.Trans(S, 26, 2);
	//inner.push_back(S);

	///*S = bo.getSquare(88, 16);
	//m.Trans(S, 25, 2);
	//inner.push_back(S);*/


	//S = bo.getCircle(5);
	//m.Trans(S, 90, 1);
	//m.Trans(S, 35, 2);
	//inner.push_back(S);

	//S = bo.getCircle(5);
	//m.Trans(S, 90, -1);
	//m.Trans(S, 35, 2);
	//inner.push_back(S);

	//S = bo.getCircle(5);
	//m.Trans(S, 90, 1);
	//m.Trans(S, 35, -2);
	//inner.push_back(S);

	//S = bo.getCircle(5);
	//m.Trans(S, 90, -1);
	//m.Trans(S, 35, -2);
	//inner.push_back(S);

	//for (auto&i : inner)
	//{
	//	for (auto&j : i)
	//	{
	//		checkLine.push_back(j);
	//	}
	//}
	//for (auto&i : outer)
	//{
	//	checkLine.push_back(i);
	//}
	//rwg.WriteSpline("F:\\Learn\\大论文\\Model\\轴承座支架\\checkLine.txt", checkLine);
	////所有轮廓线
	//varray<varray<Spline>> surf;

	////初始化surf
	//surf.push_back(outer);
	//for (auto &i : inner)
	//{
	//	surf.push_back(i);
	//}

	////所有边界线
	//varray<Spline> allSpline;
	//for (auto &i : surf)
	//{
	//	for (auto &j : i)
	//	{
	//		allSpline.push_back(j);
	//	}
	//}

	////辅助查看的数组
	//varray<Spline> chack;

	////临时变量
	//ContourData cd;

	////层次遍历所用的队列
	//queue<SfCtainTreeNode *> Qnodes;

	////层次遍历后根遍历所用队列
	//stack<SfCtainTreeNode *>Snode;
	//PublicSolution ps;
	////初略构建几何域包含树
	//SfCtainTreeNode * root = ps.CreateTree(surf);

	////层次遍历指针
	//SfCtainTreeNode* cur = nullptr;

	////辅助工具
	//varray<Spline> tempChackLine;

	//Qnodes.push(root);
	////cur节点的子节点
	//varray<SfCtainTreeNode *> allNode;
	//varray<SfCtainTreeNode *> childnodes;

	////层次遍历几何域包含树，
	//while (!Qnodes.empty())
	//{
	//	cur = Qnodes.front();
	//	Qnodes.pop();

	//	//用栈记录后根遍历的顺序
	//	Snode.push(cur);

	//	//实现层次遍历
	//	for (auto it = cur->childs.begin(); it != cur->childs.end(); ++it)
	//	{
	//		Qnodes.push(*it);
	//	}

	//}
	//varray<Spline> addLine;
	////根据后根遍历顺序，实现由内至外创建连接线
	//while (!Snode.empty())
	//{
	//	cur = Snode.top();
	//	Snode.pop();
	//	if (cur->childs.empty())
	//	{
	//		continue;
	//	}
	//	else
	//	{
	//		varray<varray<Spline>>curInner;
	//		//以cur为外轮廓，子节点为内轮廓，创建连接线
	//		for (auto it = cur->childs.begin(); it != cur->childs.end(); ++it)
	//		{
	//			curInner.push_back((*it)->outLines);
	//		}

	//		//生成连接线
	//		CDT_Operate cdto(cur->outLines, curInner);

	//		varray<Spline> allLine;
	//		allLine = cur->outLines;
	//		for (auto&i : curInner)
	//		{
	//			for (auto&j : i)
	//			{
	//				allLine.push_back(j);
	//			}
	//		}
	//		rwg.WriteSpline("F:\\Learn\\大论文\\PlaneQuadModel\\boundary.txt", allLine);
	//		for (auto&i : cdto.addLine)
	//		{
	//			allLine.push_back(i);
	//		}
	//		rwg.WriteSpline("F:\\Learn\\大论文\\PlaneQuadModel\\addLine.txt", cdto.addLine);
	//		rwg.WriteSpline("F:\\Learn\\大论文\\PlaneQuadModel\\allLines.txt", allLine);
	//		//亏格设置
	//		genus.resize(curInner.size() + 1);
	//		genus[0] = false;
	//		for (int i = 1; i < genus.size(); ++i)
	//		{
	//			genus[i] = true;
	//		}
	//		varray<SplineSurface>allSurface;
	//		varray<Spline> addLine;
	//		rwg.ReadSpline("F:\\Learn\\大论文\\Model\\轴承座支架\\baseLine.txt", addLine);
	//		//平面剖分
	//		ps.quad(cur->outLines, curInner, addLine, genus, allSurface);
	//		for (auto&i : allSurface)
	//		{
	//			CompleteSurface.push_back(i);
	//		}
	//		partSurface.push_back(allSurface);
	//		string filename = "F:\\Learn\\大论文\\Model\\轴承座支架" + std::to_string(n) + ".txt";
	//		rwg.WriteSplineSurface(filename, allSurface);
	//		n++;
	//	}
	//}
	//rwg.WriteSplineSurface("F:\\Learn\\大论文\\PlaneQuadModel\\completeSurface.txt", CompleteSurface);
	//SV = m.CreatSweepVol(CompleteSurface, 1, 3);
	//rwg.WriteSplineVolume("F:\\Learn\\大论文\\PlaneQuadModel\\Volume.txt", SV);
	rwg.ReadSplineVolume("F:\\Learn\\大论文\\PlaneQuadModel\\Volume.txt", SV);
	varray<SplineVolume> tempSV;
	
	for (int i = 0; i < SV.size(); ++i)
	{
		
		SV[i].OrderCtrlPts(SV[i]);
	}

	for (int i = 0; i < SV.size(); ++i)
	{
		if (i == 28 || i == 29 || i == 32 || i == 34)
		{
			SV[i].OrderCtrlPts(SV[i]);
			SV[i].OrderCtrlPts(SV[i]);
			continue;
		}
		tempSV.push_back(SV[i]);
	}
	putOutVTK(tempSV);

	rwg.ReadSpline("F:\\Learn\\大论文\\Model\\轴承座支架\\baseLine.txt", S);
	for (auto&i : S)
	{
		checkLine.push_back(i);
	}
	rwg.ReadSpline("F:\\Learn\\大论文\\Model\\轴承座支架\\checkLine.txt", S);
	for (auto&i : S)
	{
		checkLine.push_back(i);
	}
	rwg.WriteSpline("F:\\Learn\\大论文\\Model\\轴承座支架\\连接线.txt", checkLine);
}
void checkModelQuality6()
{
	RWGeometric rwg;
	varray<SplineVolume>Flange;
	rwg.ReadSplineVolume("F:\\Learn\\大论文\\PlaneQuadModel\\allVolume.txt", Flange);
	vector<int>index = { 2,5,8,11,12,13,14,15,16 };


	for (int i = 0; i < Flange.size(); ++i)
	{
		//Flange[i].OrderCtrlPts(Flange[i]);
	}
	for (auto &i : index)
	{
		//Flange[i].OrderCtrlPts(Flange[i]);
	}
	rwg.WriteSplineVolume("E:\\Model\\Jacobian\\Flange.txt", Flange);
}
void testProjectPointToMesh()
{
	projectPointToMesh pptm("E:\\Model\\SurfaceQuad\\完整OBJ\\ThreeHole.obj");
}
void PaPerModel_BSplineSurface()
{
	PublicSolution ps;
	Boundary bo;
	RWGeometric rwg;
	Model_Solution ms;
	double r = 10;
	Vec4 o = { 0,0,0,1 };
	Vec4 p1 = { -cos(PI / 4)*r,sin(PI / 4)*r,0,1 };
	Vec4 p2 = { cos(PI / 4)*r,sin(PI / 4)*r,0,1 };
	Spline0 s0;
	varray<Spline> S;
	varray<Spline> temp;
	Spline s = s0.getArcSpline(p1, p2, o);
	ms.Trans(s, sin(PI / 4)*r, 3);
	S.push_back(s);
	ms.Rolate(s, PI / 2, 2);
	S.push_back(s);

	ms.Rolate(s, PI / 2, 2);
	S.push_back(s);

	ms.Rolate(s, PI / 2, 2);
	S.push_back(s);
	ps.orderEdgeAntiClock0(S);
	SplineSurface ss;
	temp.push_back(S[0]);
	temp.push_back(S[3]);
	temp.push_back(S[2]);
	temp.push_back(S[1]);
	ps.sortEdg(S);
	ss.CoonsInterpolate(S);
	varray<SplineSurface>SS;
	SS.push_back(ss);
	rwg.WriteSpline("F:\\Learn\\大论文\\Model\\BSpline.txt", S);
	varray<double> u;
	varray<double> v;
	u.resize(2);
	v.resize(2);
	for (auto &i : SS) {
		u.clear();
		v.clear();
		u.push_back(0.5);
		u.push_back(0.5);
		i.KnotsRefine(u, v);
	}
	//底面
	varray<Spline> bottom = bo.getSquare(cos(PI / 4) * r * 2, cos(PI / 4) * r * 2);
	varray<SplineSurface> bottomFace;
	ps.sortEdg(bottom);
	ss.CoonsInterpolate(bottom);
	ms.Rolate(ss, PI / 2, 1);
	SS.push_back(ss);
	//放样路径
	Vec4 nl1 = { cos(PI / 4)*r,sin(PI / 4)*r,cos(PI / 4)*r,1 };
	Vec4 nl2 = { cos(PI / 4)*r,cos(PI / 4)*r,0,1 };
	//放样体
	Spline NL = s0.getSpline(nl1, nl2);
	NurbsLine nl;
	varray<NurbsSurface> ns;
	varray<NurbsSurface> ns2;
	NurbsVol nv;
	NurbsVol nv2;
	nl = NurbsTrans::SplineToCnurbsline(NL);
	ns = NurbsTrans::SplinesurfsToCsurfs(SS);
	nv.LoftingNurbsVol(nl, ns[0], ns[1]);
	SplineVolume sv;
	sv = NurbsTrans::CnurbsvolToSplinevol(nv);
	varray<SplineVolume>SV;
	SV.push_back(sv);
	//图2.2 B样条曲面
	rwg.WriteSplineSurface("F:\\Learn\\大论文\\Model\\BSplineSurface.txt", SS);

	//图2.3 B样条体
	rwg.WriteSplineVolume("F:\\Learn\\大论文\\Model\\BSplineVolume.txt", SV);


}

void PaPerModel_nurbslineSurface()
{
	Boundary bo;
	double r = 10;
	double w = cos(PI / 4);
	Spline line;
	Spline0 s0;
	varray<Spline> SL;
	varray<double> knots;
	knots.push_back(0);
	knots.push_back(0);
	knots.push_back(0);
	knots.push_back(0.25);
	knots.push_back(0.25);
	knots.push_back(0.5);
	knots.push_back(0.5);
	knots.push_back(0.75);
	knots.push_back(0.75);
	knots.push_back(1);
	knots.push_back(1);
	knots.push_back(1);

	Vec4 p01 = { -2 * r,0,0,1 };
	Vec4 p02 = { -2 * r,-r,0,w };
	Vec4 p03 = { -r,-r,0,1 };
	Vec4 p04 = { 0,-r,0,w };
	Vec4 p05 = { 0,0,0,1 };
	Vec4 p06 = { 0,r,0,w };
	Vec4 p07 = { r,r,0,1 };
	Vec4 p08 = { r*2,r,0,w };
	Vec4 p09 = { 2*r,0,0,1 };
	line.m_Degree = 2;
	line.m_Knots = knots;
	line.m_CtrlPts.push_back(p01);
	line.m_CtrlPts.push_back(p02);
	line.m_CtrlPts.push_back(p03);
	line.m_CtrlPts.push_back(p04);
	line.m_CtrlPts.push_back(p05);
	line.m_CtrlPts.push_back(p06);
	line.m_CtrlPts.push_back(p07);
	line.m_CtrlPts.push_back(p08);
	line.m_CtrlPts.push_back(p09);
	//SL = bo.getCircle(r);
	SL.push_back(line);

	Model_Solution ms;
	ms.Trans(line, r * 2, 3);
	SL.push_back(line);

	RWGeometric rwg;
	

	Vec4 v1 = { -2 * r,0,2 * r,1 };
	Vec4 v2 = { 2 * r,0,2 * r,1 };
	line = s0.getSpline(v1, p01);
	SL.push_back(line);

	line = s0.getSpline(v2, p09);
	SL.push_back(line);
	rwg.WriteSpline("F:\\Learn\\大论文\\Model\\NURBSLine.txt", SL);
	PublicSolution ps;
	ps.sortEdg(SL);
	SplineSurface ss;
	ss.CoonsInterpolate(SL);
	varray<SplineSurface>SS;
	SS.push_back(ss);
	rwg.WriteSplineSurface("F:\\Learn\\大论文\\Model\\NURBSSurface.txt", SS);
}
//测试函数
void test(varray<SplineSurface> SS)
{
	RWGeometric rwg;
	Model_Solution m;
	varray<SplineVolume> SV;
	SV = m.CreatSweepVol(SS, 0.001, 3);
	for (int i = 0; i < SV.size(); i++)
	{
		SV[i].OrderCtrlPts(SV[i]);
	}
	rwg.WriteSplineVolume("E:\\Model\\Jacobian\\Volume.txt", SV);
	putOutVTK();
}

void test01() {
	RWGeometric rwg;
	Model_Solution m;
	varray<Spline> temp;
	varray<Spline> S;
	Spline s;
	Spline0 s0;
	Boundary bo;
	varray<Spline> outer;
	varray<Spline> addLine;
	varray<varray<Spline>>inner;

	int n = 0;
	Vec4 p1 = { -76,28,0,1 };
	Vec4 p2 = { -56,28,0,1 };
	Vec4 p3 = { -0,28,0,1 };
	Vec4 p4 = { 31,28,0,1 };
	Vec4 p5 = { 60,28,0,1 };
	Vec4 p6 = { -76,11,0,1 };
	Vec4 p7 = { -60,13,0,1 };
	Vec4 p8 = { 0,-2,0,1 };
	Vec4 p9 = { 31,0,0,1 };
	Vec4 p10 = { 60,0,0,1 };
	Vec4 p11 = { -76,11,0,1 };
	Vec4 p12 = { -48.23,-3.28,0,1 };
	Vec4 p13 = { -17.76,-9.62,0,1 };
	Vec4 p14 = { 0,-2,0,1 };
	Vec4 p15 = { -76,-29,0,1 };
	Vec4 p16 = { -61,-30,0,1 };
	Vec4 p17 = { -76,-48,0,1 };
	Vec4 p18 = { -60,-48,0,1 };
	Vec4 p19 = { -22.32,-28.64,0,1 };
	Vec4 p20 = { 0,-48,0,1 };
	Vec4 p21 = { 31,-48,0,1 };
	Vec4 p22 = { 60,-48,0,1 };
	Vec4 p23 = { -52.23,-20.9,0,1 };

	Vec4 O = { -34.89,-15.57,0,1 };

	S.push_back(s0.getSpline(p1, p2));
	S.push_back(s0.getSpline(p3, p2));
	/*S.push_back(s0.getSpline(p3, p4));
	S.push_back(s0.getSpline(p4, p5));
	S.push_back(s0.getSpline(p5, p10));
	S.push_back(s0.getSpline(p10, p22));
	S.push_back(s0.getSpline(p22, p21));
	S.push_back(s0.getSpline(p21, p20));*/
	S.push_back(s0.getSpline(p20, p18));
	S.push_back(s0.getSpline(p18, p17));
	S.push_back(s0.getSpline(p17, p15));
	S.push_back(s0.getSpline(p15, p6));
	S.push_back(s0.getSpline(p6, p1));
	S.push_back(s0.getSpline(p3, p20));
	outer = S;

	temp = outer;

	S.clear();
	S.push_back(s0.getArcSpline(p23, p12, O));
	S.push_back(s0.getArcSpline(p12, p13, O));
	S.push_back(s0.getArcSpline(p13, p19, O));
	S.push_back(s0.getArcSpline(p19, p23, O));

	inner.push_back(S);

	for (const auto &i : S)
	{
		temp.push_back(i);
	}

	varray<Spline> extraAddLine;
	extraAddLine.push_back(s0.getSpline(p6, p7));
	extraAddLine.push_back(s0.getSpline(p2, p7));
	extraAddLine.push_back(s0.getSpline(p12, p7));
	extraAddLine.push_back(s0.getSpline(p15, p16));
	extraAddLine.push_back(s0.getSpline(p16, p18));
	extraAddLine.push_back(s0.getSpline(p16, p23));
	/*extraAddLine.push_back(s0.getSpline(p13, p14));
	extraAddLine.push_back(s0.getSpline(p14, p3));
	extraAddLine.push_back(s0.getSpline(p14, p20));
	extraAddLine.push_back(s0.getSpline(p4, p9));
	extraAddLine.push_back(s0.getSpline(p21, p9));
	extraAddLine.push_back(s0.getSpline(p9, p10));
	extraAddLine.push_back(s0.getSpline(p13, p10));*/

	for (const auto &i : extraAddLine)
	{
		temp.push_back(i);
	}
	rwg.WriteSpline("E:\\Model\\PlaneQuad\\test_CDT.txt", temp);
	CreatePolygon creatPoly(outer, extraAddLine, inner);
	for (const auto &polygon : creatPoly.allPolygon)
	{
		rwg.WriteSpline("E:\\Model\\PlaneQuad\\Delaunay\\Polygon.txt", polygon.p_NurbsLines);
		system("pause");
	}
}

void test02()
{
	Boundary bo;
	varray<Spline> outer;
	varray<Spline> S;
	varray<varray<Spline>> inner;
	outer = bo.getSquare(40, 40);

	S = bo.getCircle(4);
	Model_Solution m;
	for (auto &i : S)
	{
		m.Rolate(i, PI / 4, 3);
	}

	m.Trans(S, 14, -1);
	m.Trans(S, 14, -2);
	inner.push_back(S);

	S = bo.getCircle(4);
	for (auto &i : S)
	{
		m.Rolate(i, PI / 4, 3);
	}

	m.Trans(S, 14, 1);
	m.Trans(S, 14, -2);
	inner.push_back(S);
	S = bo.getCircle(4);
	for (auto &i : S)
	{
		m.Rolate(i, PI / 4, 3);
	}

	m.Trans(S, 14, -1);
	m.Trans(S, 14, 2);
	inner.push_back(S);
	S = bo.getCircle(4);
	for (auto &i : S)
	{
		m.Rolate(i, PI / 4, 3);
	}

	m.Trans(S, 14, 1);
	m.Trans(S, 14, 2);
	inner.push_back(S);

	S = bo.getCircle(10);
	for (auto &i : S)
	{
		m.Rolate(i, PI / 4, 3);
	}
	inner.push_back(S);
	varray<Spline> temp;
	for (const auto &i : outer)
	{
		temp.push_back(i);
	}

	for (const auto &i : inner)
	{
		for (const auto &j : i)
		{
			temp.push_back(j);
		}
	}

	RWGeometric rwg;
	rwg.WriteSpline("E:\\Model\\PlaneQuad\\Delaunay\\Boundary.txt", temp);
	CDT_Operate cdto(outer, inner);

	rwg.ReadSpline("E:\\Model\\PlaneQuad\\Delaunay\\AddLines.txt", temp);

	for (const auto &i : outer)
	{
		temp.push_back(i);
	}
	for (const auto &i : inner)
	{
		for (const auto &j : i)
		{
			temp.push_back(j);
		}
	}
	rwg.WriteSpline("E:\\Model\\PlaneQuad\\AllLinesAfterRefine.txt", temp);
	PublicSolution ps;
	varray<SplineSurface> allSurf;
	//记录每个轮廓的可分割性，0为不可分，1为可分割
	varray<varray<int>>seg;
	//默认设置为都可分割
	seg.push_back(varray<int>(outer.size(), 1));
	for (int i = 0; i < inner.size(); i++)
	{
		seg.push_back(varray<int>(inner[i].size(), 1));
	}
	// 亏格设置
	varray<bool> genus;
	genus.resize(6);
	genus[0] = false;
	genus[1] = true;
	genus[2] = true;
	genus[3] = true;
	genus[4] = true;
	genus[5] = true;
	//ps.quad_Lee(outer, inner, genus, allSurf);
	Graph g(outer, inner, cdto.addLine);
	varray<varray<Spline>> all_area = g.get_all_area();
	ps.quad(outer, inner, cdto.addLine, genus, allSurf); 
	rwg.WriteSplineSurface("E:\\Model\\PlaneQuad\\QuadSurface\\SquareWithFiveHoleSurface.txt", allSurf);
	varray<SplineVolume> SV = m.CreatSweepVol(allSurf, 5, 3);
	rwg.WriteSplineVolume("E:\\Model\\PlaneQuad\\QuadSurface\\SquareWithFiveHoleVolume.txt", SV);
}

void test03() {
	PublicSolution ps;
	ps.quadSurface();
}
void test04() {
	PublicSolution ps;
	double a = 5;
	double b = 5;
	double c = 7;
	Vec4 p1 = { -a,b,0,1 };
	Vec4 p2 = { a,b,0,1 };
	Vec4 p3 = { a,-b,0,1 };
	Vec4 p4 = { -a,-b,0,1 };
	Vec4 p5 = { c,0,0,1 };

	Spline0 s0;
	varray<Spline> outer;
	varray<varray<Spline>>inner;
	outer.push_back(s0.getSpline(p1, p5));
	outer.push_back(s0.getSpline(p2, p5));
	outer.push_back(s0.getSpline(p2, p3));
	outer.push_back(s0.getSpline(p3, p4));
	outer.push_back(s0.getSpline(p4, p1));

	varray<SplineSurface> allSurf;
	// 亏格设置
	varray<bool> genus;
	genus.resize(1);
	genus[0] = false;
	ps.quad(outer, inner, genus, allSurf);
	RWGeometric rwg;
	rwg.WriteSplineSurface("E:\\Model\\PlaneQuad\\allSurface.txt", allSurf);
}

/**
* @brief   : 测试nurbs曲线中间点的角度关系
* @param[I]: none
* @param[O]: none
* @return  : none
* @note    :
**/
void test05()
{
	RWGeometric rwg;
	Model_Solution m;
	varray<Spline> S;
	Vec4 point;
	Spline s1;
	Spline s2;
	rwg.ReadSpline("E:\\Model\\ModelTest\\ResultSpline.txt", S);
	s1 = S[0];
	s2 = S[4];

	m.Rolate(s2, -PI / 60, 3);

	S.clear();
	S.push_back(s1);
	S.push_back(s2);

	for (auto &i : S)
	{
		m.Rolate(i, PI / 180 * 27 + PI, 3);
	}

	m.Trans(S, -4.4, 1);
	m.Trans(S, 2.45, 2);

	point = S[1].m_CtrlPts[2];

	ResetSpline dcp(S[0]);
	dcp.showData();

	varray<Spline> temp = dcp.cutSpline(point);
	for (auto &i : temp)
	{
		S.push_back(i);
	}

	rwg.WriteSpline("E:\\Model\\ModelTest\\TestSpline.txt", S);
}

void test06()
{
	double x = 1, y = 2, z = 3;
	double r = 10;
	Vec4 v1 = { -r,0,0,1 };
	Vec4 v3 = { 0,r,0,1 };
	Vec4 O = { x,y,z,1 };

	varray<Spline> S;

	Spline0 s0;
	Spline s = s0.getArcSpline(r, PI / 2, v1, v3);
	S.push_back(s);

	Model_Solution m;

	m.Rolate(s, PI / 4, 1);
	S.push_back(s);

	m.Rolate(s, PI / 4, 2);
	S.push_back(s);

	m.Rolate(s, PI / 4, 3);
	S.push_back(s);

	m.Trans(s, x, 1);
	S.push_back(s);

	m.Trans(s, y, 2);
	S.push_back(s);

	m.Trans(s, z, 3);
	S.push_back(s);

	v1 = s.m_CtrlPts[0];
	v3 = s.m_CtrlPts[2];

	S.push_back(s0.getSpline(v1, O));
	S.push_back(s0.getSpline(v3, O));

	v1 = s.m_CtrlPts[0];
	v3 = s.m_CtrlPts[2];
	Vec4 v2 = s.m_CtrlPts[1];

	Vec4 s1 = v1 - O;

	cout << "v2 = { " << v2.x << ", " << v2.y << ", " << v2.z << ", " << v2.w << " }" << endl;

	ResetSpline dcp(s);
	S.push_back(dcp.getArcSpline(v1, v3, O));
	RWGeometric rwg;
	rwg.WriteSpline("E:\\Model\\ModelTest\\TestSpline.txt", S);
}

//平面剖分测试
void test07()
{
	varray<Spline> S;
	varray<Spline> S1;
	varray<Spline> S2;
	varray<Spline> S3;
	varray<Spline> S4;

	varray<Spline>allSpline;
	double d = 2;
	Spline s;
	Spline0 s0;
	Model_Solution m;
	RWGeometric rwg;
	double r = 2;
	double h1 = 6;
	double h2 = 20;
	double l1 = 3;
	double l2 = 9;
	double l3 = 20;

	//外轮廓
	Vec4 v1 = { -l3,h2 + d,0,1 };
	Vec4 v2 = { l3,h2 + d,0,1 };
	s = s0.getSpline(v1, v2);
	S.push_back(s);
	m.Trans(s, h2 * 2 + d * 2, -2);
	S.push_back(s);

	v1 = { -l3,h2 + d,0,1 };
	v2 = { -l3,-h2 - d,0,1 };
	s = s0.getSpline(v1, v2);
	S.push_back(s);
	m.Trans(s, l3 * 2, 1);
	S.push_back(s);

	//内轮廓1
	v1 = { -l1,h2,0,1 };
	v2 = { l1,h2,0,1 };

	s = s0.getSpline(v1, v2);
	S1.push_back(s);
	m.Trans(s, h2, -2);
	S1.push_back(s);

	Vec4 v3 = { -l1,h2,0,1 };
	Vec4 v4 = { -l1,0,0,1 };

	s = s0.getSpline(v3, v4);

	S1.push_back(s);
	m.Trans(s, l1 * 2, 1);
	S1.push_back(s);

	m.Trans(S1, h1 + 1, -2);

	//内轮廓2
	v1 = { -l2,h2,0,1 };
	v2 = { l2,h2,0,1 };

	s = s0.getSpline(v1, v2);
	S2.push_back(s);
	m.Trans(s, h1, -2);
	S2.push_back(s);

	v3 = { -l2,h2,0,1 };
	v4 = { -l2,h2 - h1,0,1 };

	s = s0.getSpline(v3, v4);
	S2.push_back(s);
	m.Trans(s, l2 * 2, 1);
	S2.push_back(s);

	//内轮廓3
	Vec4 O = { -l2 - r,0,0,1 };
	v1 = { -l2 - r * 2,0,0,1 };
	v2 = { -l2 - r,r,0,1 };
	s = s0.getArcSpline(v1, v2, O);
	S3.push_back(s);

	v1 = { -l2,0,0,1 };
	v2 = { -l2 - r,r,0,1 };
	s = s0.getArcSpline(v1, v2, O);
	S3.push_back(s);

	v1 = { -l2,0,0,1 };
	v2 = { -l2 - r,-r,0,1 };
	s = s0.getArcSpline(v1, v2, O);
	S3.push_back(s);

	v1 = { -l2 - r * 2,0,0,1 };
	v2 = { -l2 - r,-r,0,1 };
	s = s0.getArcSpline(v1, v2, O);
	S3.push_back(s);

	S4 = S3;

	m.Trans(S3, (l2 + r) * 2, 1);

	for (auto &i : S)
	{
		allSpline.push_back(i);
	}
	for (auto &i : S1)
	{
		allSpline.push_back(i);
	}

	for (auto &i : S2)
	{
		allSpline.push_back(i);
	}

	for (auto &i : S3)
	{
		allSpline.push_back(i);
	}
	for (auto &i : S4)
	{
		allSpline.push_back(i);
	}
	rwg.WriteSpline("E:\\Model\\PlaneQuad\\allBoundry.txt", allSpline);

	varray<Spline> outer;
	varray<varray<Spline>> inner;
	//亏格设置
	varray<bool> genus;
	genus.resize(5);
	genus[0] = false;
	genus[1] = true;
	genus[2] = true;
	genus[3] = true;
	genus[4] = true;

	outer = S;
	inner.push_back(S1);
	inner.push_back(S2);
	inner.push_back(S3);
	inner.push_back(S4);

	writeInnerBoundry(inner);
	//平面剖分
	quadPlane(outer, inner, genus);
}

//导轨剖分
void test08()
{
	Vec4 a = { -15,10,0,1 };
	Vec4 b = { -15,30,0,1 };
	Vec4 c = { 15,30,0,1 };
	Vec4 d = { 15,10,0,1 };
	Vec4 e = { -15,5,0,1 };
	Vec4 f = { -20,5,0,1 };
	Vec4 h = { 20,5,0,1 };
	Vec4 g = { 15,5,0,1 };
	Vec4 i = { -20,-20,0,1 };
	Vec4 j = { 20,-20,0,1 };
	Vec4 k = { -10,-20,0,1 };
	Vec4 l = { 10,-20,0,1 };
	Vec4 m = { 0,-20,0,1 };
	Vec4 n = { 0,-30,0,1 };

	RWGeometric rwg;
	Boundary bo;
	varray<Spline> outer;
	varray<Spline> S;
	varray<varray<Spline>> inner;
	varray<SplineSurface> allSurface;
	PublicSolution ps;
	Model_Solution ms;

	Curve curve;
	outer.push_back(curve.getSpline(a, b));
	outer.push_back(curve.getSpline(b, c));
	outer.push_back(curve.getSpline(c, d));
	outer.push_back(curve.getArcSpline(d, h, g));
	outer.push_back(curve.getSpline(h, j));
	outer.push_back(curve.getSpline(j, l));
	outer.push_back(curve.getArcSpline(l, n, m));
	outer.push_back(curve.getArcSpline(n, k, m));
	outer.push_back(curve.getSpline(k, i));
	outer.push_back(curve.getSpline(i, f));
	outer.push_back(curve.getArcSpline(f, a, e));

	S = bo.getCircle(10);
	ms.Rolate(S, PI / 4, 3);
	ms.Trans(S, 5, -2);
	inner.push_back(S);

	S = bo.getSquare(10, 10);
	ms.Trans(S, 15, 2);
	inner.push_back(S);

	varray<Spline> temp;
	for (const auto &i : outer)
	{
		temp.push_back(i);
	}

	for (const auto &i : inner)
	{
		for (const auto &j : i)
		{
			temp.push_back(j);
		}
	}

	rwg.WriteSpline("E:\\Model\\PlaneQuad\\Delaunay\\Boundary.txt", temp);
	CDT_Operate cdto(outer, inner);
	// 亏格设置
	varray<bool> genus;
	genus.resize(3);
	genus[0] = false;
	genus[1] = true;
	genus[2] = true;
	varray<Spline> addLines;
	for (int i = 0; i < 6; i++)
	{
		addLines.push_back(cdto.addLine[i]);
	}
	
	for (auto &i : addLines)
	{
		temp.push_back(i);
	}
	rwg.WriteSpline("E:\\Model\\PlaneQuad\\Delaunay\\AddLines.txt", temp);
	ps.quad(outer, inner, addLines, genus, allSurface);
	rwg.WriteSplineSurface("E:\\Model\\PlaneQuad\\QuadSurface\\SlidewaySurface.txt", allSurface);
}

//三角底座剖分
void test09()
{
	RWGeometric rwg;
	Boundary bo;
	varray<Spline> outer;
	varray<Spline> S;
	varray<varray<Spline>> inner;
	varray<SplineSurface> allSurface;
	PublicSolution ps;
	Model_Solution ms;
	Curve curve;
	Vec4 a = { -8,0,0,1 };
	Vec4 c = { 8,0,0,1 };
	Vec4 d = { 10,0,0,1 };
	Vec4 g = { -1.56,11.25,0,1 };
	Vec4 h = { 1.56,11.25,0,1 };
	Vec4 i = { 9.56,1.25,0,1 };
	Vec4 j = { -9.56,1.25,0,1 };
	Vec4 k = { -8,-2,0,1 };
	Vec4 l = { 8,-2,0,1 };
	Vec4 m = { 0,10,0,1 };
	Vec4 q = { 0,6,0,1 };
	Vec4 r = { -3,2,0,1 };
	Vec4 s = { 3,2,0,1 };
	Vec4 t = { -9.8,-0.87,0,1 };
	Vec4 u = { 9.8,-0.87,0,1 };
	Vec4 v = { 0,12,0,1 };

	outer.push_back(curve.getArcSpline(k, t, a));
	outer.push_back(curve.getArcSpline(t, j, a));
	outer.push_back(curve.getSpline(j, g));
	outer.push_back(curve.getArcSpline(g, v, m));
	outer.push_back(curve.getArcSpline(v, h, m));
	outer.push_back(curve.getSpline(h, i));
	outer.push_back(curve.getArcSpline(i, u, c));
	outer.push_back(curve.getArcSpline(u, l, c));
	outer.push_back(curve.getSpline(l, k));

	S = bo.getCircle(1);
	ms.Rolate(S, 2*PI-PI / 180.0*128.66 / 2, 3);
	ms.Trans(S, 8, -1);
	inner.push_back(S);
	S = bo.getCircle(1);
	ms.Rolate(S, PI / 180.0*128.66 / 2, 3);
	ms.Trans(S, 8, 1);
	inner.push_back(S);
	S = bo.getCircle(1);
	//ms.Rolate(S, PI / 4, 3);
	ms.Trans(S, 10, 2);
	inner.push_back(S);
	S.clear();
	S.push_back(curve.getSpline(r, s));
	S.push_back(curve.getSpline(s, q));
	S.push_back(curve.getSpline(q, r));
	inner.push_back(S);

	varray<Spline> temp;
	for (const auto &i : outer)
	{
		temp.push_back(i);
	}

	for (const auto &i : inner)
	{
		for (const auto &j : i)
		{
			temp.push_back(j);
		}
	}

	rwg.WriteSpline("E:\\Model\\PlaneQuad\\Delaunay\\Boundary.txt", temp);
	CDT_Operate cdto(outer, inner);
	//亏格设置
	varray<bool> genus;
	genus.resize(5);
	genus[0] = false;
	genus[1] = true;
	genus[2] = true;
	genus[3] = true;
	genus[4] = true;
	varray<Spline> addLines;
	for (int i = 0; i < 5; i++)
	{
		addLines.push_back(cdto.addLine[i]);
	}
	ps.quad(outer, inner, cdto.addLine, genus, allSurface);
	rwg.WriteSplineSurface("E:\\Model\\PlaneQuad\\Delaunay\\SlidewaySurface.txt", allSurface);
}

//于嫣的模型
void test10()
{
	YuYan yy;
	varray<SplineVolume> SV = yy.completeGearShaft(7);
	RWGeometric rwg;
	PublicSolution ps;
	//vector<int> vec = { 0,1,2,3,4,5,6,7/*,115,116,117,120,122,123,125,126,128,129,130,131,132,134,135*/ };
	//int n = 10;
	//int num = SV.size() / 2;
	//for (auto i = 0; i < 3; i++)
	//{
	//	for (auto j = 0; j < 3; j++)
	//	{
	//		vec.push_back(n + j);
	//	}
	//	n += 6;
	//}
	//n = 31;
	//for (auto i = 0; i < 7; i++)
	//{
	//	for (auto j = 0; j < 3; j++)
	//	{
	//		vec.push_back(n + j);
	//	}
	//	n += 6;
	//}
	//n = 70;
	//for (auto i = 0; i < 7; i++)
	//{
	//	for (auto j = 0; j < 3; j++)
	//	{
	//		vec.push_back(n + j);
	//	}
	//	n += 6;
	//}
	//n = num;
	//for (auto i = num; i < num + 8; i++)
	//{
	//	vec.push_back(i);
	//}
	//n = num + 10;
	//for (auto i = 0; i < 3; i++)
	//{
	//	for (auto j = 0; j < 3; j++)
	//	{
	//		vec.push_back(n + j);
	//	}
	//	n += 6;
	//}
	//n = 31 + num;
	//for (auto i = 0; i < 7; i++)
	//{
	//	for (auto j = 0; j < 3; j++)
	//	{
	//		vec.push_back(n + j);
	//	}
	//	n += 6;
	//}
	//n = 70 + num;
	//for (auto i = 0; i < 7; i++)
	//{
	//	for (auto j = 0; j < 3; j++)
	//	{
	//		vec.push_back(n + j);
	//	}
	//	n += 6;
	//}
	//for (auto &i : vec)
	//{
	//	SV[i].OrderCtrlPts(SV[i]);
	//}
	

	vector<int> vec = { 0,1,2,3,4,5,6,7,10,11,12};
	for (auto &i : vec)
	{
		SV[i].OrderCtrlPts(SV[i]);
	}
	for (auto &i : vec)
	{
		//SV[i + SV.size() / 2].OrderCtrlPts(SV[i + SV.size() / 2]);
	}
	for (int i = 0; i < SV.size(); i++)
	{
		if (i >= 240 && i < 360)
		{
			continue;
		}
		//SV[i].OrderCtrlPts(SV[i]);
	}
	varray<SplineVolume>temp;
	for (int i = 0; i < 16; i++)
	{
		temp.push_back(SV[i]);
	}

	varray<double> u;
	varray<double> v;
	varray<double> w;
	u.resize(2);
	v.resize(2);
	w.resize(2);
	//0.125 0.25 0.375 0.5 0.625 0.75 0.875
	for (auto &i : SV) {
		u.clear();
		v.clear();
		w.clear();
		/*u.push_back(0.25);
		u.push_back(0.5);
		u.push_back(0.5);
		u.push_back(0.75);

		v.push_back(0.25);
		v.push_back(0.5);
		v.push_back(0.5);
		v.push_back(0.75);

		w.push_back(0.25);
		w.push_back(0.5);
		w.push_back(0.5);
		w.push_back(0.75);*/

		
		u.push_back(0.5);
		u.push_back(0.5);
		
		v.push_back(0.5);
		v.push_back(0.5);
		
		w.push_back(0.5);
		w.push_back(0.5);
		//i.KnotsRefine(u, v, w);
	}
	rwg.WriteSplineVolume("E:\\Model\\Jacobian\\Volume.txt", SV);
	putOutVTK();
	rwg.WriteSplineVolume("E:\\Model\\WcWfFile\\VolumeTest.txt", SV);
	ps.setWCandWF("E:\\Model\\WcWfFile\\VolumeTest.txt", "E:\\Model\\WcWfFile\\VolumeTest");
}

//模型合并
void example2()
{
	YuYan yy;
	varray<SplineVolume> SV1 = yy.completeGearShaft(7);
	varray<SplineVolume> SV2 = yy.gearShaft(7);
	varray<SplineVolume> SV3 = yy.gearShaft1(7);
	RWGeometric rwg;
	PublicSolution ps;
	rwg.WriteSplineVolume("E:\\Model\\YuYanModel\\SV1.txt", SV1);
	rwg.WriteSplineVolume("E:\\Model\\YuYanModel\\SV2.txt", SV2);
	rwg.WriteSplineVolume("E:\\Model\\YuYanModel\\SV3.txt", SV3);
}
//输出VTK文件
void test11()
{
	varray<SplineVolume>SV;
	RWGeometric rwg;
	rwg.ReadSplineVolume("E:\\Model\\YuYanModel\\GearShaftVolume.txt", SV);
	vector<int>  v;
	int num = 0;

	//下面齿轮轴
	num = 0;
	for (int i = 0; i <= 7; i++)
	{
		v.push_back(i);
	}
	num = 10;
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			v.push_back(num + i * 6 + j);
		}
	}
	num = 31;
	for (int i = 0; i < 5; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			v.push_back(num + i * 6 + j);
		}
	}

	num = 58;
	for (int i = 0; i < 7; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			v.push_back(num + i * 6 + j);
		}
	}

	//上面齿轮轴
	num = 100;
	for (int i = num; i <= num + 7; i++)
	{
		v.push_back(i);
	}
	num = 110;
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			v.push_back(num + i * 6 + j);
		}
	}
	num = 131;
	for (int i = 0; i < 5; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			v.push_back(num + i * 6 + j);
		}
	}

	num = 158;
	for (int i = 0; i < 7; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			v.push_back(num + i * 6 + j);
		}
	}

	varray<SplineVolume>temp;
	temp.clear();
	for (auto&i : v)
	{
		temp.push_back(SV[i]);
	}
	rwg.WriteSplineVolume("E:\\Model\\YuYanModel\\ChackVolume.txt", temp);

	for (decltype(v.size()) i = 0; i < v.size(); i++)
	{
		SV[v[i]].OrderCtrlPts(SV[v[i]]);
	}

	varray<NurbsVol> NV;
	NV = NurbsTrans::SplinevolsToCvols(SV);
	CPolyParaVolume cp;       //输出vtk文件的类对象
	cp = NV;
	cp.OutputParaVolumeDataVTK("E:\\Model\\YuYanModel\\GearShaftVolume.vtk");
	rwg.WriteSplineVolume("E:\\Model\\WcWfFile\\VoluweTest.txt", SV);
}

//s1957测试点到曲线的距离
void test12()
{
	//前期工作
	Model_Solution m;
	Vec3 point;
	Vec4 p1 = { -5,0,0,1 };
	Vec4 p2 = { 0,0,0,1 };
	Vec4 p3 = { 0,5,0,1 };
	Spline0 s0;
	Spline s;
	s = s0.getArcSpline(p1, p3, p2);
	m.Trans(s, 3, -1);
	m.Rolate(s, PI / 4, 1);

	varray<Spline> v;
	v.push_back(s);

	RWGeometric rwg;
	rwg.WriteSpline("E:\\Model\\ModelTest\\ChackLine.txt", v);

	//SISL初始化
	SISLCurve *pcurve = NurbsLineToSislLine(s); /* Must be defined */
	double epoint[3] = { 0,0,0 }; /* Must be defined */
	int idim = 3;
	double aepsco = 1.0e-9; /* Not used */
	double aepsge = 1.0e-6;
	double gpar;
	double dist = 0;
	int jstat = 0;
	s1957(pcurve, epoint, idim, aepsco, aepsge, &gpar, &dist, &jstat);

	cout << "最近的距离为：" << dist << endl;
	cout << "曲线参数区间内最近点的参数值：" << gpar << endl;
	point = s.GetLinePoint(gpar);
	cout << "曲线上最近的点：{" << point.x << ", " << point.y << ", " << point.z << "}" << endl;
}

//施加力和约束
void test13()
{
	PublicSolution ps;
	//施加力与约束
	ps.setWCandWF("E:\\Model\\WcWfFile\\VolumeTest.txt", "E:\\Model\\WcWfFile\\VolumeTest");
}

//节点细化
void test14()
{
	RWGeometric rwg;
	varray<Spline> S;
	PublicSolution ps;
	varray<Spline> temp;
	rwg.ReadSpline("E:\\quadApp\\Model\\QtGuiApplication1\\resSpline.txt", S);
	varray<Spline> outer;
	varray<varray<Spline>> inner;
	outer.push_back(S[0]);
	outer.push_back(S[1]);
	outer.push_back(S[2]);
	outer.push_back(S[3]);
	outer.push_back(S[4]);
	outer.push_back(S[5]);
	outer.push_back(S[6]);
	outer.push_back(S[7]);
	temp.push_back(S[8]);
	temp.push_back(S[9]);
	temp.push_back(S[10]);
	temp.push_back(S[11]);
	temp.push_back(S[12]);
	temp.push_back(S[13]);
	temp.push_back(S[14]);
	temp.push_back(S[15]);
	inner.push_back(temp);
	varray<bool> genus;
	genus.resize(2);
	genus[0] = false;
	genus[1] = true;
	CreateAddlineByDelaunay cabd(outer, inner);
	varray<SplineSurface> allSurface;
	ps.quad(outer, inner, cabd.m_addLine, genus, allSurface);
	rwg.WriteSplineSurface("E:\\Model\\SurfaceQuad\\Plane\\ResultSurface.txt", allSurface);
}

//很多孔
void test15()
{
	varray<Spline> S;
	Curve curve;
	varray<Spline> outer;
	varray<varray<Spline>>inner;
	varray<Spline> temp;
	varray<SplineSurface> allSurface;
	PublicSolution ps;
	Boundary bo;
	Model_Solution m;
	RWGeometric rwg;

	S = bo.getCircle(25);
	//m.Rolate(S, PI / 4, 3);
	outer = S;

	S = bo.getCircle(2);
	//m.Rolate(S, PI / 4, 3);
	m.Trans(S, 12, -1);
	m.Trans(S, 12, 2);

	for (int i = 0; i < 4; i++)
	{
		inner.push_back(S);
		for (int j = 0; j < 3; j++)
		{
			m.Trans(S, 8, 1);
			inner.push_back(S);
		}
		m.Trans(S, 24, -1);
		m.Trans(S, 8, -2);
	}

	for (const auto &i : outer)
	{
		temp.push_back(i);
	}

	for (const auto &i : inner)
	{
		for (const auto &j : i)
		{
			temp.push_back(j);
		}
	}

	rwg.WriteSpline("E:\\Model\\PlaneQuad\\Delaunay\\Boundary.txt", temp);
	CDT_Operate cdto(outer, inner);
	// 亏格设置
	varray<bool> genus;
	genus.resize(17,true);
	genus[0] = false;
	varray<Spline> addLines;
	for (int i = 0; i < 6; i++)
	{
		addLines.push_back(cdto.addLine[i]);
		temp.push_back(cdto.addLine[i]);
	}
	ps.quad(outer, inner, cdto.addLine, genus, allSurface);

	rwg.WriteSpline("E:\\Model\\PlaneQuad\\Delaunay\\Boundary.txt", temp);
	rwg.WriteSplineSurface("E:\\Model\\PlaneQuad\\QuadSurface\\SlidewaySurface.txt", allSurface);

}

void test16()
{
	
}

//生成内外轮廓
void test17()
{
	Boundary bo;
	Spline s;
	Spline0 s0;
	Model_Solution m;
	varray<Spline> outer;
	varray<varray<Spline>> inner;
	varray<Spline> temp;
	varray<Spline> S;
	varray<Spline>extraAddLine;
	RWGeometric rwg;
	Vec4 p1 = { -5,5,0,1 };
	Vec4 p2 = { -3,5,0,1 };
	Vec4 p3 = { 5,5,0,1 };
	Vec4 p4 = { 5,-5,0,1 };
	Vec4 p5 = { -5,-5,0,1 };
	Vec4 p6 = { -5,3,0,1 };
	Vec4 p7 = { -2,2,0,1 };
	Vec4 p8 = { -3,3,0,1 };
	inner.push_back(bo.getSquare(4, 4));

	outer.push_back(s0.getSpline(p1, p2));
	outer.push_back(s0.getSpline(p2, p3));
	outer.push_back(s0.getSpline(p3, p4));
	outer.push_back(s0.getSpline(p4, p5));
	outer.push_back(s0.getSpline(p5, p6));
	outer.push_back(s0.getSpline(p6, p1));

	extraAddLine.push_back(s0.getSpline(p6, p7));
	extraAddLine.push_back(s0.getSpline(p2, p8));
	extraAddLine.push_back(s0.getSpline(p7, p8));
	temp = outer;
	for (auto &i : inner[0])
	{
		temp.push_back(i);
	}
	for (auto &i : extraAddLine)
	{
		temp.push_back(i);
	}

	rwg.WriteSpline("E:\\Model\\PlaneQuad\\test_CDT.txt", temp);

	varray<bool> genus;
	genus.resize(2);
	genus[0] = false;
	genus[1] = false;
	varray<SplineSurface> SS;
	PublicSolution ps;
	ps.quad(outer, inner, extraAddLine, genus, SS);
	//ps.quad_Lee(outer, inner, genus, SS);
	rwg.WriteSplineSurface("E:\\Model\\PlaneQuad\\boundarySurface.txt", SS);
}

//五边形加三角形
void test18()
{
	double h1 = 16;
	double h2 = 28;
	double h3 = 5;
	double d1 = 3;
	double d2 = 12;

	Vec4 v1 = { 0,h2,0,1 };
	Vec4 v2 = { d2,h1,0,1 };
	Vec4 v3 = { d2,-h1,0,1 };
	Vec4 v4 = { 0,-h2,0,1 };
	Vec4 v5 = { -d2,-h1,0,1 };
	Vec4 v6 = { -d2,h1,0,1 };
	Vec4 v7 = { 0,h3,0,1 };
	Vec4 v8 = { d1,0,0,1 };
	Vec4 v9 = { -d1,0,0,1 };

	varray<Spline> S;
	varray<Spline> S1;
	varray<Spline> S2;
	varray<Spline> S3;
	varray<Spline> S4;

	varray<Spline>allSpline;

	Spline s;
	Spline0 s0;
	Model_Solution m;
	RWGeometric rwg;

	//外轮廓
	s = s0.getSpline(v1, v2);
	S.push_back(s);

	s = s0.getSpline(v2, v3);
	S.push_back(s);

	s = s0.getSpline(v3, v4);
	S.push_back(s);

	s = s0.getSpline(v4, v5);
	S.push_back(s);

	s = s0.getSpline(v5, v6);
	S.push_back(s);

	s = s0.getSpline(v6, v1);
	S.push_back(s);

	//内轮廓
	s = s0.getSpline(v6, v1);
	S1.push_back(s);

	s = s0.getSpline(v1, v2);
	S1.push_back(s);

	s = s0.getSpline(v6, v2);
	S1.push_back(s);
	//m.Trans(S1, 2, -2);

	varray<Spline> outer;
	varray<varray<Spline>> inner;

	//赋予内外轮廓
	outer = S;
	inner.push_back(S1);

	//输出所有轮廓
	for (auto &i : outer)
	{
		allSpline.push_back(i);
	}
	for (auto &i : inner)
	{
		for (auto &j : i)
		{
			allSpline.push_back(j);
		}
	}
	rwg.WriteSpline("E:\\Model\\PlaneQuad\\allBoundry.txt", allSpline);

	//亏格设置
	varray<bool> genus;
	genus.resize(2);
	genus[0] = false;
	genus[1] = false;

	//平面剖分
	quadPlane(outer, inner, genus);
}

//连杆
void test19()
{
	double r1 = 6;
	double r2 = 8;
	double r3 = 4;
	double r4 = 1;
	double r5 = 3;
	double l1 = 10;
	double l2 = 10;
	double l3 = 12;
	double l4 = 12;

	Vec4 o1 = { -l1,0,0,1 };
	Vec4 o2 = { l2,0,0,1 };
	Vec4 o3 = { -l3,0,0,1 };
	Vec4 o4 = { l4,0,0,1 };

	Vec4 v1 = { -l1 - r2,0,0,1 };
	Vec4 v2 = { -l1,r2,0,1 };
	Vec4 v3 = { l2,r3,0,1 };
	Vec4 v4 = { l2 + r3,0,0,1 };
	Vec4 v5 = { l2,-r3,0,1 };
	Vec4 v6 = { -l1,-r2,0,1 };
	Vec4 v7 = { -l1 - r1,0,0,1 };
	Vec4 v8 = { -l1,r1,0,1 };
	Vec4 v9 = { -l1 + r1,0,0,1 };
	Vec4 v10 = { -l1,-r1,0,1 };
	Vec4 v11 = { -l3 - r4,0,0,1 };
	Vec4 v12 = { -l3,r4,0,1 };
	Vec4 v13 = { l4,r4,0,1 };
	Vec4 v14 = { l4 + r4,0,0,1 };
	Vec4 v15 = { l4,-r4,0,1 };
	Vec4 v16 = { -l3,-r4,0,1 };
	Vec4 v17 = { l2 - r5,0,0,1 };
	Vec4 v18 = { l2,r5,0,1 };
	Vec4 v19 = { l2 + r5,0,0,1 };
	Vec4 v20 = { l2,-r5,0,1 };

	varray<Spline> S;
	varray<Spline> S1;
	varray<Spline> S2;
	varray<Spline> S3;
	varray<Spline> S4;

	Spline0 s0;
	Spline s;

	s = s0.getArcSpline(v1, v2, o1);
	S.push_back(s);

	s = s0.getSpline(v2, v3);
	S.push_back(s);

	s = s0.getArcSpline(v3, v4, o2);
	S.push_back(s);

	s = s0.getArcSpline(v5, v4, o2);
	S.push_back(s);

	s = s0.getSpline(v6, v5);
	S.push_back(s);

	s = s0.getArcSpline(v6, v1, o1);
	S.push_back(s);

	RWGeometric rwg;
	rwg.WriteSpline("E:\\Model\\PlaneQuad\\OuterBoundry.txt", S);

	s = s0.getArcSpline(v7, v8, o1);
	S1.push_back(s);

	s = s0.getArcSpline(v9, v8, o1);
	S1.push_back(s);

	s = s0.getArcSpline(v10, v9, o1);
	S1.push_back(s);

	s = s0.getArcSpline(v10, v7, o1);
	S1.push_back(s);

	rwg.WriteSpline("E:\\Model\\PlaneQuad\\InnerBoundry0.txt", S1);

	s = s0.getArcSpline(v11, v12, o3);
	S2.push_back(s);

	s = s0.getSpline(v12, v13);
	S2.push_back(s);

	s = s0.getArcSpline(v14, v13, o4);
	S2.push_back(s);

	s = s0.getArcSpline(v15, v14, o4);
	S2.push_back(s);

	s = s0.getSpline(v16, v15);
	S2.push_back(s);

	s = s0.getArcSpline(v16, v11, o3);
	S2.push_back(s);

	rwg.WriteSpline("E:\\Model\\PlaneQuad\\InnerBoundry1.txt", S2);

	s = s0.getArcSpline(v17, v18, o2);
	S3.push_back(s);

	s = s0.getArcSpline(v19, v18, o2);
	S3.push_back(s);

	s = s0.getArcSpline(v20, v19, o2);
	S3.push_back(s);

	s = s0.getArcSpline(v20, v17, o2);
	S3.push_back(s);

	varray<Spline> all;

	for (auto &i : S)
	{
		all.push_back(i);
	}
	for (auto &i : S1)
	{
		all.push_back(i);
	}
	for (auto &i : S2)
	{
		all.push_back(i);
	}

	for (auto &i : S3)
	{
		all.push_back(i);
	}
	rwg.WriteSpline("E:\\Model\\PlaneQuad\\allBoundry.txt", all);

	varray<Spline> outer = S;
	varray<varray<Spline>> inner;

	inner.push_back(S1);
	//inner.push_back(S2);
	inner.push_back(S3);

	Boundary bo;
	S2 = bo.getSquare(r1 / 2, r1 / 2);

	Model_Solution m;
	for (auto &i : S2)
	{
		m.Rolate(i, PI / 4, 3);
	}

	m.Trans(S2, -l1, 1);
	//inner.push_back(S2);

	S2 = bo.getSquare(r5 / 2, r5 / 2);
	for (auto &i : S2)
	{
		m.Rolate(i, PI / 4, 3);
	}

	m.Trans(S2, l2, 1);
	//inner.push_back(S2);

	double d1 = r1 / 2 * sin(PI / 4);
	double d2 = r5 / 2 * sin(PI / 4);

	Vec4 v21 = { -d1 - l1,0,0,1 };
	Vec4 v22 = { d1 - l1,0,0,1 };

	Vec4 v23 = { -d2 + l2,0,0,1 };
	Vec4 v24 = { d2 + l2,0,0,1 };

	s = s0.getSpline(v2, v3);
	Vec4 v25 = s.m_CtrlPts[1];

	s = s0.getSpline(v6, v5);

	Vec4 v26 = s.m_CtrlPts[1];
	varray<Spline> addlines;
	varray<SplineSurface> allSurface;
	addlines.push_back(s0.getSpline(v1, v7));
	addlines.push_back(s0.getSpline(v7, v21));

	addlines.push_back(s0.getSpline(v22, v9));
	addlines.push_back(s0.getSpline(v9, v17));
	addlines.push_back(s0.getSpline(v17, v23));
	addlines.push_back(s0.getSpline(v24, v19));
	addlines.push_back(s0.getSpline(v3, v9));
	addlines.push_back(s0.getSpline(v5, v9));
	/*addlines.push_back(s0.getSpline(v9, v25));
	addlines.push_back(s0.getSpline(v9, v26));*/
	PublicSolution ps;

	varray<Spline> allLines;
	for (auto &i : outer)
	{
		allLines.push_back(i);
	}
	for (auto &i : inner)
	{
		for (auto &j : i)
		{
			allLines.push_back(j);
		}
	}
	rwg.ReadSpline("E:\\Model\\Jacobian\\OnlyAddSplines.txt", addlines);
	for (auto &i : addlines)
	{
		allLines.push_back(i);
	}

	varray<Spline> temp;
	temp = outer;
	for (const auto &i : inner)
	{
		for (const auto &j : i)
		{
			temp.push_back(j);
		}
	}
	rwg.WriteSpline("E:\\Model\\Jacobian\\OnlyAddSplines.txt", temp);
	CDT_Operate cdto(outer, inner);

	varray<SplineSurface> allSurf;
	// 亏格设置
	varray<bool> genus;
	genus.resize(3);
	genus[0] = false;
	genus[1] = true;
	genus[2] = true;
	ps.quad(outer, inner, cdto.addLine, genus, allSurf);
	rwg.WriteSplineSurface("E:\\Model\\PlaneQuad\\QuadSurface\\PitmanSurface.txt", allSurf);

	////平面剖分
	//ps.quad_Lee(outer, inner, genus, allSurface);
	//rwg.WriteSpline("E:\\Model\\PlaneQuad\\allLines.txt", allLines);
	//rwg.WriteSplineSurface("E:\\Model\\PlaneQuad\\BoundarySurface.txt", allSurface);
	//varray<SplineVolume> SV = m.CreatSweepVol(allSurface, 5, 3);
	//for (decltype(SV.size()) i = 0; i < SV.size(); i++)
	//{
	//	SV[i].OrderCtrlPts(SV[i]);
	//}
	//rwg.WriteSplineVolume("E:\\Model\\Jacobian\\Volume.txt", SV);
	//putOutVTK();

	//rwg.WriteSplineSurface("E:\\Model\\PlaneQuad\\boundarySurface.txt", allSurface);
}

//外圆内方
void test20()
{
	Model_Solution ms;
	PublicSolution ps;
	Boundary bo;
	varray<SplineSurface> allSurface;
	varray<Spline> outer;
	varray<varray<Spline>>inner;
	varray<bool> genus;
	RWGeometric rwg;

	outer = bo.getCircle(10);
	ms.Rolate(outer, PI / 4, 3);
	rwg.WriteSpline("E:\\Model\\PlaneQuad\\Delaunay\\Boundary.txt", outer);
	inner.push_back(bo.getSquare(5, 5));

	genus.resize(2);
	genus[0] = false;
	genus[1] = true;

	CDT_Operate cdto(outer, inner);
	ps.quad(outer, inner, cdto.addLine, genus, allSurface);
	rwg.WriteSplineSurface("E:\\Model\\PlaneQuad\\QuadSurface\\CircleWithOneSquare.txt", allSurface);
}

//两个矩形剖分
void test21()
{
	Model_Solution m;
	Boundary bo;
	varray<SplineSurface> SS;
	varray<Spline> outer = bo.getSquare(10, 10);
	varray<varray<Spline>>inner;
	inner.push_back(bo.getSquare(5, 5));

	CDT_Operate cdto(outer, inner);

	//亏格设置
	varray<bool> genus;
	genus.resize(2);
	genus[0] = false;
	genus[1] = true;

	PublicSolution ps;
	ps.quad(outer, inner, cdto.addLine, genus, SS);
	RWGeometric rwg;
	rwg.WriteSplineSurface("E:\\Model\\PlaneQuad\\QuadSurface\\boundarySurface.txt", SS);
}

//内圆外矩形
void test22()
{
	RWGeometric rwg;
	Boundary bo;
	varray<SplineSurface> SS;
	varray<Spline> outer;
	varray<varray<Spline>>inner;
	Model_Solution m;
	outer = bo.getSquare(40, 40);
	rwg.ReadSplineSurface("E:\\Model\\PlaneQuad\\QuadSurface\\PitmanSurface.txt", SS);
	
	rwg.WriteSplineVolume("E:\\Model\\PlaneQuad\\QuadSurface\\PitmanVolume.txt", m.CreatSweepVol(SS, 5, 3));
	
	varray<Spline> S = bo.getCircle(10);
	m.Rolate(S, PI / 4, 3);
	inner.push_back(S);
	DT_Operate dt(outer, inner);

	CDT_Operate cdto(outer, inner);
	// 亏格设置
	varray<bool> genus;
	genus.resize(2);
	genus[0] = false;
	genus[1] = true;
	/*genus[2] = false;
	genus[3] = false;
	genus[4] = false;*/

	PublicSolution ps;
	ps.quad(outer, inner, cdto.addLine, genus, SS);
	
	rwg.WriteSplineSurface("E:\\Model\\PlaneQuad\\QuadSurface\\SquareWithOneHole.txt", SS);
}

//箱体
void test23()
{//连接线算法参数改为2.0
	Spline s;
	Spline0 s0;
	Model_Solution m;
	varray<Spline> outer;
	varray<varray<Spline>> inner;
	varray<Spline> temp;
	varray<Spline> S;
	varray<SplineVolume> SV;
	varray<SplineSurface> SF;
	RWGeometric rwg;
	rwg.ReadSplineSurface("E:\\Model\\PlaneQuad\\QuadSurface\\BoxSurface.txt", SF);
	SV = m.CreatSweepVol(SF, 16, 3);
	rwg.WriteSplineVolume("E:\\Model\\PlaneQuad\\QuadSurface\\BoxVolume.txt", SV);
	double L = 30;
	double H = 15;
	double H1;
	double L1;
	double L2 = 8;
	double L3;
	double R1 = 6;
	double R2 = 8;
	double R3 = 4;
	L1 = L2 + R3 * 2;
	L3 = 2 * R2;
	L = L1 + R2 * 3 + 10;
	H1 = L2 * 2 + R3 * 2;
	H = H1 + 2 * R2;
	R1 = L1 + R2;

	Vec4 op1 = { 0,-H,0,1 };
	Vec4 op2 = { -L1,-H,0,1 };
	Vec4 op3 = { -L1 - R2,-H + R2,0,1 };
	Vec4 op4 = { -R2 * 2 - L1,-H,0,1 };
	Vec4 op5 = { -L,-H,0,1 };
	Vec4 op6 = { -L,H,0,1 };
	Vec4 op7 = { -R1,H,0,1 };
	Vec4 op8 = { 0,H + R1,0,1 };
	Vec4 oo1 = { 0,H,0,1 };
	Vec4 oo2 = { -L1 - R2,-H,0,1 };

	s = s0.getSpline(op2, op1);
	outer.push_back(s);

	s = s0.getArcSpline(op3, op2, oo2);
	outer.push_back(s);

	s = s0.getArcSpline(op3, op4, oo2);
	outer.push_back(s);

	s = s0.getSpline(op5, op4);
	outer.push_back(s);

	s = s0.getSpline(op5, op6);
	outer.push_back(s);

	s = s0.getSpline(op6, op7);
	outer.push_back(s);

	s = s0.getArcSpline(op7, op8, oo1);
	outer.push_back(s);

	m.MirrorLines(outer, temp, 1);

	for (auto &i : temp)
	{
		outer.push_back(i);
	}
	rwg.WriteSpline("E:\\Model\\PlaneQuad\\OuterBoundary.txt", outer);

	Boundary bo;
	temp = bo.getSquare(L3, H1);

	for (auto &i : temp)
	{
		m.Trans(i, L3 / 2 + L1 + R2, -1);
	}

	inner.push_back(temp);
	m.MirrorLines(temp, S, 1);
	inner.push_back(S);

	//S = bo.getCircle(L3 / 2);
	//inner.push_back(S);

	Vec4 ip1 = { -L2 - R3,0,0,1 };
	Vec4 ip2 = { -L2 - R3,L2,0,1 };
	Vec4 ip3 = { -L2 - R3 * 2,L2 + R3,0,1 };
	Vec4 ip4 = { -L2 - R3,L2 + R3 * 2,0,1 };
	Vec4 ip5 = { -L2,L2 + R3,0,1 };
	Vec4 ip6 = { 0,L2 + R3,0,1 };
	Vec4 io = { -L2 - R3,L2 + R3,0,1 };

	temp.clear();
	s = s0.getSpline(ip1, ip2);
	temp.push_back(s);

	s = s0.getArcSpline(ip2, ip3, io);
	temp.push_back(s);

	s = s0.getArcSpline(ip3, ip4, io);
	temp.push_back(s);

	s = s0.getArcSpline(ip5, ip4, io);
	temp.push_back(s);

	s = s0.getSpline(ip5, ip6);
	temp.push_back(s);

	S = temp;
	for (auto &i : temp)
	{
		m.Rolate(i, PI / 2, 3);
		S.push_back(i);
	}
	for (auto &i : temp)
	{
		m.Rolate(i, PI / 2, 3);
		S.push_back(i);
	}
	for (auto &i : temp)
	{
		m.Rolate(i, PI / 2, 3);
		S.push_back(i);
	}
	inner.push_back(S);
	S.clear();
	for (auto &i : inner)
	{
		for (auto &j : i)
		{
			S.push_back(j);
		}
	}
	rwg.WriteSpline("E:\\Model\\PlaneQuad\\InnerBoundary.txt", S);

	temp = outer;
	for (const auto &i : inner)
	{
		for (const auto &j : i)
		{
			temp.push_back(j);
		}
	}
	rwg.WriteSpline("E:\\Model\\PlaneQuad\\Delaunay\\Boundary.txt", temp);

	CDT_Operate cdto(outer, inner);
	// 亏格设置
	varray<bool> genus;
	genus.resize(4);
	genus[0] = false;
	genus[1] = true;
	genus[2] = true;
	genus[3] = true;
	/*genus[4] = false;*/
	varray<SplineSurface> SS;
	PublicSolution ps;
	varray<Spline> addLines;
	for (int i = 0; i < cdto.addLine.size(); i++)
	{
		if (i == 1)
		{
			continue;
		}
		addLines.push_back(cdto.addLine[i]);
		temp.push_back(cdto.addLine[i]);
		rwg.WriteSpline("E:\\Model\\PlaneQuad\\Delaunay\\addLines.txt", temp);
	}
	rwg.WriteSpline("E:\\Model\\PlaneQuad\\Delaunay\\addLines.txt", temp);
	Vec4 ap10 = { -R1,H,0,1 };
	Vec4 ap11 = { -L2 - R3,L2 + R3 * 2,0,1 };
	Vec4 ap20 = { R1,H,0,1 };
	Vec4 ap21 = { L2 + R3,L2 + R3 * 2,0,1 };
	Vec4 ap30 = { -L1 - R2,-H + R2,0,1 };
	Vec4 ap31 = { -L2 - R3,-L2 - R3 * 2,0,1 };
	Vec4 ap40 = { L1 + R2,-H + R2,0,1 };
	Vec4 ap41 = { L2 + R3,-L2 - R3 * 2,0,1 };
	addLines.push_back(s0.getSpline(ap10, ap11));
	addLines.push_back(s0.getSpline(ap20, ap21));
	addLines.push_back(s0.getSpline(ap30, ap31));
	addLines.push_back(s0.getSpline(ap40, ap41));
	ps.quad(outer, inner, addLines, genus, SS);
	//.quad_Lee(outer, inner, genus, SS);
	//ps.quad(outer, inner, genus, SS);
	rwg.WriteSplineSurface("E:\\Model\\PlaneQuad\\QuadSurface\\BoxSurface.txt", SS);
	
	/*CreatePolygon cp(cdto.outer, cdto.addLine, cdto.inner);

	for (const auto &polygon : cp.allPolygon)
	{
		rwg.WriteSpline("E:\\Model\\PlaneQuad\\Delaunay\\Polygon.txt", polygon.p_NurbsLines);
		system("pause");
	}*/
}

//滑块模型剖分
void test24()
{//连接线算法参数改为1.5
	varray<Spline> S;
	Curve curve;
	varray<Spline> outer;
	varray<varray<Spline>>inner;
	varray<Spline> temp;
	varray<SplineSurface> allSurface;
	varray<SplineVolume> allVolume;
	PublicSolution ps;
	Boundary bo;
	Model_Solution m;
	RWGeometric rwg;
	rwg.ReadSplineSurface("C:\\Users\\Administrator\\Desktop\\参考文献\\SlidewaySurface.txt", allSurface);
	allVolume = m.CreatSweepVol(allSurface, 4, 3);
	rwg.WriteSplineVolume("E:\\Model\\PlaneQuad\\QuadSurface\\SlidewayVolume.txt", allVolume);
	Vec4 a = { -14,12,0,1 };
	Vec4 b = { -14,0,0,1 };
	Vec4 c = { -8,0,0,1 };
	Vec4 d = { -6,-2,0,1 };
	Vec4 e = { 6,-2,0,1 };
	Vec4 f = { 8,0,0,1 };
	Vec4 g = { 14,0,0,1 };
	Vec4 h = { 14,12,0,1 };
	outer.push_back(curve.getSpline(a, (a+b)/2));
	outer.push_back(curve.getSpline((a + b) / 2, b));
	outer.push_back(curve.getSpline(b, c));
	outer.push_back(curve.getSpline(c, d));
	outer.push_back(curve.getSpline(d, (d + e) / 2));
	outer.push_back(curve.getSpline((d + e) / 2, e));
	outer.push_back(curve.getSpline(e, f));
	outer.push_back(curve.getSpline(f, g));
	outer.push_back(curve.getSpline(g, h));
	outer.push_back(curve.getSpline(h, (a + h) / 2));
	outer.push_back(curve.getSpline((a + h) / 2, a));

	S = bo.getCircle(2);
	m.Trans(S, 9, -1);
	m.Trans(S, 6, 2);
	inner.push_back(S);
	m.Trans(S, 18, 1);
	inner.push_back(S);

	S = bo.getCircle(4);
	m.Trans(S, 6, 2);
	inner.push_back(S);
	rwg.WriteSpline("E:\\Model\\PlaneQuad\\AllBoundary.txt", outer);
	ps.orderEdgeClockwise(outer);
	rwg.WriteSpline("E:\\Model\\PlaneQuad\\AllBoundary.txt", outer);
	for (const auto &i : outer)
	{
		temp.push_back(i);
	}

	for (const auto &i : inner)
	{
		for (const auto &j : i)
		{
			temp.push_back(j);
		}
	}

	rwg.WriteSpline("E:\\Model\\PlaneQuad\\Delaunay\\Boundary.txt", temp);
	CDT_Operate cdto(outer, inner);
	// 亏格设置
	varray<bool> genus;
	genus.resize(4);
	genus[0] = false;
	genus[1] = true;
	genus[2] = true;
	genus[3] = true;
	varray<Spline> addLines;
	for (int i = 0; i < cdto.addLine.size(); i++)
	{
		addLines.push_back(cdto.addLine[i]);
		temp.push_back(cdto.addLine[i]);
	}
	Graph graph(outer, inner, cdto.addLine);
	varray<varray<Spline>> all_area = graph.get_all_area();
	for (const auto &i : all_area)
	{
		rwg.WriteSpline("E:\\Model\\PlaneQuad\\Area.txt", i);
	}
	ps.quad(outer, inner, cdto.addLine, genus, allSurface);

	rwg.WriteSpline("E:\\Model\\PlaneQuad\\Delaunay\\Boundary.txt", temp);
	rwg.WriteSplineSurface("E:\\Model\\PlaneQuad\\QuadSurface\\SlidewaySurface.txt", allSurface);
}
//逸博的模型
void test25()
{
	//叉架
	//两侧薄板拉伸
	varray<Spline> Sls01;
	varray<Vec4> Vs01;
	varray<double> knots;
	knots.push_back(0);
	knots.push_back(0);
	knots.push_back(0);
	knots.push_back(1);
	knots.push_back(1);
	knots.push_back(1);

	Vec4 P0 = { 0, 22, 0, 1 };
	Vs01.push_back(P0);
	P0 = { 0, 32, 0, sqrt(2) / 2 };
	Vs01.push_back(P0);

	P0 = { 10, 32, 0, 1 };
	Vs01.push_back(P0);
	P0 = { 48, 32, 0, sqrt(2) / 2 };
	Vs01.push_back(P0);

	P0 = { 48, 70, 0, 1 };
	Vs01.push_back(P0);
	P0 = { 54, 72.5, 0, 1 };
	Vs01.push_back(P0);

	P0 = { 60, 75, 0, 1 };
	Vs01.push_back(P0);
	P0 = { 80, 75, 0, sqrt(2) / 2 };
	Vs01.push_back(P0);

	P0 = { 80, 95, 0, 1 };
	Vs01.push_back(P0);
	P0 = { 80, 115, 0, sqrt(2) / 2 };
	Vs01.push_back(P0);

	P0 = { 60, 115, 0, 1 };
	Vs01.push_back(P0);
	P0 = { 40, 115, 0, sqrt(2) / 2 };
	Vs01.push_back(P0);

	P0 = { 40, 95, 0, 1 };
	Vs01.push_back(P0);
	P0 = { 40, 82.5, 0, 1 };
	Vs01.push_back(P0);

	P0 = { 40, 70, 0, 1 };
	Vs01.push_back(P0);
	P0 = { 40, 40, 0, sqrt(2) / 2 };
	Vs01.push_back(P0);

	P0 = { 10, 40, 0, 1 };
	Vs01.push_back(P0);
	P0 = { 5, 40, 0, 1 };
	Vs01.push_back(P0);

	P0 = { 0, 40, 0, 1 };
	Vs01.push_back(P0);
	P0 = { 0, 31, 0, 1 };
	Vs01.push_back(P0);

	P0 = { 0, 22, 0, 1 };
	Vs01.push_back(P0);

	int j = (Vs01.size() - 1) / 2;
	Sls01.resize(j);

	for (int i = 0; i<j; i++)
	{
		Sls01[i].m_CtrlPts.push_back(Vs01[i * 2]);
		Sls01[i].m_CtrlPts.push_back(Vs01[i * 2 +1]);
		Sls01[i].m_CtrlPts.push_back(Vs01[i * 2 +2]);
		Sls01[i].m_Degree = 2;
		Sls01[i].m_Knots = knots;
	}

	RWGeometric rwg;
	rwg.WriteSpline("E:\\Model\\PlaneQuad\\AllBoundary.txt", Sls01);

	////中间筋板拉伸
	//varray<Spline> Sls02;
	//varray<Vec4> Vs02;
	//knots.push_back(0);
	//knots.push_back(0);
	//knots.push_back(0);
	//knots.push_back(1);
	//knots.push_back(1);
	//knots.push_back(1);
	//P0 = { 0, 22, 0, 1 };
	//Vs02.push_back(P0);
	//P0 = { 0, 32, 0, sqrt(2) / 2 };
	//Vs02.push_back(P0);
	//P0 = { 10, 32, 0, 1 };
	//Vs02.push_back(P0);
	//P0 = { 48, 32, 0, sqrt(2) / 2 };
	//Vs02.push_back(P0);
	//P0 = { 48, 70, 0, 1 };
	//Vs02.push_back(P0);
	//P0 = { 54, 72.5, 0, 1 };
	//Vs02.push_back(P0);
	//P0 = { 60, 75, 0, 1 };
	//Vs02.push_back(P0);
	//P0 = { 80, 75, 0, sqrt(2) / 2 };
	//Vs02.push_back(P0);
	//P0 = { 80, 95, 0, 1 };
	//Vs02.push_back(P0);
	//P0 = { 88.813, 69.313, 0, 0.894 };//这个点可能有问题
	//Vs02.push_back(P0);
	//P0 = { 69.442, 50.279, 0, 1 };
	//Vs02.push_back(P0);
	//P0 = { 65.837, 23.362, 0, 0.894 };
	//Vs02.push_back(P0);
	//P0 = { 40, 15, 0, 1 };
	//Vs02.push_back(P0);
	//P0 = { 20, 0, 0, 1 };
	//Vs02.push_back(P0);
	//P0 = { 0, -15, 0, 1 };
	//Vs02.push_back(P0);
	//P0 = { 0, 3.5, 0, 1 };
	//Vs02.push_back(P0);
	//P0 = { 0, 22, 0, 1 };
	//Vs02.push_back(P0);
	//j = (Vs02.size() - 1) / 2;
	//Sls02.resize(j);
	//for (int i = 0; i < j; i++)
	//{
	//	Sls02[i].m_CtrlPts.push_back(Vs02[i * 2]);
	//	Sls02[i].m_CtrlPts.push_back(Vs02[i * 2 + 1]);
	//	Sls02[i].m_CtrlPts.push_back(Vs02[i * 2 + 2]);
	//	Sls02[i].m_Degree = 2;
	//	Sls02[i].m_Knots = knots;
	//}

	//中间筋板拉伸
	varray<Spline> Sls02;
	varray<Vec4> Vs02;
	knots.clear();
	knots.push_back(0);
	knots.push_back(0);
	knots.push_back(0);
	knots.push_back(1);
	knots.push_back(1);
	knots.push_back(1);

	P0 = { 0, 22, 0, 1 };
	Vs02.push_back(P0);
	P0 = { 0, 32, 0, sqrt(2) / 2 };
	Vs02.push_back(P0);

	P0 = { 10, 32, 0, 1 };
	Vs02.push_back(P0);
	P0 = { 48, 32, 0, sqrt(2) / 2 };
	Vs02.push_back(P0);

	P0 = { 48, 70, 0, 1 };
	Vs02.push_back(P0);
	P0 = { 54, 72.5, 0, 1 };
	Vs02.push_back(P0);

	P0 = { 60, 75, 0, 1 };
	Vs02.push_back(P0);
	P0 = { 80, 75, 0, sqrt(2) / 2 };
	Vs02.push_back(P0);

	P0 = { 80, 95, 0, 1 };
	Vs02.push_back(P0);
	P0 = { 89.071, 40.464, 0, 0.6 };//这个点可能有问题
	Vs02.push_back(P0);

	P0 = { 40, 15, 0, 1 };
	Vs02.push_back(P0);
	P0 = { 20, 0, 0, 1 };
	Vs02.push_back(P0);

	P0 = { 0, -15, 0, 1 };
	Vs02.push_back(P0);
	P0 = { 0, 3.5, 0, 1 };
	Vs02.push_back(P0);

	P0 = { 0, 22, 0, 1 };
	Vs02.push_back(P0);

	j = (Vs02.size() - 1) / 2;
	Sls02.resize(j);

	for (int i = 0; i < j; i++)
	{
		Sls02[i].m_CtrlPts.push_back(Vs02[i * 2]);
		Sls02[i].m_CtrlPts.push_back(Vs02[i * 2 + 1]);
		Sls02[i].m_CtrlPts.push_back(Vs02[i * 2 + 2]);
		Sls02[i].m_Degree = 2;
		Sls02[i].m_Knots = knots;
	}
	Spline s = Sls02[4];
	varray<Spline> S;
	s.Segmentation(0.5, S);
	Sls02[4] = S[0];
	Sls02.push_back(S[1]);
	rwg.WriteSpline("E:\\Model\\PlaneQuad\\AllBoundary.txt", Sls02);

	varray<Spline> outer;
	varray<varray<Spline>> inner;
	outer = Sls02;
	//CDT_Operate cdto(outer, inner);
	varray<SplineSurface> allSurface;
	varray<Spline> temp;
	PublicSolution ps;

	// 亏格设置
	varray<bool> genus;
	genus.resize(1);
	genus[0] = false;
	/*genus[1] = true;
	genus[2] = true;
	genus[3] = true;*/
	varray<Spline> addLines;
	ps.quad(outer, inner, genus, allSurface);
	//ps.quad(outer, inner, cdto.addLine, genus, allSurface);
	rwg.WriteSpline("E:\\Model\\PlaneQuad\\Delaunay\\Boundary.txt", temp);
	rwg.WriteSplineSurface("E:\\Model\\PlaneQuad\\QuadSurface\\Yibo.txt", allSurface);

}

void test26()
{
	Vec4 p1 = { -2.73,-1.23,0,1 };
	Vec4 p2 = { -2.73,1.23,0,1 };
	Vec4 p3 = { -1.23,2.73,0,1 };
	Vec4 p4 = { 0,3,0,1 };
	Vec4 p5 = { 3,0,0,1 };
	Vec4 p6 = { 0,-3,0,1 };
	Vec4 p7 = { -1.23,-2.73,0,1 };
	Vec4 p8 = { -6,4,0,1 };
	Vec4 p9 = { -6,6,0,1 };
	Vec4 p10 = { -4,6,0,1 };
	Vec4 p11 = { -1.65,3.65,0,1 };
	Vec4 p12 = { 0,4,0,1 };
	Vec4 p13 = { 4,0,0,1 };
	Vec4 p14 = { 0,-4,0,1 };
	Vec4 p15 = { -1.65,-3.65,0,1 };
	Vec4 p16 = { -4,-6,0,1 };
	Vec4 p17 = { -6,-6,0,1 };
	Vec4 p18 = { -6,-4,0,1 };
	Vec4 p19 = { -3.65,-1.65,0,1 };
	Vec4 p20 = { -3.65,1.65,0,1 };

	Vec4 q1 = { -1.82,0.82,0,1 };
	Vec4 q2 = { -0.82,1.82,0,1 };
	Vec4 q3 = { 0,2,0,1 };
	Vec4 q4 = { 2,0,0,1 };
	Vec4 q5 = { 0,-2,0,1 };
	Vec4 q6 = { -0.82,-1.82,0,1 };
	Vec4 q7 = { -1.82,-0.82,0,1 };


	Vec4 o1 = {-5,-5,0,1};
	Vec4 o2 = { -5,5,0,1 }; 
	Vec4 o3 = { 0,0,0,1 };
	Curve curve;
	varray<Spline> outer;
	varray<Spline> S;
	varray<Spline> temp;
	varray<varray<Spline>>inner;
	Boundary bo;
	S.push_back(curve.getArcSpline(p1, p2, o3));
	S.push_back(curve.getArcSpline(p2, p3, o3));
	S.push_back(curve.getArcSpline(p3, p4, o3));
	S.push_back(curve.getArcSpline(p4, p5, o3));
	S.push_back(curve.getArcSpline(p5, p6, o3));
	S.push_back(curve.getArcSpline(p6, p7, o3));
	S.push_back(curve.getArcSpline(p7, p1, o3));
	inner.push_back(S);

	S = bo.getCircle(0.71);
	Model_Solution m;
	m.Rolate(S, PI / 4, 3);
	m.Trans(S, o1.x, 1);
	m.Trans(S, o1.y, 2);
	inner.push_back(S);

	m.Trans(S, 10, 2);
	inner.push_back(S);

	S.clear();
	S.push_back(curve.getArcSpline(q1, q2, o3));
	S.push_back(curve.getArcSpline(q2, q3, o3));
	S.push_back(curve.getArcSpline(q3, q4, o3));
	S.push_back(curve.getArcSpline(q4, q5, o3));
	S.push_back(curve.getArcSpline(q5, q6, o3));
	S.push_back(curve.getArcSpline(q6, q7, o3));
	S.push_back(curve.getArcSpline(q7, q1, o3));
	//inner.push_back(S);

	outer.push_back(curve.getArcSpline(p8, p9, o2));
	outer.push_back(curve.getArcSpline(p9, p10, o2));
	outer.push_back(curve.getSpline(p10,p11));
	outer.push_back(curve.getArcSpline(p11, p12, o3));
	outer.push_back(curve.getArcSpline(p12, p13, o3));
	outer.push_back(curve.getArcSpline(p13, p14, o3));
	outer.push_back(curve.getArcSpline(p14, p15, o3));
	outer.push_back(curve.getSpline(p15, p16));
	outer.push_back(curve.getArcSpline(p16, p17, o1));
	outer.push_back(curve.getArcSpline(p17, p18, o1));
	outer.push_back(curve.getSpline(p18, p19));
	outer.push_back(curve.getArcSpline(p19, p20, o3));
	outer.push_back(curve.getSpline(p20, p8));

	temp = outer;
	for (const auto &i : inner)
	{
		for (const auto &j : i)
		{
			temp.push_back(j);
		}
	}
	RWGeometric rwg;
	rwg.WriteSpline("E:\\Model\\PlaneQuad\\allBoundry.txt", temp);
	//CDT_Operate cdto(outer, inner);
	// 亏格设置
	varray<bool> genus;
	genus.resize(4);
	genus[0] = false;
	genus[1] = true;
	genus[2] = true;
	genus[3] = true;
	//genus[4] = true;
	varray<SplineSurface> SS;
	PublicSolution ps;
	varray<Spline> addLines;
	
	CreateAddlineByDelaunay cabd(outer, inner);

	for (const auto &i : cabd.m_addLine)
	{
		addLines.push_back(i);
	}
	/*addLines.push_back(curve.getArcSpline(p20, p11, o3));
	addLines.push_back(curve.getArcSpline(p15, p19, o3));*/
	ps.quad(outer, inner, addLines, genus, SS);
	rwg.WriteSplineSurface("E:\\Model\\PlaneQuad\\QuadSurface\\ModelTest.txt", SS);
	test(SS);
	
}
//四边形网格生成技术研究图3.12
void test27()
{
	Curve curve;
	varray<Spline> outer;
	varray<Spline> S;
	varray<Spline> temp;
	varray<varray<Spline>>inner;
	Boundary bo;
	RWGeometric rwg;
	Model_Solution m;
	varray<SplineSurface> SS1;
	varray<SplineSurface> SS2;
	PublicSolution ps;
	varray<Spline> addLines;
	Vec4 p5 = { -3.5,3.5,0,1 };
	Vec4 p6 = { -3,4,0,1 };
	Vec4 p7 = { 0,4,0,1 };
	Vec4 p8 = { 4,4,0,1 };
	Vec4 p9 = { 4,-4,0,1 };
	Vec4 p10 = { -4,-4,0,1 };
	Vec4 p11 = { -4,0,0,1 };
	Vec4 p12 = { -4,3,0,1 };
	outer.push_back(curve.getSpline(p5, p6));
	outer.push_back(curve.getSpline(p6, p7));
	outer.push_back(curve.getSpline(p7, p8));
	outer.push_back(curve.getSpline(p8, p9));
	outer.push_back(curve.getSpline(p9, p10));
	outer.push_back(curve.getSpline(p10, p11));
	outer.push_back(curve.getSpline(p11, p12));
	outer.push_back(curve.getSpline(p12, p5));
	S = bo.getCircle(2);
	m.Rolate(S,PI/4,3);
	inner.push_back(S);
	temp = outer;
	for (const auto &i : inner)
	{
		for (const auto &j : i)
		{
			temp.push_back(j);
		}
	}
	rwg.WriteSpline("E:\\Model\\PlaneQuad\\allBoundry.txt", temp);

	varray<bool> genus;
	genus.resize(2);
	genus[0] = false;
	genus[1] = true;

	/*CreateAddlineByDelaunay cabd1(outer, inner);

	for (const auto &i : cabd1.m_addLine)
	{
		addLines.push_back(i);
	}
	ps.quad(outer, inner, addLines, genus, SS1);
	rwg.WriteSplineSurface("E:\\Model\\PlaneQuad\\QuadSurface\\Model3.12.1.txt", SS1);*/


	Vec4 q1 = { -10,3,0,1 };
	Vec4 q2 = { -9,4,0,1 };
	Vec4 q3 = { -4,4,0,1 };
	Vec4 q4 = { 4,4,0,1 };
	Vec4 q5 = { 9,4,0,1 };
	Vec4 q6 = { 10,3,0,1 };
	Vec4 q7 = { 10, -3, 0, 1 };
	Vec4 q8 = { 9,-4,0,1 };
	Vec4 q9 = { 4,-4,0,1 };
	Vec4 q10 = { -4,-4,0,1 };
	Vec4 q11 = { -9,-4,0,1 };
	Vec4 q12 = { -10,-3,0,1 };
	outer.clear();
	outer.push_back(curve.getSpline(q1,q2));
	outer.push_back(curve.getSpline(q2, q3));
	outer.push_back(curve.getSpline(q3, q4));
	outer.push_back(curve.getSpline(q4, q5));
	outer.push_back(curve.getSpline(q5, q6));
	outer.push_back(curve.getSpline(q6, q7));
	outer.push_back(curve.getSpline(q7, q8));
	outer.push_back(curve.getSpline(q8, q9));
	outer.push_back(curve.getSpline(q9, q10));
	outer.push_back(curve.getSpline(q10, q11));
	outer.push_back(curve.getSpline(q11, q12));
	outer.push_back(curve.getSpline(q12, q1));
	Vec4 o1 = { -9,3,0,1 };
	Vec4 o2 = { 9,3,0,1 };
	Vec4 o3 = { 9,-3,0,1 };
	Vec4 o4 = { -9,-3,0,1 };
	inner.clear();
	S = bo.getCircle(0.5);
	//m.Rolate(S, PI / 4, 3);
	m.Trans(S, o1.x, 1);
	m.Trans(S, o1.y, 2);
	
	inner.push_back(S);
	
	S = bo.getCircle(0.5);
	//m.Rolate(S, PI / 4, 3);
	m.Trans(S, o2.x, 1);
	m.Trans(S, o2.y, 2);
	
	inner.push_back(S);
	
	S = bo.getCircle(0.5);
	//m.Rolate(S, PI / 4, 3);
	m.Trans(S, o3.x, 1);
	m.Trans(S, o3.y, 2);
	
	inner.push_back(S);
	
	S = bo.getCircle(0.5);
	//m.Rolate(S, PI / 4, 3);
	m.Trans(S, o4.x, 1);
	m.Trans(S, o4.y, 2);
	
	inner.push_back(S);
	
	S = bo.getSquare(8, 2);
	m.Trans(S, 2, 2);
	inner.push_back(S);
	
	S = bo.getSquare(8, 2);
	m.Trans(S, 2, -2);
	inner.push_back(S);
	
	temp = outer;
	for (const auto &i : inner)
	{
		for (const auto &j : i)
		{
			temp.push_back(j);
		}
	}
	rwg.WriteSpline("E:\\Model\\PlaneQuad\\allBoundry.txt", temp);

	genus.resize(7);
	genus[0] = false;
	genus[1] = true;
	genus[2] = true;
	genus[3] = true;
	genus[4] = true;
	genus[5] = false;
	genus[6] = false;

	CreateAddlineByDelaunay cabd2(outer, inner);

	for (const auto &i : cabd2.m_addLine)
	{
		addLines.push_back(i);
	}
	ps.quad(outer, inner, addLines, genus, SS2);
	rwg.WriteSplineSurface("E:\\Model\\PlaneQuad\\QuadSurface\\Model3.12.2.txt", SS2);
}

//无孔模型
void test28()
{
	Curve curve;
	varray<Spline> outer;
	varray<Spline> S;
	varray<Spline> temp;
	varray<varray<Spline>>inner;
	Boundary bo;
	RWGeometric rwg;
	Model_Solution m;
	varray<SplineSurface> SS1;
	varray<SplineSurface> SS2;
	PublicSolution ps;
	varray<Spline> addLines;
	varray<bool> genus;
	Vec4 p1 = { -8,8,0,1 };
	Vec4 p2 = { -6,10,0,1 };
	Vec4 p3 = { -2,10,0,1 };
	Vec4 p4 = { -2,6,0,1 };
	Vec4 p5 = { -1,5,0,1 };
	Vec4 p6 = { 1,5,0,1 };
	Vec4 p7 = { 2,6,0,1 };
	Vec4 p8 = { 2,10,0,1 };
	Vec4 p9 = { 6,10,0,1 };
	Vec4 p11 = { 8,8,0,1 };
	Vec4 p12 = { 8,-10,0,1 };
	Vec4 p13 = { 6,-10,0,1 };
	Vec4 p14 = { 6,-9,0,1 };
	Vec4 p15 = { 5,-8,0,1 };
	Vec4 p16 = { 4,-9,0,1 };
	Vec4 p17 = { 4,-10,0,1 };
	Vec4 p18 = { -4,-10,0,1 };
	Vec4 p19 = { -4,-9,0,1 };
	Vec4 p20 = { -5,-8,0,1 };
	Vec4 p21 = { -6,-9,0,1 };
	Vec4 p22 = { -6,-10,0,1 };
	Vec4 p23 = { -8,-10,0,1 };

	Vec4 o1 = { -8,10,0,1 };
	Vec4 o2 = { 8,10,0,1 };
	Vec4 o3 = { -1,6,0,1 };
	Vec4 o4 = { 1,6,0,1 };
	Vec4 o5 = { -5,-9,0,1 };
	Vec4 o6 = { 5,-9,0,1 };


	outer.push_back(curve.getArcSpline(p1, p2, o1));
	outer.push_back(curve.getSpline(p2,p3));
	outer.push_back(curve.getSpline(p3, p4));
	outer.push_back(curve.getArcSpline(p4, p5, o3));
	outer.push_back(curve.getSpline(p5, p6));
	outer.push_back(curve.getArcSpline(p6, p7, o4));
	outer.push_back(curve.getSpline(p7, p8));
	outer.push_back(curve.getSpline(p8, p9));
	outer.push_back(curve.getArcSpline(p9, p11, o2));
	outer.push_back(curve.getSpline(p11, p12));
	outer.push_back(curve.getSpline(p12, p13));
	outer.push_back(curve.getSpline(p13, p14));
	outer.push_back(curve.getArcSpline(p14, p15, o6));
	outer.push_back(curve.getArcSpline(p15, p16, o6));
	outer.push_back(curve.getSpline(p16, p17));
	outer.push_back(curve.getSpline(p17, p18));
	outer.push_back(curve.getSpline(p18, p19));
	outer.push_back(curve.getArcSpline(p19, p20, o5));
	outer.push_back(curve.getArcSpline(p20, p21, o5));
	outer.push_back(curve.getSpline(p21, p22));
	outer.push_back(curve.getSpline(p22, p23));
	outer.push_back(curve.getSpline(p23, p1));
	rwg.WriteSpline("E:\\Model\\PlaneQuad\\allBoundry.txt", outer);
	genus.resize(1);
	genus[0] = false;
	CreateAddlineByDelaunay cabd(outer, inner);

	for (const auto &i : cabd.m_addLine)
	{
		addLines.push_back(i);
	}
	ps.quad(outer, inner, addLines, genus, SS2);
	rwg.WriteSplineSurface("E:\\Model\\PlaneQuad\\QuadSurface\\Model_28.txt", SS2);

}
//凹凸点判断
void test_BumpPiont()
{
	PublicSolution ps;
	Spline0 s0;
	varray<Spline> S;
	RWGeometric rwg;
	ConvexDecomposition convexDec;
	convexDec.test_PositiveOrNegative();
}

//机轮
void Model_chong()
{
	RWGeometric rwg;
	Boundary bo;
	varray<Spline> outer;
	varray<Spline> S;
	varray<varray<Spline>> inner;
	varray<SplineSurface> allSurface;
	PublicSolution ps;
	Model_Solution ms;
	outer = bo.getCircle(11);
	ms.Rolate(outer, PI / 4, 3);
	S = bo.getCircle(1.5);
	ms.Rolate(S, PI / 4, 3);
	ms.Trans(S, 7.5, -1);
	inner.push_back(S);

	S = bo.getCircle(1.5);
	ms.Rolate(S, PI / 4, 3);
	ms.Trans(S, 7.5, 1);
	inner.push_back(S);

	S = bo.getCircle(1.5);
	ms.Rolate(S, PI / 4, 3);
	ms.Trans(S, 7.5, -2);
	inner.push_back(S);

	S = bo.getCircle(1.5);
	ms.Rolate(S, PI / 4, 3);
	ms.Trans(S, 7.5, 2);
	inner.push_back(S);

	S = bo.getCircle(1.5);
	ms.Rolate(S, PI / 4, 3);
	ms.Trans(S, 5.3, -1);
	ms.Trans(S, 5.3, -2);
	inner.push_back(S);

	S = bo.getCircle(1.5);
	ms.Rolate(S, PI / 4, 3);
	ms.Trans(S, 5.3, -1);
	ms.Trans(S, 5.3, 2);
	inner.push_back(S);

	S = bo.getCircle(1.5);
	ms.Rolate(S, PI / 4, 3);
	ms.Trans(S, 5.3, 1);
	ms.Trans(S, 5.3, -2);
	inner.push_back(S);
	   
	S = bo.getCircle(1.5);
	ms.Rolate(S, PI / 4, 3);
	ms.Trans(S, 5.3, 1);
	ms.Trans(S, 5.3, 2);
	inner.push_back(S);

	S = bo.getCircle(5);
	ms.Rolate(S, PI / 4, 3);
	inner.push_back(S);

	varray<Spline> temp;
	for (const auto &i : outer)
	{
		temp.push_back(i);
	}

	for (const auto &i : inner)
	{
		for (const auto &j : i)
		{
			temp.push_back(j);
		}
	}

	rwg.WriteSpline("E:\\Model\\PlaneQuad\\Delaunay\\Boundary.txt", temp);
	CDT_Operate cdto(outer, inner);
	// 亏格设置
	varray<bool> genus;
	genus.resize(10);
	genus[0] = false;
	genus[1] = true;
	genus[2] = true;
	genus[3] = true;
	genus[4] = true;
	genus[5] = true;
	genus[6] = true;
	genus[7] = true;
	genus[8] = true;
	genus[9] = true;
	varray<Spline> addLines;
	for (int i = 0; i < 6; i++)
	{
		addLines.push_back(cdto.addLine[i]);
	}
	ps.quad(outer, inner, addLines, genus, allSurface);
	rwg.WriteSplineSurface("E:\\Model\\PlaneQuad\\QuadSurface\\SlidewaySurface.txt", allSurface);
	
}

void test_CDT()
{
	Boundary bo;
	Spline s;
	Spline0 s0;
	Model_Solution m;
	varray<Spline> outer;
	varray<varray<Spline>> inner;
	varray<Spline> temp;
	varray<Spline> S;
	RWGeometric rwg;

	Vec4 p1 = { -40,0,0,1 };
	Vec4 p2 = { -5.28,0,0,1 };
	Vec4 p3 = { 7.98,18.03,0,1 };///1
	Vec4 p4 = { 26.44,5.4,0,1 };
	Vec4 p5 = { 33.43,5.4,0,1 };
	Vec4 p6 = { 33.43,0,1 };
	Vec4 p7 = { 40,0,0,1 };
	Vec4 p8 = { 18,35.55,0,1 };
	Vec4 p9 = { 18,25,0,1 };
	Vec4 p10 = { 0,25,0,1 };
	Vec4 p11 = { 0,40,0,1 };

	Vec4 O = { 0,0,0,1 };
	Vec4 O1 = { -20,15,0,1 };
	Vec4 O2 = { 10.67,2.16,0,1 };

	outer.push_back(s0.getSpline(p1, p2));
	outer.push_back(s0.getArcSpline(p2, p3, O2));
	outer.push_back(s0.getArcSpline(p3, p4, O2));
	outer.push_back(s0.getSpline(p4, p5));
	outer.push_back(s0.getSpline(p5, p6));
	outer.push_back(s0.getSpline(p6, p7));
	outer.push_back(s0.getArcSpline(p7, p8, O));
	outer.push_back(s0.getSpline(p8, p9));
	outer.push_back(s0.getSpline(p9, p10));
	outer.push_back(s0.getSpline(p10, p11));
	outer.push_back(s0.getArcSpline(p11, p1, O));

	S = bo.getCircle(10.76);
	m.Rolate(S, PI / 4, 3);
	m.Trans(S, O1.x, 1);
	m.Trans(S, O1.y, 2);
	inner.push_back(S);
	temp = outer;
	for (auto &i : S)
	{
		temp.push_back(i);
	}
	rwg.WriteSpline("E:\\Model\\PlaneQuad\\Delaunay\\Boundary.txt", temp);

	CreateAddlineByDelaunay cabd2(outer, inner);

	//亏格设置
	varray<bool> genus;
	genus.resize(2);
	genus[0] = false;
	genus[1] = false;
	varray<SplineSurface> allSurface;
	PublicSolution ps;
	//ps.quad(outer, inner, cdto.addLine, genus, allSurface);
}
void test_s1955(Spline s1, Spline s2, double &u1, double &u2)
{
	SISLCurve *curve1;					/* 指向最近点问题中的第一条曲线的指针。 */
	SISLCurve *curve2;					/* 指针指向最近点问题中的第二条曲线。 */
	double epsco = 1.0e-9;				/* 计算分辨率(未使用)。 */
	double epsge = 1.0e-6;				/*几何分辨率*/
	int numintpt = 0;					/*最近点的个数。*/
	double *intpar1 = NULL;				/*数组，其中包含第一条曲线参数区间内最接近的单个点的参数值。
										这些点按顺序排列。最接近的曲线存储在intcurve中*/

	double *intpar2 = NULL;				/*数组，其中包含第二条曲线参数区间内最接近的单个点的参数值。
										这些点按顺序排列。最接近的曲线存储在intcurve中。*/
	int numintcu = 0;					/*最近曲线的数目*/

	SISLIntcurve **intcurve = NULL;		/*指向SISLIntcurve对象的指针数组，其中包含最近曲线的描述。
										曲线仅用曲线参数区间内的起点和终点来描述。曲线指针没有指向任何东西。
										如果作为输入的曲线是退化的，则可以返回最近点作为最近曲线。*/

	int stat = 0;						/*状态消息
										> 0:警告
										= 0:ok
										< 0:错误*/
	curve1 = NurbsLineToSislLine(s1);
	curve2 = NurbsLineToSislLine(s2);

	s1955(curve1, curve2, epsco, epsge, &numintpt, &intpar1, &intpar2, &numintcu, &intcurve, &stat);

	u1 = *intpar1;
	u2 = *intpar2;
}

void create_Model()
{

}

//Coons插值测试
void test_COONS()
{
	Model_Solution ms;
	PublicSolution ps;
	RWGeometric rwg;

	SplineSurface ss;
	Curve curve;
	varray<Spline> S;
	double r = 5;
	double l = sin(PI/180*150)*r;
	
	Vec4 o = {0,l,-l,1};
	Vec4 p1 = { -l,l,0,1 };
	Vec4 p2 = {l,l,0,1};
	
	
	Spline s = curve.getArcSpline(p1, p2, o);

	S.push_back(s);
	ms.Rolate(s, PI / 2, 3);
	S.push_back(s);
	ms.Rolate(s, PI / 2, 3);
	S.push_back(s);
	ms.Rolate(s, PI / 2, 3);
	S.push_back(s);

	rwg.WriteSpline("E:\\Model\\PlaneQuad\\Delaunay\\Boundary.txt", S);
	ms.OrderCoonsLines(S);
	rwg.WriteSpline("E:\\Model\\PlaneQuad\\Delaunay\\Boundary.txt", S);
	ss.CoonsInterpolate(S);
	varray<SplineSurface>SS;
	SS.push_back(ss);
	rwg.WriteSplineSurface("E:\\Model\\PlaneQuad\\Delaunay\\SplineSurface.txt",SS);

}

void test_NurbsLine()
{
	Spline SL;
	varray<Spline> S;
	Vec4 v1 = { -0.900,0.930,-0.443,1.000 };
	Vec4 v2 = { -0.970,	0.589,0.506,1.000 };
	Vec4 v3 = { 0.033, -0.280,0.813,1.000 };
	Vec4 v4 = { 0.387,-0.649,-0.830,1.000 };
	Vec4 v5 = { 0.737,-0.668,0.408,1.000 };
	Vec4 v6 = { -0.965,0.502,-0.958,1.000 };
	varray<double> knots;
	knots.push_back(0);
	knots.push_back(0);
	knots.push_back(0);
	knots.push_back(0.25);
	knots.push_back(0.5);
	knots.push_back(0.75);
	knots.push_back(1);
	knots.push_back(1);
	knots.push_back(1);
	SL.m_Knots = knots;
	SL.m_Degree = 2;
	SL.m_CtrlPts.push_back(v1);
	SL.m_CtrlPts.push_back(v2);
	SL.m_CtrlPts.push_back(v3);
	SL.m_CtrlPts.push_back(v4);
	SL.m_CtrlPts.push_back(v5);
	SL.m_CtrlPts.push_back(v6);
	S.push_back(SL);
	RWGeometric rwg;
	rwg.WriteSpline("E:\\Model\\PlaneQuad\\Delaunay\\Line.txt", S);
	
}
void modelQualityTest()
{
	RWGeometric rwg;
	varray<SplineVolume> SV;
	varray<SplineSurface> SS;
	rwg.ReadSplineSurface("E:\\Model\\PlaneQuad\\Delaunay\\allSurface.txt", SS);
	Model_Solution m;
	SV = m.CreatSweepVol(SS, 1, 3);

	for (auto &i : SV)
	{
		i.OrderCtrlPts(i);
	}

	rwg.WriteSplineVolume("E:\\Model\\Jacobian\\Volume.txt", SV);
	putOutVTK();
}
//输出VTK文件
void putOutVTK()
{
	string path = "E:\\Model\\Jacobian\\";
	string modelName = "Flange.txt";
	string VTKName = "Volume.vtk";

	varray<SplineVolume>SV;
	RWGeometric rwg;
	rwg.ReadSplineVolume(path + modelName, SV);
	for (int i=0;i<SV.size();++i)
	{
		SV[i].OrderCtrlPts(SV[i]);
	}
	varray<NurbsVol> NV;

	NV = NurbsTrans::SplinevolsToCvols(SV);

	CPolyParaVolume cp;       //输出vtk文件的类对象
	cp = NV;
	cp.OutputParaVolumeDataVTK(path + VTKName);
}
void putOutVTK(varray<SplineVolume>SV)
{
	varray<NurbsVol> NV;
	NV = NurbsTrans::SplinevolsToCvols(SV);
	CPolyParaVolume cp;       //输出vtk文件的类对象
	cp = NV;
	cp.OutputParaVolumeDataVTK("E:\\Model\\Jacobian\\Volume.vtk");
}

void Square_Circle()
{
	Model_Solution m;
	varray<Spline> temp;
	varray<Spline> outer;
	varray<varray<Spline>> inner;
	Boundary b;
	RWGeometric rwg;

	outer = b.getSquare(40, 40);

	inner.push_back(b.getSquare(20, 20));
	//inner.push_back(b.getSquare(10, 10));

	//亏格设置
	varray<bool> genus;
	genus.resize(2);
	genus[0] = false;
	genus[1] = false;
	//genus[2] = false;

	//平面剖分
	quadPlane(outer, inner, genus);
}
int ShowPlatform(int argc, char *argv[]) {
	QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
	QApplication a(argc, argv);
	QtGuiApplication1 w;
	w.show();
	return a.exec();
}

void writeInnerBoundry(varray<varray<Spline>> inner) {
	RWGeometric rwg;
	int n = 0;
	string str = "E:\\Model\\PlaneQuad\\InnerBoundry0.txt";
	string ch;
	for (auto i : inner)
	{
		ch = to_string(n++);
		str.replace(str.begin() + 31, str.end() - 4, ch.begin(), ch.end());
		rwg.WriteSpline(str, i);
	}
}

varray<varray<Spline>> readInnerBoundry(int num) {
	RWGeometric rwg;
	int n = 0;
	string str0 = "E:\\Model\\PlaneQuad\\InnerBoundry0.txt";
	string str1 = "E:\\Model\\PlaneQuad\\InnerBoundry1.txt";
	string str2 = "E:\\Model\\PlaneQuad\\InnerBoundry2.txt";
	string str3 = "E:\\Model\\PlaneQuad\\InnerBoundry3.txt";
	string str4 = "E:\\Model\\PlaneQuad\\InnerBoundry4.txt";
	string str5 = "E:\\Model\\PlaneQuad\\InnerBoundry5.txt";
	string str6 = "E:\\Model\\PlaneQuad\\InnerBoundry6.txt";
	string str7 = "E:\\Model\\PlaneQuad\\InnerBoundry7.txt";
	string str8 = "E:\\Model\\PlaneQuad\\InnerBoundry8.txt";
	string str9 = "E:\\Model\\PlaneQuad\\InnerBoundry9.txt";
	string str10 = "E:\\Model\\PlaneQuad\\InnerBoundry10.txt";
	string str11 = "E:\\Model\\PlaneQuad\\InnerBoundry11.txt";
	string str12 = "E:\\Model\\PlaneQuad\\InnerBoundry12.txt";

	varray<string> v;
	v.push_back(str0);
	v.push_back(str1);
	v.push_back(str2);
	v.push_back(str3);
	v.push_back(str4);
	v.push_back(str5);
	v.push_back(str6);
	v.push_back(str7);
	v.push_back(str8);
	v.push_back(str9);
	v.push_back(str10);
	v.push_back(str11);
	v.push_back(str12);

	varray<varray<Spline>> inner;
	varray<Spline> temp;

	for (auto i = 0; i < num; i++)
	{
		rwg.ReadSpline(v[i], temp);
		inner.push_back(temp);
	}
	return inner;
}

//int main()
//{
//	//施加力与约束
//		PublicSolution ps;
//	ps.setWCandWF("E:\\Model\\WcWfFile\\VolumeTest.txt", "E:\\Model\\WcWfFile\\VolumeTest");
//
//	rwg.writenurbsline("splinetest.txt", nl);
//	rwg.ReadSplineSurface("E:\\Model\\BoundarySurface.txt", SS);
//	rwg.WriteSplineVolume("VolumeTest.txt", m.CreatSweepVol(SS, 10, 3));
//
//	rwg.ReadSplineVolume("zhouchengzuo2.txt", SV);
//	for (auto&i : SV) {
//		i.KnotsRefineNum(1);//细化
//	}
//	RWGeometric rwg;
//	varray<NurbsLine> nl;
//	nl.push_back(NurbsTrans::SplineToCnurbsline(createSpline(10)));
//	rwg.WriteNurbsLine("SplineTest.txt", nl);
//	rwg.WriteSplineSurface("SurfaceTest.txt", lrg.square_Circle());
//	rwg.WriteSplineVolume("VolumeTest.txt", cr.get_vol(5, 10, 10));
//	rwg.WriteSplineVolume("VolumeTest.txt", lrg.get_Model03());
//
//	quadModel();
//}