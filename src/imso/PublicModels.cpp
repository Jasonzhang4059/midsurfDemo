#pragma once
#include "PublicModels.h"

varray<SplineSurface> RandomModel::getRecArcSurface() {
	PublicSolution ps;
	varray<SplineSurface> SS, SS1, SS2;
	SplineSurface temp;
	varray<Spline> SL1, SL2;
	varray<double> knots;
	knots.push_back(0);
	knots.push_back(0);
	knots.push_back(0);
	knots.push_back(1);
	knots.push_back(1);
	knots.push_back(1);
	SL1.resize(4);
	SL2.resize(4);
	for (int i = 0; i < 4; i++) {
		SL1[i].m_Degree = 2;
		SL1[i].m_Knots = knots;
		SL2[i].m_Degree = 2;
		SL2[i].m_Knots = knots;
	}
	double r1 = ((a > b) ? b : a) / 4;//半径取边长最小的1/4

	Vec4 v1 = { -b / 2,-a / 2,0,1 };
	Vec4 v2 = { -b / 2,a / 2,0,1 };
	Vec4 v3 = { -cos(PI / 4)*r1,sin(PI / 4)*r1,0,1 };
	Vec4 v4 = { -sqrt(2)*r1,0,0,w };
	Vec4 v5 = { -cos(PI / 4)*r1,-sin(PI / 4)*r1,0,1 };
	Vec4 v6 = { 0,sqrt(2)*r1,0,w };
	Vec4 v7 = { cos(PI / 4)*r1,sin(PI / 4)*r1,0,1 };
	Vec4 v8 = { b / 2,a / 2,0,1 };

	SL1[0].m_CtrlPts.push_back(v1);
	SL1[0].m_CtrlPts.push_back((v1 + v5) / 2);
	SL1[0].m_CtrlPts.push_back(v5);
	SL1[1].m_CtrlPts.push_back(v1);
	SL1[1].m_CtrlPts.push_back((v1 + v2) / 2);
	SL1[1].m_CtrlPts.push_back(v2);
	SL1[2].m_CtrlPts.push_back(v2);
	SL1[2].m_CtrlPts.push_back((v2 + v3) / 2);
	SL1[2].m_CtrlPts.push_back(v3);
	SL1[3].m_CtrlPts.push_back(v5);
	SL1[3].m_CtrlPts.push_back(v4);
	SL1[3].m_CtrlPts.push_back(v3);
	SplineSurface ss1;
	ss1.CoonsInterpolate(SL1);
	SS.push_back(ss1);
	temp = ps.mirror(ss1, 2);
	SS.push_back(temp);

	SL2[0].m_CtrlPts.push_back(v3);
	SL2[0].m_CtrlPts.push_back(v6);
	SL2[0].m_CtrlPts.push_back(v7);
	SL2[1].m_CtrlPts.push_back(v3);
	SL2[1].m_CtrlPts.push_back((v3 + v2) / 2);
	SL2[1].m_CtrlPts.push_back(v2);
	SL2[2].m_CtrlPts.push_back(v2);
	SL2[2].m_CtrlPts.push_back((v2 + v8) / 2);
	SL2[2].m_CtrlPts.push_back(v8);
	SL2[3].m_CtrlPts.push_back(v7);
	SL2[3].m_CtrlPts.push_back((v8 + v7) / 2);
	SL2[3].m_CtrlPts.push_back(v8);
	SplineSurface ss2;
	ss2.CoonsInterpolate(SL2);
	SS.push_back(ss2);
	temp = ps.mirror(ss2, 1);
	SS.push_back(temp);
	ps.quadAdjustUV(SS);

	return SS;
}




//原始燃料棒
void fuelRod() {
	double r1 = 0.925, r2 = 4.65, l = 0.8;

	//坐标点
	Vec4 v1 = { -2 * r1 - r2,0,0,1 };
	Vec4 v2 = { -2 * r1 - r2,r1,0,1 };
	Vec4 v3 = { -r1 - r2,r1,0,1 };
	Vec4 v4 = { -r1,r1,0,1 };
	Vec4 v5 = { -r1 ,r1 + r2,0,1 };
	Vec4 v6 = { -r1,r1 * 2 + r2,0,1 };
	Vec4 v7 = { 0,r1 * 2 + r2,0,1 };
	Vec4 v8 = { r1,r1 * 2 + r2,0,1 };
	Vec4 v9 = { r1 ,r1 + r2,0,1 };
	Vec4 v10 = { r1,r1,0,1 };
	Vec4 v11 = { r1 + r2,r1,0,1 };
	Vec4 v12 = { 2 * r1 + r2,r1,0,1 };
	Vec4 v13 = { 2 * r1 + r2,0,0,1 };

	Vec4 v14 = { 2 * r1 + r2,-r1,0,1 };
	Vec4 v15 = { r1 + r2,-r1,0,1 };
	Vec4 v16 = { r1,-r1,0,1 };
	Vec4 v17 = { r1 ,-r1 - r2,0,1 };
	Vec4 v18 = { r1,-r1 * 2 - r2,0,1 };
	Vec4 v19 = { 0,-r1 * 2 - r2,0,1 };
	Vec4 v20 = { -r1,-r1 * 2 - r2,0,1 };
	Vec4 v21 = { -r1 ,-r1 - r2,0,1 };
	Vec4 v22 = { -r1,-r1,0,1 };
	Vec4 v23 = { -r1 - r2,-r1,0,1 };
	Vec4 v24 = { -2 * r1 - r2,-r1,0,1 };

	Vec4 v25 = { -l,0,0,1 };
	Vec4 v26 = { 0,l,0,1 };
	Vec4 v27 = { l,0,0,1 };
	Vec4 v28 = { 0,-l,0,1 };

	varray<Spline> sps, sps1, sps2;
	Spline0 sl;
	Spline sp;
	sp = sl.getArcSpline(v1, v2, v3);
	sps.push_back(sp);
	sp = sl.getArcSpline(v3, v4, v5);
	sps.push_back(sp);
	sp = sl.getArcSpline(v5, v6, v7);
	sps.push_back(sp);
	sp = sl.getArcSpline(v7, v8, v9);
	sps.push_back(sp);
	sp = sl.getArcSpline(v9, v10, v11);
	sps.push_back(sp);
	sp = sl.getArcSpline(v11, v12, v13);
	sps.push_back(sp);
	sp = sl.getArcSpline(v13, v14, v15);
	sps.push_back(sp);
	sp = sl.getArcSpline(v15, v16, v17);
	sps.push_back(sp);
	sp = sl.getArcSpline(v17, v18, v19);
	sps.push_back(sp);
	sp = sl.getArcSpline(v19, v20, v21);
	sps.push_back(sp);
	sp = sl.getArcSpline(v21, v22, v23);
	sps.push_back(sp);
	sp = sl.getArcSpline(v23, v24, v1);
	sps.push_back(sp);

	sp = sl.getSpline(v25, v26);
	sps1.push_back(sp);
	sp = sl.getSpline(v26, v27);
	sps1.push_back(sp);
	sp = sl.getSpline(v27, v28);
	sps1.push_back(sp);
	sp = sl.getSpline(v28, v25);
	sps1.push_back(sp);

	sp = sl.getSpline(v1, v25);
	sps2.push_back(sp);
	sp = sl.getSpline(v7, v26);
	sps2.push_back(sp);
	sp = sl.getSpline(v13, v27);
	sps2.push_back(sp);
	sp = sl.getSpline(v19, v28);
	sps2.push_back(sp);


	RWGeometric rwg;
	rwg.WriteSpline("E:\\kuang_models\\outSpline.txt", sps);
	rwg.WriteSpline("E:\\kuang_models\\inSpline.txt", sps1);
	rwg.WriteSpline("E:\\kuang_models\\addSpline.txt", sps2);

	varray<Spline>outer;//存放外轮廓曲线
	varray<Spline>inner1;//存放内轮廓曲线
	varray<varray<Spline>> inner;
	varray<Spline>addlines, allLines;//辅助线 将内外轮廓连接起来变为零亏格
	varray<bool> genus;
	varray<SplineSurface> allSurf;//存放剖分结果

	rwg.ReadSpline("E:\\kuang_models\\outSpline.txt", outer);
	rwg.ReadSpline("E:\\kuang_models\\inSpline.txt", inner1);
	rwg.ReadSpline("E:\\kuang_models\\addSpline.txt", addlines);
	inner.push_back(inner1);
	genus.resize(2);
	genus[0] = false;//我之前用的是push_back,会出现问题
	genus[1] = false;

	PublicSolution ps;
	ps.quad(outer, inner, addlines, genus, allSurf);

	rwg.WriteSplineSurface("E:\\kuang_models\\newSurface.txt", allSurf);

	Model_Solution m;
	varray<SplineVolume> SV;
	SV = m.CreatSweepVol(allSurf, 5, 3);
	for (int i = 0; i < SV.size(); i++) {
		SV[i].KnotsRefineNum(1);
	}
	rwg.WriteSplineVolume("E:\\kuang_models\\newVolume.txt", SV);

	varray<NurbsVol> NV;
	NV = NurbsTrans::SplinevolsToCvols(SV);
	CPolyParaVolume cp;       //输出vtk文件的类对象
	cp = NV;
	cp.OutputParaVolumeDataVTK("E:\\kuang_models\\newVolume.vtk");

}

