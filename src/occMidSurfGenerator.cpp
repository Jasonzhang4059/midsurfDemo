#include <TopExp_Explorer.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <TopAbs.hxx>
#include <TopoDS.hxx>
#include <BRepBndLib.hxx>
#include <TopoDS_Compound.hxx>
#include <BRep_Builder.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepOffsetAPI_MakeOffsetShape.hxx>
#include <Poly_Triangulation.hxx>

#include <BRepFill_Filling.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <TopoDS_Wire.hxx>
#include <TopoDS_Face.hxx>

#include <openvdb/openvdb.h>
#include <openvdb/tools/MeshToVolume.h>

#include <TopoDS_Face.hxx>
#include <BRep_Tool.hxx>
#include <Geom_Surface.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <Standard_Real.hxx>

#include "occMidSurfGenerator.h"
#include "occMidSurfClassification.h"
#include "occMidSurfCommFunction.h"

MidSurfGenerator::MidSurfGenerator(TopoShapeArr & shapeArr):m_orgShapeArr(shapeArr)
{
	if (!shapeArr.empty())
	{
		for (TopoDS_Shape &shape : shapeArr)
		{
			for (TopExp_Explorer exp(shape, TopAbs_FACE); exp.More(); exp.Next())
			{
				TopoDS_Face face = TopoDS::Face(exp.Current());
				m_orgFacArr.push_back(face);
			}
		}
	}
}

MidSurfGenerator::~MidSurfGenerator()
{
}

void SavePointsToVDB(const std::vector<openvdb::Vec3d>& points, const std::string& filename) {
	//openvdb::initialize();

	//std::vector<openvdb::Vec3f> positions;
	//positions.reserve(inputPoints.size());
	//for (const auto& p : inputPoints) {
	//	positions.emplace_back(static_cast<openvdb::Vec3f>(p));
	//}

	//// 创建单位体素大小的变换
	//openvdb::math::Transform::Ptr xform = openvdb::math::Transform::createLinearTransform(1.0);

	//// 直接调用推荐版本的 createPointDataGrid
	//using Codec = openvdb::points::NullCodec;
	//auto grid = openvdb::points::createPointDataGrid<Codec, openvdb::Vec3f>(positions, *xform);

	//grid->setName("points");

	//// 保存到 .vdb 文件
	//openvdb::io::File file(filename);
	//openvdb::GridPtrVec grids{ grid };
	//file.write(grids);
	//file.close();

}

bool IsSameNormalPair(
	const MidPointKey& k1,
	const MidPointKey& k2,
	double tol)
{
	return k1.PA.Distance(k2.PA) < tol &&
		k1.PB.Distance(k2.PB) < tol;

}

bool MidSurfGenerator::CalSdfMidPoint()
{
	TopoDS_Compound compound;
	BRep_Builder builder;
	builder.MakeCompound(compound);

	for (const TopoDS_Shape& shape : m_orgShapeArr) 
	{
		builder.Add(compound, shape);
	}

	BRepMesh_IncrementalMesh mesh(compound, 0.5);  // 0.5 是网格密度

	std::vector<openvdb::Vec3s> points;
	std::vector<openvdb::Vec3I> triangles;

	TopExp_Explorer explorer;
	for (explorer.Init(compound, TopAbs_FACE); explorer.More(); explorer.Next()) 
	{
		TopoDS_Face face = TopoDS::Face(explorer.Current());
		TopLoc_Location loc;
		Handle(Poly_Triangulation) tri = BRep_Tool::Triangulation(face, loc);
		if (tri.IsNull()) continue;

		const TColgp_Array1OfPnt& nodes = tri->Nodes();
		const Poly_Array1OfTriangle& tris = tri->Triangles();
		int offset = static_cast<int>(points.size());

		for (int i = nodes.Lower(); i <= nodes.Upper(); ++i) {
			gp_Pnt p = nodes(i).Transformed(loc.Transformation());
			points.emplace_back(openvdb::Vec3s(p.X(), p.Y(), p.Z()));
		}

		for (int i = tris.Lower(); i <= tris.Upper(); ++i) {
			int v1, v2, v3;
			tris(i).Get(v1, v2, v3);
			triangles.emplace_back(openvdb::Vec3I(v1 - 1 + offset, v2 - 1 + offset, v3 - 1 + offset));
		}
	}

	openvdb::initialize();

	double maxThickness = 0.0;
	for (auto& group : m_midFacGrper.GetMidSurfGroup())
	{
		maxThickness = std::max(maxThickness, group.GetMidFacThicnkess());
	}
	maxThickness = 5;
	double sdfScale = maxThickness / 5;
	auto transform = openvdb::math::Transform::createLinearTransform(sdfScale); // 体素大小
	openvdb::FloatGrid::Ptr sdfGrid = openvdb::tools::meshToLevelSet<openvdb::FloatGrid>(
		*transform, points, triangles);

	//for test

	/*for (const auto& p : points) {
		m_points.push_back(gp_Pnt(p.x(), p.y(), p.z()));
	}*/

	auto accessor = sdfGrid->getConstAccessor();

	// === 2. 局部极小值筛选 ===
	const openvdb::Coord offsets6[6] = {
	openvdb::Coord(1, 0, 0),
	openvdb::Coord(-1, 0, 0),
	openvdb::Coord(0, 1, 0),
	openvdb::Coord(0,-1, 0),
	openvdb::Coord(0, 0, 1),
	openvdb::Coord(0, 0,-1)
	};

	for (auto iter = sdfGrid->cbeginValueOn(); iter; ++iter) 
	{
		float v = iter.getValue();
		if (v >= 0.0f)
			continue; // 只考虑实体内部
		
		const openvdb::Coord& c = iter.getCoord();

		//std::cout << v << std::endl;
		//std::cout << c << std::endl;

		openvdb::Vec3d p =
			sdfGrid->indexToWorld(c);

		//std::cout << p << std::endl << std::endl;

		bool isLocalMin = true, isFind = false;

		for (int dx = -1; dx <= 1; ++dx)
		{
			for (int dy = -1; dy <= 1; ++dy)
			{
				for (int dz = -1; dz <= 1; ++dz)
				{
					if (dx == 0 && dy == 0 && dz == 0)
						continue;

					openvdb::Coord nb = c.offsetBy(dx, dy, dz);

					if (!accessor.isValueOn(nb))
						continue;
					isFind = true;
					float vn = accessor.getValue(nb);
					if (v > vn)
					{
						isLocalMin = false;
						break;
					}
				}
				if (!isLocalMin)
				{
					break;
				}
			}
			if (!isLocalMin)
			{
				break;
			}
		}

		/*for (int n = 0; n < 6; ++n)
		{
			openvdb::Coord nb = c + offsets6[n];

			if (!accessor.isValueOn(nb))
				continue;

			float vn = accessor.getValue(nb);
			if (v > vn)
			{
				isLocalMin = false;
				break;
			}
		}*/

		if (!isLocalMin || !isFind)
			continue;

		//std::cout << v << " " << c.x()<< " " << c.y() << " " << c.z() ;
		//for (int k = 0; k < 6; ++k)
		//{
		//	openvdb::Coord nb(c.x() + DX[k], c.y() + DY[k], c.z() + DZ[k]);
		//	float nbVal = sdfGrid->tree().getValue(nb);
		//	//std::cout << " " << nbVal;
		//}
		//std::cout << std::endl;

		// === 3. 体素坐标 → 世界坐标 ===
		openvdb::Vec3d wp = sdfGrid->indexToWorld(c);
		m_sdfMidPoints.emplace_back(wp.x(), wp.y(), wp.z());
	    m_points.emplace_back(wp.x(), wp.y(), wp.z());
	}

	//for test
	//std::ofstream ofs("gridPnt.txt");

	//
	//for (const auto& p : minPoints) {
	//	//m_points.push_back(gp_Pnt(p.x(), p.y(), p.z()));
	//	ofs << p.x() << " " << p.y() << " " << p.z() << "\n";
	//}

	//ofs.close();

	//openvdb::GridPtrVec grids;
	//grids.push_back(sdfGrid);
	//openvdb::io::File file("output.vdb");

	//// 添加网格到文件
	//file.write(grids);

	//// 关闭文件
	//file.close();
	//m_points.insert(m_points.end(), m_sdfMidPoints.begin(), m_sdfMidPoints.end());
	return !m_sdfMidPoints.empty();
}

bool MidSurfGenerator::BuildVirtualBoundary()
{
	MidSurfClassification mfClsfy(m_orgFacArr);
	
	mfClsfy.setThickness(m_maxThickness, m_minThickness);
	mfClsfy.Generate();

	m_sdfMidPoints = mfClsfy.m_sdfMidPoints;
	m_orgFacArr = mfClsfy.GetOrgFaces();
	m_midFacGrper = mfClsfy.GetMidSurfGrouper();
	//for test 
	m_points.insert(m_points.end(), mfClsfy.m_points.begin(), mfClsfy.m_points.end());
	m_fac.insert(m_fac.end(), mfClsfy.m_fac.begin(), mfClsfy.m_fac.end());
	testClsfyFac = mfClsfy.testClsfyFac;
	testArrow = mfClsfy.testArrow;
	testString = mfClsfy.testString;
	std::vector<OccMidSurfGroup>& grpArr = m_midFacGrper.GetMidSurfGroup();
	//m_fac.push_back(m_orgFacArr[1]);
	/*for (auto grp : grpArr)
	{
		for (auto i : grp.GetSideALabs())
		{
			m_fac.push_back(m_orgFacArr[i]);
		}

		for (auto j : grp.GetSideBLabs())
		{
			m_fac.push_back(m_orgFacArr[j]);
		}
	}*/
	//m_wire.push_back((mfClsfy.GetMidSurfGrouper().GetMidSurfGroup().begin())->GetMidWire());
	m_edges = mfClsfy.m_edges;

	return true;
}
bool MidSurfGenerator::ClassifySDFPntsByFace()
{
	auto& facLabToGrpLabMap = m_midFacGrper.GetFacLabToGrpLabMap();
	std::vector<OccMidSurfGroup>& grpArr = m_midFacGrper.GetMidSurfGroup();
	for (const gp_Pnt& p : m_sdfMidPoints)
	{
		double minDist = DBL_MAX;
		int grpId = -1;
		for (auto& [id, grp] : facLabToGrpLabMap)
		{
			TopoDS_Face f = m_orgFacArr[id];
			gp_Pnt nouse;
			double tmpDist = OccMidSurfCommFunction::PointToFaceDistance(p, f, nouse);
			if (tmpDist < minDist)
			{
				grpId = grp;
				minDist = tmpDist;
			}
		}
		std::cout << grpId << std::endl;
		m_points.push_back(p);
		grpArr[grpId].AddMidPoint(p);
	}
	return true;
}

bool MidSurfGenerator::ClassifySDFPntsByGroups()
{
	if (m_sdfMidPoints.empty())
		return false;

	std::vector<OccMidSurfGroup>& grpArr =
		m_midFacGrper.GetMidSurfGroup();

	const double DIST_RATIO_TOL = 0.1;   // 距离一致性阈值
	const double INF = 1e100;

	std::vector<MidPointKey> keys;
	double tol = 0.0;

	for (const gp_Pnt& p : m_sdfMidPoints)
	{
		OccMidSurfGroup* bestGroup = nullptr;
		double bestScore = INF;

		gp_Pnt PA, PB;
		int grpid = 0, d = 0;
		for (auto& group : grpArr)
		{
			double thickness = group.GetMidFacThicnkess();
			if (thickness <= Precision::Confusion())
				continue;

			/* ===============================
			 * Step 1: 最近 A 面
			 * =============================== */
			double minDistA = INF;
			TopoDS_Face closestAFace;

			gp_Pnt projA, projB;
			for (UINT lab : group.GetSideALabs())
			{
				const TopoDS_Face& aFac = m_orgFacArr[lab];

				Bnd_Box box;
				BRepBndLib::Add(aFac, box);
				OccMidSurfCommFunction::ExpandBndBox(box, thickness);

				if (box.IsOut(p))
					continue;
				gp_Pnt pA;
				double d = OccMidSurfCommFunction::PointToFaceDistance(p, aFac, pA);
				if (d < minDistA)
				{
					projA = pA ;
					minDistA = d;
					closestAFace = aFac;
				}
			}

			if (closestAFace.IsNull())
				continue;

			/* ===============================
			 * Step 2: 最近 B 面
			 * =============================== */
			double minDistB = INF;
			TopoDS_Face closestBFace;

			for (UINT lab : group.GetSideBLabs())
			{
				const TopoDS_Face& bFac = m_orgFacArr[lab];

				Bnd_Box box;
				BRepBndLib::Add(bFac, box);
				OccMidSurfCommFunction::ExpandBndBox(box, thickness);

				if (box.IsOut(p))
					continue;

				gp_Pnt pB;

				double d = OccMidSurfCommFunction::PointToFaceDistance(p, bFac, pB);
				if (d < minDistB)
				{
					projB = pB;
					minDistB = d;
					closestBFace = bFac;
				}
			}

			if (closestBFace.IsNull())
				continue;

			/* ===============================
			 * Step 3: 双向距离一致性（关键）
			 * =============================== */
			if (Abs(minDistA - minDistB) > thickness * DIST_RATIO_TOL)
				continue;

			gp_Vec vecA(projA, p);
			gp_Vec vecB(projB, p);

			gp_Vec nA, nB;
			if (!OccMidSurfCommFunction::GetNormalAtPoint(closestAFace, projA, nA))
				continue;
			if (!OccMidSurfCommFunction::GetNormalAtPoint(closestBFace, projB, nB))
				continue;

			// 点必须位于面法向反侧
			if (vecA.Dot(nA) > 0.0)
				continue;
			if (vecB.Dot(nB) > 0.0)
				continue;

			/* ===============================
			 * Step 5: 评分（选最优 group）
			 * =============================== */
			double score =
				Abs(minDistA - minDistB) + 0.1 * (minDistA + minDistB);

			if (score < bestScore)
			{
				tol = (OccMidSurfCommFunction::CalFacArea(closestAFace) + OccMidSurfCommFunction::CalFacArea(closestBFace)) / 20;
				bestScore = score;
				PA = projA;
				PB = projB;
				bestGroup = &group;
				d = grpid;
			}
			grpid++;
		}

		/* ===============================
		 * Step 6: 只加入最优组
		 * =============================== */
		if (bestGroup)
		{
			std::cout << d << std::endl;
			bestGroup->AddMidPoint(p);
			/*std::vector<MidPointKey>& keys = bestGroup->GetMidFacKeys();
			bool merged = false;
			for (size_t i = 0; i < keys.size(); ++i)
			{
				if (IsSameNormalPair(keys[i], { PA, PB }, tol))
				{
					merged = true;
					break;
				}
			}

			if (!merged)
			{
				bestGroup->AddMidPoint(gp_Pnt(
					0.5 * (PA.X() + PB.X()),
					0.5 * (PA.Y() + PB.Y()),
					0.5 * (PA.Z() + PB.Z())
				));
				keys.push_back({ PA, PB });
			}*/
		}
	}

	return true;
}

bool BuildFaceFromWire(const TopoDS_Wire& wire, GpPntArr& midPts, TopoDS_Face& outFace)
{
	// 1️⃣ 创建填充对象
	BRepFill_Filling filler;

	// 2️⃣ 添加边界
	for (TopExp_Explorer ex(wire, TopAbs_EDGE); ex.More(); ex.Next())
	{
		TopoDS_Edge e = TopoDS::Edge(ex.Current());
		filler.Add(e, GeomAbs_C0);
	}

	for (auto& pt : midPts)
	{
		filler.Add(pt);
	}
	// 3️⃣ 构建曲面
	filler.Build();
	if (!filler.IsDone())
		return false;

	// 4️⃣ 获取曲面
	outFace = filler.Face();
	return true;
}


bool MidSurfGenerator::FitAllGroupMidSurf()
{
	// 获取所有中面分组
	std::vector<OccMidSurfGroup>& grpArr = m_midFacGrper.GetMidSurfGroup();
	int cnt = 0;
	auto checkMidFac = [&](TopoDS_Face& f, OccMidSurfGroup& group)
	{
			GpPntArr discretePntArr;
			OccMidSurfCommFunction::GetFacDiscretePnt(f, discretePntArr);
			FaceLabArr aLabs = group.GetSideALabs();
			FaceLabArr bLabs = group.GetSideBLabs();

			double distA = DBL_MAX, distB = DBL_MAX;
			for (auto &p : discretePntArr)
			{
				gp_Pnt x;
				for (auto lab : aLabs)
				{
					distA = std::min(distA, OccMidSurfCommFunction::PointToFaceDistance(p, m_orgFacArr[lab], x));
				}
				for (auto lab : bLabs)
				{
					distB = std::min(distB, OccMidSurfCommFunction::PointToFaceDistance(p, m_orgFacArr[lab], x));
				}
				m_sc.push_back(fabs(distA - distB) / 2 * std::max(distA, distB));
				std::cout << "distA: "<<distA << " distB: " << distB << std::endl;
			}

	};
	for (OccMidSurfGroup& group : grpArr)
	{
		if (false && group.GetMidFacThicnkess()>1)
		{
			const FaceLabArr &b =group.GetSideBLabs();

			FaceLabArr aLabs = group.GetSideALabs();
			if(aLabs.size()>=b.size())
			{
				aLabs = b;
			}
			// =================================================
			// ① 构建 Shell
			// =================================================
			TopoDS_Shell shell;
			BRep_Builder builder;
			builder.MakeShell(shell);

			for (int lab : aLabs)
			{
				builder.Add(
					shell,
					m_orgFacArr[lab]);
			}

			// =================================================
			// ② 偏移
			// =================================================
			double offsetDist = - group.GetMidFacThicnkess() * 0.5;

			BRepOffsetAPI_MakeOffsetShape offsetMaker;

			offsetMaker.PerformByJoin(
				shell,
				offsetDist,
				1e-3,
				BRepOffset_Skin,
				Standard_False,
				Standard_False,
				GeomAbs_Arc,
				Standard_False);

			if (offsetMaker.IsDone())
			{
				TopoDS_Shape result =
					offsetMaker.Shape();

				// =================================================
				// ③ 提取面
				// =================================================
				for (TopExp_Explorer exp(
					result,
					TopAbs_FACE);
					exp.More();
					exp.Next())
				{
					TopoDS_Face midFace =
						TopoDS::Face(
							exp.Current());

					m_fac.push_back(
						midFace);
					checkMidFac(midFace, group);
				}
			}
			else
			{
				for (int lab : aLabs)
				{
					TopoDS_Face face =
						m_orgFacArr[lab];

					BRepOffsetAPI_MakeOffsetShape mk;

					mk.PerformBySimple(
						face, offsetDist
					);
					/*mk.PerformByJoin(
						face,
						offsetDist,
						1e-3,
						BRepOffset_Skin,
						Standard_False,
						Standard_False,
						GeomAbs_Arc,
						Standard_False);*/

					if (!mk.IsDone())
						continue;

					TopoDS_Shape res = mk.Shape();

					for (TopExp_Explorer exp(
						res, TopAbs_FACE);
						exp.More(); exp.Next())
					{
						TopoDS_Face fac = TopoDS::Face(
							exp.Current());
						m_fac.push_back(fac);
						checkMidFac(fac, group);
					}
				}
			}
			
		}
		else
		{
			GpPntArr fittingMidPtsArr;

			//const TopoDS_Wire& midWire = group.GetMidWire();
			const std::vector<TopoDS_Edge>& midWire = group.GetMidEdges();
			GpPntArr& midPts = group.GetMidPntArr();

			const FaceLabArr& b = group.GetSideBLabs();

			FaceLabArr aLabs = group.GetSideALabs();

			for (auto a : aLabs)
			{
				GpPntArr ptarr;
				OccMidSurfCommFunction::GetFacDiscretePnt(m_orgFacArr[a], ptarr);
				int steps = ptarr.size() / 20;
				for (int i = 0; i < ptarr.size(); i+=steps)
				{
					auto pt = ptarr[i];
					TopoDS_Face bf;
					double mdist = DBL_MAX;
					for (auto bld : b)
					{
						gp_Pnt x;
						double dst = OccMidSurfCommFunction::PointToFaceDistance(pt, m_orgFacArr[bld], x);
						if (dst < mdist)
						{
							mdist = dst;
							bf = m_orgFacArr[bld];
						}
					}
					gp_Pnt x;
					OccMidSurfCommFunction::ProjectPointToFace(ptarr[i], bf, x);
					gp_Pnt midPt;
					OccMidSurfCommFunction::GetMidpoint(ptarr[i], x, midPt);
				
					fittingMidPtsArr.push_back(midPt);
					m_points.emplace_back(midPt);
				}
			}
			//if (midWire.IsNull())
			//	continue; // 跳过无效组

			UINT fittingPntNum = 5;
			UINT midPtsSize = fittingMidPtsArr.size();
			// 2️⃣ 将中面点拟合到曲面
			UINT step = midPtsSize / fittingPntNum;
			if (step == 0)
			{
				step = 1;
			}
			for (size_t i = 0; i < midPtsSize; i += step)
			{
				//fittingMidPtsArr.push_back(midPts[i]);
				m_points.emplace_back(fittingMidPtsArr[i]);

			}

			//TopoDS_Face midFace = OccMidSurfCommFunction::FitPlateSurface(midWire, fittingMidPtsArr);
			//m_edges = midWire;
			TopoDS_Edge e;//
			bool isRev = false;
			TopoDS_Face midFace;
			if (cnt == 1)
			{
				isRev = true;
			}
			OccMidSurfCommFunction::BuildFaceFromWire(midWire, fittingMidPtsArr, midFace, isRev);
			
			if(cnt == 2)
			std::cout << "fittingMidPtsArr:" << fittingMidPtsArr.size() << std::endl;
			if (cnt != 3)
			{
				//cnt++;
			}
			else
			{
				//m_points = fittingMidPtsArr;
				//m_edges = midWire;
				//break;
			}
			group.SetMidFace(midFace); // 假设你给 OccMidSurfGroup 新增 SetMidFace

			m_fac.push_back(midFace);
			//checkMidFac(midFace, group);
			//break;
		}
	}
	return true;
}
void MidSurfGenerator::CheckMidSurfDistance(
	const TopoDS_Face& midFace,
	const OccMidSurfGroup& group)
{
	if (midFace.IsNull())
		return;

	GpPntArr discretePntArr;

	TopoDS_Face tmpFace = midFace;
	OccMidSurfCommFunction::GetFacDiscretePnt(tmpFace, discretePntArr);

	const FaceLabArr& aLabs = group.GetSideALabs();
	const FaceLabArr& bLabs = group.GetSideBLabs();

	if (aLabs.empty() || bLabs.empty())
		return;

	for (const auto& p : discretePntArr)
	{
		double distA = DBL_MAX;
		double distB = DBL_MAX;

		gp_Pnt proj;

		for (auto lab : aLabs)
		{
			if (lab < 0 || lab >= static_cast<int>(m_orgFacArr.size()))
				continue;

			double d = OccMidSurfCommFunction::PointToFaceDistance(
				p,
				m_orgFacArr[lab],
				proj);

			distA = std::min(distA, d);
		}

		for (auto lab : bLabs)
		{
			if (lab < 0 || lab >= static_cast<int>(m_orgFacArr.size()))
				continue;

			double d = OccMidSurfCommFunction::PointToFaceDistance(
				p,
				m_orgFacArr[lab],
				proj);

			distB = std::min(distB, d);
		}

		if (distA == DBL_MAX || distB == DBL_MAX)
			continue;

		double denom =
			std::max(std::max(distA, distB), 1.0e-6);

		double err =
			std::abs(distA - distB) / denom;

		m_sc.push_back(err);

		std::cout
			<< "distA: " << distA
			<< " distB: " << distB
			<< " err: " << err
			<< std::endl;
	}
}

bool MidSurfGenerator::BuildUniformMidSurf(
	OccMidSurfGroup& group,
	bool doCheck)
{
	FaceLabArr offsetLabs = group.GetSideALabs();

	if (offsetLabs.empty())
		return false;

	const FaceLabArr& bLabs = group.GetSideBLabs();

	// 选择面数更少的一侧偏移，通常更稳定
	if (!bLabs.empty() && bLabs.size() < offsetLabs.size())
	{
		offsetLabs = bLabs;
	}

	TopoDS_Shell shell;
	BRep_Builder builder;
	builder.MakeShell(shell);

	bool hasValidFace = false;

	for (int lab : offsetLabs)
	{
		if (lab < 0 || lab >= static_cast<int>(m_orgFacArr.size()))
			continue;

		builder.Add(shell, m_orgFacArr[lab]);
		hasValidFace = true;
	}

	if (!hasValidFace)
		return false;

	double offsetDist =
		-group.GetMidFacThicnkess() * 0.5;

	bool generated = false;

	BRepOffsetAPI_MakeOffsetShape offsetMaker;

	offsetMaker.PerformByJoin(
		shell,
		offsetDist,
		1.0e-3,
		BRepOffset_Skin,
		Standard_False,
		Standard_False,
		GeomAbs_Arc,
		Standard_False);

	if (offsetMaker.IsDone())
	{
		TopoDS_Shape result = offsetMaker.Shape();

		for (TopExp_Explorer exp(result, TopAbs_FACE);
			exp.More();
			exp.Next())
		{
			TopoDS_Face midFace =
				TopoDS::Face(exp.Current());

			if (midFace.IsNull())
				continue;

			group.SetMidFace(midFace);
			m_fac.push_back(midFace);

			if (doCheck)
				CheckMidSurfDistance(midFace, group);

			generated = true;
		}
	}

	// Shell 偏移失败，则逐面偏移兜底
	if (!generated)
	{
		for (int lab : offsetLabs)
		{
			if (lab < 0 || lab >= static_cast<int>(m_orgFacArr.size()))
				continue;

			TopoDS_Face face = m_orgFacArr[lab];

			BRepOffsetAPI_MakeOffsetShape mk;
			mk.PerformBySimple(face, offsetDist);

			if (!mk.IsDone())
				continue;

			TopoDS_Shape res = mk.Shape();

			for (TopExp_Explorer exp(res, TopAbs_FACE);
				exp.More();
				exp.Next())
			{
				TopoDS_Face midFace =
					TopoDS::Face(exp.Current());

				if (midFace.IsNull())
					continue;

				group.SetMidFace(midFace);
				m_fac.push_back(midFace);

				if (doCheck)
					CheckMidSurfDistance(midFace, group);

				generated = true;
			}
		}
	}

	return generated;
}

bool MidSurfGenerator::BuildVariableMidSurf(
	OccMidSurfGroup& group,
	bool doCheck)
{
	

	const std::vector<TopoDS_Edge>& midWire =
		group.GetMidEdges();

	if (midWire.empty())
		return false;

	//const FaceLabArr& aLabs =
	//	group.GetSideALabs();

	//const FaceLabArr& bLabs =
	//	group.GetSideBLabs();

	//if (aLabs.empty() || bLabs.empty())
	//	return false;

	//// 从 A 侧采样，投影到最近 B 面，取中点
	//for (auto aLab : aLabs)
	//{
	//	if (aLab < 0 || aLab >= static_cast<int>(m_orgFacArr.size()))
	//		continue;

	//	GpPntArr ptArr;
	//	OccMidSurfCommFunction::GetFacDiscretePnt(
	//		m_orgFacArr[aLab],
	//		ptArr);

	//	if (ptArr.empty())
	//		continue;

	//	int step =
	//		static_cast<int>(ptArr.size()) / 20;

	//	if (step <= 0)
	//		step = 1;

	//	for (int i = 0;
	//		i < static_cast<int>(ptArr.size());
	//		i += step)
	//	{
	//		const gp_Pnt& pA = ptArr[i];

	//		TopoDS_Face nearestBFace;
	//		double minDist = DBL_MAX;

	//		for (auto bLab : bLabs)
	//		{
	//			if (bLab < 0 || bLab >= static_cast<int>(m_orgFacArr.size()))
	//				continue;

	//			gp_Pnt proj;
	//			double d = OccMidSurfCommFunction::PointToFaceDistance(
	//				pA,
	//				m_orgFacArr[bLab],
	//				proj);

	//			if (d < minDist)
	//			{
	//				minDist = d;
	//				nearestBFace = m_orgFacArr[bLab];
	//			}
	//		}

	//		if (nearestBFace.IsNull())
	//			continue;

	//		gp_Pnt pB;
	//		if (!OccMidSurfCommFunction::ProjectPointToFace(
	//			pA,
	//			nearestBFace,
	//			pB))
	//		{
	//			continue;
	//		}

	//		gp_Pnt midPt;
	//		OccMidSurfCommFunction::GetMidpoint(
	//			pA,
	//			pB,
	//			midPt);

	//		fittingMidPtsArr.push_back(midPt);
	//		m_points.emplace_back(midPt);
	//	}
	//}

	// 追加分组阶段保存的中点/有效采样点
	GpPntArr fittingMidPtsArr = group.GetMidPntArr();

	if (fittingMidPtsArr.empty())
		return false;

	TopoDS_Face midFace;
	bool isRev = false;

	bool ok = OccMidSurfCommFunction::BuildFaceFromWire(
		midWire,
		fittingMidPtsArr,
		midFace,
		isRev);

	if (!ok || midFace.IsNull())
		return false;

	group.SetMidFace(midFace);
	m_fac.push_back(midFace);

	if (doCheck)
		CheckMidSurfDistance(midFace, group);

	return true;
}

bool MidSurfGenerator::BuildOneGroupMidSurf(
	OccMidSurfGroup& group,
	bool doCheck)
{
	if (group.IsUniformThickness())
	{
		return BuildUniformMidSurf(group, doCheck);
	}

	if (group.IsVariableThickness())
	{
		return BuildVariableMidSurf(group, doCheck);
	}

	// 类型未知时兜底：先拟合，失败再偏移
	if (BuildVariableMidSurf(group, doCheck))
		return true;

	return BuildUniformMidSurf(group, doCheck);
}

bool MidSurfGenerator::BuildAllGroupMidSurf()
{
	std::vector<OccMidSurfGroup>& groups =
		m_midFacGrper.GetMidSurfGroup();

	bool allOk = true;

	for (OccMidSurfGroup& group : groups)
	{
		bool ok = BuildOneGroupMidSurf(
			group,
			false);

		if (!ok)
		{
			allOk = false;

			std::cout
				<< "Warning: failed to build mid surface for group "
				<< group.GetGrpLab()
				<< std::endl;
		}
	}

	return allOk;
}

void MidSurfGenerator::OffsetSurface()
{
	auto& groups = m_midFacGrper.GetMidSurfGroup();
	auto checkMidFac = [&](TopoDS_Face& f, OccMidSurfGroup& group)
		{
			GpPntArr discretePntArr;
			OccMidSurfCommFunction::GetFacDiscretePnt(f, discretePntArr);
			FaceLabArr aLabs = group.GetSideALabs();
			FaceLabArr bLabs = group.GetSideBLabs();

			double distA = DBL_MAX, distB = DBL_MAX;
			for (auto& p : discretePntArr)
			{
				gp_Pnt x;
				for (auto lab : aLabs)
				{
					distA = std::min(distA, OccMidSurfCommFunction::PointToFaceDistance(p, m_orgFacArr[lab], x));
				}
				for (auto lab : bLabs)
				{
					distB = std::min(distB, OccMidSurfCommFunction::PointToFaceDistance(p, m_orgFacArr[lab], x));
				}
				m_sc.push_back(fabs(distA - distB) / 2 * std::max(distA, distB));
				std::cout << "distA: " << distA << " distB: " << distB << std::endl;
			}

		};
	for (auto& group : groups)
	{
		const FaceLabArr&
			aLabs =
			group.GetSideALabs();

		if (aLabs.empty())
			continue;

		// =================================================
		// ① 构建 Shell
		// =================================================
		TopoDS_Shell shell;
		BRep_Builder builder;
		builder.MakeShell(shell);

		for (int lab : aLabs)
		{
			builder.Add(
				shell,
				m_orgFacArr[lab]);
		}

		// =================================================
		// ② 偏移
		// =================================================
		double offsetDist = -group.GetMidFacThicnkess() * 0.5;

		BRepOffsetAPI_MakeOffsetShape offsetMaker;

		offsetMaker.PerformByJoin(
			shell,
			offsetDist,
			1e-3,
			BRepOffset_Skin,
			Standard_False,
			Standard_False,
			GeomAbs_Arc,
			Standard_False);

		if (offsetMaker.IsDone())
		{
			TopoDS_Shape result =
				offsetMaker.Shape();

			// =================================================
			// ③ 提取面
			// =================================================
			for (TopExp_Explorer exp(
				result,
				TopAbs_FACE);
				exp.More();
				exp.Next())
			{
				TopoDS_Face midFace =
					TopoDS::Face(
						exp.Current());

				m_fac.push_back(
					midFace);
				checkMidFac(midFace, group);
			}
		}
		else
		{
			for (int lab : aLabs)
			{
				TopoDS_Face face =
					m_orgFacArr[lab];

				BRepOffsetAPI_MakeOffsetShape mk;

				mk.PerformBySimple(
					face, offsetDist
				);
				/*mk.PerformByJoin(
					face,
					offsetDist,
					1e-3,
					BRepOffset_Skin,
					Standard_False,
					Standard_False,
					GeomAbs_Arc,
					Standard_False);*/

				if (!mk.IsDone())
					continue;

				TopoDS_Shape res = mk.Shape();

				for (TopExp_Explorer exp(
					res, TopAbs_FACE);
					exp.More(); exp.Next())
				{
					TopoDS_Face midFace = TopoDS::Face(
						exp.Current());
					m_fac.push_back(midFace	);
					checkMidFac(midFace, group);

				}
			}
		}
	}
}


openvdb::FloatGrid::Ptr MakeMinOnlyGrid(openvdb::FloatGrid::Ptr original)
{
	//openvdb::FloatGrid::Ptr result = openvdb::FloatGrid::create(/*background=*/original->background());
	//result->setTransform(original->transform().copy());

	//float minVal = std::numeric_limits<float>::max();
	//openvdb::Coord minCoord;

	//for (auto iter = original->cbeginValueOn(); iter; ++iter) {
	//	if (iter.getValue() < minVal) {
	//		minVal = iter.getValue();
	//		minCoord = iter.getCoord();
	//	}
	//}

	//result->tree().setValue(minCoord, minVal);
	//return result;
	return original;
}

bool MidSurfGenerator::Generate()
{
	if (m_orgFacArr.empty())
	{
		return false;
	}
	bool suc = false;

	BuildVirtualBoundary();

	//CalSdfMidPoint();

	//ClassifySDFPntsByGroups();
	//ClassifySDFPntsByFace();

	//FitAllGroupMidSurf();

	//OffsetSurface();

	BuildAllGroupMidSurf();

	return suc;
}
