#include "OccWin.h"
#include "occUntrimmingBuilder.h"
#include <WNT_Window.hxx>
#include <AIS_Shape.hxx>
#include <QMouseEvent>
#include <QWheelEvent>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <GC_MakeSegment.hxx>
#include <qDebug>
#include "occMidSurfGenerator.h"
#include "occMidSurfCommFunction.h"
#include <TColStd_HSequenceOfTransient.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <BRep_Builder.hxx>
#include <STEPControl_Writer.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <Interface_Static.hxx>
#include <sstream>
#include <iomanip>
#include <TopoDS.hxx>

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <Geom_BSplineCurve.hxx>
#include <AIS_Shape.hxx>
#include <TColgp_Array1OfPnt.hxx>
#include <GeomAPI_PointsToBSpline.hxx>


#include "imso/SplineSurface.h"
#include "imso/RWGeometric.h"
#include "imso/continuitymodification.h"
static int settingUsdInPaper = true;
static int debugForQuadDiv = false;
static int useConvert = false;
static int useG0 = false;

#include <BRepPrimAPI_MakeSphere.hxx>
#include <AIS_Shape.hxx>
#include <AIS_InteractiveContext.hxx>
#include <Graphic3d_NameOfMaterial.hxx>
#include <Quantity_Color.hxx>
#include <gp_Pnt.hxx>

Handle(AIS_Shape) DisplaySolidBall(const Handle(AIS_InteractiveContext)& ctx,
	const gp_Pnt& p,
	double r)
{
	TopoDS_Shape sphere = BRepPrimAPI_MakeSphere(p, r).Shape();
	Handle(AIS_Shape) aisSphere = new AIS_Shape(sphere);

	ctx->SetColor(aisSphere, Quantity_NOC_RED, Standard_False);
	ctx->SetMaterial(aisSphere, Graphic3d_NOM_PLASTIC, Standard_False);
	ctx->SetDisplayMode(aisSphere, 1, Standard_False); // 1 = shaded
	ctx->Display(aisSphere, Standard_True);

	return aisSphere;
}
static bool ProjectPointAndEvalNormal(
	const Handle(Geom_Surface)& refSurf,
	const Vec4& midCtrl,
	gp_Pnt& projPnt,
	gp_Vec& normal)
{
	if (refSurf.IsNull())
		return false;

	gp_Pnt queryP(midCtrl.x, midCtrl.y, midCtrl.z);

	GeomAPI_ProjectPointOnSurf projector;
	projector.Init(queryP, refSurf);

	if (projector.NbPoints() <= 0)
		return false;

	Standard_Real uu = 0.0, vv = 0.0;
	projector.LowerDistanceParameters(uu, vv);

	gp_Pnt p;
	gp_Vec du, dv;
	refSurf->D1(uu, vv, p, du, dv);

	gp_Vec n = du.Crossed(dv);
	if (n.Magnitude() < 1.0e-12)
		return false;

	n.Normalize();

	projPnt = p;
	normal = n;
	return true;
}
namespace
{

	static int GetSurfaceCtrlIndex(const SplineSurface& ss, int v, int u)
	{
		return v * ss.m_uNum + u;
	}

	static double ComputeGrevilleAbscissa(
		const varray<double>& knots,
		int degree,
		int ctrlIdx)
	{
		if (degree <= 0)
			return 0.0;

		double sum = 0.0;
		for (int k = 1; k <= degree; ++k)
			sum += knots.at(ctrlIdx + k);

		return sum / static_cast<double>(degree);
	}

	static void EnsureUVCtrlPtsFromLinear(SplineSurface& ss)
	{
		ss.m_UVCtrlPts.clear();
		ss.m_UVCtrlPts.resize(ss.m_vNum);

		for (int v = 0; v < ss.m_vNum; ++v)
		{
			ss.m_UVCtrlPts[v].resize(ss.m_uNum);
			for (int u = 0; u < ss.m_uNum; ++u)
			{
				ss.m_UVCtrlPts[v][u] = ss.m_CtrlPts[GetSurfaceCtrlIndex(ss, v, u)];
			}
		}
	}

	static Vec4 AverageVec4Array(const std::vector<Vec4>& pts)
	{
		Vec4 avg(0.0, 0.0, 0.0, 1.0);
		if (pts.empty())
			return avg;

		for (size_t i = 0; i < pts.size(); ++i)
		{
			avg.x += pts[i].x;
			avg.y += pts[i].y;
			avg.z += pts[i].z;
		}

		const float inv = 1.0f / static_cast<float>(pts.size());
		avg.x *= inv;
		avg.y *= inv;
		avg.z *= inv;
		return avg;
	}
}

struct SurfacePatchRecord
{
	int patchId = -1;

	SplineSurface ssMid;
	SplineSurface ssBottom;
	SplineSurface ssTop;
};

struct SurfaceCornerRef
{
	int patchId = -1;
	int cornerId = -1; // 0:p00 1:p10 2:p11 3:p01
};

struct SurfaceEdgeRelation
{
	int patchA = -1;
	int edgeA = -1;   // 0:south 1:east 2:north 3:west

	int patchB = -1;
	int edgeB = -1;   // 0:south 1:east 2:north 3:west

	bool reversed = false; // 两条边方向是否相反
};

struct XYZKey
{
	long long x = 0;
	long long y = 0;
	long long z = 0;

	bool operator<(const XYZKey& other) const
	{
		if (x != other.x) return x < other.x;
		if (y != other.y) return y < other.y;
		return z < other.z;
	}
};

namespace
{
	static double Vec4Dist(const Vec4& a, const Vec4& b)
	{
		const double dx = static_cast<double>(a.x) - static_cast<double>(b.x);
		const double dy = static_cast<double>(a.y) - static_cast<double>(b.y);
		const double dz = static_cast<double>(a.z) - static_cast<double>(b.z);
		return std::sqrt(dx * dx + dy * dy + dz * dz);
	}

	static XYZKey MakeXYZKey(const Vec4& p, double tol = 1.0e-5)
	{
		XYZKey k;
		k.x = static_cast<long long>(std::llround(static_cast<double>(p.x) / tol));
		k.y = static_cast<long long>(std::llround(static_cast<double>(p.y) / tol));
		k.z = static_cast<long long>(std::llround(static_cast<double>(p.z) / tol));
		return k;
	}


	static bool IsSurfaceCtrlLayoutValid(const SplineSurface& ss)
	{
		return (!ss.m_CtrlPts.empty() &&
			ss.m_uNum > 0 &&
			ss.m_vNum > 0 &&
			static_cast<int>(ss.m_CtrlPts.size()) == ss.m_uNum * ss.m_vNum);
	}

	static Vec4 GetSurfaceCtrlPt(const SplineSurface& ss, int v, int u)
	{
		return ss.m_CtrlPts.at(GetSurfaceCtrlIndex(ss, v, u));
	}

	static Vec4 GetSurfaceCorner(const SplineSurface& ss, int cornerId)
	{
		switch (cornerId)
		{
		case 0: return GetSurfaceCtrlPt(ss, 0, 0);                         // p00
		case 1: return GetSurfaceCtrlPt(ss, 0, ss.m_uNum - 1);             // p10
		case 2: return GetSurfaceCtrlPt(ss, ss.m_vNum - 1, ss.m_uNum - 1); // p11
		case 3: return GetSurfaceCtrlPt(ss, ss.m_vNum - 1, 0);             // p01
		default: return Vec4(0.0, 0.0, 0.0, 1.0);
		}
	}

	static void GetSurfaceEdgeCtrlPts(
		const SplineSurface& ss,
		int edgeId,
		varray<Vec4>& outPts)
	{
		outPts.clear();

		if (!IsSurfaceCtrlLayoutValid(ss))
			return;

		if (edgeId == 0) // south: v = 0, u: 0 -> uNum-1
		{
			for (int u = 0; u < ss.m_uNum; ++u)
				outPts.push_back(GetSurfaceCtrlPt(ss, 0, u));
		}
		else if (edgeId == 1) // east: u = uNum-1, v: 0 -> vNum-1
		{
			for (int v = 0; v < ss.m_vNum; ++v)
				outPts.push_back(GetSurfaceCtrlPt(ss, v, ss.m_uNum - 1));
		}
		else if (edgeId == 2) // north: v = vNum-1, u: 0 -> uNum-1
		{
			for (int u = 0; u < ss.m_uNum; ++u)
				outPts.push_back(GetSurfaceCtrlPt(ss, ss.m_vNum - 1, u));
		}
		else if (edgeId == 3) // west: u = 0, v: 0 -> vNum-1
		{
			for (int v = 0; v < ss.m_vNum; ++v)
				outPts.push_back(GetSurfaceCtrlPt(ss, v, 0));
		}
	}

	static bool ArePointArraysEqualByTol(
		const varray<Vec4>& a,
		const varray<Vec4>& b,
		double tol)
	{
		if (a.size() != b.size())
			return false;

		for (int i = 0; i < a.size(); ++i)
		{
			if (Vec4Dist(a.at(i), b.at(i)) > tol)
				return false;
		}
		return true;
	}

	static bool ArePointArraysReverseEqualByTol(
		const varray<Vec4>& a,
		const varray<Vec4>& b,
		double tol)
	{
		if (a.size() != b.size())
			return false;

		const int n = a.size();
		for (int i = 0; i < n; ++i)
		{
			if (Vec4Dist(a.at(i), b.at(n - 1 - i)) > tol)
				return false;
		}
		return true;
	}

	static void BuildSurfaceCornerGroups(
		const std::vector<SurfacePatchRecord>& patches,
		std::map<XYZKey, std::vector<SurfaceCornerRef>>& cornerGroups,
		double tol = 1.0e-5)
	{
		cornerGroups.clear();

		for (const auto& rec : patches)
		{
			if (!IsSurfaceCtrlLayoutValid(rec.ssMid))
				continue;

			for (int cid = 0; cid < 4; ++cid)
			{
				SurfaceCornerRef ref;
				ref.patchId = rec.patchId;
				ref.cornerId = cid;

				const Vec4 p = GetSurfaceCorner(rec.ssMid, cid);
				XYZKey key = MakeXYZKey(p, tol);

				cornerGroups[key].push_back(ref);
			}
		}
	}

	static void BuildSurfaceEdgeRelations(
		const std::vector<SurfacePatchRecord>& patches,
		std::vector<SurfaceEdgeRelation>& edgeRelations,
		double tol = 1.0e-5)
	{
		edgeRelations.clear();

		for (size_t i = 0; i < patches.size(); ++i)
		{
			const SplineSurface& ssi = patches[i].ssMid;
			if (!IsSurfaceCtrlLayoutValid(ssi))
				continue;

			for (size_t j = i + 1; j < patches.size(); ++j)
			{
				const SplineSurface& ssj = patches[j].ssMid;
				if (!IsSurfaceCtrlLayoutValid(ssj))
					continue;

				for (int ei = 0; ei < 4; ++ei)
				{
					varray<Vec4> edgeI;
					GetSurfaceEdgeCtrlPts(ssi, ei, edgeI);
					if (edgeI.empty())
						continue;

					for (int ej = 0; ej < 4; ++ej)
					{
						varray<Vec4> edgeJ;
						GetSurfaceEdgeCtrlPts(ssj, ej, edgeJ);
						if (edgeJ.empty())
							continue;

						if (edgeI.size() != edgeJ.size())
							continue;

						bool same = ArePointArraysEqualByTol(edgeI, edgeJ, tol);
						bool rev = false;

						if (!same)
							rev = ArePointArraysReverseEqualByTol(edgeI, edgeJ, tol);

						if (same || rev)
						{
							SurfaceEdgeRelation rel;
							rel.patchA = patches[i].patchId;
							rel.edgeA = ei;
							rel.patchB = patches[j].patchId;
							rel.edgeB = ej;
							rel.reversed = rev;

							edgeRelations.push_back(rel);
						}
					}
				}
			}
		}
	}

	static void PrintSurfaceCornerGroups(
		const std::map<XYZKey, std::vector<SurfaceCornerRef>>& cornerGroups)
	{
		std::cout << "===== Surface Corner Groups =====" << std::endl;
		for (const auto& kv : cornerGroups)
		{
			if (kv.second.size() <= 1)
				continue;

			std::cout << "Shared Corner: ";
			for (const auto& ref : kv.second)
			{
				std::cout << "[patch " << ref.patchId
					<< ", corner " << ref.cornerId << "] ";
			}
			std::cout << std::endl;
		}
	}

	static void PrintSurfaceEdgeRelations(
		const std::vector<SurfaceEdgeRelation>& edgeRelations)
	{
		std::cout << "===== Surface Edge Relations =====" << std::endl;
		for (const auto& rel : edgeRelations)
		{
			std::cout << "[patch " << rel.patchA
				<< ", edge " << rel.edgeA
				<< "] <-> [patch " << rel.patchB
				<< ", edge " << rel.edgeB
				<< "], reversed = " << (rel.reversed ? 1 : 0)
				<< std::endl;
		}
	}

	static bool BuildPatchRelationsFromSplineSurfaces(
		const std::vector<SurfacePatchRecord>& patches,
		std::map<XYZKey, std::vector<SurfaceCornerRef>>& cornerGroups,
		std::vector<SurfaceEdgeRelation>& edgeRelations,
		double cornerTol = 1.0e-5,
		double edgeTol = 1.0e-5)
	{
		cornerGroups.clear();
		edgeRelations.clear();

		if (patches.empty())
			return true;

		BuildSurfaceCornerGroups(patches, cornerGroups, cornerTol);
		BuildSurfaceEdgeRelations(patches, edgeRelations, edgeTol);

		return true;
	}
}

struct QuadPatchRecord
{
	int patchId = -1;
	TopoDS_Face refFace;
	UVQuadPatch uvPatch;

	SplineSurface ssMid;
	SplineSurface ssBottom;
	SplineSurface ssTop;
};

struct PatchCornerRef
{
	int patchId = -1;
	int localCornerId = -1; // 0:p00 1:p10 2:p11 3:p01
};

struct PatchEdgeRef
{
	int patchId = -1;
	int localEdgeId = -1;   // 0:south 1:east 2:north 3:west
	bool sameDirAsKey = true;
};

struct UVKey
{
	long long u = 0;
	long long v = 0;

	bool operator<(const UVKey& other) const
	{
		if (u != other.u) return u < other.u;
		return v < other.v;
	}
};

struct EdgeKey
{
	UVKey a;
	UVKey b;

	bool operator<(const EdgeKey& other) const
	{
		if (a < other.a) return true;
		if (other.a < a) return false;
		return b < other.b;
	}
};

struct GlobalEdgeData
{
	int edgeId = -1;
	EdgeKey key;
	Spline spline3d;
};
bool BuildSplineVolumeFromThreeLayers(
	const base::SplineSurface& bottomSS,
	const base::SplineSurface& midSS,
	const base::SplineSurface& topSS,
	base::SplineVolume& outVol)
{
	if (bottomSS.m_uNum != midSS.m_uNum || bottomSS.m_uNum != topSS.m_uNum)
		return false;
	if (bottomSS.m_vNum != midSS.m_vNum || bottomSS.m_vNum != topSS.m_vNum)
		return false;

	if (bottomSS.m_uDegree != midSS.m_uDegree || bottomSS.m_uDegree != topSS.m_uDegree)
		return false;
	if (bottomSS.m_vDegree != midSS.m_vDegree || bottomSS.m_vDegree != topSS.m_vDegree)
		return false;

	const int uNum = midSS.m_uNum;
	const int vNum = midSS.m_vNum;
	const int wNum = 3;

	const int uDegree = midSS.m_uDegree;
	const int vDegree = midSS.m_vDegree;
	const int wDegree = 2;

	varray<base::Vec4> volCtrlPts;
	volCtrlPts.clear();

	auto AppendLayer = [&](const base::SplineSurface& ss)
		{
			for (auto& pt : ss.m_CtrlPts)
			{
				volCtrlPts.push_back(pt);
			}
		};

	AppendLayer(bottomSS); // w = 0
	AppendLayer(midSS);    // w = 1
	AppendLayer(topSS);    // w = 2

	outVol.SetControlPoints(volCtrlPts, uDegree, vDegree, wDegree, uNum, vNum, wNum);

	for (auto& pt : volCtrlPts)
	{
		outVol.m_CtrlPts.push_back(pt);
	}
	// 覆盖 knots
	outVol.m_uKnots.clear();
	for (int i = 0; i < midSS.m_uKnots.size(); ++i)
		outVol.m_uKnots.push_back(midSS.m_uKnots.at(i));

	outVol.m_vKnots.clear();
	for (int i = 0; i < midSS.m_vKnots.size(); ++i)
		outVol.m_vKnots.push_back(midSS.m_vKnots.at(i));

	outVol.m_wKnots.clear();
	outVol.m_wKnots.push_back(0.0);
	outVol.m_wKnots.push_back(0.0);
	outVol.m_wKnots.push_back(0.0);
	outVol.m_wKnots.push_back(1.0);
	outVol.m_wKnots.push_back(1.0);
	outVol.m_wKnots.push_back(1.0);

	outVol.m_uDegree = uDegree;
	outVol.m_vDegree = vDegree;
	outVol.m_wDegree = wDegree;

	outVol.m_uNum = uNum;
	outVol.m_vNum = vNum;
	outVol.m_wNum = wNum;

	return true;
}
static base::Vec4 ToVec4(const gp_Pnt& p)
{
	base::Vec4 v;
	v.x = static_cast<float>(p.X());
	v.y = static_cast<float>(p.Y());
	v.z = static_cast<float>(p.Z());
	v.w = 1.0f;
	return v;
}
namespace
{
	static bool MapUVPolylineTo3D(
		const TopoDS_Face& face,
		const std::vector<gp_Pnt2d>& uvPts,
		varray<Vec4>& outPts)
	{
		outPts.clear();

		Handle(Geom_Surface) surf = BRep_Tool::Surface(face);
		if (surf.IsNull())
			return false;

		for (size_t i = 0; i < uvPts.size(); ++i)
		{
			gp_Pnt p;
			surf->D0(uvPts[i].X(), uvPts[i].Y(), p);

			Vec4 v;
			v.x = static_cast<float>(p.X());
			v.y = static_cast<float>(p.Y());
			v.z = static_cast<float>(p.Z());
			v.w = 1.0f;
			outPts.push_back(v);
		}

		return outPts.size() >= 2;
	}
	static bool FitBoundarySplineWithOptionalKnots(
		const varray<Vec4>& samplePts,
		int degree,
		int ctrlNum,
		const varray<double>* knots,
		Spline& outSpline)
	{
		if (samplePts.size() < 2)
			return false;

		bsl::FitBSpline fitter;
		if (knots)
			fitter.FittingBspl(samplePts, *knots, degree, ctrlNum);
		else
			fitter.FittingBspl(samplePts, degree, ctrlNum);

		outSpline.m_Degree = degree;
		outSpline.m_CtrlPts = fitter.m_CtrlPts;
		outSpline.m_Knots = fitter.m_Knots;

		return outSpline.m_CtrlPts.size() == ctrlNum;
	}

	static UVKey MakeUVKey(const gp_Pnt2d& p, double tol = 1.0e-6)
	{
		UVKey k;
		k.u = static_cast<long long>(std::llround(p.X() / tol));
		k.v = static_cast<long long>(std::llround(p.Y() / tol));
		return k;
	}

	static bool UVKeyLess(const UVKey& a, const UVKey& b)
	{
		if (a.u != b.u) return a.u < b.u;
		return a.v < b.v;
	}

	static EdgeKey MakeEdgeKey(const UVKey& k0, const UVKey& k1)
	{
		EdgeKey e;
		if (UVKeyLess(k1, k0))
		{
			e.a = k1;
			e.b = k0;
		}
		else
		{
			e.a = k0;
			e.b = k1;
		}
		return e;
	}

	static gp_Pnt2d GetPatchCorner(const UVQuadPatch& patch, int cid)
	{
		switch (cid)
		{
		case 0: return patch.p00;
		case 1: return patch.p10;
		case 2: return patch.p11;
		case 3: return patch.p01;
		default: return gp_Pnt2d(0.0, 0.0);
		}
	}

	static void GetPatchEdgeCorners(
		const UVQuadPatch& patch,
		int eid,
		gp_Pnt2d& p0,
		gp_Pnt2d& p1)
	{
		switch (eid)
		{
		case 0: p0 = patch.p00; p1 = patch.p10; break; // south
		case 1: p0 = patch.p10; p1 = patch.p11; break; // east
		case 2: p0 = patch.p01; p1 = patch.p11; break; // north
		case 3: p0 = patch.p00; p1 = patch.p01; break; // west
		default:
			p0 = gp_Pnt2d(0.0, 0.0);
			p1 = gp_Pnt2d(0.0, 0.0);
			break;
		}
	}

	static void BuildCornerGroups(
		const std::vector<QuadPatchRecord>& patches,
		std::map<UVKey, std::vector<PatchCornerRef>>& cornerGroups,
		double tol = 1.0e-6)
	{
		cornerGroups.clear();

		for (const auto& rec : patches)
		{
			for (int cid = 0; cid < 4; ++cid)
			{
				PatchCornerRef ref;
				ref.patchId = rec.patchId;
				ref.localCornerId = cid;

				UVKey key = MakeUVKey(GetPatchCorner(rec.uvPatch, cid), tol);
				cornerGroups[key].push_back(ref);
			}
		}
	}

	static void BuildEdgeGroups(
		const std::vector<QuadPatchRecord>& patches,
		std::map<EdgeKey, std::vector<PatchEdgeRef>>& edgeGroups,
		double tol = 1.0e-6)
	{
		edgeGroups.clear();

		for (const auto& rec : patches)
		{
			for (int eid = 0; eid < 4; ++eid)
			{
				gp_Pnt2d p0, p1;
				GetPatchEdgeCorners(rec.uvPatch, eid, p0, p1);

				UVKey k0 = MakeUVKey(p0, tol);
				UVKey k1 = MakeUVKey(p1, tol);

				EdgeKey key = MakeEdgeKey(k0, k1);

				PatchEdgeRef ref;
				ref.patchId = rec.patchId;
				ref.localEdgeId = eid;
				ref.sameDirAsKey = !UVKeyLess(k1, k0);

				edgeGroups[key].push_back(ref);
			}
		}
	}

	static void PrintCornerGroups(
		const std::map<UVKey, std::vector<PatchCornerRef>>& cornerGroups)
	{
		std::cout << "===== Corner Groups =====" << std::endl;
		for (const auto& kv : cornerGroups)
		{
			if (kv.second.size() <= 1)
				continue;

			std::cout << "Shared Corner: ";
			for (const auto& ref : kv.second)
			{
				std::cout << "[patch " << ref.patchId
					<< ", corner " << ref.localCornerId << "] ";
			}
			std::cout << std::endl;
		}
	}

	static void PrintEdgeGroups(
		const std::map<EdgeKey, std::vector<PatchEdgeRef>>& edgeGroups)
	{
		std::cout << "===== Edge Groups =====" << std::endl;
		for (const auto& kv : edgeGroups)
		{
			if (kv.second.size() <= 1)
				continue;

			std::cout << "Shared Edge: ";
			for (const auto& ref : kv.second)
			{
				std::cout << "[patch " << ref.patchId
					<< ", edge " << ref.localEdgeId
					<< ", sameDir=" << (ref.sameDirAsKey ? 1 : 0) << "] ";
			}
			std::cout << std::endl;
		}
	}
	

	static const std::vector<gp_Pnt2d>& GetPatchEdgePolyline(const UVQuadPatch& patch, int eid)
	{
		switch (eid)
		{
		case 0: return patch.south;
		case 1: return patch.east;
		case 2: return patch.north;
		default: return patch.west;
		}
	}
	static bool FitBoundarySpline(
		const varray<Vec4>& samplePts,
		int degree,
		int ctrlNum,
		Spline& outSpline)
	{
		if (samplePts.size() < 2)
			return false;

		bsl::FitBSpline fitter;
		fitter.FittingBspl(samplePts, degree, ctrlNum);

		outSpline.m_Degree = degree;
		outSpline.m_CtrlPts = fitter.m_CtrlPts;
		outSpline.m_Knots = fitter.m_Knots;

		return outSpline.m_CtrlPts.size() == ctrlNum;
	}

	static bool FitBoundarySplineWithKnots(
		const varray<Vec4>& samplePts,
		const varray<double>& knots,
		int degree,
		int ctrlNum,
		Spline& outSpline)
	{
		if (samplePts.size() < 2)
			return false;

		bsl::FitBSpline fitter;

		fitter.FittingBspl(samplePts, knots, degree, ctrlNum);

		outSpline.m_Degree = degree;
		outSpline.m_CtrlPts = fitter.m_CtrlPts;
		outSpline.m_Knots = fitter.m_Knots;

		return outSpline.m_CtrlPts.size() == ctrlNum;
	}

	static bool BuildSplineSurfaceFromUVQuadPatch(
		const TopoDS_Face& face,
		const UVQuadPatch& patch,
		SplineSurface& outSurf)
	{
		// 这里你可以后面再改成配置项
		const int uDegree = 2;
		const int vDegree = 2;
		const int uCtrlNum = 3; // south / north
		const int vCtrlNum = 3; // east / west

		varray<Vec4> southPts, eastPts, northPts, westPts;
		if (!MapUVPolylineTo3D(face, patch.south, southPts)) return false;
		if (!MapUVPolylineTo3D(face, patch.east, eastPts))  return false;
		if (!MapUVPolylineTo3D(face, patch.north, northPts)) return false;
		if (!MapUVPolylineTo3D(face, patch.west, westPts))  return false;

		Spline southSp, eastSp, northSp, westSp;

		// 先拟合 south，得到 u 向 knots
		if (!FitBoundarySpline(southPts, uDegree, uCtrlNum, southSp))
			return false;

		// 再拟合 east，得到 v 向 knots
		if (!FitBoundarySpline(eastPts, vDegree, vCtrlNum, eastSp))
			return false;

		// north 强制复用 south 的 knots
		if (!FitBoundarySplineWithKnots(northPts, southSp.m_Knots, uDegree, uCtrlNum, northSp))
			return false;

		// west 强制复用 east 的 knots
		if (!FitBoundarySplineWithKnots(westPts, eastSp.m_Knots, vDegree, vCtrlNum, westSp))
			return false;

		varray<Spline> edgeLines;
		// 顺序必须满足：
		// F(u,v=0), F(v,u=0), F(u,v=1), F(v,u=1)
		edgeLines.push_back(southSp);
		edgeLines.push_back(eastSp);
		edgeLines.push_back(northSp);
		edgeLines.push_back(westSp);

		outSurf.Clear();
		outSurf.CoonsInterpolate(edgeLines);
		return true;
	}
	static Spline ReverseSplineCtrlPts(const Spline& sp)
	{
		Spline out = sp;
		std::reverse(out.m_CtrlPts.begin(), out.m_CtrlPts.end());
		return out;
	}

	static bool FindPatchEdgeSpline(
		const QuadPatchRecord& rec,
		int localEdgeId,
		const std::map<EdgeKey, GlobalEdgeData>& globalEdgeMap,
		Spline& outSp)
	{
		gp_Pnt2d p0, p1;
		GetPatchEdgeCorners(rec.uvPatch, localEdgeId, p0, p1);

		UVKey k0 = MakeUVKey(p0);
		UVKey k1 = MakeUVKey(p1);
		EdgeKey ekey = MakeEdgeKey(k0, k1);

		auto it = globalEdgeMap.find(ekey);
		if (it == globalEdgeMap.end())
			return false;

		outSp = it->second.spline3d;

		const bool localSameDir = !UVKeyLess(k1, k0);
		if (!localSameDir)
			outSp = ReverseSplineCtrlPts(outSp);

		return true;
	}

	static bool BuildConformMidSurfaceFromGlobalEdges(
		const QuadPatchRecord& rec,
		const std::map<EdgeKey, GlobalEdgeData>& globalEdgeMap,
		SplineSurface& outSurf)
	{
		Spline southSp, eastSp, northSp, westSp;
		if (!FindPatchEdgeSpline(rec, 0, globalEdgeMap, southSp)) return false;
		if (!FindPatchEdgeSpline(rec, 1, globalEdgeMap, eastSp))  return false;
		if (!FindPatchEdgeSpline(rec, 2, globalEdgeMap, northSp)) return false;
		if (!FindPatchEdgeSpline(rec, 3, globalEdgeMap, westSp))  return false;

		varray<Spline> edgeLines;
		// 顺序：F(u,v=0), F(v,u=0), F(u,v=1), F(v,u=1)
		edgeLines.push_back(southSp);
		edgeLines.push_back(eastSp);
		edgeLines.push_back(northSp);
		edgeLines.push_back(westSp);

		outSurf.Clear();
		outSurf.CoonsInterpolate(edgeLines);
		return true;
	}
	static bool BuildGlobalEdgeSplineMap(
		const std::vector<QuadPatchRecord>& patches,
		const std::map<EdgeKey, std::vector<PatchEdgeRef>>& edgeGroups,
		std::map<EdgeKey, GlobalEdgeData>& globalEdgeMap)
	{
		globalEdgeMap.clear();

		const int uDegree = 2;
		const int vDegree = 2;
		const int uCtrlNum = 3;
		const int vCtrlNum = 3;

		int eidAlloc = 0;

		for (const auto& kv : edgeGroups)
		{
			const EdgeKey& key = kv.first;
			const std::vector<PatchEdgeRef>& refs = kv.second;
			if (refs.empty())
				continue;

			const PatchEdgeRef& ref0 = refs.front();
			const QuadPatchRecord& rec = patches[ref0.patchId];

			const std::vector<gp_Pnt2d>& uvPoly =
				GetPatchEdgePolyline(rec.uvPatch, ref0.localEdgeId);

			varray<Vec4> edgePts3d;
			if (!MapUVPolylineTo3D(rec.refFace, uvPoly, edgePts3d))
				return false;

			const bool isHorizontal =
				(ref0.localEdgeId == 0 || ref0.localEdgeId == 2);

			const int degree = isHorizontal ? uDegree : vDegree;
			const int ctrlNum = isHorizontal ? uCtrlNum : vCtrlNum;

			Spline sp;
			if (!FitBoundarySplineWithOptionalKnots(
				edgePts3d, degree, ctrlNum, nullptr, sp))
			{
				return false;
			}

			GlobalEdgeData ge;
			ge.edgeId = eidAlloc++;
			ge.key = key;
			ge.spline3d = sp;

			globalEdgeMap[key] = ge;
		}

		return true;
	}
}

struct OffsetCtrlRef
{
	int patchId = -1;
	int idx = -1;
	int u = -1;
	int v = -1;

	Vec4 midPt;
	Vec4 topPt;
	Vec4 bottomPt;
};

namespace
{
	static bool IsBoundaryCtrl(const SplineSurface& ss, int v, int u)
	{
		return (u == 0 ||
			u == ss.m_uNum - 1 ||
			v == 0 ||
			v == ss.m_vNum - 1);
	}

	static bool EvalPointNormalOnRefSurf(
		const Handle(Geom_Surface)& surf,
		double u,
		double v,
		gp_Pnt& p,
		gp_Vec& n)
	{
		if (surf.IsNull())
			return false;

		gp_Vec du, dv;
		surf->D1(u, v, p, du, dv);

		n = du.Crossed(dv);
		if (n.Magnitude() < 1.0e-12)
			return false;

		n.Normalize();
		return true;
	}

	static void SetCtrlPtLinear(
		SplineSurface& ss,
		int v,
		int u,
		const Vec4& p)
	{
		const int idx = GetSurfaceCtrlIndex(ss, v, u);
		ss.m_CtrlPts.at(idx) = p;

		if (static_cast<int>(ss.m_UVCtrlPts.size()) == ss.m_vNum &&
			static_cast<int>(ss.m_UVCtrlPts.at(v).size()) == ss.m_uNum)
		{
			ss.m_UVCtrlPts.at(v).at(u) = p;
		}
	}

	
	static bool BuildOffsetSurfacesUnified(
		const varray<SplineSurface>& midSurfs,
		const std::vector<Handle(Geom_Surface)>& refSurfs,
		double thickness,
		varray<SplineSurface>& bottomSurfs,
		varray<SplineSurface>& topSurfs,
		double groupTol = 1.0e-5)
	{
		bottomSurfs.clear();
		topSurfs.clear();

		if (midSurfs.size() != static_cast<int>(refSurfs.size()))
			return false;

		const int patchNum = midSurfs.size();
		if (patchNum == 0)
			return true;

		bottomSurfs = midSurfs;
		topSurfs = midSurfs;

		// 先保证 UVCtrlPts 与 m_CtrlPts 对应
		for (int pid = 0; pid < patchNum; ++pid)
		{
			if (!IsSurfaceCtrlLayoutValid(midSurfs.at(pid)))
				return false;

			EnsureUVCtrlPtsFromLinear(bottomSurfs[pid]);
			EnsureUVCtrlPtsFromLinear(topSurfs[pid]);
		}

		// 记录所有边界控制点的偏移结果，按中面控制点分组
		std::map<XYZKey, std::vector<OffsetCtrlRef>> groups;

		for (int pid = 0; pid < patchNum; ++pid)
		{
			const SplineSurface& midSS = midSurfs.at(pid);
			const Handle(Geom_Surface)& refSurf = refSurfs.at(pid);

			if (refSurf.IsNull())
				return false;

			for (int v = 0; v < midSS.m_vNum; ++v)
			{
				for (int u = 0; u < midSS.m_uNum; ++u)
				{
					const int idx = GetSurfaceCtrlIndex(midSS, v, u);
					const Vec4& midCtrl = midSS.m_CtrlPts.at(idx);

					const double uu = ComputeGrevilleAbscissa(
						midSS.m_uKnots, midSS.m_uDegree, u);
					const double vv = ComputeGrevilleAbscissa(
						midSS.m_vKnots, midSS.m_vDegree, v);

					gp_Pnt p;
					gp_Vec n;
					if (!EvalPointNormalOnRefSurf(refSurf, uu, vv, p, n))
						return false;

					Vec4 topCtrl = midCtrl;
					topCtrl.x += static_cast<float>(0.5 * thickness * n.X());
					topCtrl.y += static_cast<float>(0.5 * thickness * n.Y());
					topCtrl.z += static_cast<float>(0.5 * thickness * n.Z());

					Vec4 botCtrl = midCtrl;
					botCtrl.x -= static_cast<float>(0.5 * thickness * n.X());
					botCtrl.y -= static_cast<float>(0.5 * thickness * n.Y());
					botCtrl.z -= static_cast<float>(0.5 * thickness * n.Z());

					// 先写一遍，内部点直接就这样保留
					SetCtrlPtLinear(topSurfs[pid], v, u, topCtrl);
					SetCtrlPtLinear(bottomSurfs[pid], v, u, botCtrl);

					// 只对边界控制点做共享组处理
					if (IsBoundaryCtrl(midSS, v, u))
					{
						OffsetCtrlRef rec;
						rec.patchId = pid;
						rec.idx = idx;
						rec.u = u;
						rec.v = v;
						rec.midPt = midCtrl;
						rec.topPt = topCtrl;
						rec.bottomPt = botCtrl;

						const XYZKey key = MakeXYZKey(midCtrl, groupTol);
						groups[key].push_back(rec);
					}
				}
			}
		}

		// 对共享组统一平均 top / bottom
		for (auto& kv : groups)
		{
			std::vector<OffsetCtrlRef>& refs = kv.second;
			if (refs.size() < 2)
				continue;

			Vec4 topAvg(0.0, 0.0, 0.0, 1.0);
			Vec4 botAvg(0.0, 0.0, 0.0, 1.0);

			for (size_t i = 0; i < refs.size(); ++i)
			{
				topAvg.x += refs[i].topPt.x;
				topAvg.y += refs[i].topPt.y;
				topAvg.z += refs[i].topPt.z;

				botAvg.x += refs[i].bottomPt.x;
				botAvg.y += refs[i].bottomPt.y;
				botAvg.z += refs[i].bottomPt.z;
			}

			const float inv = 1.0f / static_cast<float>(refs.size());
			topAvg.x *= inv;
			topAvg.y *= inv;
			topAvg.z *= inv;

			botAvg.x *= inv;
			botAvg.y *= inv;
			botAvg.z *= inv;

			for (size_t i = 0; i < refs.size(); ++i)
			{
				const OffsetCtrlRef& rec = refs[i];
				SetCtrlPtLinear(topSurfs[rec.patchId], rec.v, rec.u, topAvg);
				SetCtrlPtLinear(bottomSurfs[rec.patchId], rec.v, rec.u, botAvg);
			}
		}

		return true;
	}
}


bool OffsetSplineSurfaceByCtrlPts(
	const base::SplineSurface& midSS,
	const Handle(Geom_BSplineSurface)& midOccSurf,
	double thickness,
	bool upper,
	base::SplineSurface& outSS)
{
	if (midOccSurf.IsNull())
		return false;

	const int uNum = midSS.m_uNum;
	const int vNum = midSS.m_vNum;

	if (uNum <= 0 || vNum <= 0)
		return false;

	if (midSS.m_CtrlPts.size() != uNum * vNum)
		return false;

	outSS = midSS;

	const double sign = upper ? 0.5 : -0.5;

	outSS.m_CtrlPts.clear();
	outSS.m_UVCtrlPts.clear();
	outSS.m_UVCtrlPts.resize(vNum);

	for (int v = 0; v < vNum; ++v)
	{
		outSS.m_UVCtrlPts.at(v).resize(uNum);

		for (int u = 0; u < uNum; ++u)
		{
			const double uu = ComputeGrevilleAbscissa(
				midSS.m_uKnots, midSS.m_uDegree, u);
			const double vv = ComputeGrevilleAbscissa(
				midSS.m_vKnots, midSS.m_vDegree, v);

			gp_Pnt p;
			gp_Vec du, dv;
			midOccSurf->D1(uu, vv, p, du, dv);

			gp_Vec n = du.Crossed(dv);
			if (n.Magnitude() < 1.0e-12)
				return false;

			n.Normalize();
			n *= (sign * thickness);

			const int idx = v * uNum + u;
			base::Vec4 ctrl = midSS.m_CtrlPts.at(idx);

			ctrl.x += static_cast<float>(n.X());
			ctrl.y += static_cast<float>(n.Y());
			ctrl.z += static_cast<float>(n.Z());

			outSS.m_UVCtrlPts.at(v).at(u) = ctrl;
			outSS.m_CtrlPts.push_back(ctrl);
		}
	}

	return true;
}
static bool BuildConformPatchesAndVolumes(
	std::vector<QuadPatchRecord>& allPatches,
	double wallThickness,
	varray<SplineSurface>& outSurfaces,
	varray<SplineVolume>& outVolumes)
{
	outSurfaces.clear();
	outVolumes.clear();

	if (allPatches.empty())
		return true;

	// 1) 建共角点 / 共边映射
	std::map<UVKey, std::vector<PatchCornerRef>> cornerGroups;
	std::map<EdgeKey, std::vector<PatchEdgeRef>> edgeGroups;

	BuildCornerGroups(allPatches, cornerGroups, 1.0e-6);
	BuildEdgeGroups(allPatches, edgeGroups, 1.0e-6);


	// 2) 拟合共享边 spline
	std::map<EdgeKey, GlobalEdgeData> globalEdgeMap;
	if (!BuildGlobalEdgeSplineMap(allPatches, edgeGroups, globalEdgeMap))
	{
		std::cout << "BuildGlobalEdgeSplineMap failed." << std::endl;
		return false;
	}

	// 3) 用共享边 spline 重建 conform 中面
	for (size_t i = 0; i < allPatches.size(); ++i)
	{
		if (!BuildConformMidSurfaceFromGlobalEdges(
			allPatches[i],
			globalEdgeMap,
			allPatches[i].ssMid))
		{
			std::cout << "BuildConformMidSurfaceFromGlobalEdges failed, patchId = "
				<< allPatches[i].patchId << std::endl;
			continue;
		}

		outSurfaces.push_back(allPatches[i].ssMid);
	}

	// 4) 由中面偏移出 top / bottom，并生成 volume
	for (size_t i = 0; i < allPatches.size(); ++i)
	{
		QuadPatchRecord& rec = allPatches[i];

		if (rec.ssMid.m_CtrlPts.empty())
			continue;

		Handle(Geom_Surface) surf = BRep_Tool::Surface(rec.refFace);
		if (surf.IsNull())
		{
			std::cout << "Reference surface is null, patchId = "
				<< rec.patchId << std::endl;
			continue;
		}

		Handle(Geom_BSplineSurface) bsRef =
			Handle(Geom_BSplineSurface)::DownCast(surf);
		if (bsRef.IsNull())
		{
			std::cout << "Reference surface is not Geom_BSplineSurface, patchId = "
				<< rec.patchId << std::endl;
			continue;
		}

		if (!OffsetSplineSurfaceByCtrlPts(
			rec.ssMid, bsRef, wallThickness, false, rec.ssBottom))
		{
			std::cout << "Offset bottom failed, patchId = "
				<< rec.patchId << std::endl;
			continue;
		}

		if (!OffsetSplineSurfaceByCtrlPts(
			rec.ssMid, bsRef, wallThickness, true, rec.ssTop))
		{
			std::cout << "Offset top failed, patchId = "
				<< rec.patchId << std::endl;
			continue;
		}

		SplineVolume sv;
		if (!BuildSplineVolumeFromThreeLayers(
			rec.ssBottom, rec.ssMid, rec.ssTop, sv))
		{
			std::cout << "BuildSplineVolumeFromThreeLayers failed, patchId = "
				<< rec.patchId << std::endl;
			continue;
		}

		outVolumes.push_back(sv);
	}

	return true;
}

//static TopoDS_Edge Make3DPolylineEdgeOnSurface(
//	const Handle(Geom_Surface)& surf,
//	const std::vector<gp_Pnt2d>& uvPts)
//{
//	if (uvPts.size() < 2)
//		return TopoDS_Edge();
//
//	TColgp_Array1OfPnt arr(1, static_cast<int>(uvPts.size()));
//	for (int i = 0; i < static_cast<int>(uvPts.size()); ++i)
//	{
//		arr.SetValue(i + 1, surf->Value(uvPts[i].X(), uvPts[i].Y()));
//	}
//
//	GeomAPI_PointsToBSpline builder(arr);
//	Handle(Geom_BSplineCurve) c3d = builder.Curve();
//	if (c3d.IsNull())
//		return TopoDS_Edge();
//
//	return BRepBuilderAPI_MakeEdge(c3d);
//}
#include <TopoDS_Compound.hxx>
#include <BRep_Builder.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRep_Tool.hxx>






static void ExpandUKnots(
	const Handle(Geom_BSplineSurface)& bs,
	varray<double>& uKnots)
{
	uKnots.clear();
	const int n = bs->NbUKnots();
	for (int i = 1; i <= n; ++i)
	{
		const double k = bs->UKnot(i);
		const int mult = bs->UMultiplicity(i);
		for (int t = 0; t < mult; ++t)
		{
			uKnots.push_back(k);
		}
	}
}

static void ExpandVKnots(
	const Handle(Geom_BSplineSurface)& bs,
	varray<double>& vKnots)
{
	vKnots.clear();
	const int n = bs->NbVKnots();
	for (int i = 1; i <= n; ++i)
	{
		const double k = bs->VKnot(i);
		const int mult = bs->VMultiplicity(i);
		for (int t = 0; t < mult; ++t)
		{
			vKnots.push_back(k);
		}
	}
}

bool ConvertOccBSplineSurfaceToSplineSurface(
	const Handle(Geom_BSplineSurface)& bs,
	base::SplineSurface& outSurf)
{
	if (bs.IsNull())
		return false;

	const int uDegree = bs->UDegree();
	const int vDegree = bs->VDegree();
	const int uNum = bs->NbUPoles();
	const int vNum = bs->NbVPoles();

	if (uNum < 2 || vNum < 2)
		return false;

	outSurf.Clear();
	outSurf.SetSurfaceDegree(uDegree, vDegree);

	// 1) 构造线性控制点：p00,p10,p20,...,p(u-1,0),p01,p11,...
	outSurf.m_CtrlPts.clear();
	for (int u = 1; u <= uNum; ++u)
	{
		for (int v = 1; v <= vNum; ++v)
		{
			gp_Pnt p = bs->Pole(u, v);
			outSurf.m_CtrlPts.push_back(ToVec4(p));
		}
	}

	// 2) 构造二维控制网 m_UVCtrlPts[v][u]
	outSurf.m_UVCtrlPts.clear();
	outSurf.m_UVCtrlPts.resize(vNum);
	for (int v = 0; v < vNum; ++v)
	{
		outSurf.m_UVCtrlPts.at(v).resize(uNum);
		for (int u = 0; u < uNum; ++u)
		{
			outSurf.m_UVCtrlPts.at(v).at(u) = outSurf.m_CtrlPts.at(v * uNum + u);
		}
	}

	outSurf.m_uNum = uNum;
	outSurf.m_vNum = vNum;

	// 3) knot 展开
	ExpandUKnots(bs, outSurf.m_uKnots);
	ExpandVKnots(bs, outSurf.m_vKnots);

	return true;
}

static TopoDS_Compound Make3DPolylineCompoundOnSurface(
	const Handle(Geom_Surface)& surf,
	const std::vector<gp_Pnt2d>& uvPts)
{
	TopoDS_Compound comp;
	BRep_Builder builder;
	builder.MakeCompound(comp);

	if (surf.IsNull() || uvPts.size() < 2)
		return comp;

	for (size_t i = 0; i + 1 < uvPts.size(); ++i)
	{
		gp_Pnt p0 = surf->Value(uvPts[i].X(), uvPts[i].Y());
		gp_Pnt p1 = surf->Value(uvPts[i + 1].X(), uvPts[i + 1].Y());


		// 跳过过短段
		if (p0.Distance(p1) < 1.0e-7)
			continue;

		TopoDS_Edge e = BRepBuilderAPI_MakeEdge(p0, p1);
		if (!e.IsNull())
			builder.Add(comp, e);
	}

	return comp;
}

static std::string SanitizeFileName(const std::string& s)
{
	std::string out;
	out.reserve(s.size());

	for (char c : s)
	{
		// Windows 非法字符：\ / : * ? " < > |
		if (c == '\\' || c == '/' || c == ':' || c == '*' ||
			c == '?' || c == '"' || c == '<' || c == '>' || c == '|')
		{
			out.push_back('_');
		}
		else if (std::isspace(static_cast<unsigned char>(c)))
		{
			out.push_back('_');
		}
		else
		{
			out.push_back(c);
		}
	}

	// 避免空字符串
	if (out.empty())
		out = "frame";

	return out;
}

static TopoDS_Compound Make3DPolylineCompound(
	const std::vector<gp_Pnt>& pts)
{
	TopoDS_Compound comp;
	BRep_Builder builder;
	builder.MakeCompound(comp);

	if (pts.size() < 2)
		return comp;

	for (size_t i = 0; i + 1 < pts.size(); ++i)
	{
		if (pts[i].Distance(pts[i + 1]) < 1.0e-7)
			continue;

		TopoDS_Edge e = BRepBuilderAPI_MakeEdge(pts[i], pts[i + 1]);
		if (!e.IsNull())
			builder.Add(comp, e);
	}

	return comp;
}

static TopoDS_Edge Make3DSegmentOnSurface(
	const Handle(Geom_Surface)& surf,
	const gp_Pnt2d& uv0,
	const gp_Pnt2d& uv1)
{
	gp_Pnt p0 = surf->Value(uv0.X(), uv0.Y());
	gp_Pnt p1 = surf->Value(uv1.X(), uv1.Y());
	return BRepBuilderAPI_MakeEdge(p0, p1);
}

void OccWin::DisplayDebugFramesOnFace(
	const TopoDS_Face& refFace,
	const std::vector<DebugFrame>& debugFrames)
{
	Handle(Geom_Surface) surf = BRep_Tool::Surface(refFace);
	if (surf.IsNull())
		return;
	int bridgeCnt = 0;
	int segmentCnt = 0;
	for (size_t k = 0; k < debugFrames.size(); ++k)
	{
		const DebugFrame& frame = debugFrames[k];

		// 1) 显示操作前的 loops
		if(/*false &&*/ k==0)
		{
			int pntCnt = 0;
			for (const auto& loop : frame.loopsBefore)
			{
				TopoDS_Compound e = Make3DPolylineCompoundOnSurface(surf, loop.points);
				if (e.IsNull())
					continue;

				Handle(AIS_Shape) ais = new AIS_Shape(e);
				ais->SetColor(loop.isOuter ? Quantity_NOC_BLUE1 : Quantity_NOC_GREEN);
				m_context->Display(ais, Standard_False);
				m_quadDivisionDebugAisArr.push_back(ais);

				const size_t n =
					(loop.points.size() >= 2 &&
						loop.points.front().Distance(loop.points.back()) < 1.0e-8)
					? loop.points.size() - 1
					: loop.points.size();

				for (size_t i = 0; i < n; ++i)
				{
					const size_t j = (i + 1) % n;

					const gp_Pnt2d& p0 = loop.points[i];
					const gp_Pnt2d& p1 = loop.points[j];

					gp_Pnt2d mid2d(
						0.5 * (p0.X() + p1.X()),
						0.5 * (p0.Y() + p1.Y())
					);

					gp_Pnt pt;
					gp_Pnt pt2;
					surf->D0(mid2d.X(), mid2d.Y(), pt);
					surf->D0(p0.X(), p0.Y(), pt2);

					//DisplaySolidBall(m_context, pt2, 0.5);
					/*DisplayTextLabel(
						m_context,
						pt,
						std::to_string(++pntCnt),
						Quantity_NOC_BLACK,
						26.0);*/
				}
			}
		}

		// 2) 显示中间操作线（bridge / split）
		if (true)
		{
			for (const auto& seg : frame.segments)
			{
				TopoDS_Shape segShape;

				if (!seg.polyline.empty() && seg.polyline.size() >= 2)
				{
					segShape = Make3DPolylineCompoundOnSurface(surf, seg.polyline);
				}
				else
				{
					segShape = Make3DSegmentOnSurface(surf, seg.p0, seg.p1);
				}

				if (segShape.IsNull())
					continue;

				Handle(AIS_Shape) ais = new AIS_Shape(segShape);

				std::string labelText;
				gp_Pnt2d mid2d;

				if (seg.type == DebugStepSegmentType::Bridge)
				{
					ais->SetColor(Quantity_NOC_BLACK);
					labelText = std::string(u8"桥接线") + std::to_string(++bridgeCnt);
				}
				else if (seg.type == DebugStepSegmentType::Split)
				{
					ais->SetColor(Quantity_NOC_DARKORANGE3);
					labelText = std::string(u8"分割线") + std::to_string(++segmentCnt);
				}
				else
				{
					ais->SetColor(Quantity_NOC_WHITE);
					labelText = "seg";
				}

				m_context->Display(ais, Standard_False);
				m_quadDivisionDebugAisArr.push_back(ais);

				// 计算标注位置：优先取 polyline 的弧长中点，否则取端点中点
				if (!seg.polyline.empty() && seg.polyline.size() >= 2)
				{
					mid2d = OccUntrimUtil::SamplePolylineByArcLength(seg.polyline, 0.5);
				}
				else
				{
					mid2d = gp_Pnt2d(
						0.5 * (seg.p0.X() + seg.p1.X()),
						0.5 * (seg.p0.Y() + seg.p1.Y()));
				}

				gp_Pnt mid3d;
				surf->D0(mid2d.X(), mid2d.Y(), mid3d);

				DisplayTextLabel(
					m_context,
					mid3d,
					labelText,
					Quantity_NOC_BLACK,
					45.0);
			}
		}
		

		// 3) 显示操作后的 loops
		if (false)
		{
			for (const auto& loop : frame.loopsAfter)
			{
				int pntCnt = 0;
				TopoDS_Compound e = Make3DPolylineCompoundOnSurface(surf, loop.points);
				if (e.IsNull())
					continue;

				Handle(AIS_Shape) ais = new AIS_Shape(e);
				ais->SetColor(loop.isOuter ? Quantity_NOC_MAGENTA1 : Quantity_NOC_YELLOW);
				m_context->Display(ais, Standard_False);
				m_quadDivisionDebugAisArr.push_back(ais);

				// 对每一段的中点做标注
				for (size_t i = 0; i + 1 < loop.points.size(); ++i)
				{
					const gp_Pnt2d& p0 = loop.points[i];
					const gp_Pnt2d& p1 = loop.points[i + 1];

					gp_Pnt2d mid2d(
						0.5 * (p0.X() + p1.X()),
						0.5 * (p0.Y() + p1.Y())
					);

					gp_Pnt pt;
					surf->D0(mid2d.X(), mid2d.Y(), pt);

					DisplayTextLabel(m_context, pt, std::to_string(++pntCnt));
				}
			}
		}
		
		if (false)
		{
			// 4) 显示 split 后的子域 loops
			for (const auto& loop : frame.splitChildLoops)
			{
				int pntCnt = 0;
				TopoDS_Compound e = Make3DPolylineCompoundOnSurface(surf, loop.points);
				if (e.IsNull())
					continue;

				Handle(AIS_Shape) ais = new AIS_Shape(e);
				ais->SetColor(Quantity_NOC_ORANGE);
				m_context->Display(ais, Standard_False);
				m_quadDivisionDebugAisArr.push_back(ais);

				const size_t n =
					(loop.points.size() >= 2 &&
						loop.points.front().Distance(loop.points.back()) < 1.0e-8)
					? loop.points.size() - 1
					: loop.points.size();

				for (size_t i = 0; i < n; ++i)
				{
					const size_t j = (i + 1) % n;

					const gp_Pnt2d& p0 = loop.points[i];
					const gp_Pnt2d& p1 = loop.points[j];

					gp_Pnt2d mid2d(
						0.5 * (p0.X() + p1.X()),
						0.5 * (p0.Y() + p1.Y())
					);

					gp_Pnt pt;
					surf->D0(mid2d.X(), mid2d.Y(), pt);

					DisplayTextLabel(
						m_context,
						pt,
						std::to_string(++pntCnt),
						Quantity_NOC_BLACK,
						20.0);
				}
			}
		}
		
	}
}
OccWin::OccWin(QWidget *parent) : QWidget(parent)
{
	setAttribute(Qt::WA_NativeWindow);      // 确保 winId() 有效
	setMouseTracking(true);
	setFocusPolicy(Qt::StrongFocus);        // 接收键盘事件

	initViewer();

	Handle(Prs3d_Drawer) style = m_context->HighlightStyle(Prs3d_TypeOfHighlight_Selected);
	style->SetColor(Quantity_NOC_YELLOW);
	style->SetDisplayMode(1);
	style->SetTransparency(0.8);
	style->SetTransparency(0.0f);
	style->SetMethod(Aspect_TOHM_COLOR);

}

OccWin::~OccWin() 
{

}

void OccWin::initViewer()
{
	m_displayConnection = new Aspect_DisplayConnection();
	m_graphicDriver = new OpenGl_GraphicDriver(m_displayConnection);
	m_viewer = new V3d_Viewer(m_graphicDriver);
	m_viewer->SetDefaultLights();
	m_viewer->SetLightOn();

	m_context = new AIS_InteractiveContext(m_viewer);
	m_view = m_viewer->CreateView();

	Handle(WNT_Window) win = new WNT_Window((Aspect_Handle)winId());
	m_view->SetWindow(win);
	if (!win->IsMapped())
		win->Map();
	if (settingUsdInPaper)
	{
		m_view->SetBackgroundColor(Quantity_NOC_WHITE);
	}
	else
	{
		m_view->SetBackgroundColor(Quantity_NOC_GRAY60);  
	}
	m_view->MustBeResized();
	m_view->TriedronDisplay(Aspect_TOTP_LEFT_LOWER, Quantity_NOC_WHITE, 0.08, V3d_ZBUFFER);
	m_view->SetShadingModel(Graphic3d_TypeOfShadingModel::Graphic3d_TOSM_FACET); 
	m_view->SetProj(V3d_Zpos);
	m_view->Update();
}
void OccWin::DisplayTextLabel(
    const Handle(AIS_InteractiveContext)& context,
    const gp_Pnt& pnt,
    const std::string& text,
    Quantity_NameOfColor color,
    double height)
{
    Handle(AIS_TextLabel) aisText = new AIS_TextLabel();

    // 按 UTF-8 解释 std::string
    QString qText = QString::fromUtf8(text.c_str());
    std::wstring wText = qText.toStdWString();

    aisText->SetText(
        TCollection_ExtendedString(
            reinterpret_cast<const Standard_ExtString>(wText.c_str())));

    aisText->SetPosition(pnt);
    aisText->SetColor(color);
    aisText->SetHeight(height);

    context->Display(aisText, Standard_False);
}
void OccWin::DisplayShape(const TopoDS_Shape& shape)
{
	m_context->RemoveAll(false);

	Handle(AIS_Shape) aisShape = new AIS_Shape(shape);

	if (settingUsdInPaper)
	{
		aisShape->SetColor(Quantity_NOC_GRAY);        // 注释掉
		//aisShape->SetColor(Quantity_NOC_SLATEGRAY);        // 注释掉
	}
	else
	{
		aisShape->SetColor(Quantity_NOC_BLUE2);        // 注释掉
	}
	aisShape->SetTransparency(0.3); // 注释掉

	m_context->SetDisplayMode(aisShape, AIS_Shaded, false);

	// ✅ 显示图形
	m_context->Display(aisShape, true);

	// ✅ 激活 Shape 的选择模式（mode 0 = 整体）
	m_context->Activate(aisShape, 0, true);  // ← 非常关键

	m_view->FitAll();
}



void OccWin::DisplayArrow(const Handle(AIS_InteractiveContext)& context, const gp_Pnt& startPnt, const gp_Vec& direction, double length, Quantity_NameOfColor color)
{
	if (direction.Magnitude() < 1e-6)
		return;

	// ---------- ① 单位化方向 ----------
	gp_Vec dir = direction;
	dir.Normalize();
	//DisplaySolidBall(m_context, startPnt,0.5);
	// ---------- ② 箭头参数 ----------
	double coneLen = length * 0.1;     // 头部长度
	

	// ---------- ③ 箭杆终点 ----------
	gp_Pnt shaftEnd =
		startPnt.Translated(dir * (length - coneLen));

	//length /= 4;
	double coneRadius = length * 0.06;  // 头部半径
	// ---------- ④ 箭杆 ----------
	Handle(Geom_TrimmedCurve) line =
		GC_MakeSegment(startPnt, shaftEnd);

	TopoDS_Edge edge =
		BRepBuilderAPI_MakeEdge(line);

	Handle(AIS_Shape) aisLine =
		new AIS_Shape(edge);

	aisLine->SetColor(color);
	aisLine->SetWidth(4.0);

	context->Display(aisLine, Standard_False);

	// ---------- ⑤ 箭头圆锥 ----------
	gp_Ax2 ax2(shaftEnd, dir);

	TopoDS_Shape cone =
		BRepPrimAPI_MakeCone(
			ax2,
			coneRadius,
			0.0,
			coneLen);

	Handle(AIS_Shape) aisCone =
		new AIS_Shape(cone);

	aisCone->SetColor(color);

	context->Display(aisCone, Standard_False);
}


void OccWin::resizeEvent(QResizeEvent *event)
{
	QWidget::resizeEvent(event);
	if (!m_view.IsNull())
		m_view->MustBeResized();
}

void OccWin::paintEvent(QPaintEvent *event)
{
	QWidget::paintEvent(event);
	if (!m_view.IsNull())
		m_view->Redraw();
}

void OccWin::mousePressEvent(QMouseEvent* event)
{
	if (event->button() == Qt::LeftButton)
	{
		m_context->MoveTo(event->pos().x(), event->pos().y(), m_view, Standard_True);
		m_context->Select(Standard_True); // 自动高亮

		if (m_context->NbSelected() > 0)
		{
			Handle(AIS_InteractiveObject) selectedObj;
			for (m_context->InitSelected(); m_context->MoreSelected(); m_context->NextSelected())
			{
				selectedObj = m_context->SelectedInteractive();
				qDebug() << "Selected: " << selectedObj->DynamicType()->Name() ;
			}
		}
		m_view->Redraw();
	}
	else if (event->button() == Qt::RightButton)
	{
		m_view->StartRotation(event->x(), event->y());
	}
	m_lastMousePos = event->pos();
}

void OccWin::mouseReleaseEvent(QMouseEvent * event)
{
	m_lastMousePos = event->pos();
	QWidget::mouseReleaseEvent(event);
}


void OccWin::mouseMoveEvent(QMouseEvent *event)
{
	if (!m_view.IsNull())
	{
		if (event->buttons() & Qt::RightButton)
		{
			m_view->Rotation(event->x(), event->y());
			m_view->Redraw();
		}
		else if ((event->buttons() & Qt::LeftButton) && (event->modifiers() & Qt::ControlModifier))
		{
			QPoint delta = event->pos() - m_lastMousePos;

			m_view->Pan(delta.x(), -delta.y());
			m_view->Redraw();
		}
	}
	m_lastMousePos = event->pos();

	QWidget::mouseMoveEvent(event);
}


void OccWin::wheelEvent(QWheelEvent *event)
{
	if (m_view.IsNull())
		return;

	QPointF pos = event->position();

	// 模拟拖动距离，正数向上，负数向下
	int delta = (event->angleDelta().y() > 0) ? 10 : -10;

	// start 是鼠标当前位置
	int startX = pos.x();
	int startY = pos.y();

	// end 模拟拖动结束点，比如竖直方向偏移 delta
	int endX = startX;
	int endY = startY + delta;

	m_view->StartZoomAtPoint(startX, startY);
	m_view->ZoomAtPoint(startX, startY, endX, endY);

	event->accept();
	m_view->Redraw();
}

#include <AIS_Point.hxx>
#include <Geom_CartesianPoint.hxx>
#include <Quantity_Color.hxx>
#include <Aspect_TypeOfMarker.hxx>
#include <AIS_InteractiveContext.hxx>


bool OccWin::GenerateMidSurface(double maxThickness, double minThickness)
{
	TopoShapeArr shapeArr;
	GetSelectedShape(shapeArr);
	bool suc = false;
	MidSurfGenerator midsurfGenerator(shapeArr);
	midsurfGenerator.setThickness(maxThickness, minThickness);
	suc = midsurfGenerator.Generate();


	//for test 

	//delete all context interactive objects
	// 1️⃣ 收集所有对象
	Handle(TColStd_HSequenceOfTransient) allObjects = new TColStd_HSequenceOfTransient();

	for (m_context->InitCurrent(); m_context->MoreCurrent(); m_context->NextCurrent())
	{
		Handle(AIS_InteractiveObject) obj = m_context->Current();
		if (!obj.IsNull())
		{
			allObjects->Append(obj);
		}
	}

	// 2️⃣ 遍历临时列表，安全 Erase
	for (Standard_Integer i = 1; i <= allObjects->Length(); ++i)
	{
		Handle(AIS_InteractiveObject) obj = Handle(AIS_InteractiveObject)::DownCast(allObjects->Value(i));
		if (!obj.IsNull())
		{
			//m_context->Erase(obj, Standard_True);
		}
	}

	// 3️⃣ 刷新视图
	m_context->UpdateCurrentViewer();

	// ---------- ① 预定义颜色表 ----------
	std::vector<Quantity_NameOfColor> colorTable =
	{
		Quantity_NOC_RED,
		Quantity_NOC_GREEN,
		Quantity_NOC_BLUE1,
		Quantity_NOC_YELLOW,
		Quantity_NOC_CYAN1,
		Quantity_NOC_MAGENTA1,
		Quantity_NOC_ORANGE,
		Quantity_NOC_PURPLE,
		Quantity_NOC_SALMON,
		Quantity_NOC_TURQUOISE,
		Quantity_NOC_GOLD,
		Quantity_NOC_PINK
	};

	//test for fac
	for (const auto& fac : midsurfGenerator.m_fac) 
	{
		Handle(AIS_Shape) aisShape = new AIS_Shape(fac);

		// 2️⃣ 设置颜色为红色
		aisShape->SetColor(Quantity_NOC_RED);
		aisShape->SetDisplayMode(AIS_Shaded);
		// 3️⃣ 显示
		m_context->Display(aisShape, Standard_False);
	}

	/*OccMidSurfCommFunction::SaveToObj(
		OccMidSurfCommFunction::MergeFacesToCompound(midsurfGenerator.m_fac),
		"./obj/midfac.obj");*/

	
	//test for testClsfyFac
	int grpId = 0;
	int cnt = 0;
	auto xstest = midsurfGenerator.m_sc;
	int tsize = xstest.size();

	double x01 = 0,x1 = 0,x10 = 0,x20 = 0,xelse = 0;
	for (auto i : xstest)
	{
		if (i < 0.001)
		{
			x01++;
		}
		else if (i < 0.01)
		{
			x1++;
		}
		else if (i < 0.1)
		{
			x10++;
		}
		else if (i < 0.2)
		{
			x20++;
		}
		else
		{
			xelse++;
		}
	}
	std::cout << "conclusion:" << x01/tsize << " " << x1 / tsize << " " << x10 / tsize << " " << x20 / tsize << " " << xelse / tsize << std::endl;
	std::sort(xstest.begin(), xstest.end());
	for (const auto& facArr : midsurfGenerator.testString)
	{
		//DisplayTextLabel(m_context, facArr.first, facArr.second);
	}
	for (const auto& facArr : midsurfGenerator.testArrow)
	{
		//DisplayArrow(m_context, facArr.first, facArr.second);
	}
	for (const auto& facArr : midsurfGenerator.testClsfyFac)
	{
		Quantity_NameOfColor col =
			colorTable[grpId % colorTable.size()];
		/*if (grpId >= colorTable.size())
		{
			break;
		}*/
		for (auto& fac : facArr)
		{
			Handle(AIS_Shape) aisShape = new AIS_Shape(fac);

			// 2️⃣ 设置颜色为红色
			aisShape->SetColor(col);
			aisShape->SetDisplayMode(AIS_Shaded);
			// 3️⃣ 显示
			//m_context->Display(aisShape, Standard_False);
		}
		grpId++;
	}

	//test for midwire
	for (const auto& wire : midsurfGenerator.m_wire)
	{
		Handle(AIS_Shape) aisWire = new AIS_Shape(wire);
		// 2️⃣ 设置颜色为红色
		aisWire->SetColor(Quantity_NOC_RED);

		// 3️⃣ 显示
		//m_context->Display(aisWire, Standard_True);
	}
	int c = 0;
	for (const auto& edg : midsurfGenerator.m_edges)
	{
		Handle(AIS_Shape) aisEdg = new AIS_Shape(edg);
		//// 2️⃣ 设置颜色为红色
		aisEdg->SetColor(Quantity_NOC_RED);
		aisEdg->SetWidth(2.0);
		gp_Vec t;
		gp_Pnt mid = OccMidSurfCommFunction::GetEdgeMidPointAndDeriv(edg, t);
		//DisplayTextLabel(m_context, mid, std::to_string(c++),Quantity_NOC_BLACK,40);
		//DisplayArrow(m_context, mid, t,1);
		// 3️⃣ 显示
		//m_context->Display(aisEdg, Standard_True);
	}

	//test for sdfPnts
	int pntCnt = 0;
	for (const auto& pt : midsurfGenerator.m_points) 
	{
		Handle(Geom_Point) geomPoint = new Geom_CartesianPoint(pt);
		Handle(AIS_Point) aisPoint = new AIS_Point(geomPoint);

		aisPoint->SetColor(Quantity_NOC_RED);
		aisPoint->SetMarker(Aspect_TOM_PLUS);
		//aisPoint->SetWidth(pntSize);  // 设置点的大小
		//DisplaySolidBall(m_context, pt, 0.5);
		//m_context->Display(aisPoint, Standard_False);
		//DisplayTextLabel(m_context, pt, std::to_string(++pntCnt));
		//OccMidSurfCommFunction::ShowTextAtPoint(pt, (std::to_string(pntSize)).c_str(), m_context);
		//pntSize *= 10;
	}
	m_context->UpdateCurrentViewer();
	//TODO midsurf generator && postprocess
	return suc;
}
static bool BuildOffsetSurfacesByMaps(
	const varray<SplineSurface>& midSurfs,
	const std::vector<Handle(Geom_Surface)>& refSurfs,
	const std::vector<int>& patchSigns,
	double thickness,
	varray<SplineSurface>& bottomSurfs,
	varray<SplineSurface>& topSurfs,
	double tol = 1.0e-5)
{
	bottomSurfs.clear();
	topSurfs.clear();

	if (midSurfs.size() != static_cast<int>(refSurfs.size()))
		return false;

	const int surfNum = midSurfs.size();
	if (surfNum == 0)
		return true;

	bottomSurfs = midSurfs;
	topSurfs = midSurfs;

	for (int i = 0; i < surfNum; ++i)
	{
		if (!IsSurfaceCtrlLayoutValid(midSurfs.at(i)))
			return false;

		EnsureUVCtrlPtsFromLinear(bottomSurfs[i]);
		EnsureUVCtrlPtsFromLinear(topSurfs[i]);
	}

	std::map<XYZKey, std::vector<Vec4>> bottomMap;
	std::map<XYZKey, std::vector<Vec4>> topMap;

	// 第一次遍历：计算每个控制点偏移后的 top / bottom 候选点，并放到 map 里
	for (int sid = 0; sid < surfNum; ++sid)
	{
		const SplineSurface& midSS = midSurfs.at(sid);
		const Handle(Geom_Surface)& refSurf = refSurfs.at(sid);

		if (refSurf.IsNull())
			return false;

		for (int v = 0; v < midSS.m_vNum; ++v)
		{
			for (int u = 0; u < midSS.m_uNum; ++u)
			{
				const int idx = GetSurfaceCtrlIndex(midSS, v, u);
				const Vec4& midCtrl = midSS.m_CtrlPts.at(idx);
				if (fabs(midCtrl.x + 165) < 1 && fabs(midCtrl.y + 20) < 1)
				{
					int x = 0;
				}
				const double uu = ComputeGrevilleAbscissa(
					midSS.m_uKnots, midSS.m_uDegree, u);
				const double vv = ComputeGrevilleAbscissa(
					midSS.m_vKnots, midSS.m_vDegree, v);

				gp_Pnt projP;
				gp_Vec n;
				if (!ProjectPointAndEvalNormal(refSurf, midCtrl, projP, n))
					return false;
				if (patchSigns[sid] < 0)
					n.Reverse();
				Vec4 topPt = midCtrl;
				topPt.x += static_cast<float>(0.5 * thickness * n.X());
				topPt.y += static_cast<float>(0.5 * thickness * n.Y());
				topPt.z += static_cast<float>(0.5 * thickness * n.Z());

				Vec4 botPt = midCtrl;
				botPt.x -= static_cast<float>(0.5 * thickness * n.X());
				botPt.y -= static_cast<float>(0.5 * thickness * n.Y());
				botPt.z -= static_cast<float>(0.5 * thickness * n.Z());
				
				const XYZKey key = MakeXYZKey(midCtrl, tol);
				
				topMap[key].push_back(topPt);
				bottomMap[key].push_back(botPt);
			}
		}
	}

	// 第二次遍历：从 map 中取平均值或唯一值，回写到每个 patch
	for (int sid = 0; sid < surfNum; ++sid)
	{
		const SplineSurface& midSS = midSurfs.at(sid);

		for (int v = 0; v < midSS.m_vNum; ++v)
		{
			for (int u = 0; u < midSS.m_uNum; ++u)
			{
				const int idx = GetSurfaceCtrlIndex(midSS, v, u);
				const Vec4& midCtrl = midSS.m_CtrlPts.at(idx);
				const XYZKey key = MakeXYZKey(midCtrl, tol);

				auto itTop = topMap.find(key);
				auto itBot = bottomMap.find(key);
				if (itTop == topMap.end() || itBot == bottomMap.end())
					return false;

				const Vec4 topFinal = AverageVec4Array(itTop->second);
				const Vec4 botFinal = AverageVec4Array(itBot->second);

				SetCtrlPtLinear(topSurfs[sid], v, u, topFinal);
				SetCtrlPtLinear(bottomSurfs[sid], v, u, botFinal);
			}
		}
	}

	return true;
}
static bool ComputePatchSignFromFaceAndBSpline(
	const TopoDS_Face& refFace,
	const Handle(Geom_BSplineSurface)& bs,
	int& signOut)
{
	signOut = 1;

	if (bs.IsNull())
		return false;

	Handle(Geom_Surface) refSurf = BRep_Tool::Surface(refFace);
	if (refSurf.IsNull())
		return false;

	// 1) 取 bs 的参数中点
	Standard_Real u1, u2, v1, v2;
	bs->Bounds(u1, u2, v1, v2);

	const Standard_Real uMid = 0.5 * (u1 + u2);
	const Standard_Real vMid = 0.5 * (v1 + v2);

	// 2) 在 bs 上求中点和法向
	gp_Pnt pMid;
	gp_Vec duPatch, dvPatch;
	bs->D1(uMid, vMid, pMid, duPatch, dvPatch);

	gp_Vec nPatch = duPatch.Crossed(dvPatch);
	if (nPatch.Magnitude() < 1.0e-12)
		return false;
	nPatch.Normalize();

	// 3) 将 bs 中点投影到原始 face 对应曲面
	GeomAPI_ProjectPointOnSurf projector;
	projector.Init(pMid, refSurf);
	if (projector.NbPoints() <= 0)
		return false;

	Standard_Real uRef = 0.0, vRef = 0.0;
	projector.LowerDistanceParameters(uRef, vRef);

	// 4) 在原始曲面上求法向
	gp_Pnt pRef;
	gp_Vec duRef, dvRef;
	refSurf->D1(uRef, vRef, pRef, duRef, dvRef);

	gp_Vec nRef = duRef.Crossed(dvRef);
	if (nRef.Magnitude() < 1.0e-12)
		return false;
	nRef.Normalize();
	if (refFace.Orientation() == TopAbs_REVERSED)
	{
		nRef.Reverse();
	}
	// 5) 点积判断方向
	signOut = (nPatch.Dot(nRef) < 0.0) ? -1 : 1;
	return true;
}
void OccWin::GenerateQuadDivision()
{
	TopoShapeArr shapeArr;
	GetSelectedShape(shapeArr);

	if (shapeArr.empty())
	{
		std::cout << "No shape selected." << std::endl;
		return;
	}

	if (!debugForQuadDiv)
		HideSelectedOriginalShapes();

	// 清理上一次 patch 显示
	for (size_t i = 0; i < m_quadDivisionAisArr.size(); ++i)
	{
		if (!m_quadDivisionAisArr[i].IsNull())
		{
			m_context->Remove(m_quadDivisionAisArr[i], Standard_False);
		}
	}
	m_quadDivisionAisArr.clear();

	// 清理上一次 debug step 显示
	ClearQuadDivisionDebugDisplay();

	UntrimBuildOptions opt;
	opt.boundarySampleCountPerEdge = 30;
	opt.coonsSampleU = 15;
	opt.coonsSampleV = 15;
	opt.fitDegreeU = 3;
	opt.fitDegreeV = 3;
	opt.verbose = true;

	OccUntrimmingBuilder builder(opt);

	TopoShapeArr resultFaceArr;

	RWGeometric rwg;
	varray <SplineSurface> sss;
	std::vector<Handle(Geom_Surface)> refSurfs;
	varray <SplineVolume> svs,noG0;
	const double wallThickness = 2.0; // 先固定
	std::vector<QuadPatchRecord> allPatches;
	std::vector<int> patchSigns;
	
	auto ProcessOneFace = [&](const TopoDS_Face& face)
		{
			UntrimBuildResult result;
			if (!builder.Build(face, result))
			{
				std::cout << "Untrimming failed: " << result.message << std::endl;
				return;
			}
			//DisplayMixedUvPoints(face, builder);
			// 显示每个 face 对应的 debug frames
			if (!result.debugFrames.empty() && debugForQuadDiv)
			{
				DisplayDebugFramesOnFace(face, result.debugFrames);
			}
			SaveDebugFramesToStep(face, result.debugFrames, "D:/debug_steps");
			std::cout << "Generated patch count = " << result.patches.size() << std::endl;

			for (size_t i = 0; i < result.patches.size(); ++i)
			{
				Handle(Geom_BSplineSurface) bs = result.patches[i];
				if (bs.IsNull())
					continue;

				SplineSurface ssMid;
				QuadPatchRecord rec;
				rec.patchId = static_cast<int>(allPatches.size());
				rec.refFace = face;
				rec.uvPatch = result.uvPatches[i];

				allPatches.push_back(rec);
				refSurfs.push_back(bs);
				if (useConvert)
				{
					ConvertOccBSplineSurfaceToSplineSurface(bs, ssMid);
				}
				else
				{
					if (!BuildSplineSurfaceFromUVQuadPatch(face, result.uvPatches[i], ssMid))
					{
						std::cout << "BuildSplineSurfaceFromUVQuadPatch failed." << std::endl;
						continue;
					}
					
				}
				if(useG0)
				{
					SplineSurface ssBottom, ssTop;
					if (!OffsetSplineSurfaceByCtrlPts(ssMid, bs, wallThickness, false, ssBottom))
					{
						std::cout << "Offset bottom surface failed." << std::endl;
						continue;
					}
					if (!OffsetSplineSurfaceByCtrlPts(ssMid, bs, wallThickness, true, ssTop))
					{
						std::cout << "Offset top surface failed." << std::endl;
						continue;
					}

					SplineVolume sv;
					if (!BuildSplineVolumeFromThreeLayers(ssBottom, ssMid, ssTop, sv))
					{
						std::cout << "BuildSplineVolumeFromThreeLayers failed." << std::endl;
						continue;
					}
					noG0.push_back(sv);
				}

				int sign = 1;
				if (!ComputePatchSignFromFaceAndBSpline(face, bs, sign))
				{
					std::cout << "ComputePatchSignFromFaceAndBSpline failed, use +1 by default." << std::endl;
					sign = 1;
				}
				patchSigns.push_back(sign);
				sss.push_back(ssMid);

				/**/

				std::cout << "Patch " << i
					<< ", UDegree = " << bs->UDegree()
					<< ", VDegree = " << bs->VDegree()
					<< std::endl;

				BRepBuilderAPI_MakeFace mkFace(bs, Precision::Confusion());
				if (!mkFace.IsDone())
					continue;

				resultFaceArr.push_back(mkFace.Face());
			}
		};
	
	for (const TopoDS_Shape& shape : shapeArr)
	{
		if (shape.IsNull())
			continue;

		if (shape.ShapeType() == TopAbs_FACE)
		{
			ProcessOneFace(TopoDS::Face(shape));
		}
		else
		{
			for (TopExp_Explorer exp(shape, TopAbs_FACE); exp.More(); exp.Next())
			{
				ProcessOneFace(TopoDS::Face(exp.Current()));
			}
		}
	}
	if (!debugForQuadDiv)
	{
		if (useG0)
		{
			rwg.WriteSplineVolume("./nurbs/vol_rslt_noG0.txt", noG0);
		}
		
		rwg.WriteSplineSurface("./nurbs/rslt.txt", sss);
		/*varray<SplineSurface> bottomSurfs, topSurfs;
		if (!BuildOffsetSurfacesUnified(
			sss,
			refSurfs,
			wallThickness,
			bottomSurfs,
			topSurfs,
			1.0e-4))
		{
			std::cout << "BuildOffsetSurfacesUnified failed." << std::endl;
		}
		svs.clear();*/
		varray<SplineSurface> bottomSurfs, topSurfs;
		if (!BuildOffsetSurfacesByMaps(
			sss,
			refSurfs,
			patchSigns,
			wallThickness,
			bottomSurfs,
			topSurfs,
			1.0e-4))
		{
			std::cout << "BuildOffsetSurfacesByMaps failed." << std::endl;
		}
		else
		{
			svs.clear();
			for (int i = 0; i < sss.size(); ++i)
			{
				SplineVolume sv;
				if (!BuildSplineVolumeFromThreeLayers(
					bottomSurfs[i],
					sss[i],
					topSurfs[i],
					sv))
				{
					std::cout << "BuildSplineVolumeFromThreeLayers failed: " << i << std::endl;
					continue;
				}
				svs.push_back(sv);
			}
		}

	/*sss.clear();
	if (!BuildConformPatchesAndVolumes(allPatches, wallThickness, sss, svs))
	{
		std::cout << "BuildConformPatchesAndVolumes failed." << std::endl;
	}
	rwg.WriteSplineSurface("./nurbs/rslt.txt", sss);*/
		rwg.WriteSplineVolume("./nurbs/vol_rslt.txt", svs);
		continuityModification cm("./nurbs/vol_rslt.txt");
		cm.adjustVolContinuity();
	}

	if (resultFaceArr.empty())
	{
		std::cout << "No quad-division faces generated." << std::endl;
		m_context->UpdateCurrentViewer();
		return;
	}

	static Quantity_NameOfColor colorArr[] =
	{
		Quantity_NOC_RED,
		Quantity_NOC_GREEN,
		Quantity_NOC_BLUE1,
		Quantity_NOC_YELLOW,
		Quantity_NOC_CYAN1,
		Quantity_NOC_MAGENTA1,
		Quantity_NOC_ORANGE,
		Quantity_NOC_VIOLET
	};
	const int colorNum = sizeof(colorArr) / sizeof(colorArr[0]);

	for (size_t i = 0; i < resultFaceArr.size(); ++i)
	{
		Handle(AIS_Shape) aisShape = new AIS_Shape(resultFaceArr[i]);
		aisShape->SetColor(colorArr[i % colorNum]);
		aisShape->SetMaterial(Graphic3d_NOM_PLASTIC);
		aisShape->SetTransparency(0.0f);
		if (!debugForQuadDiv)
		{
			m_context->Display(
				aisShape,
				AIS_Shaded,      // 显示模式
				0,               // selection mode
				Standard_False,  // 不立即刷新
				Standard_True);  // 允许分解
		}
		

		m_quadDivisionAisArr.push_back(aisShape);
	}

	m_context->UpdateCurrentViewer();
}

void OccWin::ClearQuadDivisionDebugDisplay()
{
	for (size_t i = 0; i < m_quadDivisionDebugAisArr.size(); ++i)
	{
		if (!m_quadDivisionDebugAisArr[i].IsNull())
			m_context->Remove(m_quadDivisionDebugAisArr[i], Standard_False);
	}
	m_quadDivisionDebugAisArr.clear();
}

void OccWin::GetSelectedShape(TopoShapeArr& shapeArr)
{
	if (!m_context || m_context->NbSelected() == 0)
	{
		return;
	}

	for (m_context->InitSelected(); m_context->MoreSelected(); m_context->NextSelected())
	{
		Handle(AIS_InteractiveObject) selectedObj = m_context->SelectedInteractive();
		Handle(AIS_Shape) aisShape = Handle(AIS_Shape)::DownCast(selectedObj);
		if (!aisShape.IsNull())
		{
			shapeArr.push_back(aisShape->Shape());
		}
	}
}

void OccWin::HideSelectedOriginalShapes()
{
	std::vector<Handle(AIS_InteractiveObject)> selectedAisArr;

	// 先收集
	for (m_context->InitSelected(); m_context->MoreSelected(); m_context->NextSelected())
	{
		Handle(AIS_InteractiveObject) aisObj = m_context->SelectedInteractive();
		if (aisObj.IsNull())
			continue;

		selectedAisArr.push_back(aisObj);
	}

	// 再统一隐藏
	for (size_t i = 0; i < selectedAisArr.size(); ++i)
	{
		if (!selectedAisArr[i].IsNull())
		{
			m_context->Erase(selectedAisArr[i], Standard_False);
		}
	}
}

void OccWin::SaveDebugFramesToStep(
	const TopoDS_Face& refFace,
	const std::vector<DebugFrame>& debugFrames,
	const std::string& outDir)
{
	Handle(Geom_Surface) surf = BRep_Tool::Surface(refFace);
	if (surf.IsNull())
		return;

	for (size_t k = 0; k < debugFrames.size(); ++k)
	{
		const DebugFrame& frame = debugFrames[k];

		std::string msg = frame.message.empty() ? "frame" : frame.message;
		msg = SanitizeFileName(msg);

		auto BuildFilePath = [&](const std::string& suffix) -> std::string
			{
				std::ostringstream oss;
				oss << outDir << "/frame_"
					<< std::setw(3) << std::setfill('0') << k
					<< "_"
					<< msg
					<< "_"
					<< suffix
					<< ".step";
				return oss.str();
			};

		// 1) beforeloop
		if (!frame.loopsBefore.empty())
		{
			TopoDS_Compound comp = BuildLoopsCompound(surf, frame.loopsBefore);
			if (!comp.IsNull())
				SaveShapeToStep(comp, BuildFilePath("beforeloop"));
		}

		// 2) afterloop
		if (!frame.loopsAfter.empty())
		{
			TopoDS_Compound comp = BuildLoopsCompound(surf, frame.loopsAfter);
			if (!comp.IsNull())
				SaveShapeToStep(comp, BuildFilePath("afterloop"));
		}

		// 3) splitchild
		if (!frame.splitChildLoops.empty())
		{
			TopoDS_Compound comp = BuildLoopsCompound(surf, frame.splitChildLoops);
			if (!comp.IsNull())
				SaveShapeToStep(comp, BuildFilePath("splitchild"));
		}

		// 4) bridge
		{
			TopoDS_Compound comp = BuildSegmentsCompound(surf, frame.segments, DebugStepSegmentType::Bridge);
			if (!comp.IsNull())
				SaveShapeToStep(comp, BuildFilePath("bridge"));
		}

		// 5) segment（这里专指 split segment）
		{
			TopoDS_Compound comp = BuildSegmentsCompound(surf, frame.segments, DebugStepSegmentType::Split);
			if (!comp.IsNull())
				SaveShapeToStep(comp, BuildFilePath("segment"));
		}
	}
}

TopoDS_Compound OccWin::BuildSegmentsCompound(
	const Handle(Geom_Surface)& surf,
	const std::vector<DebugStepSegment2d>& segments,
	DebugStepSegmentType segType)
{
	BRep_Builder builder;
	TopoDS_Compound comp;
	builder.MakeCompound(comp);

	if (surf.IsNull())
		return comp;

	for (const auto& seg : segments)
	{
		if (seg.type != segType)
			continue;

		TopoDS_Shape segShape;
		if (!seg.polyline.empty() && seg.polyline.size() >= 2)
			segShape = Make3DPolylineCompoundOnSurface(surf, seg.polyline);
		else
			segShape = Make3DSegmentOnSurface(surf, seg.p0, seg.p1);

		if (!segShape.IsNull())
			builder.Add(comp, segShape);
	}

	return comp;
}

TopoDS_Compound OccWin::BuildLoopsCompound(
	const Handle(Geom_Surface)& surf,
	const std::vector<DebugLoop2d>& loops)
{
	BRep_Builder builder;
	TopoDS_Compound comp;
	builder.MakeCompound(comp);

	if (surf.IsNull())
		return comp;

	for (const auto& loop : loops)
	{
		TopoDS_Compound e = Make3DPolylineCompoundOnSurface(surf, loop.points);
		if (!e.IsNull())
			builder.Add(comp, e);
	}

	return comp;
}

bool OccWin::SaveShapeToStep(
	const TopoDS_Shape& shape,
	const std::string& filePath)
{
	if (shape.IsNull())
		return false;

	STEPControl_Writer writer;
	Interface_Static::SetCVal("write.step.schema", "AP214");

	IFSelect_ReturnStatus stat = writer.Transfer(shape, STEPControl_AsIs);
	if (stat != IFSelect_RetDone)
		return false;

	stat = writer.Write(filePath.c_str());
	return (stat == IFSelect_RetDone);
}

void OccWin::DisplayMixedUvPoints(
	const TopoDS_Face& refFace,
	const OccUntrimmingBuilder& builder)
{
	Handle(Geom_Surface) surf = BRep_Tool::Surface(refFace);
	if (surf.IsNull())
		return;

	const auto& uvTypeMap = builder.GetUvPointTypeMap();

	for (const auto& kv : uvTypeMap)
	{
		const gp_Pnt2d& uv = kv.first;
		const UVPointCurveType type = kv.second;

		if (type != UVPointCurveType::Mixed)
			continue;

		gp_Pnt pt;
		surf->D0(uv.X(), uv.Y(), pt);
		if(debugForQuadDiv)
			DisplaySolidBall(m_context, pt, 0.055);
	}
}