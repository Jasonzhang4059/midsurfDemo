#pragma once
#include<TopoDS_Shape.hxx>
#include"occTypeDefine.h"
#include "occMidSurfGrouper.h"
class MidSurfGenerator 
{
public:
	explicit MidSurfGenerator(TopoShapeArr& shapeArr);
	~MidSurfGenerator();
	
	void setThickness(double maxThickness, double minThickness)
	{
		m_minThickness = minThickness;
		m_maxThickness = maxThickness;
	}
	bool Generate();

	//for test
	std::vector<gp_Pnt> m_points;
	std::vector<double> m_sc;
	std::vector<TopoDS_Face> m_fac;
	std::vector<TopoDS_Wire> m_wire;
	std::vector<TopoDS_Edge> m_edges;
	std::vector< TopoFaceArr> testClsfyFac;
	std::vector<std::pair<gp_Pnt, gp_Vec>> testArrow;
	std::vector<std::pair<gp_Pnt, std::string>> testString;
private:

	bool CalSdfMidPoint();

	bool BuildVirtualBoundary();

	bool ClassifySDFPntsByFace();

	bool ClassifySDFPntsByGroups();

	bool FitAllGroupMidSurf();

	void OffsetSurface();

	bool BuildAllGroupMidSurf();

	bool BuildOneGroupMidSurf(
		OccMidSurfGroup& group,
		bool doCheck = true);

	bool BuildUniformMidSurf(
		OccMidSurfGroup& group,
		bool doCheck = true);

	bool BuildVariableMidSurf(
		OccMidSurfGroup& group,
		bool doCheck = true);

	void CheckMidSurfDistance(
		const TopoDS_Face& midFace,
		const OccMidSurfGroup& group);

	TopoFaceArr m_orgFacArr;
	TopoShapeArr m_orgShapeArr;
	GpPntArr m_sdfMidPoints;
	double m_maxThickness;
	double m_minThickness;
	OccMidSurfGrouper m_midFacGrper;
};