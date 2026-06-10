#pragma once

#include <QWidget>

#include <AIS_InteractiveContext.hxx>
#include <V3d_View.hxx>
#include <Graphic3d_GraphicDriver.hxx>
#include <Aspect_Handle.hxx>
#include <OpenGl_GraphicDriver.hxx>
#include <Aspect_DisplayConnection.hxx>
#include <AIS_Shape.hxx>
#include <TopoDS_Compound.hxx>
#include <Geom_Surface.hxx>
#include "occTypeDefine.h"
#include "occUntrimmingBuilder.h"

class OccWin : public QWidget
{
	Q_OBJECT

public:
	OccWin(QWidget *parent = nullptr);
	virtual ~OccWin();

	void DisplayMixedUvPoints(
		const TopoDS_Face& refFace,
		const OccUntrimmingBuilder& builder);
	void DisplayTextLabel(const Handle(AIS_InteractiveContext)& context, const gp_Pnt& pnt, const std::string& text, Quantity_NameOfColor color = Quantity_NOC_BLACK, double height = 16.0);
	/*void DisplayTextLabel(
		const Handle(AIS_InteractiveContext)& context,
		const gp_Pnt& pnt,
		const QString& text,
		Quantity_NameOfColor color = Quantity_NOC_BLACK,
		double height = 16.0);*/
	void DisplayShape(const TopoDS_Shape &shape);

	void DisplayArrow(
		const Handle(AIS_InteractiveContext)& context,
		const gp_Pnt& startPnt,
		const gp_Vec& direction,
		double length = 10.0,
		Quantity_NameOfColor color = Quantity_NOC_RED);

	Handle(V3d_View) getView() { return m_view; }
	Handle(AIS_InteractiveContext) GetContext() { return m_context; }
protected:
	void resizeEvent(QResizeEvent *event) override;
	void paintEvent(QPaintEvent *event) override;
	void mousePressEvent(QMouseEvent *event) override;
	void mouseReleaseEvent(QMouseEvent *event) override;
	void mouseMoveEvent(QMouseEvent *event) override;
	void wheelEvent(QWheelEvent *event) override;
	void GetSelectedShape(std::vector<TopoDS_Shape>&shapeArr);
private:
	void initViewer();

	Handle(Aspect_DisplayConnection) m_displayConnection;
	Handle(Graphic3d_GraphicDriver) m_graphicDriver;
	Handle(V3d_Viewer) m_viewer;
	Handle(V3d_View) m_view;
	Handle(AIS_InteractiveContext) m_context;

    std::vector<Handle(AIS_Shape)> m_quadDivisionAisArr;
    std::vector<Handle(AIS_Shape)> m_quadDivisionDebugAisArr;

	void HideSelectedOriginalShapes();
	void SaveDebugFramesToStep(
		const TopoDS_Face& refFace,
		const std::vector<DebugFrame>& debugFrames,
		const std::string& outDir);

	bool SaveShapeToStep(
		const TopoDS_Shape& shape,
		const std::string& filePath);

	TopoDS_Compound BuildLoopsCompound(
		const Handle(Geom_Surface)& surf,
		const std::vector<DebugLoop2d>& loops);

	TopoDS_Compound BuildSegmentsCompound(
		const Handle(Geom_Surface)& surf,
		const std::vector<DebugStepSegment2d>& segments,
		DebugStepSegmentType segType);
    void ClearQuadDivisionDebugDisplay();
    void DisplayDebugFramesOnFace(
        const TopoDS_Face& refFace,
        const std::vector<DebugFrame>& debugFrames);

	QPoint m_lastMousePos;
public slots:
	bool GenerateMidSurface(double maxThickness, double minThickness);
	void GenerateQuadDivision();
};
