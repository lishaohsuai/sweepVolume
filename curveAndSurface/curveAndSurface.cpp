#include "curveAndSurface.h"

curveAndSurface::curveAndSurface(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);
	createAgent();
	createActions();
	createMenu();
}


void curveAndSurface::createMenu() {
	QMenu *bezier = menuBar()->addMenu(tr("&beizer"));
	bezier->addAction(bezierCurveAct);
	bezier->addAction(bezierCurveShortCutAct);
	bezier->addAction(bezierSurfaceAct);
	bezier->addAction(bezierVolumeAct);

	QMenu *bspline = menuBar()->addMenu(tr("b&spline"));
	bspline->addAction(bsplineCurveAct);
	bspline->addAction(bsplineCurve3dAct);
	bspline->addAction(bsplineSurfaceAct);
	bspline->addAction(bsplineVolumeAct);
	bspline->addAction(nurbsCurveAct);
	bspline->addAction(nurbsSurfaceAct);
	

	QMenu *swept = menuBar()->addMenu(tr("s&wept"));
	swept->addAction(sweptSurfaceAct);
	swept->addAction(sweptVolumeAct);
	swept->addAction(sweptSurfaceTransformAct);
	swept->addAction(sweptSurfaceTransformFrentFrameAct);
	swept->addAction(sweptSurfaceTransformFrentFrameScaleAct);
	swept->addAction(sweptSurfaceSiltAct);
	swept->addAction(sweptVolumeScaleAct);
	swept->addAction(sweptVolumeSiltAct);

	QMenu *tool = menuBar()->addMenu(tr("tool"));
	tool->addAction(exportMeshFileAct);
	tool->addAction(filpAct);
}

void curveAndSurface::createActions() {
	bezierCurveAct = new QAction(tr("bezier Curve"),this);
	connect(bezierCurveAct, SIGNAL(triggered()), agent, SLOT(drawBezierCurve()));
	bezierCurveShortCutAct = new QAction(tr("bezier Curve shortCut"), this);
	connect(bezierCurveShortCutAct, SIGNAL(triggered()), agent, SLOT(drawBezierCurveShortCut()));
	bezierSurfaceAct = new QAction(tr("bezier Surface"), this);
	connect(bezierSurfaceAct, SIGNAL(triggered()), agent, SLOT(drawBezierSurface()));
	bezierVolumeAct = new QAction(tr("bezier Volume"), this);
	connect(bezierVolumeAct, SIGNAL(triggered()), agent, SLOT(drawBezierVolume()));

	bsplineCurveAct = new QAction(tr("bspline Curve"), this);
	connect(bsplineCurveAct, SIGNAL(triggered()), agent, SLOT(drawBsplineCurve()));
	bsplineCurve3dAct = new QAction(tr("bspline Curve 3d"), this);
	connect(bsplineCurve3dAct, SIGNAL(triggered()), agent, SLOT(drawBsplineCurve3d()));
	bsplineSurfaceAct = new QAction(tr("bspline Surface"), this);
	connect(bsplineSurfaceAct, SIGNAL(triggered()), agent, SLOT(drawBsplineSurface()));
	nurbsCurveAct = new QAction(tr("nurbs Curve"), this);
	bsplineVolumeAct = new QAction(tr("bspline Volume"), this);
	connect(bsplineVolumeAct, SIGNAL(triggered()), agent, SLOT(drawBsplineVolume()));
	connect(nurbsCurveAct, SIGNAL(triggered()), agent, SLOT(drawNurbsCurve()));
	nurbsSurfaceAct = new QAction(tr("nurbs Surface"), this);
	connect(nurbsSurfaceAct, SIGNAL(triggered()), agent, SLOT(drawNurbsSurface()));

	sweptSurfaceAct = new QAction(tr("swept Surface"), this);
	connect(sweptSurfaceAct, SIGNAL(triggered()), agent, SLOT(drawSweptSurface()));
	sweptVolumeAct = new QAction(tr("swept Volume"), this);
	connect(sweptVolumeAct, SIGNAL(triggered()), agent, SLOT(drawSweptVolume()));
	sweptSurfaceTransformAct = new QAction(tr("swept Surface Transform"), this);
	connect(sweptSurfaceTransformAct, SIGNAL(triggered()), agent, SLOT(drawSweptSurfaceTransform()));
	sweptSurfaceTransformFrentFrameAct = new QAction(tr("swept Surface Transform 3d"), this);
	connect(sweptSurfaceTransformFrentFrameAct, SIGNAL(triggered()), agent, SLOT(drawSweptSurfaceTransformFrentFrame()));
	sweptSurfaceTransformFrentFrameScaleAct = new QAction(tr("sweptSurfaceTransformScale"), this);
	connect(sweptSurfaceTransformFrentFrameScaleAct, SIGNAL(triggered()), agent, SLOT(drawSweptSurfaceTransformFrentFrameScale()));
	sweptSurfaceSiltAct = new QAction(tr("sweptSurfaceSilt"), this);
	connect(sweptSurfaceSiltAct, SIGNAL(triggered()), agent, SLOT(drawSweptSurfaceSilt()));
	sweptVolumeScaleAct = new QAction(tr("swept Volume scale"), this);
	connect(sweptVolumeScaleAct, SIGNAL(triggered()), agent, SLOT(drawSweptVolumeScale()));
	sweptVolumeSiltAct = new QAction(tr("swept Volume Silt"), this);
	connect(sweptVolumeSiltAct, SIGNAL(triggered()), agent, SLOT(drawSweptVolumeSilt()));

	exportMeshFileAct = new QAction(tr("Export Mesh File"), this);
	connect(exportMeshFileAct, SIGNAL(triggered()), agent, SLOT(exportOBJ()));
	filpAct = new QAction(tr("filp"), this);
	connect(filpAct, SIGNAL(triggered()), agent, SLOT(filpWithMidLine()));
}

void curveAndSurface::createAgent() {
	agent = new Agent(NULL);
}