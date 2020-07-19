#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_curveAndSurface.h"
#include "agent.h"

class curveAndSurface : public QMainWindow
{
	Q_OBJECT

public:
	curveAndSurface(QWidget *parent = Q_NULLPTR);
	QAction *bezierCurveAct;
	QAction *bezierCurveShortCutAct;
	QAction *bezierSurfaceAct;
	QAction *bezierVolumeAct;

	QAction *bsplineCurveAct;
	QAction *bsplineCurve3dAct;
	QAction *bsplineSurfaceAct;
	QAction *bsplineVolumeAct;
	QAction *nurbsCurveAct;
	QAction *nurbsSurfaceAct;
	QAction *sweptSurfaceAct;
	QAction *sweptSurfaceTransformAct;
	QAction *sweptSurfaceTransformFrentFrameAct;
	QAction *sweptSurfaceTransformFrentFrameScaleAct;
	QAction *sweptSurfaceSiltAct;
	QAction *sweptVolumeAct;
	QAction *sweptVolumeScaleAct;
	QAction *sweptVolumeSiltAct;
	
	QAction *exportMeshFileAct;
	QAction *filpAct;
	void createMenu();
	void createActions();
	void createAgent();
private:
	Ui::curveAndSurfaceClass ui;
	Agent *agent;
};
