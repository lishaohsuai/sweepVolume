#include "agent.h"



Agent::Agent(QWidget *parent)
{
}


Agent::~Agent()
{
}


void Agent::drawBezierCurve() {
	strategy = new BezierCurve(NULL);
	strategy->genPoints("Non-shortcut");
	((BezierCurve*)strategy)->show();
}


void Agent::drawBezierCurveShortCut() {
	strategy = new BezierCurve(NULL);
	strategy->genPoints("deCasteljau");
	((BezierCurve*)strategy)->show();
}

void Agent::drawBezierSurface() {
	strategy = new BezierSurface(NULL);
	strategy->genPoints("deCasteljau");
	((BezierSurface*)strategy)->resizeGL(500, 500);
	((BezierSurface*)strategy)->show();
}
void Agent::drawBezierVolume() {
	strategy = new BezierVolume(NULL);
	strategy->genPoints("");
	((BezierVolume*)strategy)->resizeGL(500, 500);
	((BezierVolume*)strategy)->show();
}

void Agent::drawBsplineCurve() {
	strategy = new BsplineCurve(NULL);
	strategy->genPoints("C:\\Users\\lee\\Desktop\\code\\curve_and_surface\\curve_surface\\curveAndSurface\\config\\bsplineCurve\\bsplineCurve.json");
	((BsplineCurve*)strategy)->resizeGL(700, 700);
	((BsplineCurve*)strategy)->show();
}

void Agent::drawBsplineCurve3d() {
	strategy = new BsplineCurve3d(NULL);
	//strategy->genPoints("C:\\Users\\lee\\Desktop\\code\\curve_and_surface\\curve_surface\\curveAndSurface\\config\\bsplineCurve3d\\bsplineCurve3d.json");
	strategy->genPoints("");
	((BsplineCurve3d*)strategy)->initializeGL();
	((BsplineCurve3d*)strategy)->resizeGL(700, 700);
	((BsplineCurve3d*)strategy)->show();
}

void Agent::drawBsplineSurface() {
	strategy = new BsplineSurface3d(NULL);
	//strategy->genPoints("C:\\Users\\lee\\Desktop\\code\\curve_and_surface\\curve_surface\\curveAndSurface\\config\\bsplineSurface\\bsplineSurface.json");
	strategy->genPoints("");
	((BsplineCurve3d*)strategy)->initializeGL();
	((BsplineCurve3d*)strategy)->resizeGL(700, 700);
	((BsplineCurve3d*)strategy)->show();
}


void Agent::drawBsplineVolume() {
	strategy = new BSplineVolume(NULL);
	//strategy->genPoints("C:\\Users\\lee\\Desktop\\code\\curve_and_surface\\curve_surface\\curveAndSurface\\config\\bsplineSurface\\bsplineSurface.json");
	strategy->genPoints("");
	((BsplineCurve3d*)strategy)->initializeGL();
	((BsplineCurve3d*)strategy)->resizeGL(700, 700);
	((BsplineCurve3d*)strategy)->show();
}

void Agent::drawNurbsCurve() {
	strategy = new NurbsCurve(NULL);
	//strategy->genPoints("C:\\Users\\lee\\Desktop\\code\\curve_and_surface\\curve_surface\\curveAndSurface\\config\\nurbsCurve\\nurbsCurve.json");
	strategy->genPoints("");
	((NurbsCurve*)strategy)->initializeGL();
	((NurbsCurve*)strategy)->resizeGL(700, 700);
	((NurbsCurve*)strategy)->show();
}


void Agent::drawNurbsSurface() {
	strategy = new NurbsSurface(NULL);
	//strategy->genPoints("C:\\Users\\lee\\Desktop\\code\\curve_and_surface\\curve_surface\\curveAndSurface\\config\\nurbsSurface\\nurbsSurface.json");
	strategy->genPoints("");
	((NurbsSurface*)strategy)->initializeGL();
	((NurbsSurface*)strategy)->resizeGL(700, 700);
	((NurbsSurface*)strategy)->show();
}

void Agent::drawSweptSurface() {
	strategy = new SweptSurface(NULL);
	//strategy->genPoints("C:\\Users\\lee\\Desktop\\code\\curve_and_surface\\curve_surface\\curveAndSurface\\config\\sweptSurface\\sweptSurface.json");
	strategy->genPoints("");
	((SweptSurface*)strategy)->initializeGL();
	((SweptSurface*)strategy)->resizeGL(700, 700);
	((SweptSurface*)strategy)->show();
}


void Agent::drawSweptVolume() {
	strategy = new SweptVolume(NULL);
	//strategy->genPoints("C:\\Users\\lee\\Desktop\\code\\curve_and_surface\\curve_surface\\curveAndSurface\\config\\sweptVolume\\sweptVolume.json");
	strategy->genPoints("");
	((SweptVolume*)strategy)->initializeGL();
	((SweptVolume*)strategy)->resizeGL(700, 700);
	((SweptVolume*)strategy)->show();
}



void Agent::drawSweptSurfaceTransform() {
	strategy = new SweptSurfaceTransform(NULL);
	strategy->genPoints("C:\\Users\\lee\\Desktop\\code\\curve_and_surface\\curve_surface\\curveAndSurface\\config\\sweptSurface\\sweptSurfacePlanar.json");
	((SweptVolume*)strategy)->initializeGL();
	((SweptVolume*)strategy)->resizeGL(700, 700);
	((SweptVolume*)strategy)->show();
}

void Agent::drawSweptSurfaceTransformFrentFrame() {
	strategy = new SweptSurfaceTransformFrentFrame(NULL);
	//strategy->genPoints("C:\\Users\\lee\\Desktop\\code\\curve_and_surface\\curve_surface\\curveAndSurface\\config\\sweptSurface\\sweptSurfaceFrentFrame.json");
	strategy->genPoints("");
	((SweptVolume*)strategy)->initializeGL();
	((SweptVolume*)strategy)->resizeGL(700, 700);
	((SweptVolume*)strategy)->show();
}


void Agent::drawSweptSurfaceTransformFrentFrameScale() {
	strategy = new SweptSurfaceTransformFrentFrameScale(NULL);
	strategy->genPoints("C:\\Users\\lee\\Desktop\\code\\curve_and_surface\\curve_surface\\curveAndSurface\\config\\sweptSurface\\sweptSurfaceFrentFrameScale.json");
	//strategy->genPoints("");
	((SweptVolume*)strategy)->initializeGL();
	((SweptVolume*)strategy)->resizeGL(700, 700);
	((SweptVolume*)strategy)->show();
}


void Agent::drawSweptVolumeScale() {
	strategy = new SweptVolumeTransformFrentFrameScale(NULL);
	//strategy->genPoints("C:\\Users\\lee\\Desktop\\code\\curve_and_surface\\curve_surface\\curveAndSurface\\config\\sweptSurface\\sweptSurfaceFrentFrameScale.json");
	strategy->genPoints("");
	((SweptVolume*)strategy)->initializeGL();
	((SweptVolume*)strategy)->resizeGL(700, 700);
	((SweptVolume*)strategy)->show();
}

void Agent::drawSweptVolumeSilt() {
	strategy = new SweptVolumeTransformSilt(NULL);
	//strategy->genPoints("C:\\Users\\lee\\Desktop\\code\\curve_and_surface\\curve_surface\\curveAndSurface\\config\\sweptSurface\\sweptSurfaceFrentFrameScale.json");
	strategy->genPoints("");
	((SweptVolume*)strategy)->initializeGL();
	((SweptVolume*)strategy)->resizeGL(700, 700);
	((SweptVolume*)strategy)->show();
}

void Agent::drawSweptSurfaceSilt() {
	strategy = new SweptSurfaceTransformSilt(NULL);
	//strategy->genPoints("C:\\Users\\lee\\Desktop\\code\\curve_and_surface\\curve_surface\\curveAndSurface\\config\\sweptSurface\\sweptSurfaceFrentFrameScale.json");
	strategy->genPoints("");
	((SweptVolume*)strategy)->initializeGL();
	((SweptVolume*)strategy)->resizeGL(700, 700);
	((SweptVolume*)strategy)->show();
}

void Agent::exportOBJ() {
	strategy->exportOBJ();
}

void Agent::filpWithMidLine() {
	strategy = new Tool(NULL);
	strategy->genPoints("");
	((Tool*)strategy)->initializeGL();
	((Tool*)strategy)->resizeGL(700, 700);
	((Tool*)strategy)->show();
}
