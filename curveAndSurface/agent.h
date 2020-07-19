#ifndef __AGENT_H__
#define __AGENT_H__
#include "Strategy.h"
#include "bezierCurve.h"
#include "bezierSurface.h"
#include "BezierVolume.h"
#include "bsplineCurve.h"
#include "bsplineCurve3d.h"
#include "bsplineSurface.h"
#include "bsplineVolume.h"
#include "nurbsCurve.h"
#include "nurbsSurface.h"
#include "sweptSurface.h"
#include "sweptVolume.h"
#include "sweptSurfaceTransform.h"
#include "sweptSurfaceTransformFrentFrame.h"
#include "sweptSurfaceTransformFrentFrameScale.h"
#include "sweptVolumeTransformFrentFrameScale.h"
#include "sweptVolumeTransformSilt.h"
#include "sweptSurfaceSilt.h"
#include "tool.h"
#include <QtWidgets/QMainWindow>
#include "qfiledialog.h"
class Strategy;
class BezierCurve;
class BezierSurface;
class BezierVolume;
class BsplineCurve;
class BsplineCurve3d;
class BsplineSurface3d;
class BSplineVolume;
class NurbsCurve;
class NurbsSurface;
class SweptSurface;
class SweptVolume;
class SweptSurfaceTransform;
class SweptSurfaceTransformFrentFrame;
class SweptSurfaceTransformFrentFrameScale;
class SweptVolumeTransformFrentFrameScale;
class Tool;
class SweptVolumeTransformSilt;
class SweptSurfaceTransformSilt;
class Agent:public QObject
{
	Q_OBJECT

public:
	Agent(QWidget *parent = Q_NULLPTR);
	~Agent();
public slots:
	void drawBezierCurve();
	void drawBezierCurveShortCut();
	void drawBezierSurface();
	void drawBezierVolume();

	void drawBsplineCurve();
	void drawBsplineCurve3d();
	void drawBsplineSurface();
	void drawBsplineVolume();
	void drawNurbsCurve();
	void drawNurbsSurface();
	void drawSweptSurface();
	void drawSweptVolume();
	void drawSweptSurfaceTransform();
	void drawSweptSurfaceTransformFrentFrame();
	void drawSweptSurfaceTransformFrentFrameScale();
	void drawSweptVolumeScale();
	void drawSweptVolumeSilt();
	void drawSweptSurfaceSilt();
	void exportOBJ();
	void filpWithMidLine();
private:
	Strategy *strategy;
};


#endif // !__AGENT_H__



