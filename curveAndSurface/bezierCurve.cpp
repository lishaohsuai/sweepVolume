#include "bezierCurve.h"

//参考 The NURBS Book 2nd (38/660)
BezierCurve::BezierCurve(QWidget *parent)//就是确定控制顶点数目和控制顶点
	:Strategy(),QWidget(parent) {

}

BezierCurve::~BezierCurve() {}

//使用公式计算
double BezierCurve::calcBernstein(double u, int n, int i, std::vector<int> &factorialN) {
	return factorialN[n] * pow(u, i)*pow((1 - u), n - i) / (double)(factorialN[i] * factorialN[n - i]);
}
//生成一个对应的点
QPointF BezierCurve::calcBernsteinDeCasteljau(double u, int n, int i) {
	std::vector<QPointF> Q;
	for (int i = 0; i <= n; i++) {
		Q.push_back(bezierCurveConfig.ctrlPoints[i]);
	}
	for (int k = 1; k <= n; k++) {
		for (int i = 0; i <= n - k; i++) {
			Q[i] = (1.0 - u) * Q[i] + u * Q[i + 1];
		}
	}
	return Q[0];
}


void BezierCurve::genPoints(std::string method) {
	jsonReader();
	std::vector<int> factorialN;
	calcfactorialN(factorialN);
	if (method == "deCasteljau") {
		for (int index = 0; index <= bezierCurveConfig.genPointNum; index++) {
			double u = (double)index / (double)bezierCurveConfig.genPointNum;
			bezierCurveConfig.curvaturePoints.push_back(
				calcBernsteinDeCasteljau(u,bezierCurveConfig.n,index));
		}
	}
	else {
		//开始生成新的点
		for (int index = 0; index <= bezierCurveConfig.genPointNum; index++) {
			double u = (double)index / (double)bezierCurveConfig.genPointNum;
			QPointF genPoint(0, 0);
			for (int i = 0; i <= bezierCurveConfig.n; i++) {
				genPoint += calcBernstein(u, bezierCurveConfig.n, i, factorialN)*bezierCurveConfig.ctrlPoints[i];
			}
			bezierCurveConfig.curvaturePoints.push_back(genPoint);
		}
	}
	
}


void BezierCurve::paintEvent(QPaintEvent *){
	std::cout << "[DEBUG] BezierCurve paintEvent" << std::endl;
	QPen pen;//画笔
	QPainter painter(this);//执行绘图操作
	//画出控制点
	pen.setWidth(3);                     // 1 表示点的大小（形状为方形）
	pen.setColor(Qt::black);
	painter.setPen(pen);
	painter.drawPoints(bezierCurveConfig.ctrlPoints.data(), bezierCurveConfig.ctrlPoints.size());

	//画控制多边形
	pen.setWidth(2);
	pen.setColor(Qt::black);
	painter.setPen(pen);
	for (int i = 0; i < bezierCurveConfig.ctrlPoints.size() - 1; i++) {
		painter.drawLine(bezierCurveConfig.ctrlPoints[i].x(), bezierCurveConfig.ctrlPoints[i].y()
			, bezierCurveConfig.ctrlPoints[i + 1].x(), bezierCurveConfig.ctrlPoints[i + 1].y());
	}

	//画bezier曲线, 把计算得到的点连线
	pen.setWidth(2);
	pen.setColor(Qt::red);
	painter.setPen(pen);
	for (int i = 0; i < bezierCurveConfig.curvaturePoints.size() - 1; i++) //-1是因为先调用的这个，不然会有溢出
		painter.drawLine(bezierCurveConfig.curvaturePoints[i].x(), bezierCurveConfig.curvaturePoints[i].y(),
			bezierCurveConfig.curvaturePoints[i + 1].x(), bezierCurveConfig.curvaturePoints[i + 1].y());
}

void BezierCurve::jsonReader() {
	QString QfileName = QFileDialog::getOpenFileName(NULL,
		QObject::tr("Open json file"),
		QObject::tr("./config"),
		QObject::tr("mesh (*.json);;"
			"All Files (*)"));
	std::string fileName = QfileName.toStdString();
	Json::Value root;
	Json::Reader reader;
	std::ifstream ifs(fileName.c_str());//open file example.json

	if (!reader.parse(ifs, root)) {
		// fail to parse
		std::cout << "fail to parse json \n";
	}
	else {
		// success
		std::cout << "[DEBUG] config file have found!!\n";
		std::cout << "name: " << root["name"].asString() << std::endl;
		bezierCurveConfig.genPointNum = root["genPointNum"].asInt();
		bezierCurveConfig.name = root["name"].asString();
		Json::Value points = root["Point"];
		for (int i = 0; i < points.size(); i++) {
			QPointF p(0, 0);
			p.setX(points[i][0].asDouble());
			p.setY(points[i][1].asDouble());
			bezierCurveConfig.ctrlPoints.push_back(p);
		}
		bezierCurveConfig.n = bezierCurveConfig.ctrlPoints.size() - 1;//阶是点的个数减去1
	}
}


void BezierCurve::calcfactorialN(std::vector<int> &factorialN) {
	int n = 1;
	for (int i = 0; i <= bezierCurveConfig.n; i++) {
		if (i == 0) {}
		else {
			n = n * i;
		}
		factorialN.push_back(n);
	}
	/*for (auto &vh : factorialN) {
		DEBUG("%d",vh);
	}*/
}