#ifndef __TOOLBAR_H__
#define __TOOLBAR_H__
#include <QWidget>
#include <QtGui>
#include <QtWidgets>
class ToolBar : public QWidget {
	Q_OBJECT
public:
	ToolBar(QWidget * parent = 0);
	~ToolBar(void);
private:
	void CreateToolBarWidget(void);
};

#endif