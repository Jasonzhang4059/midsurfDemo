#pragma once

#include <QOpenGLWidget>
#include <QOpenGLFunctions_4_5_Core>
#include <QOpenGLShaderProgram>

#include "Camera.h"
#include "Option.h"

class win : public QOpenGLWidget, protected QOpenGLFunctions_4_5_Core
{
	Q_OBJECT

public:
	/*void createData(QOpenGLContext * p);*/
	void createFragment();
	win(QWidget *parent);
	~win();
protected:
	void initializeGL()  Q_DECL_OVERRIDE;
	void resizeGL(int w, int h) Q_DECL_OVERRIDE;
	void paintGL() Q_DECL_OVERRIDE;
	void keyPressEvent(QKeyEvent *event) Q_DECL_OVERRIDE;
	void keyReleaseEvent(QKeyEvent *event) Q_DECL_OVERRIDE;
	void mousePressEvent(QMouseEvent *event) Q_DECL_OVERRIDE;
	void mouseReleaseEvent(QMouseEvent *event) Q_DECL_OVERRIDE;
	void mouseMoveEvent(QMouseEvent *event) Q_DECL_OVERRIDE;
	void wheelEvent(QWheelEvent *event) Q_DECL_OVERRIDE;
private:
	bool createShader();
	//设置纹理对象，暂时没用
	uint loadTexture(const QString& path);
	//着色器程序
	QOpenGLShaderProgram triangleShader, coorShader;
	//定时器进行渲染
	QTimer* m_pTimer = nullptr;
	int     m_nTimeValue = 0;
	//坐标轴的顶点数组和索引数组
	uint VAO, VBO;
	//纹理，暂时没啥用
	uint diffuseMap, specularMap;
	// 照相机
	std::unique_ptr<Camera> camera;
	bool m_bLeftPressed;
	QPoint m_lastPos;

	QVector3D ro;
};
