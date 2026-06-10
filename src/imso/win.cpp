#include "win.h"
#include <QDebug>
#include <QTimer>
#include <QKeyEvent>
#include <QDateTime>
#include "MyDoc.h"
#include "Fragment.h"
#include "qapplication.h"

win::win(QWidget *parent) : QOpenGLWidget(parent)
{
	MyDoc::Ptr pdoc = MyDoc::getInstance();
	//setGeometry(0, 0, 800, 800);
	camera = std::make_unique<Camera>(QVector3D(pdoc->cameraX, pdoc->cameraY, pdoc->cameraZ));//照相机的初始位置
	m_bLeftPressed = false;
	QSurfaceFormat surfaceFormat;
	surfaceFormat.setSamples(6);//多重采样
	setFormat(surfaceFormat); //setFormat是QOpenGLWidget的函数
	m_pTimer = new QTimer(this);
	connect(m_pTimer, &QTimer::timeout, this, [=] {
		m_nTimeValue += 1;
		update();
	});
	m_pTimer->start(10);//这里是设置fps，数值越高fps越低
}

win::~win()
{
	glDeleteVertexArrays(1, &VAO);
	glDeleteBuffers(1, &VBO);
}

void win::initializeGL() {

	this->initializeOpenGLFunctions();
	createShader();

	//先把坐标系设置好
	//----------
	float X[] = {

		0.0f,0.0f,0.0f,  1.0f,0.0f,0.0f,
		1000.0f,0.0f,0.0f,  1.0f,0.0f,0.0f,

		0.0f,0.0f,0.0f,  0.0f,1.0f,0.0f,
		0.0f,1000.0f,0.0f,  0.0f,1.0f,0.0f,

		0.0f,0.0f,0.0f,  0.0f,0.0f,1.0f,
		0.0f,0.0f,1000.0f,  0.0f,0.0f,1.0f

	};
	//setGeometry(300, 300, 800, 800);

	//坐标轴的顶点数组对象
	//---------
	glGenVertexArrays(1, &VAO);
	glGenBuffers(1, &VBO);
	glBindVertexArray(VAO);
	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(X), X, GL_STATIC_DRAW);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), (GLvoid*)0);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), (GLvoid*)(3 * sizeof(float)));
	glEnableVertexAttribArray(1);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);

	
	// 设置OpenGL的全局状态
	// --------
	glEnable(GL_DEPTH_TEST);//开启深度缓冲
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

void win::resizeGL(int w, int h) {
	glViewport(0,0 ,w, h);
}

void win::paintGL() {

	//存储文件的获取
	// -----
	MyDoc::Ptr pdoc = MyDoc::getInstance();
	// 设定每次键盘按下之后的速度
	// -----
	camera->processInput(pdoc->_keyBordSensitive);//速度
	camera->setMouseSpeed(pdoc->_mouseSenstive);
	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	//重新设置摄像机位置
	//----------
	if (pdoc->m_restView == true)
	{
		camera->setLocation(QVector3D(pdoc->cameraX, pdoc->cameraY, pdoc->cameraZ));
		pdoc->m_restView = false;
	}

	//显示模式选择
	//----------
	switch (pdoc->m_GLMode)//选择显示的模式，线，面，点
	{
	case 0:
		glPointSize(pdoc->m_dotSZ);
		glPolygonMode(GL_FRONT_AND_BACK, GL_POINT);
		break;
	case 3:
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		break;
	case 7:
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		break;
	default:
		break;
	}

	//画出来三角形
	//----------
	triangleShader.bind();
	QMatrix4x4 projection;
	projection.perspective(camera->zoom, 1.0f * width() / height(), 0.1f, 2000.0f);
	triangleShader.setUniformValue("projection", projection);
	QMatrix4x4 view;
	view = camera->getViewMatrix();
	triangleShader.setUniformValue("view", view);
	QMatrix4x4 model;
	//按下ctrl键之后进行物体的旋转
	model.rotate(-ro.x(), 1, 0, 0);
	model.rotate(ro.y(), 0, 1, 0);
	model.rotate(ro.z(), 0, 0, 1);
	//旋转结束
	triangleShader.setUniformValue("model", model);
	triangleShader.setUniformValue("ourColor", pdoc->_surfaceColor);

	//是否需要去分片显示
	//----------
	vector<QVector3D> n = { {0.3,0.5,0.5},{0.5,1.0,0.5},{0.5,0.5,1.0},{0.5,1.0,1.0},
	{1.0,0.5,1.0}, {0.5,0.8,0.5} };
	if (pdoc->m_CellIdx <= -1 && pdoc->m_Fragment == false)//不要分片显示
	{
		for (int i = 0; i != pdoc->VAO_D_ARRAY.size(); ++i)
		{
			triangleShader.setUniformValue("ourColor", n[i%6]);
			glBindVertexArray(pdoc->VAO_D_ARRAY[i]);
			glDrawElements(GL_TRIANGLES, pdoc->EACH_VAO_D_POINT_NUMBER[i], GL_UNSIGNED_INT, 0);
		}
	}
	else if (pdoc->m_CellIdx >= 0)//要分片显示
	{
		for (int i = pdoc->m_CellIdx * pdoc->m_NUMPic; i != pdoc->m_CellIdx * pdoc->m_NUMPic + pdoc->m_NUMPic; ++i)
		{
			glBindVertexArray(pdoc->VAO_D_ARRAY[i]);
			glDrawElements(GL_TRIANGLES, pdoc->EACH_VAO_D_POINT_NUMBER[i], GL_UNSIGNED_INT, 0);
		}
	}

	//画轮廓线
	//----------
	triangleShader.setUniformValue("ourColor", pdoc->_bordLineColor);
	glLineWidth(pdoc->_lineWidth);
	for (int i = 0; i != pdoc->VAO_L_ARRAY.size(); ++i)
	{
		glBindVertexArray(pdoc->VAO_L_ARRAY[i]);
		glDrawArrays(GL_LINE_STRIP, 0, pdoc->EACH_VAO_L_POINT_NUMBER[i]);
	}

	//画控制点
	//----------
	if (pdoc->m_showCtrlPts == true)//如果可以显示控制点，我们就显示控制点
	{
		triangleShader.setUniformValue("ourColor", pdoc->_controlPoinrColor);
		glPointSize(pdoc->m_CPTSdotSZ);
		if (pdoc->m_CellIdx == -1)//显示全部的控制点
		{
			for (int i = 0; i != pdoc->VAO_CP_ARRAY.size(); ++i)
			{
				glBindVertexArray(pdoc->VAO_CP_ARRAY[i]);
				glDrawArrays(GL_POINTS, 0, pdoc->EACH_VAO_CP_POINT_NUMBER[i]);
			}
		}
		else//显示选中体模型的控制点
		{
			for (int i = pdoc->m_CellIdx /** 6*/; i != pdoc->m_CellIdx+1 /** 6 + 6*/; ++i)
			{
				glBindVertexArray(pdoc->VAO_CP_ARRAY[i]);
				glDrawArrays(GL_POINTS, 0, pdoc->EACH_VAO_CP_POINT_NUMBER[i]);
			}
		}

		
	}

	//如果需要切片
	//----------
	if (pdoc->m_Fragment == true)
	{
		glPointSize(10);
		triangleShader.setUniformValue("ourColor", pdoc->_sliceSurfaceColor);
		
		for (int i = 0; i != pdoc->VAO_FRAGMENT_ARRAY.size(); ++i)
		{
			glBindVertexArray(pdoc->VAO_FRAGMENT_ARRAY[i]);
			glDrawElements(GL_TRIANGLES, pdoc->EACH_VAO_F_NUMBER[i], GL_UNSIGNED_INT, 0);
		}

		triangleShader.setUniformValue("ourColor", pdoc->_sliceSuppotrColor);
		for (int i = 0; i != pdoc->VAO_SUPPORT_ARRAY.size(); ++i)
		{
			glBindVertexArray(pdoc->VAO_SUPPORT_ARRAY[i]);
			glDrawArrays(GL_LINES, 0, pdoc->EACH_VAO_SUPPORT_NUMBER[i]);
		}
	}
	triangleShader.release();

	//画坐标系
	//----------
	coorShader.bind();
	if (pdoc->m_showCoordinates == true)
	{
		coorShader.setUniformValue("projection", projection);
		coorShader.setUniformValue("view", view);
		QMatrix4x4 model1;
		coorShader.setUniformValue("model", model1);
		glLineWidth(1.0f);
		glBindVertexArray(VAO);
		glDrawArrays(GL_LINE_STRIP, 0, 6);
		glDrawArrays(GL_POINTS, 0, 100000);

		//借用坐标轴的着色器进行非均质模型的渲染
		/*glPointSize(10.0f);
		for (int i = 0; i != pdoc->VAO_D.size(); ++i)
		{
			glBindVertexArray(pdoc->VAO_D[i]);
			glDrawElements(GL_TRIANGLES, pdoc->VAO_D_SIZE[i], GL_UNSIGNED_INT, 0);
		}*/
	}
	coorShader.release();
}

void win::keyPressEvent(QKeyEvent *event)
{
	int key = event->key();
	if (key >= 0 && key < 1024)
		camera->keys[key] = true;
}

void win::keyReleaseEvent(QKeyEvent *event)
{
	int key = event->key();
	if (key >= 0 && key < 1024)
		camera->keys[key] = false;
}

void win::mousePressEvent(QMouseEvent *event)
{
	if (event->button() == Qt::LeftButton) {
		m_bLeftPressed = true;
		m_lastPos = event->pos();
	}
}

void win::mouseReleaseEvent(QMouseEvent *event)
{
	Q_UNUSED(event);

	m_bLeftPressed = false;
}

void win::mouseMoveEvent(QMouseEvent *event)
{
	int xpos = event->pos().x();
	int ypos = event->pos().y();

	int xoffset = xpos - m_lastPos.x();
	int yoffset = m_lastPos.y() - ypos;
	m_lastPos = event->pos();

	if ((QApplication::keyboardModifiers() == Qt::ControlModifier))
	{
		float a = ro.x() + yoffset / 10.0;
		ro.setX(a);
		if (ro.x() > 360.0 || ro.x() < -360.0)
			ro.setX(0.0);
		float b = ro.y() + xoffset / 10.0;
		ro.setY(b);
		if (ro.y() > 360.0 || ro.y() < -360.0)
			ro.setY(0.0);
	}
	else
	{
		camera->processMouseMovement(xoffset, yoffset);
	}
}

void win::wheelEvent(QWheelEvent *event)
{
	QPoint offset = event->angleDelta();
	camera->processMouseScroll(offset.y() / 20.0f);
}

bool win::createShader()
{
	
	bool success = triangleShader.addShaderFromSourceFile(QOpenGLShader::Vertex, ":/triangle.vert");
	if (!success) {
		qDebug() << "shaderProgram addShaderFromSourceFile failed!" << triangleShader.log();
		return success;
	}

	success = triangleShader.addShaderFromSourceFile(QOpenGLShader::Fragment, ":/triangle.frag");
	if (!success) {
		qDebug() << "shaderProgram addShaderFromSourceFile failed!" << triangleShader.log();
		return success;
	}

	success = triangleShader.link();
	if (!success) {
		qDebug() << "shaderProgram link failed!" << triangleShader.log();
	}

	success = coorShader.addShaderFromSourceFile(QOpenGLShader::Vertex, ":/coor.vert");
	if (!success) {
		qDebug() << "shaderProgram addShaderFromSourceFile failed!" << coorShader.log();
		return success;
	}

	success = coorShader.addShaderFromSourceFile(QOpenGLShader::Fragment, ":/coor.frag");
	if (!success) {
		qDebug() << "shaderProgram addShaderFromSourceFile failed!" << coorShader.log();
		return success;
	}

	success = coorShader.link();
	if (!success) {
		qDebug() << "shaderProgram link failed!" << coorShader.log();
	}

	return success;
}

uint win::loadTexture(const QString& path)
{
	uint textureID;
	glGenTextures(1, &textureID);

	QImage image = QImage(path).convertToFormat(QImage::Format_RGBA8888).mirrored(true, true);
	if (!image.isNull()) {
		glBindTexture(GL_TEXTURE_2D, textureID);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, image.width(), image.height(), 0, GL_RGBA, GL_UNSIGNED_BYTE, image.bits());
		glGenerateMipmap(GL_TEXTURE_2D);

		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	}

	return textureID;
}

//void win::createData(QOpenGLContext * p)
//{
//	//设置当前的渲染环境
//	QSurface *a = p->surface();
//	p->makeCurrent(a);
//
//	MyDoc::Ptr pdoc = MyDoc::getInstance();
//	pdoc->m_CellIdx = -1;
//
//	std::vector<unsigned int> indices;
//	std::vector<unsigned int> base = { 0,1,2,2,3,0 };
//
//	//这里我门创造一个一维数组，保存原来的数据
//	//画面的数据保存
//	for (int i = 0; i != pdoc->m_ShowData.size(); ++i)
//	{
//		int pointsize = 0;
//		for (int j = 0; j != pdoc->m_ShowData[i].size(); ++j)
//		{
//			for (int k = 0; k != pdoc->m_ShowData[i][j].size(); ++k)
//			{
//				pdoc->showDta_vertices.push_back(pdoc->m_ShowData[i][j][k].x);
//				pdoc->showDta_vertices.push_back(pdoc->m_ShowData[i][j][k].y);
//				pdoc->showDta_vertices.push_back(pdoc->m_ShowData[i][j][k].z);
//			}
//			for (int k = 0; k != base.size(); ++k)
//			{
//				indices.push_back(base[k]);
//				base[k] += 4;
//			}
//			pointsize += 6;
//		}
//
//		uint VAO, VBO, EBO;
//		glGenVertexArrays(1, &VAO);//顶点数组对象
//		glGenBuffers(1, &VBO);//顶点数组
//		glGenBuffers(1, &EBO);//索引数组
//		glBindVertexArray(VAO);//开始绑定
//		glBindBuffer(GL_ARRAY_BUFFER, VBO);
//		glBufferData(GL_ARRAY_BUFFER, pdoc->showDta_vertices.size() * sizeof(float), pdoc->showDta_vertices.data(), GL_STATIC_DRAW);
//		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
//		glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(float)*indices.size(), indices.data(), GL_STATIC_DRAW);
//		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), (GLvoid *)0);
//		glEnableVertexAttribArray(0);
//		glBindBuffer(GL_ARRAY_BUFFER, 0);
//		glBindVertexArray(0);
//		pdoc->EACH_VAO_D_POINT_NUMBER.push_back(pointsize);
//		pdoc->VAO_D_ARRAY.push_back(VAO);
//		pdoc->showDta_vertices.clear();
//		indices.clear();
//		base = { 0,1,2,2,3,0 };
//	}
//
//	//保存线的数据，线不需要索引数组，线因为特殊需要一组一组的分别画出来
//	for (int i = 0; i != pdoc->m_ShowLines.size(); ++i)
//	{
//		for (int j = 0; j != pdoc->m_ShowLines[i].size(); ++j)
//		{
//			int pointSize = 0;//计算这一组线段里面有多少个点
//			for (int k = 0; k != pdoc->m_ShowLines[i][j].size(); ++k)
//			{
//				pdoc->showLines_vertices.push_back(pdoc->m_ShowLines[i][j][k].x);
//				pdoc->showLines_vertices.push_back(pdoc->m_ShowLines[i][j][k].y);
//				pdoc->showLines_vertices.push_back(pdoc->m_ShowLines[i][j][k].z);
//				pointSize++;
//			}
//			unsigned int VBO, VAO;
//			glGenVertexArrays(1, &VAO);
//			glGenBuffers(1, &VBO);
//			glBindVertexArray(VAO);
//			glBindBuffer(GL_ARRAY_BUFFER, VBO);
//			glBufferData(GL_ARRAY_BUFFER, pdoc->showLines_vertices.size() * sizeof(float), pdoc->showLines_vertices.data(), GL_STATIC_DRAW);
//			glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), (GLvoid *)0);
//			glEnableVertexAttribArray(0);
//			glBindBuffer(GL_ARRAY_BUFFER, 0);
//			glBindVertexArray(0);
//			pdoc->VAO_L_ARRAY.push_back(VAO);//把这一组的顶点缓冲对象记录下来
//			pdoc->EACH_VAO_L_POINT_NUMBER.push_back(pointSize);//把这一组有多少个点记录下来
//			pdoc->showLines_vertices.clear();//数组清空，方便下行下一个循环
//		}
//	}
//	//控制点的数据
//	for (int i = 0; i != pdoc->m_ShowData4D.size(); ++i)
//	{
//		int pointsize = 0;
//		for (int j = 0; j != pdoc->m_ShowData4D[i].size(); ++j)
//		{
//			pdoc->showCp_vertices.push_back(pdoc->m_ShowData4D[i][j].x);
//			pdoc->showCp_vertices.push_back(pdoc->m_ShowData4D[i][j].y);
//			pdoc->showCp_vertices.push_back(pdoc->m_ShowData4D[i][j].z);
//			pointsize++;
//		}
//		unsigned int VAO, VBO;
//		glGenVertexArrays(1, &VAO);
//		glGenBuffers(1, &VBO);
//		glBindVertexArray(VAO);
//		glBindBuffer(GL_ARRAY_BUFFER, VBO);
//		glBufferData(GL_ARRAY_BUFFER, pdoc->showCp_vertices.size() * sizeof(float), pdoc->showCp_vertices.data(), GL_STATIC_DRAW);
//		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), (GLvoid *)0);
//		glEnableVertexAttribArray(0);
//		glBindBuffer(GL_ARRAY_BUFFER, 0);
//		glBindVertexArray(0);
//		pdoc->EACH_VAO_CP_POINT_NUMBER.push_back(pointsize);
//		pdoc->VAO_CP_ARRAY.push_back(VAO);
//		pdoc->showCp_vertices.clear();
//	}
//}

void win::createFragment()
{
	makeCurrent();//设定当前的渲染场景

	MyDoc::Ptr pdoc = MyDoc::getInstance();
	pdoc->m_Fragment = !pdoc->m_Fragment;

	//这里做出来一个切片类，平行于xoy面，从begin开始到end结束，中间切分degree个平面
	Fragment f(pdoc->m_fragmentA, pdoc->m_fragmentB, pdoc->m_fragmentC, pdoc->m_fragmentBegin, pdoc->m_fragmentEnd, pdoc->m_fragmentDegree);

	vector<vector<float>> temp;
	temp = f.getPoints(pdoc->m_ShowData);//接受返回的顶点数组，分别是每个切面的所有点
	for (int i = 0; i != temp.size(); ++i)//我们把每个切面分开显示
	{
		vector<int> indices;//创造出来一个顶点数组，方便画图
		vector<int> Indices = { 0,1,2 };
		for (int j = 0; j != temp[i].size() / 3 - 2; ++j)
		{
			indices.insert(indices.end(), Indices.begin(), Indices.end());
			Indices[1] += 1;
			Indices[2] += 1;
		}
		unsigned int VAO, VBO, EBO;
		glGenVertexArrays(1, &VAO);//顶点数组对象
		glGenBuffers(1, &VBO);//顶点数组
		glGenBuffers(1, &EBO);//索引数组
		glBindVertexArray(VAO);//开始绑定
		glBindBuffer(GL_ARRAY_BUFFER, VBO);
		glBufferData(GL_ARRAY_BUFFER, temp[i].size() * sizeof(float), temp[i].data(), GL_STATIC_DRAW);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(float)*indices.size(), indices.data(), GL_STATIC_DRAW);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), (GLvoid *)0);
		glEnableVertexAttribArray(0);
		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindVertexArray(0);
		pdoc->VAO_FRAGMENT_ARRAY.push_back(VAO);
		pdoc->EACH_VAO_F_NUMBER.push_back(temp[i].size());
	}

	vector<float> aa = f.getSupportPoints();
	unsigned int VAO, VBO, EBO;
	glGenVertexArrays(1, &VAO);//顶点数组对象
	glGenBuffers(1, &VBO);//顶点数组
	glBindVertexArray(VAO);//开始绑定
	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, aa.size() * sizeof(float), aa.data(), GL_STATIC_DRAW);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), (GLvoid *)0);
	glEnableVertexAttribArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
	pdoc->VAO_SUPPORT_ARRAY.push_back(VAO);
	pdoc->EACH_VAO_SUPPORT_NUMBER.push_back(aa.size() / 3);
}