#include "Option.h"
#include "MyDoc.h"
#include "qprogressdialog.h"

Option::Option()
{
}

Option::~Option()
{
}

//Cnurbs数据结构的显示函数
void Option::OnBnClickedCreateModel(QOpenGLContext * p,QWidget * father)
{
	MyDoc::Ptr pdoc = MyDoc::getInstance();
	
	int nIndex = pdoc->modelType;
	RWGeometric rwg;
	string path = string((const char *)pdoc->path.toLocal8Bit());
	varray<varray<point3d>> p3d2, l2;
	pdoc->m_ShowData.clear();
	pdoc->m_ShowLines.clear();

	switch (nIndex)
	{
	case 0:
	{
		rwg.ReadPoint(path, p3d2);
		for (int i = 0; i < p3d2.size(); ++i)
		{
			l2.clear();
			l2.push_back(p3d2[i]);
			pdoc->m_ShowData.push_back(l2);
		}
		pdoc->m_tolNum = pdoc->m_ShowData.size() / pdoc->m_NUMPic;
		if (pdoc->m_tolNum*pdoc->m_NUMPic < pdoc->m_ShowData.size())
			++pdoc->m_tolNum;
		break;
	}
	case 1:
	{
		varray<varray<point4d>> p4d2;
		rwg.ReadPoint(path, p4d2);
		for (int i = 0; i < p4d2.size(); ++i)
		{
			varray<point3d> l3d;
			for (int j = 0; j < p4d2[i].size(); ++j)
				l3d.push_back(p4d2[i][j]);
			l2.clear();
			l2.push_back(l3d);
			pdoc->m_ShowData.push_back(l2);
		}
		pdoc->m_tolNum = pdoc->m_ShowData.size() / pdoc->m_NUMPic;
		if (pdoc->m_tolNum*pdoc->m_NUMPic < pdoc->m_ShowData.size())
			++pdoc->m_tolNum;
		break;
	}
	case 2:
	{
		varray<BezierLine> bzls;
		rwg.ReadBezierLine(path, bzls);
		int size = 0;
		if (m_again != 0)//如果在运行过程中发生了拖动点，我们把拖动后的点作为最新的控制点
		{
			for (int i = 0; i != pdoc->m_ShowData4D.size(); ++i)
			{
				bzls[i].m_CtrlPts = pdoc->m_ShowData4D[i];
			}
		}
		for (int i = 0; i < bzls.size(); ++i)
		{
			varray<point3d> l3d;
			bzls[i].CalLinePoint(pdoc->m_Unum, l3d);
			l2.clear();
			l2.push_back(l3d);
			pdoc->m_ShowData.push_back(l2);
			if (m_again == 0)
			{
				pdoc->m_ShowData4D.push_back(bzls[i].m_CtrlPts);
			}
			createData(p, pdoc->m_ShowData, pdoc->m_ShowLines);
			size += pdoc->m_ShowData.size();
			pdoc->cleanUp();
		}
		pdoc->m_tolNum = size / pdoc->m_NUMPic;
		if (pdoc->m_tolNum*pdoc->m_NUMPic < size)
			++pdoc->m_tolNum;
		break;
	}
	case 3:
	{
		varray<NurbsLine> nls;
		rwg.ReadNurbsLine(path, nls);
		int size = 0;
		if (m_again != 0)//如果在运行过程中发生了拖动点，我们把拖动后的点作为最新的控制点
		{
			for (int i = 0; i != pdoc->m_ShowData4D.size(); ++i)
			{
				nls[i].m_CtrlPts = pdoc->m_ShowData4D[i];
			}
		}
		for (int i = 0; i < nls.size(); ++i)
		{
			varray<point3d> l3d;
			nls[i].CalLinePoint(pdoc->m_Unum, l3d);
			l2.clear();
			l2.push_back(l3d);
			pdoc->m_ShowData.push_back(l2);
			if (m_again == 0)
			{
				pdoc->m_ShowData4D.push_back(nls[i].m_CtrlPts);
			}
			createData(p, pdoc->m_ShowData, pdoc->m_ShowLines);
			size += pdoc->m_ShowData.size();
			pdoc->cleanUp();
		}
		pdoc->m_tolNum = size / pdoc->m_NUMPic;
		if (pdoc->m_tolNum*pdoc->m_NUMPic < size)
			++pdoc->m_tolNum;
		break;
	}
	case 4:
	{
		varray<NurbsSurface> nsfs;
		rwg.ReadNurbsSurface(path, nsfs);
		int size = 0;
		if (m_again != 0)//如果在运行过程中发生了拖动点，我们把拖动后的点作为最新的控制点
		{
			for (int i = 0; i != pdoc->m_ShowData4D.size(); ++i)
			{
				nsfs[i].m_CtrlPts = pdoc->m_ShowData4D[i];
			}
		}
		else
		{
			for (int i = 0; i < nsfs.size(); ++i)
			{
				pdoc->m_ShowData4D.push_back(nsfs[i].m_CtrlPts);
			}
		}
		///////////////多线程计算
		vector<future<threadParam>> v;
		int count = 0;//多线程的计数，我们设定一次创造的线程最多不能多余7个，太多了会造成上下文切换，浪费
		for (int i = 0; i < nsfs.size(); i++)
		{
			count++;//创造一个线程
			v.push_back(async(&NurbsSurface::CalQuads, &nsfs[i], pdoc->m_Unum, pdoc->m_Vnum));
			if (count == pdoc->THREADNUMBERS || i == nsfs.size() - 1)//当线程数量到7的时候我们就分别等线程结束，然后回收
			{
				for (int j = 0; j < count; j++)//开始回收七个线程里面的数据
				{
					v[j].wait();
					threadParam p = v[j].get();
					pdoc->m_ShowData.push_back(p.l2);
					pdoc->m_ShowLines.push_back(p.p3d2);
				}
				count = 0;//清空计数
				v.clear();//清空列表，等待下一轮的7个线程

				createData(p, pdoc->m_ShowData, pdoc->m_ShowLines);
				size += pdoc->m_ShowData.size();
				pdoc->cleanUp();
			}
		}
		////////多线程计算结束
		pdoc->m_tolNum = size / pdoc->m_NUMPic;
		if (pdoc->m_tolNum*pdoc->m_NUMPic < size)
			++pdoc->m_tolNum;
		break;
	}
	case 5:
	{
		varray<NurbsVol> nvs;
		rwg.ReadNurbsVol(path, nvs);
		if (m_again != 0)//如果在运行过程中发生了拖动点，我们把拖动后的点作为最新的控制点
		{
			for (int i = 0; i != pdoc->m_ShowData4D.size(); ++i)
			{
				nvs[i].m_CtrlPts = pdoc->m_ShowData4D[i];
			}
		}
		else
		{
			
			for (int i = 0; i < nvs.size(); ++i)
			{
				pdoc->m_ShowData4D.push_back(nvs[i].m_CtrlPts);
			}
		}
		//////////////多线程计算，修改于2020-2-8
		vector<future<threadParamVOL>> v;
		int count = 0;//多线程的计数，我们设定一次创造的线程最多不能多余7个，太多了会造成上下文切换，浪费
		int size = 0;
		for (int i = 0; i < nvs.size(); i++)
		{
			count++;//创造一个线程
			varray<varray<varray<point3d>>>  q3, l3;
			v.push_back(async(&NurbsVol::CalQuads, &nvs[i], pdoc->m_Unum, pdoc->m_Vnum, pdoc->m_Wnum));
			if (count == pdoc->THREADNUMBERS || i == nvs.size() - 1)//当线程数量到7的时候我们就分别等线程结束，然后回收
			{
				for (int j = 0; j < count; j++)//开始回收七个线程里面的数据
				{
					v[j].wait();
					threadParamVOL p = v[j].get();
					for (int k = 0; k < p.q3.size(); ++k)
						pdoc->m_ShowData.push_back(p.q3[k]);
					for (int k = 0; k < p.l3.size(); ++k)
						pdoc->m_ShowLines.push_back(p.l3[k]);
				}
				count = 0;//清空计数
				v.clear();//清空列表，等待下一轮的7个线程

				//修改于2020-05-14，删除掉中间的文件存储，直接进行显示
				createData(p,pdoc->m_ShowData, pdoc->m_ShowLines);
				size += pdoc->m_ShowData.size();
				pdoc->cleanUp();
			}
		}

		pdoc->m_tolNum = size / pdoc->m_NUMPic;
		if (pdoc->m_tolNum*pdoc->m_NUMPic < size)
			++pdoc->m_tolNum;
		break;
	}
	default:
		break;
	}
}

//CPolyParaVolume数据结构转换为Cnurbs再显示函数
void Option::OnBnClickedCreateModel(QOpenGLContext * p, QWidget * father, CPolyParaVolume polyv)
{
	MyDoc::Ptr pdoc = MyDoc::getInstance();

	int nIndex = pdoc->modelType;
	RWGeometric rwg;
	string path = string((const char *)pdoc->path.toLocal8Bit());
	varray<varray<point3d>> p3d2, l2;
	pdoc->m_ShowData.clear();
	pdoc->m_ShowLines.clear();

	//下面的代码是从CPolyParaVolume转为NurbsVol
	varray<NurbsVol> nvs;
	TransCnurbsvolToPolyIGA(nvs, polyv);//转换为nurbsvol，然后再进行显示
	if (m_again != 0)//如果在运行过程中发生了拖动点，我们把拖动后的点作为最新的控制点
	{
		for (int i = 0; i != pdoc->m_ShowData4D.size(); ++i)
		{
			nvs[i].m_CtrlPts = pdoc->m_ShowData4D[i];
		}
	}
	else
	{
		for (int i = 0; i < nvs.size(); ++i)
		{
			pdoc->m_ShowData4D.push_back(nvs[i].m_CtrlPts);
		}
	}
	//////////////多线程计算，修改于2020-2-8
	vector<future<threadParamVOL>> v;
	int count = 0;//多线程的计数，我们设定一次创造的线程最多不能多余7个，太多了会造成上下文切换，浪费
	int size = 0;
	for (int i = 0; i < nvs.size(); i++)
	{
		count++;//创造一个线程
		varray<varray<varray<point3d>>>  q3, l3;
		v.push_back(async(&NurbsVol::CalQuads, &nvs[i], pdoc->m_Unum, pdoc->m_Vnum, pdoc->m_Wnum));
		if (count == pdoc->THREADNUMBERS || i == nvs.size() - 1)//当线程数量到7的时候我们就分别等线程结束，然后回收
		{
			for (int j = 0; j < count; j++)//开始回收七个线程里面的数据
			{
				v[j].wait();
				threadParamVOL p = v[j].get();
				for (int k = 0; k < p.q3.size(); ++k)
					pdoc->m_ShowData.push_back(p.q3[k]);
				for (int k = 0; k < p.l3.size(); ++k)
					pdoc->m_ShowLines.push_back(p.l3[k]);
			}
			count = 0;//清空计数
			v.clear();//清空列表，等待下一轮的7个线程

			//修改于2020-05-14，删除掉中间的文件存储，直接进行显示
			createData(p, pdoc->m_ShowData, pdoc->m_ShowLines);
			size += pdoc->m_ShowData.size();
			pdoc->cleanUp();
		}
	}
	pdoc->m_tolNum = size / pdoc->m_NUMPic;
	if (pdoc->m_tolNum*pdoc->m_NUMPic < size)
		++pdoc->m_tolNum;
}

//CPolyParaVolume数据结构显示函数
void Option::createAllData(QOpenGLContext * p, QWidget * father, CPolyParaVolume polyv)
{
	MyDoc::Ptr pdoc = MyDoc::getInstance();

	int nIndex = pdoc->modelType;
	RWGeometric rwg;
	string path = string((const char *)pdoc->path.toLocal8Bit());
	varray<varray<point3d>> p3d2, l2;
	pdoc->m_ShowData.clear();
	pdoc->m_ShowLines.clear();

	//根据CPolyParaVolume 计算模型的内部渲染点
	vector<float> data;
	for (int i = 0; i != polyv.m_HexVolumes.size(); ++i)
	{
		//给模型控制点一个初始的材料颜色
		for (int j = 0; j != polyv.m_HexVolumes[i].m_splVol.m_vNum*polyv.m_HexVolumes[i].m_splVol.m_wNum; ++j)
		{
			int size = polyv.m_HexVolumes[i].m_splVol.m_uNum;
			if (size % 2 == 0)
			{
				for (int k = 0; k != size / 2; ++k)
				{
					polyv.m_HexVolumes[i].m_splVol.m_vAllCtrlPts[k + j * size].m_matval.x = k * 1.0 / (size / 2);
					polyv.m_HexVolumes[i].m_splVol.m_vAllCtrlPts[k + j * size].m_matval.y = k * 1.0 / (size / 2);
					polyv.m_HexVolumes[i].m_splVol.m_vAllCtrlPts[k + j * size].m_matval.z = k * 1.0 / (size / 2);
					polyv.m_HexVolumes[i].m_splVol.m_vAllCtrlPts[size - k - 1 + j * size].m_matval.x = k * 1.0 / (size / 2);
					polyv.m_HexVolumes[i].m_splVol.m_vAllCtrlPts[size - k - 1 + j * size].m_matval.y = k * 1.0 / (size / 2);
					polyv.m_HexVolumes[i].m_splVol.m_vAllCtrlPts[size - k - 1 + j * size].m_matval.z = k * 1.0 / (size / 2);
				}
			}
			else
			{
				for (int k = 0; k != size / 2; ++k)
				{
					polyv.m_HexVolumes[i].m_splVol.m_vAllCtrlPts[k + j * size].m_matval.x = k * 1.0 / (size / 2);
					polyv.m_HexVolumes[i].m_splVol.m_vAllCtrlPts[k + j * size].m_matval.y = k * 1.0 / (size / 2);
					polyv.m_HexVolumes[i].m_splVol.m_vAllCtrlPts[k + j * size].m_matval.z = k * 1.0 / (size / 2);
					polyv.m_HexVolumes[i].m_splVol.m_vAllCtrlPts[size - k - 1 + j * size].m_matval.x = k * 1.0 / (size / 2);
					polyv.m_HexVolumes[i].m_splVol.m_vAllCtrlPts[size - k - 1 + j * size].m_matval.y = k * 1.0 / (size / 2);
					polyv.m_HexVolumes[i].m_splVol.m_vAllCtrlPts[size - k - 1 + j * size].m_matval.z = k * 1.0 / (size / 2);
				}
				polyv.m_HexVolumes[i].m_splVol.m_vAllCtrlPts[size / 2 + j * size].m_matval.x = 1.0;
				polyv.m_HexVolumes[i].m_splVol.m_vAllCtrlPts[size / 2 + j * size].m_matval.y = 1.0;
				polyv.m_HexVolumes[i].m_splVol.m_vAllCtrlPts[size / 2 + j * size].m_matval.z = 1.0;
			}
		}
		//另外一种给控制点颜色的方法
		/*for (int j = 0; j != polyv.m_HexVolumes[i].m_splVol.m_vAllCtrlPts.size(); ++j)
		{
			polyv.m_HexVolumes[i].m_splVol.m_vAllCtrlPts[j].m_matval.x = (j *1.0/ polyv.m_HexVolumes[i].m_splVol.m_vAllCtrlPts.size());
			polyv.m_HexVolumes[i].m_splVol.m_vAllCtrlPts[j].m_matval.y = j *1.0/ polyv.m_HexVolumes[i].m_splVol.m_vAllCtrlPts.size();
			polyv.m_HexVolumes[i].m_splVol.m_vAllCtrlPts[j].m_matval.z = j *1.0/ polyv.m_HexVolumes[i].m_splVol.m_vAllCtrlPts.size();
		}*/
		//获取模型的内部点，以及内部点的颜色 
		polyv.m_HexVolumes[i].m_splVol.CreateAllVolumeRenderPts(pdoc->m_Unum, pdoc->m_Vnum, pdoc->m_Wnum);
		//把计算好的数据传入到一个一维数组之中
		for (int j = 0; j != polyv.m_HexVolumes[i].m_splVol.m_vVolumePts.size(); ++j)
		{
			data.push_back(polyv.m_HexVolumes[i].m_splVol.m_vVolumePts[j].m_pt.x);
			data.push_back(polyv.m_HexVolumes[i].m_splVol.m_vVolumePts[j].m_pt.y);
			data.push_back(polyv.m_HexVolumes[i].m_splVol.m_vVolumePts[j].m_pt.z);
			data.push_back(polyv.m_HexVolumes[i].m_splVol.m_vVolumePts[j].m_matval.x);
			data.push_back(polyv.m_HexVolumes[i].m_splVol.m_vVolumePts[j].m_matval.y);
			data.push_back(polyv.m_HexVolumes[i].m_splVol.m_vVolumePts[j].m_matval.z);
		}
		createData(p, data);//把一个体模型拿去渲染
		data.clear();
	}
}

//void Option::SaveSetting()
//{
//	MyDoc::Ptr pdoc = MyDoc::getInstance();
//	using namespace std;
//	std::ofstream outf;
//	outf.open("Setting", ios::binary);
//	outf << "<path>" << "\n\r" << endl;
//	outf << pdoc->path.toStdString() << "\n\r" << endl;
//	outf << "<para>" << "\n\r" << endl;
//	outf << "Unum=" << pdoc->m_Unum << "\n\r" << endl;
//	outf << "Vnum=" << pdoc->m_Vnum << "\n\r" << endl;
//	outf << "Wnum=" << pdoc->m_Wnum << "\n\r" << endl;
//	outf << "Type=" << pdoc->modelType << "\n\r" << endl;
//	outf << "dotSZ=" << pdoc->m_dotSZ << "\n\r" << endl;
//	outf << "cptsDotSZ=" << pdoc->m_CPTSdotSZ << "\n\r" << endl;
//	outf << "NUMPic=" << pdoc->m_NUMPic << "\n\r" << endl;
//	outf << "<end>" << endl;
//	outf.close();
//}
//
//
//void Option::ReflashPara()
//{
//	using namespace std;
//	ifstream inf;
//	inf.open("setting", ios::binary);
//	string temp = "", path = "", knots_str = "";
//	varray<double> para;
//	int flag = 0;
//	while (!inf.eof())
//	{
//		inf >> temp;
//		//判断标识
//		if (temp == "<path>")
//		{
//			flag = 1;
//			continue;
//		}
//		else if (temp == "<para>")
//		{
//			flag = 2;
//			continue;
//		}
//		else if (temp == "<end>")
//			break;
//		//根据标识对temp操作
//		if (flag == 1)//path
//		{
//			if (path != "")
//				path += " ";
//			path += temp;
//			continue;
//		}
//		else if (flag == 2)//para
//		{
//			varray<string> vstr;
//			Split(temp, vstr, "=");
//			para.push_back(atof(vstr[1].c_str()));
//			continue;
//		}
//	}
//	inf.close();
//	//初始化控件文本
//	if (path != "")
//		m_modelPath = path.c_str();
//	if (para.size() == 11)
//	{
//		MyDoc::Ptr pdoc = MyDoc::getInstance();
//		pdoc->m_Unum = para[0];
//		pdoc->m_Vnum = para[1];
//		pdoc->m_Wnum = para[2];
//		pdoc->modelType = para[3];
//		pdoc->m_dotSZ = para[4];
//		pdoc->m_CPTSdotSZ = para[5];
//		pdoc->m_NUMPic = para[6];
//	}
//}


void Option::Split(const string& s, varray<string>& v, const string& c)
{
	string::size_type pos1, pos2;
	pos2 = s.find(c);
	pos1 = 0;
	while (string::npos != pos2)
	{
		v.push_back(s.substr(pos1, pos2 - pos1));

		pos1 = pos2 + c.size();
		pos2 = s.find(c, pos1);
	}
	if (pos1 != s.length())
		v.push_back(s.substr(pos1));
}


//视角还原
//----------
void Option::OnBnClickedViewre()
{
	MyDoc::Ptr pdoc = MyDoc::getInstance();
	pdoc->m_restView = true;
}


//生成3MF文件
//----------
void Option::OnBnClicked3MF()
{
	MyDoc::Ptr pdoc = MyDoc::getInstance();

	vector<sLib3MFPosition> vectice;
	vector<sLib3MFTriangle> triangle;
	vector<int> base = { 0,1,2,2,3,0 };

	for (int i = 0; i != pdoc->m_ShowData.size(); ++i)
	{
		for (int j = 0; j != pdoc->m_ShowData[i].size(); ++j)
		{
			for (int k = 0; k != pdoc->m_ShowData[i][j].size(); ++k)
			{
				sLib3MFPosition temp;
				temp.m_Coordinates[0] = pdoc->m_ShowData[i][j][k].x;
				temp.m_Coordinates[1] = pdoc->m_ShowData[i][j][k].y;
				temp.m_Coordinates[2] = pdoc->m_ShowData[i][j][k].z;
				vectice.push_back(temp);
			}
		}
	}
	for (int i = 0; i != vectice.size() / 4; ++i)
	{
		for (int k = 0; k != base.size(); k += 3)
		{
			sLib3MFTriangle temp;
			temp.m_Indices[0] = base[k];
			base[k] += 4;
			temp.m_Indices[1] = base[k + 1];
			base[k + 1] += 4;
			temp.m_Indices[2] = base[k + 2];
			base[k + 2] += 4;
			triangle.push_back(temp);
		}
	}

	Lib3MF::PWrapper wrapper = Lib3MF::CWrapper::loadLibrary();
	Lib3MF::PModel model = wrapper->CreateModel();
	Lib3MF::PMeshObject meshObject = model->AddMeshObject();
	meshObject->SetName("vol");
	meshObject->SetGeometry(vectice, triangle);
	model->AddBuildItem(meshObject.get(), wrapper->GetIdentityTransform());
	Lib3MF::PWriter writer = model->QueryWriter("3mf");
	writer->WriteToFile("vol.3mf");
}

//渲染非均质模型使用
void Option::createData(QOpenGLContext * p, vector<float> data)
{
	//设置当前渲染环境
	initializeOpenGLFunctions();
	QSurface *a = p->surface();
	p->makeCurrent(a);

	MyDoc::Ptr pdoc = MyDoc::getInstance();
	//我们还需要一个索引数组来表示每个点之间的联系，形成一个轮廓模型
	vector<int> index;
	vector<int> INDEX;
	//首先是uv面，有两个
	vector<int> temp = { 0,1,pdoc->m_Unum,pdoc->m_Unum,1,pdoc->m_Unum + 1 };
	for (int k = 1; k != pdoc->m_Unum; ++k)
	{
		for (int i = 1; i < pdoc->m_Vnum; ++i)
		{
			for (int j = 0; j != temp.size(); ++j)
			{
				index.push_back(temp[j]);
				temp[j]++;
			}
		}
		for (int i = 0; i != temp.size(); ++i)
		{
			temp[i]++;
		}
	}
	int size = index.size();//继续创建第二个uv面的索引数组
	for (int i = 0; i != size; ++i)
	{
		index.push_back(index[i] + pdoc->m_Unum*pdoc->m_Vnum*(pdoc->m_Wnum - 1));
	}
	INDEX.insert(INDEX.end(), index.begin(), index.end());
	index.clear();
	//接下来是uw面，也是两个
	temp = { 0,1,pdoc->m_Unum*pdoc->m_Vnum,pdoc->m_Unum*pdoc->m_Vnum ,1,pdoc->m_Unum*pdoc->m_Vnum + 1 };
	for (int k = 1; k != pdoc->m_Wnum; ++k)
	{
		for (int i = 1; i < pdoc->m_Unum; ++i)
		{
			for (int j = 0; j != temp.size(); ++j)
			{
				index.push_back(temp[j]);
				temp[j] ++;
			}
		}
		for (int i = 0; i != temp.size(); ++i)
		{
			temp[i] += (pdoc->m_Unum*(pdoc->m_Vnum-1)+1);
		}
	}
	size = index.size();
	for (int i = 0; i != size; ++i)
	{
		index.push_back(index[i] + pdoc->m_Wnum*(pdoc->m_Vnum - 1));
	}
	INDEX.insert(INDEX.end(), index.begin(), index.end());
	index.clear();
	//接下来是vw面，同样是两个
	temp = { 0,pdoc->m_Unum,pdoc->m_Unum*pdoc->m_Vnum,pdoc->m_Unum*pdoc->m_Vnum ,pdoc->m_Unum ,pdoc->m_Unum*pdoc->m_Vnum + pdoc->m_Unum };
	for (int k = 1; k != pdoc->m_Vnum; ++k)
	{
		for (int i = 1; i != pdoc->m_Vnum; ++i)
		{
			for (int j = 0; j != temp.size(); ++j)
			{
				index.push_back(temp[j]);
				temp[j] += pdoc->m_Unum;
			}
		}
		for (int i = 0; i != temp.size(); ++i)
		{
			temp[i] += pdoc->m_Unum;
		}
	}
	size = index.size();
	for (int i = 0; i != size; ++i)
	{
		index.push_back(index[i] + (pdoc->m_Unum - 1));
	}
	INDEX.insert(INDEX.end(), index.begin(), index.end());

	unsigned int VAO, VBO, EBO;
	glGenVertexArrays(1, &VAO);
	glGenBuffers(1, &VBO);
	glGenBuffers(1, &EBO);
	glBindVertexArray(VAO);
	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, data.size()*sizeof(float), data.data(), GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(float)*INDEX.size(), INDEX.data(), GL_STATIC_DRAW);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), (GLvoid*)0);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), (GLvoid*)(3 * sizeof(float)));
	glEnableVertexAttribArray(1);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);

	pdoc->VAO_D.push_back(VAO);
	pdoc->VAO_D_SIZE.push_back(6*pdoc->m_Unum*pdoc->m_Vnum*pdoc->m_Wnum);
}

void Option::createMode()
{

}

//渲染均质模型
void Option::createData(QOpenGLContext * p, varray<varray<varray<point3d>>> varrayPoint, varray<varray<varray<point3d>>> varrayLine)
{
	//设置当前的渲染环境
	initializeOpenGLFunctions();
	QSurface *a = p->surface();
	p->makeCurrent(a);

	MyDoc::Ptr pdoc = MyDoc::getInstance();
	pdoc->m_CellIdx = -1;//设置当前不需要进行分片显示

	std::vector<unsigned int> indices;
	std::vector<unsigned int> base = { 0,1,2,2,3,0 };//索引数组

	//这里我门创造一个一维数组，保存原来的数据
	//画面的数据保存
	for (int i = 0; i != varrayPoint.size(); ++i)
	{
		int pointsize = 0;
		//先把所有的数据整合到一个一维数组里面
		for (int j = 0; j != varrayPoint[i].size(); ++j)
		{
			for (int k = 0; k != varrayPoint[i][j].size(); ++k)
			{
				pdoc->showDta_vertices.push_back(varrayPoint[i][j][k].x);
				pdoc->showDta_vertices.push_back(varrayPoint[i][j][k].y);
				pdoc->showDta_vertices.push_back(varrayPoint[i][j][k].z);
			}
			for (int k = 0; k != base.size(); ++k)
			{
				indices.push_back(base[k]);
				base[k] += 4;
			}
			pointsize += 6;
		}
		//修改于2020-07-03，计算法向量，用来计算光照
		
		//光照计算结束
		uint VAO, VBO, EBO;
		glGenVertexArrays(1, &VAO);//顶点数组对象
		glGenBuffers(1, &VBO);//顶点数组
		glGenBuffers(1, &EBO);//索引数组
		glBindVertexArray(VAO);//开始绑定
		glBindBuffer(GL_ARRAY_BUFFER, VBO);
		glBufferData(GL_ARRAY_BUFFER, pdoc->showDta_vertices.size() * sizeof(float), pdoc->showDta_vertices.data(), GL_STATIC_DRAW);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(float)*indices.size(), indices.data(), GL_STATIC_DRAW);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), (GLvoid *)0);
		glEnableVertexAttribArray(0);
		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindVertexArray(0);
		pdoc->EACH_VAO_D_POINT_NUMBER.push_back(pointsize);
		pdoc->VAO_D_ARRAY.push_back(VAO);
		pdoc->showDta_vertices.clear();
		indices.clear();
		base = { 0,1,2,2,3,0 };
	}

	//保存线的数据，线不需要索引数组，线因为特殊需要一组一组的分别画出来
	for (int i = 0; i != varrayLine.size(); ++i)
	{
		for (int j = 0; j != varrayLine[i].size(); ++j)
		{
			int pointSize = 0;//计算这一组线段里面有多少个点
			for (int k = 0; k != varrayLine[i][j].size(); ++k)
			{
				pdoc->showLines_vertices.push_back(varrayLine[i][j][k].x);
				pdoc->showLines_vertices.push_back(varrayLine[i][j][k].y);
				pdoc->showLines_vertices.push_back(varrayLine[i][j][k].z);
				pointSize++;
			}
			unsigned int VBO, VAO;
			glGenVertexArrays(1, &VAO);
			glGenBuffers(1, &VBO);
			glBindVertexArray(VAO);
			glBindBuffer(GL_ARRAY_BUFFER, VBO);
			glBufferData(GL_ARRAY_BUFFER, pdoc->showLines_vertices.size() * sizeof(float), pdoc->showLines_vertices.data(), GL_STATIC_DRAW);
			glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), (GLvoid *)0);
			glEnableVertexAttribArray(0);
			glBindBuffer(GL_ARRAY_BUFFER, 0);
			glBindVertexArray(0);
			pdoc->VAO_L_ARRAY.push_back(VAO);//把这一组的顶点缓冲对象记录下来
			pdoc->EACH_VAO_L_POINT_NUMBER.push_back(pointSize);//把这一组有多少个点记录下来
			pdoc->showLines_vertices.clear();//数组清空，方便下行下一个循环
		}
	}	
}

//渲染均质模型
void Option::createData(QOpenGLContext * p, varray<varray<varray<Vec3>>> varrayPoint, varray<varray<varray<Vec3>>> varrayLine)
{
	//设置当前的渲染环境
	initializeOpenGLFunctions();
	QSurface *a = p->surface();
	p->makeCurrent(a);

	MyDoc::Ptr pdoc = MyDoc::getInstance();
	pdoc->m_CellIdx = -1;//设置当前不需要进行分片显示

	std::vector<unsigned int> indices;
	std::vector<unsigned int> base = { 0,1,2,2,3,0 };//索引数组

	//这里我门创造一个一维数组，保存原来的数据
	//画面的数据保存
	for (int i = 0; i != varrayPoint.size(); ++i)
	{
		int pointsize = 0;
		//先把所有的数据整合到一个一维数组里面
		for (int j = 0; j != varrayPoint[i].size(); ++j)
		{
			for (int k = 0; k != varrayPoint[i][j].size(); ++k)
			{
				pdoc->showDta_vertices.push_back(varrayPoint[i][j][k].x);
				pdoc->showDta_vertices.push_back(varrayPoint[i][j][k].y);
				pdoc->showDta_vertices.push_back(varrayPoint[i][j][k].z);
			}
			for (int k = 0; k != base.size(); ++k)
			{
				indices.push_back(base[k]);
				base[k] += 4;
			}
			pointsize += 6;
		}
		//修改于2020-07-03，计算法向量，用来计算光照

		//光照计算结束
		uint VAO, VBO, EBO;
		glGenVertexArrays(1, &VAO);//顶点数组对象
		glGenBuffers(1, &VBO);//顶点数组
		glGenBuffers(1, &EBO);//索引数组
		glBindVertexArray(VAO);//开始绑定
		glBindBuffer(GL_ARRAY_BUFFER, VBO);
		glBufferData(GL_ARRAY_BUFFER, pdoc->showDta_vertices.size() * sizeof(float), pdoc->showDta_vertices.data(), GL_STATIC_DRAW);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(float)*indices.size(), indices.data(), GL_STATIC_DRAW);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), (GLvoid *)0);
		glEnableVertexAttribArray(0);
		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindVertexArray(0);
		pdoc->EACH_VAO_D_POINT_NUMBER.push_back(pointsize);
		pdoc->VAO_D_ARRAY.push_back(VAO);
		pdoc->showDta_vertices.clear();
		indices.clear();
		base = { 0,1,2,2,3,0 };
	}

	//保存线的数据，线不需要索引数组，线因为特殊需要一组一组的分别画出来
	for (int i = 0; i != varrayLine.size(); ++i)
	{
		for (int j = 0; j != varrayLine[i].size(); ++j)
		{
			int pointSize = 0;//计算这一组线段里面有多少个点
			for (int k = 0; k != varrayLine[i][j].size(); ++k)
			{
				pdoc->showLines_vertices.push_back(varrayLine[i][j][k].x);
				pdoc->showLines_vertices.push_back(varrayLine[i][j][k].y);
				pdoc->showLines_vertices.push_back(varrayLine[i][j][k].z);
				pointSize++;
			}
			unsigned int VBO, VAO;
			glGenVertexArrays(1, &VAO);
			glGenBuffers(1, &VBO);
			glBindVertexArray(VAO);
			glBindBuffer(GL_ARRAY_BUFFER, VBO);
			glBufferData(GL_ARRAY_BUFFER, pdoc->showLines_vertices.size() * sizeof(float), pdoc->showLines_vertices.data(), GL_STATIC_DRAW);
			glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), (GLvoid *)0);
			glEnableVertexAttribArray(0);
			glBindBuffer(GL_ARRAY_BUFFER, 0);
			glBindVertexArray(0);
			pdoc->VAO_L_ARRAY.push_back(VAO);//把这一组的顶点缓冲对象记录下来
			pdoc->EACH_VAO_L_POINT_NUMBER.push_back(pointSize);//把这一组有多少个点记录下来
			pdoc->showLines_vertices.clear();//数组清空，方便下行下一个循环
		}
	}
}

void Option::createCP(QOpenGLContext * p)
{
	initializeOpenGLFunctions();
	QSurface *a = p->surface();
	
	p->makeCurrent(a);
	MyDoc::Ptr pdoc = MyDoc::getInstance();
	//控制点的数据
	for (int i = 0; i != pdoc->m_ShowData4D.size(); ++i)
	{
		int pointsize = 0;
		for (int j = 0; j != pdoc->m_ShowData4D[i].size(); ++j)
		{
			pdoc->showCp_vertices.push_back(pdoc->m_ShowData4D[i][j].x);
			pdoc->showCp_vertices.push_back(pdoc->m_ShowData4D[i][j].y);
			pdoc->showCp_vertices.push_back(pdoc->m_ShowData4D[i][j].z);
			pointsize++;
		}
		unsigned int VAO, VBO;
		glGenVertexArrays(1, &VAO);
		glGenBuffers(1, &VBO);
		glBindVertexArray(VAO);
		glBindBuffer(GL_ARRAY_BUFFER, VBO);
		glBufferData(GL_ARRAY_BUFFER, pdoc->showCp_vertices.size() * sizeof(float), pdoc->showCp_vertices.data(), GL_STATIC_DRAW);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), (GLvoid *)0);
		glEnableVertexAttribArray(0);
		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindVertexArray(0);
		pdoc->EACH_VAO_CP_POINT_NUMBER.push_back(pointsize);
		pdoc->VAO_CP_ARRAY.push_back(VAO);
		pdoc->showCp_vertices.clear();
	}
	//for (int i = 0; i != pdoc->m_DisplayData4D.size(); ++i)
	//{
	//	int pointsize = 0;
	//	for (int j = 0; j != pdoc->m_DisplayData4D[i].size(); ++j)
	//	{
	//		pdoc->showCp_vertices.push_back(pdoc->m_DisplayData4D[i][j].x);
	//		pdoc->showCp_vertices.push_back(pdoc->m_DisplayData4D[i][j].y);
	//		pdoc->showCp_vertices.push_back(pdoc->m_DisplayData4D[i][j].z);
	//		pointsize++;
	//	}
	//	unsigned int VAO, VBO;
	//	glGenVertexArrays(1, &VAO);
	//	glGenBuffers(1, &VBO);
	//	glBindVertexArray(VAO);
	//	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	//	glBufferData(GL_ARRAY_BUFFER, pdoc->showCp_vertices.size() * sizeof(float), pdoc->showCp_vertices.data(), GL_STATIC_DRAW);
	//	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), (GLvoid *)0);
	//	glEnableVertexAttribArray(0);
	//	glBindBuffer(GL_ARRAY_BUFFER, 0);
	//	glBindVertexArray(0);
	//	pdoc->EACH_VAO_CP_POINT_NUMBER.push_back(pointsize);
	//	pdoc->VAO_CP_ARRAY.push_back(VAO);
	//	pdoc->showCp_vertices.clear();
	//}
}

//转换函数
void Option::TransCnurbsvolToPolyIGA(varray<NurbsVol> &nurbsVols,const CPolyParaVolume poly)
{
	nurbsVols.resize(poly.m_HexVolumes.size());
	for (int i = 0; i != poly.m_HexVolumes.size(); ++i)
	{
		//u节点向量
		nurbsVols.at(i).m_uKnots.resize(poly.m_HexVolumes.at(i).m_splVol.m_vKnots.size());
		for (int j = 0; j != poly.m_HexVolumes.at(i).m_splVol.m_vKnots.size(); ++j)
		{
			nurbsVols.at(i).m_uKnots[j] = poly.m_HexVolumes.at(i).m_splVol.m_vKnots[j];
		}
		//v节点向量
		for (int j = 0; j != poly.m_HexVolumes.at(i).m_splVol.m_uKnots.size(); ++j)
		{
			nurbsVols.at(i).m_vKnots.push_back(poly.m_HexVolumes.at(i).m_splVol.m_uKnots[j]);
		}
		//w节点向量
		for (int j = 0; j != poly.m_HexVolumes.at(i).m_splVol.m_wKnots.size(); ++j)
		{
			nurbsVols.at(i).m_wKnots.push_back(poly.m_HexVolumes.at(i).m_splVol.m_wKnots[j]);
		}
		//阶数
		nurbsVols.at(i).m_vDegree = poly.m_HexVolumes.at(i).m_splVol.m_uDegree;
		nurbsVols.at(i).m_uDegree = poly.m_HexVolumes.at(i).m_splVol.m_vDegree;
		nurbsVols.at(i).m_wDegree = poly.m_HexVolumes.at(i).m_splVol.m_wDegree;

		nurbsVols.at(i).m_vNum = poly.m_HexVolumes.at(i).m_splVol.m_uNum;
		nurbsVols.at(i).m_uNum = poly.m_HexVolumes.at(i).m_splVol.m_vNum;
		nurbsVols.at(i).m_wNum = poly.m_HexVolumes.at(i).m_splVol.m_wNum;

		nurbsVols.at(i).m_CtrlPts.resize(poly.m_HexVolumes.at(i).m_splVol.m_vAllCtrlPts.size());
		for (int j = 0; j != poly.m_HexVolumes.at(i).m_splVol.m_vAllCtrlPts.size(); ++j)
		{
			nurbsVols.at(i).m_CtrlPts[j].x = poly.m_HexVolumes.at(i).m_splVol.m_vAllCtrlPts[j].m_pt.x;
			nurbsVols.at(i).m_CtrlPts[j].y = poly.m_HexVolumes.at(i).m_splVol.m_vAllCtrlPts[j].m_pt.y;
			nurbsVols.at(i).m_CtrlPts[j].z = poly.m_HexVolumes.at(i).m_splVol.m_vAllCtrlPts[j].m_pt.z;
		}
	}
}

//计算法向量
point3d Option::LegalVector(point3d a, point3d b, point3d c)
{
	point3d temp;
	//计算出来两个向量
	point3d ab(b.x - a.x, b.y - a.y, b.z - a.z);
	point3d ac(c.x - a.x, c.y - a.y, c.z - a.z);
	//计算法向量
	temp.x = ab.y*ac.z - ac.y*ab.z;
	temp.y = ab.z*ac.x - ac.z*ab.x;
	temp.z = ab.x*ac.y - ac.x*ab.y;
	return temp;
}

//绘制函数
void Option::drawNurbsVol(QOpenGLContext * p, QWidget * father, varray<NurbsVol> nvs)
{
	MyDoc::Ptr pdoc = MyDoc::getInstance();

	varray<varray<point3d>> p3d2, l2;
	//把原来画面进行清空
	pdoc->m_ShowData.clear();
	pdoc->m_ShowLines.clear();

	//下面的代码是从CPolyParaVolume转为NurbsVol
	//varray<NurbsVol> nvs;
	
	if (m_again != 0)//如果在运行过程中发生了拖动点，我们把拖动后的点作为最新的控制点
	{
		for (int i = 0; i != pdoc->m_ShowData4D.size(); ++i)
		{
			nvs[i].m_CtrlPts = pdoc->m_ShowData4D[i];
		}
	}
	else
	{
		for (int i = 0; i < nvs.size(); ++i)
		{
			pdoc->m_ShowData4D.push_back(nvs[i].m_CtrlPts);
		}
	}
	//////////////多线程计算，修改于2020-2-8
	vector<future<threadParamVOL>> v;
	int count = 0;//多线程的计数，我们设定一次创造的线程最多不能多余7个，太多了会造成上下文切换，浪费
	int size = 0;
	for (int i = 0; i < nvs.size(); i++)
	{
		count++;//创造一个线程
		varray<varray<varray<point3d>>>  q3, l3;
		v.push_back(async(&NurbsVol::CalQuads, &nvs[i], pdoc->m_Unum, pdoc->m_Vnum, pdoc->m_Wnum));
		if (count == pdoc->THREADNUMBERS || i == nvs.size() - 1)//当线程数量到7的时候我们就分别等线程结束，然后回收
		{
			for (int j = 0; j < count; j++)//开始回收七个线程里面的数据
			{
				v[j].wait();
				threadParamVOL p = v[j].get();
				for (int k = 0; k < p.q3.size(); ++k)
					pdoc->m_ShowData.push_back(p.q3[k]);
				for (int k = 0; k < p.l3.size(); ++k)
					pdoc->m_ShowLines.push_back(p.l3[k]);
			}
			count = 0;//清空计数
			v.clear();//清空列表，等待下一轮的7个线程

			//修改于2020-05-14，删除掉中间的文件存储，直接进行显示
			createData(p, pdoc->m_ShowData, pdoc->m_ShowLines);
			size += pdoc->m_ShowData.size();
			pdoc->cleanUp();
		}
	}
	pdoc->m_tolNum = size / pdoc->m_NUMPic;
	if (pdoc->m_tolNum*pdoc->m_NUMPic < size)
		++pdoc->m_tolNum;
}

void Option::drawNurbsSurface(QOpenGLContext * p, QWidget * father, varray<NurbsSurface>nsfs)
{
	MyDoc::Ptr pdoc = MyDoc::getInstance();

	varray<varray<point3d>> p3d2, l2;
	pdoc->m_ShowData.clear();
	pdoc->m_ShowLines.clear();

	int size = 0;
	if (m_again != 0)//如果在运行过程中发生了拖动点，我们把拖动后的点作为最新的控制点
	{
		for (int i = 0; i != pdoc->m_ShowData4D.size(); ++i)
		{
			nsfs[i].m_CtrlPts = pdoc->m_ShowData4D[i];
		}
	}
	else
	{
		for (int i = 0; i < nsfs.size(); ++i)
		{
			pdoc->m_ShowData4D.push_back(nsfs[i].m_CtrlPts);
		}
	}
	///////////////多线程计算
	vector<future<threadParam>> v;
	int count = 0;//多线程的计数，我们设定一次创造的线程最多不能多余7个，太多了会造成上下文切换，浪费
	for (int i = 0; i < nsfs.size(); i++)
	{
		count++;//创造一个线程
		v.push_back(async(&NurbsSurface::CalQuads, &nsfs[i], pdoc->m_Unum, pdoc->m_Vnum));
		if (count == pdoc->THREADNUMBERS || i == nsfs.size() - 1)//当线程数量到7的时候我们就分别等线程结束，然后回收
		{
			for (int j = 0; j < count; j++)//开始回收七个线程里面的数据
			{
				v[j].wait();
				threadParam p = v[j].get();
				pdoc->m_ShowData.push_back(p.l2);
				pdoc->m_ShowLines.push_back(p.p3d2);
			}
			count = 0;//清空计数
			v.clear();//清空列表，等待下一轮的7个线程

			createData(p, pdoc->m_ShowData, pdoc->m_ShowLines);
			size += pdoc->m_ShowData.size();
			pdoc->cleanUp();
		}
	}
	////////多线程计算结束
	pdoc->m_tolNum = size / pdoc->m_NUMPic;
	if (pdoc->m_tolNum*pdoc->m_NUMPic < size)
		++pdoc->m_tolNum;
}

void Option::drawNurbsLine(QOpenGLContext * p, QWidget * father, varray<NurbsLine> nls)
{
	MyDoc::Ptr pdoc = MyDoc::getInstance();

	varray<varray<point3d>> p3d2, l2;
	pdoc->m_ShowData.clear();
	pdoc->m_ShowLines.clear();

	int size = 0;
	if (m_again != 0)//如果在运行过程中发生了拖动点，我们把拖动后的点作为最新的控制点
	{
		for (int i = 0; i != pdoc->m_ShowData4D.size(); ++i)
		{
			nls[i].m_CtrlPts = pdoc->m_ShowData4D[i];
		}
	}
	for (int i = 0; i < nls.size(); ++i)
	{
		varray<point3d> l3d;
		nls[i].CalLinePoint(pdoc->m_Unum, l3d);
		l2.clear();
		l2.push_back(l3d);
		pdoc->m_ShowData.push_back(l2);
		if (m_again == 0)
		{
			pdoc->m_ShowData4D.push_back(nls[i].m_CtrlPts);
		}
		createData(p, pdoc->m_ShowData, pdoc->m_ShowLines);
		size += pdoc->m_ShowData.size();
		pdoc->cleanUp();
	}
	pdoc->m_tolNum = size / pdoc->m_NUMPic;
	if (pdoc->m_tolNum*pdoc->m_NUMPic < size)
		++pdoc->m_tolNum;
}

void Option::drawBezierLine(QOpenGLContext * p, QWidget * father, varray<BezierLine> bzls)
{
	MyDoc::Ptr pdoc = MyDoc::getInstance();

	varray<varray<point3d>> p3d2, l2;
	pdoc->m_ShowData.clear();
	pdoc->m_ShowLines.clear();

	int size = 0;
	if (m_again != 0)//如果在运行过程中发生了拖动点，我们把拖动后的点作为最新的控制点
	{
		for (int i = 0; i != pdoc->m_ShowData4D.size(); ++i)
		{
			bzls[i].m_CtrlPts = pdoc->m_ShowData4D[i];
		}
	}
	for (int i = 0; i < bzls.size(); ++i)
	{
		varray<point3d> l3d;
		bzls[i].CalLinePoint(pdoc->m_Unum, l3d);
		l2.clear();
		l2.push_back(l3d);
		pdoc->m_ShowData.push_back(l2);
		if (m_again == 0)
		{
			pdoc->m_ShowData4D.push_back(bzls[i].m_CtrlPts);
		}
		createData(p, pdoc->m_ShowData, pdoc->m_ShowLines);
		size += pdoc->m_ShowData.size();
		pdoc->cleanUp();
	}
	pdoc->m_tolNum = size / pdoc->m_NUMPic;
	if (pdoc->m_tolNum*pdoc->m_NUMPic < size)
		++pdoc->m_tolNum;
}

void Option::drawpoint4d(QOpenGLContext * p, QWidget * father, varray<varray<point4d>> p4d2)
{
	MyDoc::Ptr pdoc = MyDoc::getInstance();

	varray<varray<point3d>> p3d2, l2;
	pdoc->m_ShowData.clear();
	pdoc->m_ShowLines.clear();

	for (int i = 0; i < p4d2.size(); ++i)
	{
		varray<point3d> l3d;
		for (int j = 0; j < p4d2[i].size(); ++j)
			l3d.push_back(p4d2[i][j]);
		l2.clear();
		l2.push_back(l3d);
		pdoc->m_ShowData.push_back(l2);
	}
	createData(p, pdoc->m_ShowData, pdoc->m_ShowLines);
	pdoc->m_tolNum = pdoc->m_ShowData.size() / pdoc->m_NUMPic;
	if (pdoc->m_tolNum*pdoc->m_NUMPic < pdoc->m_ShowData.size())
		++pdoc->m_tolNum;
}

void Option::drawpoint3d(QOpenGLContext * p, QWidget * father, varray<varray<point3d>> p3d2)
{
	MyDoc::Ptr pdoc = MyDoc::getInstance();

	varray<varray<point3d>> /*p3d2,*/ l2;
	pdoc->m_ShowData.clear();
	pdoc->m_ShowLines.clear();
	
	for (int i = 0; i < p3d2.size(); ++i)
	{
		l2.clear();
		l2.push_back(p3d2[i]);
		pdoc->m_ShowData.push_back(l2);
	}
	pdoc->m_tolNum = pdoc->m_ShowData.size() / pdoc->m_NUMPic;
	if (pdoc->m_tolNum*pdoc->m_NUMPic < pdoc->m_ShowData.size())
		++pdoc->m_tolNum;
}

void Option::drawSplineVolume(QOpenGLContext * p, QWidget * father, varray<SplineVolume> nvs)
{
	MyDoc::Ptr pdoc = MyDoc::getInstance();

	varray<varray<Vec3>> p3d2, l2;
	//把原来画面进行清空
	pdoc->m_DisplayData.clear();
	pdoc->m_DisplayLines.clear();

	//下面的代码是从CPolyParaVolume转为SplineVolume
	//varray<SplineVolume> nvs;

	if (m_again != 0)//如果在运行过程中发生了拖动点，我们把拖动后的点作为最新的控制点
	{
		for (int i = 0; i != pdoc->m_DisplayData4D.size(); ++i)
		{
			nvs[i].m_CtrlPts = pdoc->m_DisplayData4D[i];
		}
	}
	else
	{
		for (int i = 0; i < nvs.size(); ++i)
		{
			pdoc->m_DisplayData4D.push_back(nvs[i].m_CtrlPts);
		}
	}
	//////////////多线程计算，修改于2020-2-8
	vector<future<threadParamSplineVOL>> v;
	int count = 0;//多线程的计数，我们设定一次创造的线程最多不能多余7个，太多了会造成上下文切换，浪费
	int size = 0;
	for (int i = 0; i < nvs.size(); i++)
	{
		count++;//创造一个线程
		varray<varray<varray<Vec3>>>  q3, l3;
		v.push_back(async(&SplineVolume::CalQuads, &nvs[i], pdoc->m_Unum, pdoc->m_Vnum, pdoc->m_Wnum));
		if (count == pdoc->THREADNUMBERS || i == nvs.size() - 1)//当线程数量到7的时候我们就分别等线程结束，然后回收
		{
			for (int j = 0; j < count; j++)//开始回收七个线程里面的数据
			{
				v[j].wait();
				threadParamSplineVOL p = v[j].get();
				for (int k = 0; k < p.q3.size(); ++k)
					pdoc->m_DisplayData.push_back(p.q3[k]);
				for (int k = 0; k < p.l3.size(); ++k)
					pdoc->m_DisplayLines.push_back(p.l3[k]);
			}
			count = 0;//清空计数
			v.clear();//清空列表，等待下一轮的7个线程

			//修改于2020-05-14，删除掉中间的文件存储，直接进行显示
			createData(p, pdoc->m_DisplayData, pdoc->m_DisplayLines);
			size += pdoc->m_DisplayData.size();
			pdoc->cleanUp();
		}
	}
	pdoc->m_tolNum = size / pdoc->m_NUMPic;
	if (pdoc->m_tolNum*pdoc->m_NUMPic < size)
		++pdoc->m_tolNum;
}

void Option::drawSplineSurface(QOpenGLContext * p, QWidget * father, varray<SplineSurface>nsfs)
{
	MyDoc::Ptr pdoc = MyDoc::getInstance();

	varray<varray<Vec3>> p3d2, l2;
	pdoc->m_DisplayData.clear();
	pdoc->m_DisplayLines.clear();

	int size = 0;
	if (m_again != 0)//如果在运行过程中发生了拖动点，我们把拖动后的点作为最新的控制点
	{
		for (int i = 0; i != pdoc->m_DisplayData4D.size(); ++i)
		{
			nsfs[i].m_CtrlPts = pdoc->m_DisplayData4D[i];
		}
	}
	else
	{
		for (int i = 0; i < nsfs.size(); ++i)
		{
			pdoc->m_DisplayData4D.push_back(nsfs[i].m_CtrlPts);
		}
	}
	///////////////多线程计算
	vector<future<threadParamSpline>> v;
	int count = 0;//多线程的计数，我们设定一次创造的线程最多不能多余7个，太多了会造成上下文切换，浪费
	for (int i = 0; i < nsfs.size(); i++)
	{
		count++;//创造一个线程
		v.push_back(async(&SplineSurface::CalQuads, &nsfs[i], pdoc->m_Unum, pdoc->m_Vnum));
		if (count == pdoc->THREADNUMBERS || i == nsfs.size() - 1)//当线程数量到7的时候我们就分别等线程结束，然后回收
		{
			for (int j = 0; j < count; j++)//开始回收七个线程里面的数据
			{
				v[j].wait();
				threadParamSpline p = v[j].get();
				pdoc->m_DisplayData.push_back(p.l2);
				pdoc->m_DisplayLines.push_back(p.p3d2);
			}
			count = 0;//清空计数
			v.clear();//清空列表，等待下一轮的7个线程

			createData(p, pdoc->m_DisplayData, pdoc->m_DisplayLines);
			size += pdoc->m_DisplayData.size();
			pdoc->cleanUp();
		}
	}
	////////多线程计算结束
	pdoc->m_tolNum = size / pdoc->m_NUMPic;
	if (pdoc->m_tolNum*pdoc->m_NUMPic < size)
		++pdoc->m_tolNum;
}

void Option::drawSpline(QOpenGLContext * p, QWidget * father, varray<Spline> nls)
{
	MyDoc::Ptr pdoc = MyDoc::getInstance();

	varray<varray<Vec3>> p3d2, l2;
	pdoc->m_DisplayData.clear();
	pdoc->m_DisplayLines.clear();

	int size = 0;
	if (m_again != 0)//如果在运行过程中发生了拖动点，我们把拖动后的点作为最新的控制点
	{
		for (int i = 0; i != pdoc->m_DisplayData4D.size(); ++i)
		{
			nls[i].m_CtrlPts = pdoc->m_DisplayData4D[i];
		}
	}
	for (int i = 0; i < nls.size(); ++i)
	{
		varray<Vec3> l3d;
		nls[i].CalLinePoint(pdoc->m_Unum, l3d);
		l2.clear();
		l2.push_back(l3d);
		pdoc->m_DisplayData.push_back(l2);
		if (m_again == 0)
		{
			pdoc->m_DisplayData4D.push_back(nls[i].m_CtrlPts);
		}
		createData(p, pdoc->m_DisplayData, pdoc->m_DisplayLines);
		size += pdoc->m_DisplayData.size();
		pdoc->cleanUp();
	}
	pdoc->m_tolNum = size / pdoc->m_NUMPic;
	if (pdoc->m_tolNum*pdoc->m_NUMPic < size)
		++pdoc->m_tolNum;
}

void Option::drawVec4(QOpenGLContext * p, QWidget * father, varray<varray<Vec4>> p4d2)
{
	MyDoc::Ptr pdoc = MyDoc::getInstance();

	varray<varray<Vec3>> p3d2, l2;
	pdoc->m_DisplayData.clear();
	pdoc->m_DisplayLines.clear();

	for (int i = 0; i < p4d2.size(); ++i)
	{
		varray<Vec3> l3d;
		for (int j = 0; j < p4d2[i].size(); ++j)
			l3d.push_back(p4d2[i][j]);
		l2.clear();
		l2.push_back(l3d);
		pdoc->m_DisplayData.push_back(l2);
	}
	createData(p, pdoc->m_DisplayData, pdoc->m_DisplayLines);
	pdoc->m_tolNum = pdoc->m_DisplayData.size() / pdoc->m_NUMPic;
	if (pdoc->m_tolNum*pdoc->m_NUMPic < pdoc->m_DisplayData.size())
		++pdoc->m_tolNum;
}

void Option::drawVec3(QOpenGLContext * p, QWidget * father, varray<varray<Vec3>> p3d2)
{
	MyDoc::Ptr pdoc = MyDoc::getInstance();

	varray<varray<Vec3>> /*p3d2,*/ l2;
	pdoc->m_DisplayData.clear();
	pdoc->m_DisplayLines.clear();

	for (int i = 0; i < p3d2.size(); ++i)
	{
		l2.clear();
		l2.push_back(p3d2[i]);
		pdoc->m_DisplayData.push_back(l2);
	}
	pdoc->m_tolNum = pdoc->m_DisplayData.size() / pdoc->m_NUMPic;
	if (pdoc->m_tolNum*pdoc->m_NUMPic < pdoc->m_DisplayData.size())
		++pdoc->m_tolNum;
}


void Option::setLockOn()
{
	_modifyLock = true;
}

void Option::setLockClose()
{
	_modifyLock = false;
}

bool Option::isLcokOn()
{
	return _modifyLock;
}

void Option::reDraw(QOpenGLContext * p, QWidget * father)
{
	//MyDoc::Ptr pdoc = MyDoc::getInstance();
	////设置两个辅助数组
	//varray<varray<point3d>> p3d2, l2;
	////把原来画面进行清空
	//pdoc->m_ShowData.clear();
	//pdoc->m_ShowLines.clear();

	//if (m_again != 0)//如果在运行过程中发生了拖动点，我们把拖动后的点作为最新的控制点
	//{
	//	for (int i = 0; i != pdoc->m_ShowData4D.size(); ++i)
	//	{
	//		nvs[i].m_CtrlPts = pdoc->m_ShowData4D[i];
	//	}
	//}
	//else
	//{
	//	for (int i = 0; i < nvs.size(); ++i)
	//	{
	//		pdoc->m_ShowData4D.push_back(nvs[i].m_CtrlPts);
	//	}
	//}
	////////////////多线程计算，修改于2020-2-8
	//vector<future<threadParamVOL>> v;
	//int count = 0;//多线程的计数，我们设定一次创造的线程最多不能多余7个，太多了会造成上下文切换，浪费
	//int size = 0;
	//for (int i = 0; i < nvs.size(); i++)
	//{
	//	count++;//创造一个线程
	//	varray<varray<varray<point3d>>>  q3, l3;
	//	v.push_back(async(&NurbsVol::CalQuads, &nvs[i], pdoc->m_Unum, pdoc->m_Vnum, pdoc->m_Wnum, q3, l3));
	//	if (count == pdoc->THREADNUMBERS || i == nvs.size() - 1)//当线程数量到7的时候我们就分别等线程结束，然后回收
	//	{
	//		for (int j = 0; j < count; j++)//开始回收七个线程里面的数据
	//		{
	//			v[j].wait();
	//			threadParamVOL p = v[j].get();
	//			for (int k = 0; k < p.q3.size(); ++k)
	//				pdoc->m_ShowData.push_back(p.q3[k]);
	//			for (int k = 0; k < p.l3.size(); ++k)
	//				pdoc->m_ShowLines.push_back(p.l3[k]);
	//		}
	//		count = 0;//清空计数
	//		v.clear();//清空列表，等待下一轮的7个线程

	//		//修改于2020-05-14，删除掉中间的文件存储，直接进行显示
	//		createData(p, pdoc->m_ShowData, pdoc->m_ShowLines);
	//		size += pdoc->m_ShowData.size();
	//		pdoc->cleanUp();
	//	}
	//}
	//pdoc->m_tolNum = size / pdoc->m_NUMPic;
	//if (pdoc->m_tolNum*pdoc->m_NUMPic < size)
	//	++pdoc->m_tolNum;
}