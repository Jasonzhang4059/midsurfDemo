#include "WingStruct.h"

using YN::NurbsVol;
using YN::NurbsSurface;
using YN::NurbsLine;
using YN::Cube;
using YN::Surface;

YN::WingStruct::WingStruct()
{
	_root = NULL;
}

YN::WingStruct::WingStruct(string path, bool tag1 = true)
{
	vector<Cube *> m_HexVolumes=ReadVolumeTxt(path,tag1);
	abstractInfo(m_HexVolumes);
}

YN::WingStruct::WingStruct(vector<NurbsVol> nurbsVols)
{
	vector<Cube *> m_HexVolumes;//所有体的集合
	m_HexVolumes = transNurbsVoltoCube(nurbsVols);//体的转化
	abstractInfo(m_HexVolumes);
}

YN::WingStruct::~WingStruct()
{
	vector<Cube *> a= bordFirstSearch();
	for (int i = 0; i != a.size(); ++i)
	{
		//将面参数和片参数全部删除，防止内存泄露
		delete a[i]->_u0v0;
		a[i]->_u0v0 = NULL;
		delete a[i]->_u1v1;
		a[i]->_u1v1 = NULL;
		delete a[i]->_v0w0;
		a[i]->_v0w0 = NULL;
		delete a[i]->_v1w1;
		a[i]->_v1w1 = NULL;
		delete a[i]->_u0w0;
		a[i]->_u0w0 = NULL;
		delete a[i]->_u1w1;
		a[i]->_u1w1 = NULL;
		delete a[i];
		a[i] = NULL;
	}
}

vector<Cube *> YN::WingStruct::ReadVolumeTxt(string path,bool tag=true)
{
	vector<Cube *> m_HexVolumes;
	ifstream ifs;
	ifs.open(path);
	assert(ifs.is_open());

	int i;

	while (!ifs.eof())
	{
		string str;
		ifs >> str;
	contin:
		if (str == "PN")
		{
			int count;
			ifs >> count;
			for (int i = 0; i != count; ++i)
			{
				Cube *t = new Cube;
				m_HexVolumes.push_back(t);
			}
		}
		if (str == "PI")
		{
			ifs >> i;
		}
		if (str == "OD")
		{
			ifs >> m_HexVolumes.at(i)->_Vol_Data._u_Degree;
			ifs >> m_HexVolumes.at(i)->_Vol_Data._v_Degree;
			ifs >> m_HexVolumes.at(i)->_Vol_Data._w_Degree;
			//这里添加下面的代码是因为读取文件的时候计算不正确，如果手动计算正确了，可以把下面的注释
			m_HexVolumes.at(i)->_Vol_Data._u_Degree--;
			m_HexVolumes.at(i)->_Vol_Data._v_Degree--;
			m_HexVolumes.at(i)->_Vol_Data._w_Degree--;
		}
		if (str == "UK")
		{
			int count;
			ifs >> count;
			m_HexVolumes.at(i)->_Vol_Data._u_Knots->resize(count);
			for (int j = 0; j < count; j++)
			{
				double temp;
				ifs >> temp;
				(*m_HexVolumes.at(i)->_Vol_Data._u_Knots)[j] = temp;
			}
		}
		if (str == "VK")
		{
			int count;
			ifs >> count;
			m_HexVolumes.at(i)->_Vol_Data._v_Knots->resize(count);
			for (int j = 0; j < count; j++)
			{
				double temp;
				ifs >> temp;
				(*m_HexVolumes.at(i)->_Vol_Data._v_Knots)[j] = temp;
			}
		}
		if (str == "WK")
		{
			int count;
			ifs >> count;
			m_HexVolumes.at(i)->_Vol_Data._w_Knots->resize(count);
			for (int j = 0; j < count; j++)
			{
				double temp;
				ifs >> temp;
				(*m_HexVolumes.at(i)->_Vol_Data._w_Knots)[j] = temp;
			}
		}
		if (str == "CP")
		{
			ifs >> m_HexVolumes.at(i)->_Vol_Data._u_Num;
			ifs >> m_HexVolumes.at(i)->_Vol_Data._v_Num;
			ifs >> m_HexVolumes.at(i)->_Vol_Data._w_Num;
			ifs >> str;
			while (str != "PI")
			{
				point4d v;
				v.x = stof(str);
				ifs >> v.y;
				ifs >> v.z;
				if (tag == true)
				{
					ifs >> v.w;
				}
				m_HexVolumes.at(i)->_Vol_Data._ControlPts->push_back(v);
				ifs >> str;
				if (ifs.eof())
				{
					return m_HexVolumes;
				}
			}
			goto contin;
		}
	}
	
	//读取结束之后检查数据是否正确
	for (auto iter = m_HexVolumes.begin(); iter != m_HexVolumes.end(); ++iter)
	{
		//检查控制点个数
		int num = (*iter)->_Vol_Data._u_Num*(*iter)->_Vol_Data._v_Num*(*iter)->_Vol_Data._w_Num;
		if (num != (*iter)->_Vol_Data._ControlPts->size())
		{
			m_HexVolumes.clear();
		}
		//检查 节点矢量=控制点个数+阶数+1
		if((*iter)->_Vol_Data._u_Knots->size()!= (*iter)->_Vol_Data._u_Num+ 
			(*iter)->_Vol_Data._u_Degree+1)
			m_HexVolumes.clear();
		if ((*iter)->_Vol_Data._v_Knots->size() != (*iter)->_Vol_Data._v_Num +
			(*iter)->_Vol_Data._v_Degree + 1)
			m_HexVolumes.clear();
		if ((*iter)->_Vol_Data._w_Knots->size() != (*iter)->_Vol_Data._w_Num +
			(*iter)->_Vol_Data._w_Degree + 1)
			m_HexVolumes.clear();
	}

	return m_HexVolumes;
}

vector<point4d>  YN::WingStruct::OutputParaVolumeDataTxt(string filename, string Cp)
{
	vector<Cube *> m_HexVolumes;
	m_HexVolumes = bordFirstSearch();
	//输出模型信息
	ofstream ofs(filename);
	ofs << "PN " << " " << m_HexVolumes.size() << "\n";  //patch number
	for (int i = 0; i < m_HexVolumes.size(); i++)
	{
		ofs << "PI" << " " << i << "\n";   //当前片的id号。

		ofs << "OD" << " " << m_HexVolumes.at(i)->_Vol_Data._u_Degree + 1 << " " << m_HexVolumes.at(i)->_Vol_Data._v_Degree + 1 << " " << m_HexVolumes.at(i)->_Vol_Data._w_Degree + 1 << "\n"; //Order

		ofs << "UK" << " " << m_HexVolumes.at(i)->_Vol_Data._u_Knots->size() << "\n";
		for (int i = 0; i < m_HexVolumes.at(i)->_Vol_Data._u_Knots->size(); i++)
			ofs << m_HexVolumes.at(i)->_Vol_Data._u_Knots->at(i) << " ";   //节点向量
		ofs << "\n";
		ofs << "VK" << " " << m_HexVolumes.at(i)->_Vol_Data._v_Knots->size() << "\n";
		for (int i = 0; i < m_HexVolumes.at(i)->_Vol_Data._v_Knots->size(); i++)
			ofs << m_HexVolumes.at(i)->_Vol_Data._v_Knots->at(i) << " ";   //节点向量
		ofs << "\n";
		ofs << "WK" << " " << m_HexVolumes.at(i)->_Vol_Data._w_Knots->size() << "\n";
		for (int i = 0; i < m_HexVolumes.at(i)->_Vol_Data._w_Knots->size(); i++)
			ofs << m_HexVolumes.at(i)->_Vol_Data._w_Knots->at(i) << " ";   //节点向量
		ofs << "\n";

		ofs << "CP" << " " << m_HexVolumes.at(i)->_Vol_Data._u_Num << " " << m_HexVolumes.at(i)->_Vol_Data._v_Num << " " << m_HexVolumes.at(i)->_Vol_Data._w_Num << "\n"; //control point number
		for (int i = 0; i < m_HexVolumes.at(i)->_Vol_Data._ControlPts->size(); i++)
			ofs << m_HexVolumes.at(i)->_Vol_Data._ControlPts->at(i).x << " " 
			<< m_HexVolumes.at(i)->_Vol_Data._ControlPts->at(i).y << " " 
			<< m_HexVolumes.at(i)->_Vol_Data._ControlPts->at(i).z << " " 
			<< m_HexVolumes.at(i)->_Vol_Data._ControlPts->at(i).w << "\n";
	}
	ofs.close();
	//输出控制点全局编号
	ofstream idofs(Cp);
	idofs << "PN " << " " << m_HexVolumes.size() << "\n";  //patch number
	for (int i = 0; i < m_HexVolumes.size(); i++)
	{
		int idxnum = m_HexVolumes.at(i)->_volGloblControlPointID.size();
		idofs << "PN" << " " << i << "\n";
		idofs << "PI" << " " << idxnum << "\n";
		for (int j = 0; j < idxnum; j++)
		{
			idofs << m_HexVolumes.at(i)->_volGloblControlPointID[j] << " ";
		}
		idofs << "\n";
	}

	//输出施加约束和力的控制点
	vector<point4d> Poin;
	point4d po;
	//输出点和面之间的距离
	auto function = [](point4d p, float A, float B, float C, float D) {
		float temp = sqrt(A*A + B * B + C * C);
		return 1.0*abs(p.x*A + p.y*B + p.z*C + D) / temp;
	};
	//输入平面参数，输出距离平面最近的控制点
	auto Function = [&](float A, float B, float C, float D)
	{
		vector <int> result;
		float distance;
		for (int i = 0; i != m_HexVolumes.size(); ++i)
		{
			for (int j = 0; j != m_HexVolumes[i]->_Vol_Data._ControlPts->size(); ++j)
			{
				if (result.size() == 0)
				{
					distance = function(m_HexVolumes[i]->_Vol_Data._ControlPts->at(j) , A, B, C, D);
					result.push_back(m_HexVolumes[i]->_volGloblControlPointID[j]);
					po = m_HexVolumes[i]->_Vol_Data._ControlPts->at(j);
					Poin.push_back(po);
					continue;
				}
				else
				{
					float temp = function(m_HexVolumes[i]->_Vol_Data._ControlPts->at(j), A, B, C, D);
					if (abs(temp - distance) <= 0.1)
					{
						result.push_back(m_HexVolumes[i]->_volGloblControlPointID[j]);
						po = m_HexVolumes[i]->_Vol_Data._ControlPts->at(j);
						Poin.push_back(po);
					}
					else
					{
						if (distance - temp > 0)
						{
							result.clear();
							Poin.clear();
							distance = function(m_HexVolumes[i]->_Vol_Data._ControlPts->at(j), A, B, C, D);
							result.push_back(m_HexVolumes[i]->_volGloblControlPointID[j]);
							po = m_HexVolumes[i]->_Vol_Data._ControlPts->at(j);
							Poin.push_back(po);
						}
						else
						{
							continue;
						}
					}
				}
			}
		}
		return result;
	};
	//去除重复控制点
	auto single = [](vector<int> &a)
	{
		for (auto iter = a.begin(); iter != a.end(); ++iter)
		{
			for (auto iter2 = iter + 1; iter2 != a.end(); )
			{
				if (*iter2 == *iter)
				{
					iter2 = a.erase(iter2);
				}
				else
				{
					iter2++;
				}
			}
		}
	};

	if (m_HexVolumes.size() != 0)
	{
		vector<int> a = Function(0, 0, 1, -50);
		single(a);
		vector<point4d> aaaa = Poin;
		idofs << "WC" << " ";
		for (int i = 0; i != a.size(); ++i)
		{
			idofs << a[i] << " ";
		}
		idofs << "\n";
		a.clear();
		a = Function(0, 0, 1, 50);
		Poin.insert(Poin.end(), aaaa.begin(), aaaa.end());
		single(a);
		idofs << "WF" << " ";
		for (int i = 0; i != a.size(); ++i)
		{
			idofs << a[i] << " ";
		}
		idofs << "\n";
	}

	return Poin;
}

void  YN::WingStruct::OutputParaVolumeDataVTK(string filename)
{
	vector<Cube *> m_HexVolumes;
	m_HexVolumes = bordFirstSearch();
	ofstream file; //以输出方式打开文件
	file.open(filename);
	
	int uSegnum, vSegnum, wSegnum, patchPtNumCount, patchNumCount;
	uSegnum = vSegnum = wSegnum = 10;
	patchPtNumCount = (uSegnum + 1)*(vSegnum + 1)*(wSegnum + 1);  //点的数目
	int patchNum = m_HexVolumes.size();
	point4d pt;
	int started = 0;

	file << "# vtk DataFile Version 2.0" << "\n" << "TET" << "\n" << "ASCII" << "\n";
	file << "\n" << "DATASET UNSTRUCTURED_GRID" << "\n";
	file << "POINTS " << patchNum * patchPtNumCount << " float" << "\n";

	for (int patID = 0; patID < patchNum; patID++)
	{
		for (int k = 0; k <= wSegnum; k++)
		{
			for (int j = 0; j <= vSegnum; j++)
			{
				for (int i = 0; i <= uSegnum; i++)
				{
					pt = m_HexVolumes[patID]->_Vol_Data.GetVolPoint(i*1.f / uSegnum, j*1.f / vSegnum, k*1.f / wSegnum);
					file << pt.x << " " << pt.y << " " << pt.z << " " << "\n";
				}
			}
		}
	}

	patchNumCount = uSegnum * vSegnum*wSegnum;   //单元数目。
	file << "CELLS " << patchNum * patchNumCount << " " << 9 * patchNum * patchNumCount << "\n";
	for (int patID = 0; patID < patchNum; patID++)
	{
		started = patID * patchPtNumCount;
		for (int k = 0; k < wSegnum; k++)
		{
			for (int j = 0; j < vSegnum; j++)
			{
				for (int i = 0; i < uSegnum; i++)
				{
					file << "8 " << started + k * (uSegnum + 1)*(vSegnum + 1) + (uSegnum + 1)*j + i << " "
						<< started + k * (uSegnum + 1)*(vSegnum + 1) + (uSegnum + 1)*j + i + 1 << " "
						<< started + k * (uSegnum + 1)*(vSegnum + 1) + (uSegnum + 1)*(j + 1) + i + 1 << " "
						<< started + k * (uSegnum + 1)*(vSegnum + 1) + (uSegnum + 1)*(j + 1) + i << " "
						<< started + (k + 1)*(uSegnum + 1)*(vSegnum + 1) + (uSegnum + 1)*j + i << " "
						<< started + (k + 1)*(uSegnum + 1)*(vSegnum + 1) + (uSegnum + 1)*j + i + 1 << " "
						<< started + (k + 1)*(uSegnum + 1)*(vSegnum + 1) + (uSegnum + 1)*(j + 1) + i + 1 << " "
						<< started + (k + 1)*(uSegnum + 1)*(vSegnum + 1) + (uSegnum + 1)*(j + 1) + i << "\n";
				}
			}
		}
	}

	file << "CELL_TYPES " << patchNum * patchNumCount << "\n";
	for (int l = 0; l < patchNum * patchNumCount; l++)
	{
		file << "12" << "\n";
	}
	file.close();
}

void YN::WingStruct::abstractInfo(vector<Cube *>& m_HexVolumes)
{
	vector<Surface *> m_all4sidePatch;//所有面片的集合
	//提取几何信息
	m_all4sidePatch = upDateSurface(m_HexVolumes);//面的更新

	//提取拓扑信息
	//浪费时间!速度太慢
	setTwinPatch(m_HexVolumes, m_all4sidePatch);

	//提取参数域信息,要求所有的片UVW方向的控制点个数全部相同才可以进行
	Order(m_HexVolumes);

	//设置根节点
	_root = m_HexVolumes[0];

	//设置全局控制点序号
	setGloblControlPointId();
}

bool YN::WingStruct::setTwinPatch(vector<Cube *>& m_HexVolumes, vector<Surface *>& m_all4sidePatch)
{
	int patchnum = m_all4sidePatch.size();
	if (patchnum < 2)
		return false;
	for (int i = 0; i < patchnum; i++)
	{
		m_all4sidePatch.at(i)->_haveNextSuf = false;
		m_all4sidePatch.at(i)->_nextSurface = nullptr;
		m_all4sidePatch.at(i)->_pointOther = nullptr;
	}
	vector<int> idxarr;
	for (int i = 0; i < patchnum - 1; i++)
	{
		idxarr.clear();
		for (int j = i + 1; j < patchnum; j++)
		{
			if (m_all4sidePatch.at(j)->_haveNextSuf != false)
				continue;
			if (m_all4sidePatch.at(i)->_surfaceData.isTwoSurfaceSame(m_all4sidePatch.at(j)->_surfaceData))
			{
				idxarr.push_back(j);
				break;
			}
		}
		if (idxarr.size() < 1)
			continue;
		int minidx = idxarr.at(0);
		if (idxarr.size() > 1)  //找到的面不止一个？目前是有问题的。如果找到的面片不止一个，就找相对距离最小的面。
		{
			float minlen = m_all4sidePatch.at(i)->_surfaceData.GetDistanceBetweenTwoSurface(m_all4sidePatch.at(idxarr.at(0))->_surfaceData);
			for (int k = 1; k < idxarr.size(); k++)
			{
				float templen = m_all4sidePatch.at(i)->_surfaceData.GetDistanceBetweenTwoSurface(m_all4sidePatch.at(idxarr.at(k))->_surfaceData);
				if (templen < minlen)
				{
					minidx = idxarr.at(k);
					minlen = templen;
				}
			}
		}
		m_all4sidePatch.at(i)->_haveNextSuf = true;
		m_all4sidePatch.at(i)->_nextSurface = m_all4sidePatch.at(minidx);
		m_all4sidePatch.at(i)->_pointOther = m_all4sidePatch.at(minidx)->_pointThis;
		m_all4sidePatch.at(minidx)->_haveNextSuf = true;
		m_all4sidePatch.at(minidx)->_nextSurface = m_all4sidePatch.at(i);
		m_all4sidePatch.at(minidx)->_pointOther = m_all4sidePatch.at(i)->_pointThis;
	}
	return true;
}

vector<Cube *> YN::WingStruct::transNurbsVoltoCube(vector<NurbsVol> nurbsVols)
{
	vector<Cube *> m_HexVolumes;
	for (int i = 0; i != nurbsVols.size(); ++i)
	{
		Cube * c = new Cube;
		m_HexVolumes.push_back(c);
	}
	for (int i = 0; i != nurbsVols.size(); i++)
	{
		//u节点向量
		for (int j = 0; j != nurbsVols.at(i)._u_Knots->size(); ++j)
		{
			m_HexVolumes.at(i)->_Vol_Data._v_Knots->push_back(nurbsVols.at(i)._u_Knots->at(j));
		}
		//v节点向量
		for (int j = 0; j != nurbsVols.at(i)._v_Knots->size(); ++j)
		{
			m_HexVolumes.at(i)->_Vol_Data._u_Knots->push_back(nurbsVols.at(i)._v_Knots->at(j));
		}
		//w节点向量
		for (int j = 0; j != nurbsVols.at(i)._w_Knots->size(); ++j)
		{
			m_HexVolumes.at(i)->_Vol_Data._w_Knots->push_back(nurbsVols.at(i)._w_Knots->at(j));
		}
		//uvw阶数
		m_HexVolumes.at(i)->_Vol_Data._v_Degree = nurbsVols.at(i)._u_Degree;
		m_HexVolumes.at(i)->_Vol_Data._u_Degree = nurbsVols.at(i)._v_Degree;
		m_HexVolumes.at(i)->_Vol_Data._w_Degree = nurbsVols.at(i)._w_Degree;

		m_HexVolumes.at(i)->_Vol_Data._v_Num = nurbsVols.at(i)._u_Num;
		m_HexVolumes.at(i)->_Vol_Data._u_Num = nurbsVols.at(i)._v_Num;
		m_HexVolumes.at(i)->_Vol_Data._w_Num = nurbsVols.at(i)._w_Num;

		//控制点
		for (int j = 0; j != nurbsVols.at(i)._ControlPts->size(); ++j)
		{
			point3d v;
			v.x = nurbsVols.at(i)._ControlPts->at(j).x;
			v.y = nurbsVols.at(i)._ControlPts->at(j).y;
			v.z = nurbsVols.at(i)._ControlPts->at(j).z;
			m_HexVolumes.at(i)->_Vol_Data._ControlPts->push_back(v);
		}
	}
	return m_HexVolumes;
}

vector<Surface *> YN::WingStruct::upDateCubeSurface(Cube * cube)
{
	vector<Surface *> m_all4sidePatch;
	//u0v0面
	//如果第一次使用首先创建 new
	if (cube->_u0v0== NULL)
	{
		cube->_u0v0 = new Surface;
	}
	else
	{
		cube->_u0v0->_surfaceData._ControlPts->clear();
	}
	cube->_u0v0->_surfaceData._v_Knots = cube->_Vol_Data._v_Knots;
	cube->_u0v0->_surfaceData._u_Knots = cube->_Vol_Data._u_Knots;
	cube->_u0v0->_surfaceData._u_Num = cube->_Vol_Data._u_Num;
	cube->_u0v0->_surfaceData._v_Num = cube->_Vol_Data._v_Num;
	cube->_u0v0->_surfaceData._u_Degree = cube->_Vol_Data._u_Degree;
	cube->_u0v0->_surfaceData._v_Degree = cube->_Vol_Data._v_Degree;
	//u0v0的控制点
	for (int j = 0; j != cube->_Vol_Data._v_Num; ++j)
	{
		for (int k = 0; k != cube->_Vol_Data._u_Num; ++k)
		{
			cube->_u0v0->_surfaceData._ControlPts->push_back(cube->_Vol_Data.getControlPoint(k, j, 0));
		}
	}
	m_all4sidePatch.push_back(cube->_u0v0);
	cube->_u0v0->_pointThis= cube;
	cube->_u0v0->_pointThisNumber = u0v0;

	//u1v1面
	if (cube->_u1v1 == NULL)
	{
		cube->_u1v1 = new Surface;
	}
	else
	{
		cube->_u1v1->_surfaceData._ControlPts->clear();
	}
	cube->_u1v1->_surfaceData._v_Knots = cube->_Vol_Data._v_Knots;
	cube->_u1v1->_surfaceData._u_Knots = cube->_Vol_Data._u_Knots;
	cube->_u1v1->_surfaceData._u_Num = cube->_Vol_Data._u_Num;
	cube->_u1v1->_surfaceData._v_Num = cube->_Vol_Data._v_Num;
	cube->_u1v1->_surfaceData._u_Degree = cube->_Vol_Data._u_Degree;
	cube->_u1v1->_surfaceData._v_Degree = cube->_Vol_Data._v_Degree;
	for (int j = 0; j != cube->_Vol_Data._v_Num; ++j)
	{
		for (int k = 0; k != cube->_Vol_Data._u_Num; ++k)
		{
			cube->_u1v1->_surfaceData._ControlPts->push_back(cube->_Vol_Data.getControlPoint(k, j, cube->_Vol_Data._w_Num - 1));
		}
	}
	m_all4sidePatch.push_back(cube->_u1v1);
	cube->_u1v1->_pointThis = cube;
	cube->_u1v1->_pointThisNumber = u1v1;

	//v0w0面
	if (cube->_v0w0 == NULL)
	{
		cube->_v0w0 = new Surface;
	}
	else
	{
		cube->_v0w0->_surfaceData._ControlPts->clear();
	}
	cube->_v0w0->_surfaceData._u_Knots = cube->_Vol_Data._v_Knots;
	cube->_v0w0->_surfaceData._v_Knots = cube->_Vol_Data._w_Knots;
	cube->_v0w0->_surfaceData._u_Num = cube->_Vol_Data._v_Num;
	cube->_v0w0->_surfaceData._v_Num = cube->_Vol_Data._w_Num;
	cube->_v0w0->_surfaceData._u_Degree = cube->_Vol_Data._v_Degree;
	cube->_v0w0->_surfaceData._v_Degree = cube->_Vol_Data._w_Degree;
	//v0w0的控制点
	for (int j = 0; j != cube->_Vol_Data._w_Num; ++j)
	{
		for (int k = 0; k != cube->_Vol_Data._v_Num; ++k)
		{
			cube->_v0w0->_surfaceData._ControlPts->push_back(cube->_Vol_Data.getControlPoint(0, k, j));
		}
	}
	m_all4sidePatch.push_back(cube->_v0w0);
	cube->_v0w0->_pointThis = cube;
	cube->_v0w0->_pointThisNumber = v0w0;

	//v1w1面
	if (cube->_v1w1 == NULL)
	{
		cube->_v1w1 = new Surface;
	}
	else
	{
		cube->_v1w1->_surfaceData._ControlPts->clear();
	}
	cube->_v1w1->_surfaceData._u_Knots = cube->_Vol_Data._v_Knots;
	cube->_v1w1->_surfaceData._v_Knots = cube->_Vol_Data._w_Knots;
	cube->_v1w1->_surfaceData._u_Num = cube->_Vol_Data._v_Num;
	cube->_v1w1->_surfaceData._v_Num = cube->_Vol_Data._w_Num;
	cube->_v1w1->_surfaceData._u_Degree = cube->_Vol_Data._v_Degree;
	cube->_v1w1->_surfaceData._v_Degree = cube->_Vol_Data._w_Degree;
	for (int j = 0; j != cube->_Vol_Data._w_Num; ++j)
	{
		for (int k = 0; k != cube->_Vol_Data._v_Num; ++k)
		{
			cube->_v1w1->_surfaceData._ControlPts->push_back(cube->_Vol_Data.getControlPoint(cube->_Vol_Data._u_Num - 1, k, j));
		}
	}
	m_all4sidePatch.push_back(cube->_v1w1);
	cube->_v1w1->_pointThis = cube;
	cube->_v1w1->_pointThisNumber = v1w1;

	//u0w0面
	if (cube->_u0w0 == NULL)
	{
		cube->_u0w0 = new Surface;
	}
	else
	{
		cube->_u0w0->_surfaceData._ControlPts->clear();
	}
	cube->_u0w0->_surfaceData._u_Knots = cube->_Vol_Data._u_Knots;
	cube->_u0w0->_surfaceData._v_Knots = cube->_Vol_Data._w_Knots;
	cube->_u0w0->_surfaceData._u_Num = cube->_Vol_Data._u_Num;
	cube->_u0w0->_surfaceData._v_Num = cube->_Vol_Data._w_Num;
	cube->_u0w0->_surfaceData._u_Degree = cube->_Vol_Data._u_Degree;
	cube->_u0w0->_surfaceData._v_Degree = cube->_Vol_Data._w_Degree;
	for (int j = 0; j != cube->_Vol_Data._w_Num; ++j)
	{
		for (int k = 0; k != cube->_Vol_Data._u_Num; ++k)
		{
			cube->_u0w0->_surfaceData._ControlPts->push_back(cube->_Vol_Data.getControlPoint(k, 0, j));
		}
	}
	m_all4sidePatch.push_back(cube->_u0w0);
	cube->_u0w0->_pointThis = cube;
	cube->_u0w0->_pointThisNumber = u0w0;

	//u1w1面
	if (cube->_u1w1 == NULL)
	{
		cube->_u1w1 = new Surface;
	}
	else
	{
		cube->_u1w1->_surfaceData._ControlPts->clear();
	}
	cube->_u1w1->_surfaceData._u_Knots = cube->_Vol_Data._u_Knots;
	cube->_u1w1->_surfaceData._v_Knots = cube->_Vol_Data._w_Knots;
	cube->_u1w1->_surfaceData._u_Num = cube->_Vol_Data._u_Num;
	cube->_u1w1->_surfaceData._v_Num = cube->_Vol_Data._w_Num;
	cube->_u1w1->_surfaceData._u_Degree = cube->_Vol_Data._u_Degree;
	cube->_u1w1->_surfaceData._v_Degree = cube->_Vol_Data._w_Degree;
	for (int j = 0; j != cube->_Vol_Data._w_Num; ++j)
	{
		for (int k = 0; k != cube->_Vol_Data._u_Num; ++k)
		{
			cube->_u1w1->_surfaceData._ControlPts->push_back(cube->_Vol_Data.getControlPoint(k, cube->_Vol_Data._v_Num - 1, j));
		}
	}
	m_all4sidePatch.push_back(cube->_u1w1);
	cube->_u1w1->_pointThis = cube;
	cube->_u1w1->_pointThisNumber = u1w1;

	return m_all4sidePatch;
}

vector<Surface *> YN::WingStruct::upDateSurface(vector<Cube *>& m_HexVolumes)
{
	//更新面信息
	vector<Surface *> m_all4sidePatch;
	for (int i = 0; i != m_HexVolumes.size(); ++i)
	{
		vector<Surface *> temp = upDateCubeSurface(m_HexVolumes[i]);
		m_all4sidePatch.insert(m_all4sidePatch.end(),temp.begin(),temp.end());
	}
	return m_all4sidePatch;
}

bool YN::WingStruct::Order(vector<Cube *>& m_HexVolumes)
{
	if (m_HexVolumes.empty())
	{
		return false;
	}
	std::list<Cube *> List;
	m_HexVolumes[0]->_isOrdered = true;
	List.push_back(m_HexVolumes[0]);
	while (!List.empty())
	{
		Cube* a = List.front();
		List.pop_front();
		//u0v0
		if (a->_u0v0->_pointOther != NULL && a->_u0v0->_pointOther->_isOrdered != true)
		{
			setNearCubeWithSameUVW(a, a->_u0v0->_pointOther, u0v0);
			a->_u0v0->_pointOther->_isOrdered = true;
			List.push_back(a->_u0v0->_pointOther);
		}
		//u1v1
		if (a->_u1v1->_pointOther != NULL && a->_u1v1->_pointOther->_isOrdered != true)
		{
			setNearCubeWithSameUVW(a, a->_u1v1->_pointOther, u1v1);
			a->_u1v1->_pointOther->_isOrdered = true;
			List.push_back(a->_u1v1->_pointOther);
		}
		//v0w0
		if (a->_v0w0->_pointOther != NULL && a->_v0w0->_pointOther->_isOrdered != true)
		{
			setNearCubeWithSameUVW(a, a->_v0w0->_pointOther, v0w0);
			a->_v0w0->_pointOther->_isOrdered = true;
			List.push_back(a->_v0w0->_pointOther);
		}
		//v1w1
		if (a->_v1w1->_pointOther != NULL && a->_v1w1->_pointOther->_isOrdered != true)
		{
			setNearCubeWithSameUVW(a, a->_v1w1->_pointOther, v1w1);
			a->_v1w1->_pointOther->_isOrdered = true;
			List.push_back(a->_v1w1->_pointOther);
		}
		//u0w0
		if (a->_u0w0->_pointOther != NULL && a->_u0w0->_pointOther->_isOrdered != true)
		{
			setNearCubeWithSameUVW(a, a->_u0w0->_pointOther, u0w0);
			a->_u0w0->_pointOther->_isOrdered = true;
			List.push_back(a->_u0w0->_pointOther);
		}
		//u1w1
		if (a->_u1w1->_pointOther != NULL && a->_u1w1->_pointOther->_isOrdered != true)
		{
			setNearCubeWithSameUVW(a, a->_u1w1->_pointOther, u1w1);
			a->_u1w1->_pointOther->_isOrdered = true;
			List.push_back(a->_u1w1->_pointOther);
		}
	}
	//检查所有体是否都经过排序,存在没有排序的片，返回false
	for (auto iter = m_HexVolumes.begin(); iter != m_HexVolumes.end(); ++iter)
	{
		if ((*iter)->_isOrdered == false)
		{
			return false;
		}
	}
	return true;
}

bool YN::WingStruct::setNearCubeWithSameUVW(Cube * StandarHexCell, Cube *WaitHexCell,int index)
{
	if (StandarHexCell == NULL || WaitHexCell == NULL || index < 0 || index>5)
	{
		return false;
	}
	Surface temp,temp2;
	vector<vector<int>> continor, continor2;
	vector<int> IndxOrder, IndxOrder_old, IndxOrder_orient, FinalOrder;
	vector<point4d> tt, tt1, tt2, tt3;
	int PatchIndx_r;

	//设置两个临时向量temp，temp2方便管理
	switch (index)
	{
	case u0v0:
		temp = *StandarHexCell->_u0v0;
		temp2 = *StandarHexCell->_u0v0->_nextSurface;
		PatchIndx_r = 1;
		break;
	case u1v1:
		temp = *StandarHexCell->_u1v1;
		temp2 = *StandarHexCell->_u1v1->_nextSurface;
		PatchIndx_r = 0;
		break;
	case v0w0:
		temp = *StandarHexCell->_v0w0;
		temp2 = *StandarHexCell->_v0w0->_nextSurface;
		PatchIndx_r = 3;
		break;
	case v1w1:
		temp = *StandarHexCell->_v1w1;
		temp2 = *StandarHexCell->_v1w1->_nextSurface;
		PatchIndx_r = 2;
		break;
	case u0w0:
		temp = *StandarHexCell->_u0w0;
		temp2 = *StandarHexCell->_u0w0->_nextSurface;
		PatchIndx_r = 5;
		break;
	case u1w1:
		temp = *StandarHexCell->_u1w1;
		temp2 = *StandarHexCell->_u1w1->_nextSurface;
		PatchIndx_r = 4;
		break;
	default:
		break;
	}

	//控制点对应的体模型内编号
	auto findNumber = [](vector<point4d> tt, Cube * cell)
	{
		vector<int> order;
		for (int i = 0; i != tt.size(); ++i)
		{
			for (int j = 0; j != cell->_Vol_Data._ControlPts->size(); ++j)
			{
				if (abs(tt[i].x - cell->_Vol_Data._ControlPts->at(j).x) < 0.001
					&&abs(tt[i].y - cell->_Vol_Data._ControlPts->at(j).y) < 0.001
					&&abs(tt[i].z - cell->_Vol_Data._ControlPts->at(j).z) < 0.001)
				{
					order.push_back(j);
				}
			}
		}
		return order;
	};
	
	//获取finalorder
	auto GetFinalOrder = [&]()
	{
		tt1 = *temp._surfaceData._ControlPts;
		//在待排序的体内寻找到这些控制点,得到一个旧控制点下标
		IndxOrder_old = findNumber(tt1, WaitHexCell);
		

		tt2 = *temp2._surfaceData._ControlPts;
		IndxOrder_orient = findNumber(tt2, WaitHexCell);

		if (tt1.size() != IndxOrder_old.size() || tt2.size() != IndxOrder_orient.size())
		{
			int a = 0;
			a++;
		}

		//finalorder的计算
		for (int i = 0; i != IndxOrder_orient.size(); ++i)
		{
			for (int j = 0; j != IndxOrder_old.size(); ++j)
			{
				if (IndxOrder_old[j] == IndxOrder_orient[i])
				{
					FinalOrder.push_back(j);
					break;
				}
			}
		}
	};

	//片内其余所有控制点提取，x为识别序号
	auto MySwitch = [&](int x, Cube * b)
	{
		vector<vector<int>> c;
		switch (x)
		{

		case u0v0://uv0方向
			for (int i = b->_Vol_Data._w_Num-1; i != -1; i--)
			{
				for (int j = 0; j != b->_Vol_Data._v_Num; ++j)
				{
					for (int k = 0; k != b->_Vol_Data._u_Num; ++k)
					{
						tt3.push_back(b->_Vol_Data.getControlPoint(k, j, i));
					}
				}
				c.push_back(findNumber(tt3, b));
				tt3.clear();
			};
			break;
		case u1v1://uv1方向
			for (int i = 0; i != b->_Vol_Data._w_Num; i++)
			{
				for (int j = 0; j != b->_Vol_Data._v_Num; ++j)
				{
					for (int k = 0; k != b->_Vol_Data._u_Num; ++k)
					{
						tt3.push_back(b->_Vol_Data.getControlPoint(k, j, i));
					}
				}
				c.push_back(findNumber(tt3, b));
				tt3.clear();
			};
			break;
		case v0w0://vw0方向
			for (int i = b->_Vol_Data._u_Num - 1; i != -1; i--)
			{
				for (int j = 0; j != b->_Vol_Data._w_Num; ++j)
				{
					for (int k = 0; k != b->_Vol_Data._v_Num; ++k)
					{
						tt3.push_back(b->_Vol_Data.getControlPoint(i, k, j));
					}
				}
				c.push_back(findNumber(tt3, b));
				tt3.clear();
			};
			break;
		case v1w1://vw1方向
			for (int i = 0; i != b->_Vol_Data._u_Num; i++)
			{
				for (int j = 0; j != b->_Vol_Data._w_Num; ++j)
				{
					for (int k = 0; k != b->_Vol_Data._v_Num; ++k)
					{
						tt3.push_back(b->_Vol_Data.getControlPoint(i, k, j));
					}
				}
				c.push_back(findNumber(tt3, b));
				tt3.clear();
			};
			break;
		case u0w0://uw0方向
			for (int i = b->_Vol_Data._v_Num - 1; i != -1; i--)
			{
				for (int j = 0; j != b->_Vol_Data._w_Num; ++j)
				{
					for (int k = 0; k != b->_Vol_Data._u_Num; ++k)
					{
						tt3.push_back(b->_Vol_Data.getControlPoint(k, i, j));
					}
				}
				c.push_back(findNumber(tt3, b));
				tt3.clear();
			};
			break;
		case u1w1://uw1方向
			for (int i = 0; i != b->_Vol_Data._v_Num; i++)
			{
				for (int j = 0; j != b->_Vol_Data._w_Num; ++j)
				{
					for (int k = 0; k != b->_Vol_Data._u_Num; ++k)
					{
						tt3.push_back(b->_Vol_Data.getControlPoint(k, i, j));
					}
				}
				c.push_back(findNumber(tt3, b));
				tt3.clear();
			};
			break;
		default:
			break;
		}
		return c;
	};

	//获取辅助排序数组fianlOreder
	GetFinalOrder();
	//提取出待排序数组
	continor = MySwitch(PatchIndx_r, StandarHexCell);
	continor2 = MySwitch(temp2._pointThisNumber, WaitHexCell);

	//然后根据finalOrder进行一次排序
	for (int i = 0; i != continor2.size(); ++i)
	{
		vector<int> c;
		c.resize(continor2[i].size());
		for (int j = 0; j != continor2[i].size(); ++j)
		{
			c[FinalOrder[j]] = continor2[i][j];
			//c.push_back(continor2[i][FinalOrder[j]]);
		}
		continor2[i] = c;
	}

	//待排序的体模型进行排序
	vector<point4d> replace;
	replace.resize(WaitHexCell->_Vol_Data._ControlPts->size());
	for (int i = 0; i != continor.size(); ++i)
	{
		for (int j = 0; j != continor[i].size(); ++j)
		{
			replace[continor[i][j]] = (*WaitHexCell->_Vol_Data._ControlPts)[continor2[i][j]];
		}
	}
	//完成替换
	WaitHexCell->_Vol_Data._ControlPts->clear();
	for (auto iter = replace.begin(); iter != replace.end(); ++iter)
	{
		WaitHexCell->_Vol_Data._ControlPts->push_back(*iter);
	}

	//完成替换之后需要把原先的数据进行更新
	//首先更新面

	upDateCubeSurface(WaitHexCell);

	//面更新完之后，面的pointOther指针需要更新

	vector<Surface *> tempSurface;
	vector<Surface *> cubeSurface;
	auto getCubeSurfaces = [&](Surface *surface)
	{
		cubeSurface.push_back(surface);
		if (surface->_nextSurface != NULL)
		{
			tempSurface.push_back(surface->_nextSurface);
			surface->_haveNextSuf = false;
			surface->_pointOther = NULL;
			surface->_nextSurface = NULL;
		}
	};

	getCubeSurfaces(WaitHexCell->_u0v0);
	getCubeSurfaces(WaitHexCell->_u1v1);
	getCubeSurfaces(WaitHexCell->_u0w0);
	getCubeSurfaces(WaitHexCell->_u1w1);
	getCubeSurfaces(WaitHexCell->_v0w0);
	getCubeSurfaces(WaitHexCell->_v1w1);

	for (auto iter = tempSurface.begin(); iter != tempSurface.end(); ++iter)
	{
		for (auto iter1 = cubeSurface.begin(); iter1 != cubeSurface.end(); ++iter1)
		{
			if ((*iter1)->_surfaceData.isTwoSurfaceSame((*iter)->_surfaceData))
			{
				(*iter1)->_haveNextSuf = true;
				(*iter1)->_nextSurface = (*iter);
				(*iter1)->_pointOther = (*iter)->_pointThis;
				break;
			}
		}
	}

	return true;
}

void YN::WingStruct::setGloblControlPointId()
{
	//函数先设置全局控制点序号，之后返回
	int idx = 0;

	//获取控制点所在序号
	auto getIndex = [&](int u,int v,int w,int _u_Num,int _v_Num)
	{
		return u + v * _u_Num + w * (_u_Num*_v_Num);
	};
	//输入一个经有全局控制点的片，设置面的全局控制点序号
	auto setSurfaceGolblControlPointID = [&](Cube * cube)
	{
		int u_Num = cube->_Vol_Data._u_Num;
		int v_Num = cube->_Vol_Data._v_Num;
		int w_Num = cube->_Vol_Data._w_Num;
		int index;
		//u0v0的序号
		for (int j = 0; j != cube->_Vol_Data._v_Num; ++j)
		{
			for (int k = 0; k != cube->_Vol_Data._u_Num; ++k)
			{
				index=cube->_volGloblControlPointID.at(getIndex(k,j,0,u_Num, v_Num));
				cube->_u0v0->_surfaceGloblControlPointID.push_back(index);
			}
		}
		//u1v1的序号
		for (int j = 0; j != cube->_Vol_Data._v_Num; ++j)
		{
			for (int k = 0; k != cube->_Vol_Data._u_Num; ++k)
			{
				index = cube->_volGloblControlPointID.at(getIndex(k, j, w_Num - 1, u_Num, v_Num));
				cube->_u1v1->_surfaceGloblControlPointID.push_back(index);
			}
		}
		//v0w0的序号
		for (int j = 0; j != cube->_Vol_Data._w_Num; ++j)
		{
			for (int k = 0; k != cube->_Vol_Data._v_Num; ++k)
			{
				index = cube->_volGloblControlPointID.at(getIndex(0, k, j, u_Num, v_Num));
				cube->_v0w0->_surfaceGloblControlPointID.push_back(index);
			}
		}
		//v1w1的序号
		for (int j = 0; j != cube->_Vol_Data._w_Num; ++j)
		{
			for (int k = 0; k != cube->_Vol_Data._v_Num; ++k)
			{
				index = cube->_volGloblControlPointID.at(getIndex(u_Num - 1, k, j, u_Num, v_Num));
				cube->_v1w1->_surfaceGloblControlPointID.push_back(index);
			}
		}
		//u0w0的序号
		for (int j = 0; j != cube->_Vol_Data._w_Num; ++j)
		{
			for (int k = 0; k != cube->_Vol_Data._u_Num; ++k)
			{
				index = cube->_volGloblControlPointID.at(getIndex(k, 0, j, u_Num, v_Num));
				cube->_u0w0->_surfaceGloblControlPointID.push_back(index);
			}
		}
		//u1w1的序号
		for (int j = 0; j != cube->_Vol_Data._w_Num; ++j)
		{
			for (int k = 0; k != cube->_Vol_Data._u_Num; ++k)
			{
				index = cube->_volGloblControlPointID.at(getIndex(k, v_Num - 1, j, u_Num, v_Num));
				cube->_u1w1->_surfaceGloblControlPointID.push_back(index);
			}
		}
	};
	//将相邻面控制点全局序号放入cube中
	auto setControlPointID = [&](Surface * surface,Cube * cube)
	{
		if (surface->_haveNextSuf &&
			!surface->_nextSurface->_surfaceGloblControlPointID.empty())
		{
			for (int i = 0; i != surface->_nextSurface->_surfaceData._ControlPts->size(); ++i)
			{
				for (int j = 0; j != cube->_Vol_Data._ControlPts->size(); ++j)
				{
					if ((*surface->_nextSurface->_surfaceData._ControlPts)[i] ==
						(*cube->_Vol_Data._ControlPts)[j])
					{
						cube->_volGloblControlPointID[j] = surface->_nextSurface->_surfaceGloblControlPointID.at(i);
					}
				}
			}
		}
	};
	//输入一个片，将片内控制点全局编号
	auto setVolGloblControlPointID = [&](Cube * cube)
	{
		cube->_volGloblControlPointID.resize(cube->_Vol_Data._u_Num*cube->_Vol_Data._v_Num*cube->_Vol_Data._w_Num);
		std::fill(cube->_volGloblControlPointID.begin(), cube->_volGloblControlPointID.end(), -1);
		setControlPointID(cube->_u0v0, cube);
		setControlPointID(cube->_u1v1, cube);
		setControlPointID(cube->_v0w0, cube);
		setControlPointID(cube->_v1w1, cube);
		setControlPointID(cube->_u0w0, cube);
		setControlPointID(cube->_u1w1, cube);
		for (auto iter = cube->_volGloblControlPointID.begin(); iter != cube->_volGloblControlPointID.end(); ++iter)
		{
			if (*iter == -1)
			{
				*iter = idx++;
			}
		}
	};

	//生成全局控制点
	std::list<Cube *> List;
	List.push_back(_root);
	_root->_isOrdered = !_root->_isOrdered;
	while (!List.empty())
	{
		Cube * a = List.front();
		List.pop_front();
		setVolGloblControlPointID(a);
		setSurfaceGolblControlPointID(a);
		if (a->_u0v0->_pointOther != NULL && a->_u0v0->_pointOther->_isOrdered != a->_isOrdered)
		{
			List.push_back(a->_u0v0->_pointOther);
			a->_u0v0->_pointOther->_isOrdered = !a->_u0v0->_pointOther->_isOrdered;
		}	
		if (a->_u1v1->_pointOther != NULL && a->_u1v1->_pointOther->_isOrdered != a->_isOrdered)
		{
			List.push_back(a->_u1v1->_pointOther);
			a->_u1v1->_pointOther->_isOrdered = !a->_u1v1->_pointOther->_isOrdered;
		}	
		if (a->_v0w0->_pointOther != NULL && a->_v0w0->_pointOther->_isOrdered != a->_isOrdered)
		{
			List.push_back(a->_v0w0->_pointOther);
			a->_v0w0->_pointOther->_isOrdered = !a->_v0w0->_pointOther->_isOrdered;
		}
		if (a->_v1w1->_pointOther != NULL && a->_v1w1->_pointOther->_isOrdered != a->_isOrdered)
		{
			List.push_back(a->_v1w1->_pointOther);
			a->_v1w1->_pointOther->_isOrdered = !a->_v1w1->_pointOther->_isOrdered;
		}
		if (a->_u0w0->_pointOther != NULL && a->_u0w0->_pointOther->_isOrdered != a->_isOrdered)
		{
			List.push_back(a->_u0w0->_pointOther);
			a->_u0w0->_pointOther->_isOrdered = !a->_u0w0->_pointOther->_isOrdered;
		}		
		if (a->_u1w1->_pointOther != NULL && a->_u1w1->_pointOther->_isOrdered != a->_isOrdered)
		{
			List.push_back(a->_u1w1->_pointOther);
			a->_u1w1->_pointOther->_isOrdered = !a->_u1w1->_pointOther->_isOrdered;
		}
	}

}

vector<Cube *> YN::WingStruct::deepFirstSearch(Cube * startCube, int index)
{
	vector<Cube *> temp;
	switch (index)
	{
	case u0v0:
		while (startCube!= NULL)
		{
			temp.push_back(startCube);
			startCube = startCube->_u0v0->_pointOther;
		}
		break;
	case u1v1:
		while (startCube != NULL)
		{
			temp.push_back(startCube);
			startCube = startCube->_u1v1->_pointOther;
		}
		break;
	case v0w0:
		while (startCube != NULL)
		{
			temp.push_back(startCube);
			startCube = startCube->_v0w0->_pointOther;
		}
		break;
	case v1w1:
		while (startCube != NULL)
		{
			temp.push_back(startCube);
			startCube = startCube->_v1w1->_pointOther;
		}
		break;
	case u0w0:
		while (startCube != NULL)
		{
			temp.push_back(startCube);
			startCube = startCube->_u0w0->_pointOther;
		}
		break;
	case u1w1:
		while (startCube != NULL)
		{
			temp.push_back(startCube);
			startCube = startCube->_u1w1->_pointOther;
		}
		break;
	default:
		break;
	}

	return temp;
}

vector<Cube *> YN::WingStruct::bordFirstSearch()
{
	//每次遍历之前所有片的_bordFirstSearch都是相同的,
	//把第一片_bordFirstSearch设置为不相同的标记，方便遍历使用
	_root->_bordFirstSearch = !_root->_bordFirstSearch;
	vector<Cube *> re;
	std::list<Cube *> List;
	List.push_back(_root);
	
	while (!List.empty())
	{
		Cube * temp = List.front();
		List.pop_front();
		re.push_back(temp);
		if (temp->_u0v0->_pointOther!=NULL&&temp->_u0v0->_pointOther->_bordFirstSearch != temp->_bordFirstSearch)
		{
			temp->_u0v0->_pointOther->_bordFirstSearch = temp->_bordFirstSearch;
			List.push_back(temp->_u0v0->_pointOther);
		}
		if (temp->_u1v1->_pointOther != NULL && temp->_u1v1->_pointOther->_bordFirstSearch != temp->_bordFirstSearch)
		{
			temp->_u1v1->_pointOther->_bordFirstSearch = temp->_bordFirstSearch;
			List.push_back(temp->_u1v1->_pointOther);
		}
		if (temp->_u0w0->_pointOther != NULL && temp->_u0w0->_pointOther->_bordFirstSearch != temp->_bordFirstSearch)
		{
			temp->_u0w0->_pointOther->_bordFirstSearch = temp->_bordFirstSearch;
			List.push_back(temp->_u0w0->_pointOther);
		}
		if (temp->_u1w1->_pointOther != NULL && temp->_u1w1->_pointOther->_bordFirstSearch != temp->_bordFirstSearch)
		{
			temp->_u1w1->_pointOther->_bordFirstSearch = temp->_bordFirstSearch;
			List.push_back(temp->_u1w1->_pointOther);
		}
		if (temp->_v0w0->_pointOther != NULL && temp->_v0w0->_pointOther->_bordFirstSearch != temp->_bordFirstSearch)
		{
			temp->_v0w0->_pointOther->_bordFirstSearch = temp->_bordFirstSearch;
			List.push_back(temp->_v0w0->_pointOther);
		}
		if (temp->_v1w1->_pointOther != NULL && temp->_v1w1->_pointOther->_bordFirstSearch != temp->_bordFirstSearch)
		{
			temp->_v1w1->_pointOther->_bordFirstSearch = temp->_bordFirstSearch;
			List.push_back(temp->_v1w1->_pointOther);
		}
	}
	return re;
}

vector<NurbsSurface> YN::WingStruct::getModelSurface()
{
	vector<Cube *> a;
	vector<NurbsSurface> re;
	a = bordFirstSearch();
	for (auto iter = a.begin(); iter != a.end(); ++iter)
	{
		if ((*iter)->_u0v0->_haveNextSuf == false)
		{
			re.push_back((*iter)->_u0v0->_surfaceData);
		}
		if ((*iter)->_u1v1->_haveNextSuf == false)
		{
			re.push_back((*iter)->_u1v1->_surfaceData);
		}
		if ((*iter)->_v0w0->_haveNextSuf == false)
		{
			re.push_back((*iter)->_v0w0->_surfaceData);
		}
		if ((*iter)->_v1w1->_haveNextSuf == false)
		{
			re.push_back((*iter)->_v1w1->_surfaceData);
		}
		if ((*iter)->_u0w0->_haveNextSuf == false)
		{
			re.push_back((*iter)->_u0w0->_surfaceData);
		}
		if ((*iter)->_u1w1->_haveNextSuf == false)
		{
			re.push_back((*iter)->_u1w1->_surfaceData);
		}
	}
	return re;
}

bool YN::WingStruct::removeVol(point4d target)
{
	//通过快速搜索找到目标片
	vector<Cube *> a = quickSearch(target, _root);
	if (a.empty())
	{
		return false;
	}
	Cube * b = a.back();

	//删除面
	auto deleteSurface = [&](Surface * surface)
	{
		if (surface != NULL)
		{
			surface->_nextSurface->_pointOther = nullptr;
			surface->_nextSurface->_nextSurface = nullptr;
			surface->_nextSurface->_haveNextSuf = false;
			delete surface;
			surface = nullptr;
		}
	};
	//六个面删除
	if (b->_u0v0->_haveNextSuf)
	{
		deleteSurface(b->_u0v0);
	}
	if (b->_u1v1->_haveNextSuf)
	{
		deleteSurface(b->_u1v1);
	}
	if (b->_v0w0->_haveNextSuf)
	{
		deleteSurface(b->_v0w0);
	}
	if (b->_v1w1->_haveNextSuf)
	{
		deleteSurface(b->_v1w1);
	}
	if (b->_u0w0->_haveNextSuf)
	{
		deleteSurface(b->_u0w0);
	}
	if (b->_u1w1->_haveNextSuf)
	{
		deleteSurface(b->_u1w1);
	}
	//片删除
	delete b;
	b = nullptr;

	return true;
}

bool YN::WingStruct::addVolToSurface(point4d target, NurbsLine line, NurbsSurface surface)
{
	//快速搜索找到目标片
	vector<Cube *> a = quickSearch(target, _root);
	NurbsVol vol;
	Cube * b = a.back();
	int surfaceNumber = isPointInSurface(target, b);
	if (surfaceNumber == -1)
		return false;
	//放样函数构造nurbsvol
	switch (surfaceNumber)
	{
	case 0:
		vol.LoftingNurbsVol(line, surface, b->_u0v0->_surfaceData);
		break;
	case 1:
		vol.LoftingNurbsVol(line, surface, b->_u1v1->_surfaceData);
		break;
	case 2:
		vol.LoftingNurbsVol(line, surface, b->_v0w0->_surfaceData);
		break;
	case 3:
		vol.LoftingNurbsVol(line, surface, b->_v1w1->_surfaceData);
		break;
	case 4:
		vol.LoftingNurbsVol(line, surface, b->_u0w0->_surfaceData);
		break;
	case 5:
		vol.LoftingNurbsVol(line, surface, b->_u1w1->_surfaceData);
		break;
	default:
		break;
	}

	Cube * newCube = new Cube;
	newCube->_Vol_Data = vol;
	upDateCubeSurface(newCube);

	auto function = [&](Surface * surface)
	{
		surface->_haveNextSuf = true;
		surface->_pointOther = newCube;
		surface->_nextSurface = newCube->_u1v1;
		setNearCubeWithSameUVW(b, newCube, surfaceNumber);
	};

	switch (surfaceNumber)
	{
	case 0:
		function(b->_u0v0);
		break;
	case 1:
		function(b->_u1v1);
		break;
	case 2:
		function(b->_v0w0);
		break;
	case 3:
		function(b->_v1w1);
		break;
	case 4:
		function(b->_u0w0);
		break;
	case 5:
		function(b->_u1w1);
		break;
	default:
		break;
	}

	return true;
}

int YN::WingStruct::isPointInSurface(point4d target, Cube * cube)
{
	auto compare = [&](Surface * surface)
	{
		for (auto i = surface->_surfaceData._ControlPts->begin();
			i != surface->_surfaceData._ControlPts->end(); ++i)
		{
			if (*i == target)
				return surface->_pointThisNumber;
		}
		return -1;
	};
	
	int re;
	re = compare(cube->_u0v0);
	if (re != -1)
	{
		return re;
	}	
	
	re = compare(cube->_u1v1);
	if (re != -1)
	{
		return re;
	}
	
	re = compare(cube->_v0w0);
	if (re != -1)
	{
		return re;
	}

	re = compare(cube->_v1w1);
	if (re != -1)
	{
		return re;
	}

	re = compare(cube->_u0w0);
	if (re != -1)
	{
		return re;
	}

	re = compare(cube->_u1w1);
	if (re != -1)
	{
		return re;
	}

	return -1;
}

vector<Cube *> YN::WingStruct::quickSearch(point4d target, Cube * cubes)
{
	//目前的程序还不能达到logN的平均时间复杂度，仍需要进行改进
	//使用RTT随机搜索树的方法，但是在搜索过程中给出一个方向，防止漫无目的搜索
	//边简历搜索树边给出方向，当搜索到目标时，确定树的最大高度，如果出现更短的路径则更换较短的
	//高度，以此找出最短的路径

	struct Node;
	vector<Cube *> re;
	std::stack<Node *> Stack;

	//定义树节点
	struct Node
	{
		Node * father;
		Cube * Data;
		vector<Node *> childs;
	};

	//从根节点root开始
	//计算可能面,将可能面设置为子节点
	Node * head = new Node;
	if (/*相交函数，求点是否在片的内部，如果在内部)*/true)
	{
		//求出路径的长度，和已有的路径长度做对比
		int size;

	}
	//如果不在内部则 将可能的面 所对应的片全部设置为子节点
	else
	{
		vector<Surface *> maybeSurfce;
		for (auto iter = maybeSurfce.begin(); iter != maybeSurfce.end(); ++iter)
		{
			Node * newNode = new Node;
			newNode->Data = (*iter)->_pointOther;
			newNode->father = head;
			head->childs.push_back(newNode);
		}
	}

	


	return re;
}

void YN::WingStruct::Modify(Cube* target, int direct, int u, int v, int w)
{
	//修改包含了控制点的细化
	//UVW按照 0 0 0 的方式记录
	//举例：如果传入需要操作方向为 UW 方向 二进制
	//U V W
	//1 0 1
	//传入参数为5，传入之后挨个进行 与 & 操作

	vector<Cube *> uDirect, vDirect, wDirect,tempDirect,totalDirect;

	//如果U不为0
	if (direct & 4 != 0)
	{
		tempDirect = deepFirstSearch(target, 0);
		uDirect.insert(uDirect.end(), tempDirect.begin() + 1, tempDirect.end());
		tempDirect = deepFirstSearch(target, 1);
		uDirect.insert(uDirect.end(), tempDirect.begin() + 1, tempDirect.end());
	}
	//如果V不为0
	if (direct & 2 != 0)
	{
		for (auto iter = uDirect.begin(); iter != uDirect.end(); ++iter)
		{
			tempDirect = deepFirstSearch(*iter, 2);
			vDirect.insert(vDirect.end(), tempDirect.begin() + 1, tempDirect.end());
			tempDirect = deepFirstSearch(*iter, 3);
			vDirect.insert(vDirect.end(), tempDirect.begin() + 1, tempDirect.end());
		}
	}
	//如果w不为0
	if (direct & 4 != 0)
	{
		for (auto iter = vDirect.begin(); iter != vDirect.end(); ++iter)
		{
			tempDirect = deepFirstSearch(*iter, 4);
			wDirect.insert(wDirect.end(), tempDirect.begin() + 1, tempDirect.end());
			tempDirect = deepFirstSearch(*iter, 5);
			wDirect.insert(wDirect.end(), tempDirect.begin() + 1, tempDirect.end());
		}
	}

	for (auto iter = wDirect.begin(); iter != wDirect.end(); ++iter)
	{
		int U = max((*iter)->_Vol_Data._u_Degree, u);
		int V = max((*iter)->_Vol_Data._v_Degree, v);
		int W = max((*iter)->_Vol_Data._w_Degree, w);
		(*iter)->_Vol_Data.DegreeElevate(U, V, W);
		upDateCubeSurface((*iter));
	}
}