#include "stdafx.h"
#include "RWGeometric.h"
#include <fstream>

#include "Nurbs.h"
//读取3d离散点
int RWGeometric::ReadPoint(const string & path, varray<varray<point3d>>& allPts)
{
	allPts.clear();
	ifstream inf(path, ios::binary);
	if (!inf)return 0;
	string temp;
	int num = 0, flag = 0;
	varray<point3d> l;
	while (!inf.eof())
	{
		inf >> temp;
		if (temp == "<idx>") { inf >> temp; continue; }
		if (temp == "<Num>") { inf >> num; l.clear(); continue; }
		if (temp == "<pts/>") { flag = 1; continue; }
		if (temp == "</pts>")flag = 2;
		if (temp == "<allend>") { break; }
		if (flag == 1) 
		{
			point3d pts;
			pts.x = atof(temp.c_str());
			inf >> pts.y;
			inf >> pts.z;
			l.push_back(pts);
		}
		if (flag == 2 && l.size() == num)
			allPts.push_back(l);
	}
	inf.close();
	return allPts.size();
}

//读取4d离散点
int RWGeometric::ReadPoint(const string & path, varray<varray<point4d>>& allPts)
{
	allPts.clear();
	ifstream inf(path, ios::binary);
	if (!inf)return 0;
	string temp;
	int num = 0, flag = 0;
	varray<point4d> l;
	while (!inf.eof())
	{
		inf >> temp;
		if (temp == "<idx>") { inf >> temp; continue; }
		if (temp == "<Num>") { inf >> num; continue; }
		if (temp == "<pts/>") { flag = 1; l.clear(); continue; }
		if (temp == "</pts>")flag = 2;
		if (temp == "<allend>") { break; }
		if (flag == 1)
		{
			point4d pts;
			pts.x = atof(temp.c_str());
			inf >> pts.y;
			inf >> pts.z;
			inf >> pts.w;
			l.push_back(pts);
		}
		if (flag == 2 && l.size() == num)
			allPts.push_back(l);
	}
	inf.close();
	return allPts.size();
}

int RWGeometric::ReadBezierLine(const string & path, varray<BezierLine>& lines)
{
	lines.clear();
	ifstream inf(path, ios::binary);
	if (!inf)return 0;
	string temp = "";
	int flag = 0;
	BezierLine line;
	while (!inf.eof())
	{
		inf >> temp;
		//判断标识
		if (temp == "<idx>") { inf >> temp; continue; }
		if (temp == "<line/>")flag = 1;
		else if (temp == "<degree>")flag = 2;
		else if (temp == "<ctrlPts/>") { flag = 3; continue; }
		else if (temp == "</ctrlPts>")continue;
		else if (temp == "</line>")flag = 4;
		else if (temp == "<allend>")break;
		//根据标识对temp操作
		switch (flag)
		{
		case 1://<line/>
			line.m_CtrlPts.clear();
			break;
		case 2://<degree>
			inf >> line.m_Degree;
			break;
		case 3://<ctrlPts/>
		{
			point4d pts;
			pts.x = atof(temp.c_str());
			inf >> pts.y;
			inf >> pts.z;
			inf >> pts.w;
			line.m_CtrlPts.push_back(pts);
			break;
		}
		case 4://</line>
		{
			lines.push_back(line);
			break;
		}
		default:
			break;
		}
	}
	inf.close();
	return lines.size();
}

//读取NURBS曲线
//返回曲线数量
int RWGeometric::ReadNurbsLine(const string& path, varray<NurbsLine>& lines)
{
	lines.clear();
	ifstream inf(path, ios::binary);
	if (!inf)return 0;
	string temp = "";
	int flag = 0;
	NurbsLine line;
	while (!inf.eof())
	{
		inf >> temp;
		//判断标识
		if (temp == "<idx>") { inf >> temp; continue; }
		if (temp == "<line/>")flag = 1;
		else if (temp == "<degree>")flag = 2;
		else if (temp == "<knots/>") { flag = 3; continue; }
		else if (temp == "</knots>")continue;
		else if (temp == "<ctrlPts/>") { flag = 4; continue; }
		else if (temp == "</ctrlPts>")continue;
		else if (temp == "</line>")flag = 5;
		else if (temp == "<allend>")break;
		//根据标识对temp操作
		switch (flag)
		{
		case 1://<line/>
			line.m_Knots.clear();
			line.m_CtrlPts.clear();
			break;
		case 2://<degree>
			inf >> line.m_Degree;
			break;
		case 3://<knots/>
		{
			double k = atof(temp.c_str());
			line.m_Knots.push_back(k);
			break;
		}
		case 4://<ctrlPts/>
		{
			point4d pts;
			pts.x = atof(temp.c_str());
			inf >> pts.y;
			inf >> pts.z;
			inf >> pts.w;
			line.m_CtrlPts.push_back(pts);
			break;
		}
		case 5://</line>
		{
		    lines.push_back(line);
			break;
		}
		default:
			break;
		}
	}
	inf.close();
	return lines.size();
}

//读取NURBS曲面
//返回曲面数量
int RWGeometric::ReadNurbsSurface(const string& path, varray<NurbsSurface>& surfaces)
{
	surfaces.clear();
	ifstream inf(path, ios::binary);
	if (!inf)return 0;
	string temp = "";
	int flag = 0;
	NurbsSurface surface;
	while (!inf.eof())
	{
		inf >> temp;
		//判断标识
		if (temp == "<idx>") { inf >> temp; continue; }
		if (temp == "<surface/>")flag = 1;
		else if (temp == "<udegree>")flag = 2;
		else if (temp == "<vdegree>")flag = 3;
		else if (temp == "<uNum>")flag = 4;
		else if (temp == "<vNum>")flag = 5;
		else if (temp == "<uknots/>") { flag = 6; continue; }
		else if (temp == "</uknots>")continue;
		else if (temp == "<vknots/>") { flag = 7; continue; }
		else if (temp == "</vknots>")continue;
		else if (temp == "<ctrlPts/>") { flag = 8; continue; }
		else if (temp == "</ctrlPts>")continue;
		else if (temp == "</surface>")flag = 9;
		else if (temp == "<allend>")break;
		//根据标识对temp操作
		switch (flag)
		{
		case 1://<surface/>
			surface.m_uKnots.clear();
			surface.m_vKnots.clear();
			surface.m_CtrlPts.clear();
			break;
		case 2://<udegree>
			inf >> surface.m_uDegree;
			break;
		case 3://<vdegree>
			inf >> surface.m_vDegree;
			break;
		case 4://<uNum>
			inf >> surface.m_uNum;
			break;
		case 5://<vNum>
			inf >> surface.m_vNum;
			break;
		case 6://<uknots/>
		{
			double k = atof(temp.c_str());
			surface.m_uKnots.push_back(k);
			break;
		}
		case 7://<vknots/>
		{
			double k = atof(temp.c_str());
			surface.m_vKnots.push_back(k);
			break;
		}
		case 8://<ctrlPts/>
		{
			point4d pts;
			pts.x = atof(temp.c_str());
			inf >> pts.y;
			inf >> pts.z;
			inf >> pts.w;
			surface.m_CtrlPts.push_back(pts);
			break;
		}
		case 9://</surface>
		{
			surfaces.push_back(surface);
			break;
		}
		default:
			break;
		}
	}
	inf.close();
	return surfaces.size();
}

//读取NURBS体
//返回体模型数量
int RWGeometric::ReadNurbsVol(const string& path, varray<NurbsVol>& vols)
{
	vols.clear();
	ifstream inf(path, ios::binary);
	if (!inf)return 0;
	string temp = "";
	int flag = 0;
	NurbsVol vol;
	while (!inf.eof())
	{
		inf >> temp;
		//判断标识
		if (temp == "<idx>") { inf >> temp; continue; }
		if (temp == "<vol/>")flag = 1;
		else if (temp == "<udegree>")flag = 2;
		else if (temp == "<vdegree>")flag = 3;
		else if (temp == "<wdegree>")flag = 4;
		else if (temp == "<uNum>")flag = 5;
		else if (temp == "<vNum>")flag = 6;
		else if (temp == "<wNum>")flag = 7;
		else if (temp == "<uknots/>") { flag = 8; continue; }
		else if (temp == "</uknots>")continue;
		else if (temp == "<vknots/>") { flag = 9; continue; }
		else if (temp == "</vknots>")continue;
		else if (temp == "<wknots/>") { flag = 10; continue; }
		else if (temp == "</wknots>")continue;
		else if (temp == "<ctrlPts/>") { flag = 11; continue; }
		else if (temp == "</ctrlPts>")continue;
		else if (temp == "</vol>")flag = 12;
		else if (temp == "<allend>")break;
		//根据标识对temp操作
		switch (flag)
		{
		case 1://<surface/>
			vol.m_uKnots.clear();
			vol.m_vKnots.clear();
			vol.m_wKnots.clear();
			vol.m_CtrlPts.clear();
			break;
		case 2://<udegree>
			inf >> vol.m_uDegree;
			break;
		case 3://<vdegree>
			inf >> vol.m_vDegree;
			break;
		case 4://<wdegree>
			inf >> vol.m_wDegree;
			break;
		case 5://<uNum>
			inf >> vol.m_uNum;
			break;
		case 6://<vNum>
			inf >> vol.m_vNum;
			break;
		case 7://<wNum>
			inf >> vol.m_wNum;
			break;
		case 8://<uknots/>
		{
			double k = atof(temp.c_str());
			vol.m_uKnots.push_back(k);
			break;
		}
		case 9://<vknots/>
		{
			double k = atof(temp.c_str());
			vol.m_vKnots.push_back(k);
			break;
		}
		case 10://<wknots/>
		{
			double k = atof(temp.c_str());
			vol.m_wKnots.push_back(k);
			break;
		}
		case 11://<ctrlPts/>
		{
			point4d pts;
			pts.x = atof(temp.c_str());
			inf >> pts.y;
			inf >> pts.z;
			inf >> pts.w;
			vol.m_CtrlPts.push_back(pts);
			break;
		}
		case 12://</vol>
		{
			vols.push_back(vol);
			break;
		}
		default:
			break;
		}
	}
	inf.close();
	return vols.size();
}



int RWGeometric::WritePoint(const string & path, const varray<varray<point3d>>& allPts)
{
	ofstream outf(path, ios::binary);
	if (!outf)return 0;
	for (int i = 0; i < allPts.size(); ++i)
	{
		outf << "<idx>" << " " << i << '\r' << '\n' << endl;
		outf << "<Num>" << " " << allPts[i].size() << '\r'<<'\n' << endl;
		outf << "<pts/>" << '\r'<<'\n' << endl;
		for (int j = 0; j < allPts[i].size();++j)
			outf << allPts[i][j].x << " " << allPts[i][j].y << " "
			<< allPts[i][j].z << '\r'<<'\n' << endl;
		outf << "</pts>" << '\r'<<'\n' << endl;
	}
	outf << "<allend>" << endl;
	outf.close();
	return allPts.size();
}


int RWGeometric::WritePoint(const string & path, const varray<varray<Vec4>>& allPts)
{
	ofstream outf(path, ios::binary);
	if (!outf)return 0;
	for (int i = 0; i < allPts.size(); ++i)
	{
		outf << "<idx>" << " " << i << '\r' << '\n' << endl;
		outf << "<Num>" << " " << allPts[i].size() << '\r' << '\n' << endl;
		outf << "<pts/>" << '\r' << '\n' << endl;
		for (int j = 0; j < allPts[i].size(); ++j)
			outf << allPts[i][j].x << " " << allPts[i][j].y << " "
			<< allPts[i][j].z << " " << allPts[i][j].w << '\r' << '\n' << endl;
		outf << "</pts>" << '\r' << '\n' << endl;
	}
	outf << "<allend>" << endl;
	outf.close();
	return allPts.size();
}

int RWGeometric::WritePoint(const string & path, const varray<varray<point4d>>& allPts)
{
	ofstream outf(path, ios::binary);
	if (!outf)return 0;
	for (int i = 0; i < allPts.size(); ++i)
	{
		outf << "<idx>" << " " << i << '\r' << '\n' << endl;
		outf << "<Num>" << " " << allPts[i].size() << '\r'<<'\n' << endl;
		outf << "<pts/>" << '\r'<<'\n' << endl;
		for (int j = 0; j < allPts[i].size(); ++j)
			outf << allPts[i][j].x << " " << allPts[i][j].y << " "
			<< allPts[i][j].z << " " << allPts[i][j].w << '\r'<<'\n' << endl;
		outf << "</pts>" << '\r'<<'\n' << endl;
	}
	outf << "<allend>" << endl;
	outf.close();
	return allPts.size();
}

int RWGeometric::WriteBezierLine(const string & path, const varray<BezierLine>& lines)
{
	ofstream outf(path, ios::binary);
	if (!outf)return 0;
	for (int i = 0; i < lines.size(); ++i)
	{
		outf << "<idx>" << " " << i << '\r' << '\n' << endl;
		outf << "<line/>" << '\r' << '\n' << endl;
		outf << "<degree>" << " " << lines[i].m_Degree << '\r' << '\n' << endl;
		outf << "<ctrlPts/>" << '\r' << '\n' << endl;
		for (int j = 0; j < lines[i].m_CtrlPts.size(); ++j)
			outf << lines[i].m_CtrlPts[j].x << " " << lines[i].m_CtrlPts[j].y << " "
			<< lines[i].m_CtrlPts[j].z << " " << lines[i].m_CtrlPts[j].w << '\r' << '\n' << endl;
		outf << "</ctrlPts>" << '\r' << '\n' << endl;
		outf << "</line>" << '\r' << '\n' << endl;
	}
	outf << "<allend>" << endl;
	outf.close();
	return lines.size();
}

//写出NURBS曲线
//返回写出的曲线数量
int RWGeometric::WriteNurbsLine(const string& path, const varray<NurbsLine>& lines)
{
	ofstream outf(path, ios::binary);
	if (!outf)return 0;
	for (int i = 0; i < lines.size(); ++i)
	{
		outf << "<idx>" << " " << i << '\r' << '\n' << endl;
		outf << "<line/>" << '\r'<<'\n' << endl;
		outf << "<degree>" << " " << lines[i].m_Degree << '\r'<<'\n' << endl;
		outf << "<knots/>" << " ";
		for (int j = 0; j < lines[i].m_Knots.size(); ++j)
			outf << lines[i].m_Knots[j] << " ";
		outf << "</knots>" << '\r'<<'\n' << endl;
		outf << "<ctrlPts/>" << '\r'<<'\n' << endl;
		for (int j = 0; j < lines[i].m_CtrlPts.size(); ++j)
			outf << lines[i].m_CtrlPts[j].x << " " << lines[i].m_CtrlPts[j].y << " "
			<< lines[i].m_CtrlPts[j].z << " " << lines[i].m_CtrlPts[j].w << '\r'<<'\n' << endl;
		outf << "</ctrlPts>" << '\r'<<'\n' << endl;
		outf << "</line>" << '\r'<<'\n' << endl;
	}
	outf << "<allend>" << endl;
	outf.close();
	return lines.size();
}

//写出NURBS曲面
//返回写出的曲面数量
int RWGeometric::WriteNurbsSurface(const string& path, const varray<NurbsSurface>& surfaces)
{
	ofstream outf(path, ios::binary);
	if (!outf)return 0;
	for (int i = 0; i < surfaces.size(); ++i)
	{
		outf << "<idx>" << " " << i << '\r' << '\n' << endl;
		outf << "<surface/>" << '\r'<<'\n' << endl;
		outf << "<udegree>" << " " << surfaces[i].m_uDegree << '\r'<<'\n' << endl;
		outf << "<vdegree>" << " " << surfaces[i].m_vDegree << '\r'<<'\n' << endl;
		outf << "<uNum>" << " " << surfaces[i].m_uNum << '\r'<<'\n' << endl;
		outf << "<vNum>" << " " << surfaces[i].m_vNum << '\r'<<'\n' << endl;
		outf << "<uknots/>" << " ";
		for (int j = 0; j < surfaces[i].m_uKnots.size(); ++j)
			outf << surfaces[i].m_uKnots[j] << " ";
		outf << "</uknots>" << '\r'<<'\n' << endl;
		outf << "<vknots/>" << " ";
		for (int j = 0; j < surfaces[i].m_vKnots.size(); ++j)
			outf << surfaces[i].m_vKnots[j] << " ";
		outf << "</vknots>" << '\r'<<'\n' << endl;
		outf << "<ctrlPts/>" << '\r'<<'\n' << endl;
		for (int j = 0; j < surfaces[i].m_CtrlPts.size(); ++j)
			outf << surfaces[i].m_CtrlPts[j].x << " " << surfaces[i].m_CtrlPts[j].y << " "
			<< surfaces[i].m_CtrlPts[j].z << " " << surfaces[i].m_CtrlPts[j].w << '\r'<<'\n' << endl;
		outf << "</ctrlPts>" << '\r'<<'\n' << endl;
		outf << "</surface>" << '\r'<<'\n' << endl;
	}
	outf << "<allend>" << endl;
	outf.close();
	return surfaces.size();
}

//写出NURBS体
//返回写出的体模型数量
int RWGeometric::WriteNurbsVol(const string& path, const varray<NurbsVol>& vols)
{
	ofstream outf(path, ios::binary);
	if (!outf)return 0;
	for (int i = 0; i < vols.size(); ++i)
	{
		outf << "<idx>" << " " << i << '\r' << '\n' << endl;
		outf << "<vol/>" << '\r'<<'\n' << endl;
		outf << "<udegree>" << " " << vols[i].m_uDegree << '\r'<<'\n' << endl;
		outf << "<vdegree>" << " " << vols[i].m_vDegree << '\r'<<'\n' << endl;
		outf << "<wdegree>" << " " << vols[i].m_wDegree << '\r'<<'\n' << endl;
		outf << "<uNum>" << " " << vols[i].m_uNum << '\r'<<'\n' << endl;
		outf << "<vNum>" << " " << vols[i].m_vNum << '\r'<<'\n' << endl;
		outf << "<wNum>" << " " << vols[i].m_wNum << '\r'<<'\n' << endl;
		outf << "<uknots/>" << " ";
		for (int j = 0; j < vols[i].m_uKnots.size(); ++j)
			outf << vols[i].m_uKnots[j] << " ";
		outf << "</uknots>" << '\r'<<'\n' << endl;
		outf << "<vknots/>" << " ";
		for (int j = 0; j < vols[i].m_vKnots.size(); ++j)
			outf << vols[i].m_vKnots[j] << " ";
		outf << "</vknots>" << '\r'<<'\n' << endl;
		outf << "<wknots/>" << " ";
		for (int j = 0; j < vols[i].m_wKnots.size(); ++j)
			outf << vols[i].m_wKnots[j] << " ";
		outf << "</wknots>" << '\r'<<'\n' << endl;
		outf << "<ctrlPts/>" << '\r'<<'\n' << endl;
		for (int j = 0; j < vols[i].m_CtrlPts.size(); ++j)
			outf << vols[i].m_CtrlPts[j].x << " " << vols[i].m_CtrlPts[j].y << " "
			<< vols[i].m_CtrlPts[j].z << " " << vols[i].m_CtrlPts[j].w << '\r'<<'\n' << endl;
		outf << "</ctrlPts>" << '\r'<<'\n' << endl;
		outf << "</vol>" << '\r'<<'\n' << endl;
	}
	outf << "<allend>" << endl;
	outf.close();
	return vols.size();
}



//--------------------------------------------------------------//
//读取NURBS曲线
//返回曲线数量
int RWGeometric::ReadSpline(const string& path, varray<Spline>& lines)
{
	lines.clear();
	ifstream inf(path, ios::binary);
	if (!inf)return 0;
	string temp = "";
	int flag = 0;
	Spline line;
	while (!inf.eof())
	{
		inf >> temp;
		//判断标识
		if (temp == "<idx>") { inf >> temp; continue; }
		if (temp == "<line/>")flag = 1;
		else if (temp == "<degree>")flag = 2;
		else if (temp == "<knots/>") { flag = 3; continue; }
		else if (temp == "</knots>")continue;
		else if (temp == "<ctrlPts/>") { flag = 4; continue; }
		else if (temp == "</ctrlPts>")continue;
		else if (temp == "</line>")flag = 5;
		else if (temp == "<allend>")break;
		//根据标识对temp操作
		switch (flag)
		{
		case 1://<line/>
			line.m_Knots.clear();
			line.m_CtrlPts.clear();
			break;
		case 2://<degree>
			inf >> line.m_Degree;
			break;
		case 3://<knots/>
		{
			double k = atof(temp.c_str());
			line.m_Knots.push_back(k);
			break;
		}
		case 4://<ctrlPts/>
		{
			Vec4 pts;
			pts.x = atof(temp.c_str());
			inf >> pts.y;
			inf >> pts.z;
			inf >> pts.w;
			line.m_CtrlPts.push_back(pts);
			break;
		}
		case 5://</line>
		{
			lines.push_back(line);
			break;
		}
		default:
			break;
		}
	}
	inf.close();
	return lines.size();
}

//读取NURBS曲面
//返回曲面数量
int RWGeometric::ReadSplineSurface(const string& path, varray<SplineSurface>& surfaces)
{
	surfaces.clear();
	ifstream inf(path, ios::binary);
	if (!inf)return 0;
	string temp = "";
	int flag = 0;
	SplineSurface surface;
	while (!inf.eof())
	{
		inf >> temp;
		//判断标识
		if (temp == "<idx>") { inf >> temp; continue; }
		if (temp == "<surface/>")flag = 1;
		else if (temp == "<udegree>")flag = 2;
		else if (temp == "<vdegree>")flag = 3;
		else if (temp == "<uNum>")flag = 4;
		else if (temp == "<vNum>")flag = 5;
		else if (temp == "<uknots/>") { flag = 6; continue; }
		else if (temp == "</uknots>")continue;
		else if (temp == "<vknots/>") { flag = 7; continue; }
		else if (temp == "</vknots>")continue;
		else if (temp == "<ctrlPts/>") { flag = 8; continue; }
		else if (temp == "</ctrlPts>")continue;
		else if (temp == "</surface>")flag = 9;
		else if (temp == "<allend>")break;
		//根据标识对temp操作
		switch (flag)
		{
		case 1://<surface/>
			surface.m_uKnots.clear();
			surface.m_vKnots.clear();
			surface.m_CtrlPts.clear();
			break;
		case 2://<udegree>
			inf >> surface.m_uDegree;
			break;
		case 3://<vdegree>
			inf >> surface.m_vDegree;
			break;
		case 4://<uNum>
			inf >> surface.m_uNum;
			break;
		case 5://<vNum>
			inf >> surface.m_vNum;
			break;
		case 6://<uknots/>
		{
			double k = atof(temp.c_str());
			surface.m_uKnots.push_back(k);
			break;
		}
		case 7://<vknots/>
		{
			double k = atof(temp.c_str());
			surface.m_vKnots.push_back(k);
			break;
		}
		case 8://<ctrlPts/>
		{
			Vec4 pts;
			pts.x = atof(temp.c_str());
			inf >> pts.y;
			inf >> pts.z;
			inf >> pts.w;
			surface.m_CtrlPts.push_back(pts);
			break;
		}
		case 9://</surface>
		{
			surfaces.push_back(surface);
			break;
		}
		default:
			break;
		}
	}
	inf.close();
	return surfaces.size();
}

//读取NURBS体
//返回体模型数量
int RWGeometric::ReadSplineVolume(const string& path, varray<SplineVolume>& vols)
{
	vols.clear();
	ifstream inf(path, ios::binary);
	if (!inf)
	{
		cout << "------" << endl;
		return 0;
	}
	string temp = "";
	int flag = 0;
	SplineVolume vol;
	while (!inf.eof())
	{
		inf >> temp;
		//判断标识
		if (temp == "<idx>") { inf >> temp; continue; }
		if (temp == "<vol/>")flag = 1;
		else if (temp == "<udegree>")flag = 2;
		else if (temp == "<vdegree>")flag = 3;
		else if (temp == "<wdegree>")flag = 4;
		else if (temp == "<uNum>")flag = 5;
		else if (temp == "<vNum>")flag = 6;
		else if (temp == "<wNum>")flag = 7;
		else if (temp == "<uknots/>") { flag = 8; continue; }
		else if (temp == "</uknots>")continue;
		else if (temp == "<vknots/>") { flag = 9; continue; }
		else if (temp == "</vknots>")continue;
		else if (temp == "<wknots/>") { flag = 10; continue; }
		else if (temp == "</wknots>")continue;
		else if (temp == "<ctrlPts/>") { flag = 11; continue; }
		else if (temp == "</ctrlPts>")continue;
		else if (temp == "</vol>")flag = 12;
		else if (temp == "<allend>")break;
		//根据标识对temp操作
		switch (flag)
		{
		case 1://<surface/>
			vol.m_uKnots.clear();
			vol.m_vKnots.clear();
			vol.m_wKnots.clear();
			vol.m_CtrlPts.clear();
			break;
		case 2://<udegree>
			inf >> vol.m_uDegree;
			break;
		case 3://<vdegree>
			inf >> vol.m_vDegree;
			break;
		case 4://<wdegree>
			inf >> vol.m_wDegree;
			break;
		case 5://<uNum>
			inf >> vol.m_uNum;
			break;
		case 6://<vNum>
			inf >> vol.m_vNum;
			break;
		case 7://<wNum>
			inf >> vol.m_wNum;
			break;
		case 8://<uknots/>
		{
			double k = atof(temp.c_str());
			vol.m_uKnots.push_back(k);
			break;
		}
		case 9://<vknots/>
		{
			double k = atof(temp.c_str());
			vol.m_vKnots.push_back(k);
			break;
		}
		case 10://<wknots/>
		{
			double k = atof(temp.c_str());
			vol.m_wKnots.push_back(k);
			break;
		}
		case 11://<ctrlPts/>
		{
			Vec4 pts;
			pts.x = atof(temp.c_str());
			inf >> pts.y;
			inf >> pts.z;
			inf >> pts.w;
			vol.m_CtrlPts.push_back(pts);
			break;
		}
		case 12://</vol>
		{
			vols.push_back(vol);
			break;
		}
		default:
			break;
		}
	}
	inf.close();
	return vols.size();
}


//写出NURBS曲线
//返回写出的曲线数量
int RWGeometric::WriteSpline(const string& path, const varray<Spline>& lines)
{
	ofstream outf(path, ios::binary);
	if (!outf)return 0;
	for (int i = 0; i < lines.size(); ++i)
	{
		outf << "<idx>" << " " << i << '\r' << '\n' << endl;
		outf << "<line/>" << '\r' << '\n' << endl;
		outf << "<degree>" << " " << lines[i].m_Degree << '\r' << '\n' << endl;
		outf << "<knots/>" << " ";
		for (int j = 0; j < lines[i].m_Knots.size(); ++j)
			outf << lines[i].m_Knots[j] << " ";
		outf << "</knots>" << '\r' << '\n' << endl;
		outf << "<ctrlPts/>" << '\r' << '\n' << endl;
		for (int j = 0; j < lines[i].m_CtrlPts.size(); ++j)
			outf << lines[i].m_CtrlPts[j].x << " " << lines[i].m_CtrlPts[j].y << " "
			<< lines[i].m_CtrlPts[j].z << " " << lines[i].m_CtrlPts[j].w << '\r' << '\n' << endl;
		outf << "</ctrlPts>" << '\r' << '\n' << endl;
		outf << "</line>" << '\r' << '\n' << endl;
	}
	outf << "<allend>" << endl;
	outf.close();
	return lines.size();
}

//写出NURBS曲面
//返回写出的曲面数量
int RWGeometric::WriteSplineSurface(const string& path, const varray<SplineSurface>& surfaces)
{
	ofstream outf(path, ios::binary);
	if (!outf)return 0;
	for (int i = 0; i < surfaces.size(); ++i)
	{
		outf << "<idx>" << " " << i << '\r' << '\n' << endl;
		outf << "<surface/>" << '\r' << '\n' << endl;
		outf << "<udegree>" << " " << surfaces[i].m_uDegree << '\r' << '\n' << endl;
		outf << "<vdegree>" << " " << surfaces[i].m_vDegree << '\r' << '\n' << endl;
		outf << "<uNum>" << " " << surfaces[i].m_uNum << '\r' << '\n' << endl;
		outf << "<vNum>" << " " << surfaces[i].m_vNum << '\r' << '\n' << endl;
		outf << "<uknots/>" << " ";
		for (int j = 0; j < surfaces[i].m_uKnots.size(); ++j)
			outf << surfaces[i].m_uKnots[j] << " ";
		outf << "</uknots>" << '\r' << '\n' << endl;
		outf << "<vknots/>" << " ";
		for (int j = 0; j < surfaces[i].m_vKnots.size(); ++j)
			outf << surfaces[i].m_vKnots[j] << " ";
		outf << "</vknots>" << '\r' << '\n' << endl;
		outf << "<ctrlPts/>" << '\r' << '\n' << endl;
		for (int j = 0; j < surfaces[i].m_CtrlPts.size(); ++j)
			outf << surfaces[i].m_CtrlPts[j].x << " " << surfaces[i].m_CtrlPts[j].y << " "
			<< surfaces[i].m_CtrlPts[j].z << " " << surfaces[i].m_CtrlPts[j].w << '\r' << '\n' << endl;
		outf << "</ctrlPts>" << '\r' << '\n' << endl;
		outf << "</surface>" << '\r' << '\n' << endl;
	}
	outf << "<allend>" << endl;
	outf.close();
	return surfaces.size();
}

//写出NURBS体
//返回写出的体模型数量
int RWGeometric::WriteSplineVolume(const string& path, const varray<SplineVolume>& vols)
{
	ofstream outf(path, ios::binary);
	if (!outf)return 0;
	for (int i = 0; i < vols.size(); ++i)
	{
		outf << "<idx>" << " " << i << '\r' << '\n' << endl;
		outf << "<vol/>" << '\r' << '\n' << endl;
		outf << "<udegree>" << " " << vols[i].m_uDegree << '\r' << '\n' << endl;
		outf << "<vdegree>" << " " << vols[i].m_vDegree << '\r' << '\n' << endl;
		outf << "<wdegree>" << " " << vols[i].m_wDegree << '\r' << '\n' << endl;
		outf << "<uNum>" << " " << vols[i].m_uNum << '\r' << '\n' << endl;
		outf << "<vNum>" << " " << vols[i].m_vNum << '\r' << '\n' << endl;
		outf << "<wNum>" << " " << vols[i].m_wNum << '\r' << '\n' << endl;
		outf << "<uknots/>" << " ";
		for (int j = 0; j < vols[i].m_uKnots.size(); ++j)
			outf << vols[i].m_uKnots[j] << " ";
		outf << "</uknots>" << '\r' << '\n' << endl;
		outf << "<vknots/>" << " ";
		for (int j = 0; j < vols[i].m_vKnots.size(); ++j)
			outf << vols[i].m_vKnots[j] << " ";
		outf << "</vknots>" << '\r' << '\n' << endl;
		outf << "<wknots/>" << " ";
		for (int j = 0; j < vols[i].m_wKnots.size(); ++j)
			outf << vols[i].m_wKnots[j] << " ";
		outf << "</wknots>" << '\r' << '\n' << endl;
		outf << "<ctrlPts/>" << '\r' << '\n' << endl;
		for (int j = 0; j < vols[i].m_CtrlPts.size(); ++j)
			outf << vols[i].m_CtrlPts[j].x << " " << vols[i].m_CtrlPts[j].y << " "
			<< vols[i].m_CtrlPts[j].z << " " << vols[i].m_CtrlPts[j].w << '\r' << '\n' << endl;
		outf << "</ctrlPts>" << '\r' << '\n' << endl;
		outf << "</vol>" << '\r' << '\n' << endl;
	}
	outf << "<allend>" << endl;
	outf.close();
	return vols.size();
}
