#include<occIgesTrans.h>
#include<QtGui/qwindowdefs_win.h>

#include "RWGeometric.h"
#include<TopoDS_Compound.hxx>
#include<BRep_Builder.hxx>
#include<IGESControl_Writer.hxx>
#include<BRepBuilderAPI_MakeFace.hxx>

#include <Geom_BSplineSurface.hxx>

OccIgesTrans::OccIgesTrans()
{
}
void OccIgesTrans::outputIges(const string & path,const string &outputPath)
{
	cout << "outputIges" << endl;
	varray<SplineSurface> surfs;
	RWGeometric rwg;
	varray<SplineVolume> svs;
	rwg.ReadSplineVolume(path, svs);
	varray<SplineVolume> tmp;
	TopoDS_Compound resultCompound;//最终的面
	BRep_Builder builder;
	builder.MakeCompound(resultCompound);

	varray<Vec4> pt;
	varray<varray<Vec4>> pts;

	//varray<SplineVolume> svs;
	if (!svs.size())
	{
		return;
	}
	int count = 0;
	for (int id = 0; id < svs.size(); id++) {
		auto vol = svs[id];
		varray<Vec4> pt;
		varray<varray<Vec4>> pts;
		varray<SplineSurface> sfs;

		SplineSurface sf;
		sf.m_uNum = vol.m_uNum;
		sf.m_uDegree = vol.m_uDegree;
		sf.m_vDegree = vol.m_vDegree;
		sf.m_vNum = vol.m_vNum;
		sf.m_uKnots = vol.m_uKnots;
		sf.m_vKnots = vol.m_vKnots;
		//顶底面
		for (int i = 0; i < vol.m_uNum*vol.m_vNum; i++) {
			sf.m_CtrlPts.push_back(vol.m_CtrlPts[i]);
		}
		sfs.push_back(sf);
		sf.m_CtrlPts.clear();
		for (int i = vol.m_uNum*vol.m_vNum*(vol.m_wNum - 1); i < vol.m_uNum*vol.m_vNum*vol.m_wNum; i++) {
			sf.m_CtrlPts.push_back(vol.m_CtrlPts[i]);
		}
		sfs.push_back(sf);

		//前后侧面
		sf.m_vNum = vol.m_wNum;
		sf.m_CtrlPts.clear();
		sf.m_vDegree = vol.m_wDegree;
		sf.m_vKnots = vol.m_wKnots;
		for (int i = 0; i < vol.m_wNum; i++) {
			for (int j = 0; j < vol.m_uNum; j++) {
				sf.m_CtrlPts.push_back(vol.m_CtrlPts[i*vol.m_uNum*vol.m_vNum + j]);
			}
		}
		sfs.push_back(sf);
		sf.m_CtrlPts.clear();

		for (int i = 0; i < vol.m_wNum; i++) {
			for (int j = 0; j < vol.m_uNum; j++) {
				sf.m_CtrlPts.push_back(vol.m_CtrlPts[i*vol.m_uNum*vol.m_vNum + j + vol.m_uNum*(vol.m_vNum - 1)]);
			}
		}
		sfs.push_back(sf);
		//左右侧面
		sf.m_uNum = vol.m_vNum;
		sf.m_CtrlPts.clear();
		sf.m_uDegree = vol.m_vDegree;
		sf.m_uKnots = vol.m_vKnots;

		for (int i = 0; i < vol.m_wNum; i++) {
			for (int j = 0; j < vol.m_vNum; j++) {
				sf.m_CtrlPts.push_back(vol.m_CtrlPts[i*vol.m_uNum*vol.m_vNum + j * vol.m_uNum]);
			}
		}
		sfs.push_back(sf);
		sf.m_CtrlPts.clear();
		for (int i = 0; i < vol.m_wNum; i++) {
			for (int j = 0; j < vol.m_vNum; j++) {
				sf.m_CtrlPts.push_back(vol.m_CtrlPts[i*vol.m_uNum*vol.m_vNum + (j + 1) * vol.m_uNum - 1]);
			}
		}
		sfs.push_back(sf);
		//}

	//pts.push_back(pt);

		/*if (id == 26)
			rwg.WriteSplineSurface("./model/testsurf.txt", sfs);*/

		for (auto &suf : sfs) {
			surfs.push_back(suf);
			int numU = suf.m_uNum; // U方向控制点数量
			int numV = suf.m_vNum; // V方向控制点数量
			TColgp_Array2OfPnt controlPoints(1, numV, 1, numU);
			for (int i = 0; i < numV; i++) {
				for (int j = 0; j < numU; j++) {
					Vec4 p;
					//if ((id == 22 || id == 26 || id == 30) && path == "bearing block") 
					p = suf.m_CtrlPts[j*numV + i];
					/*else {
						p= suf.m_CtrlPts[j + i * numV];
					}*/
					pt.push_back(p);

					controlPoints(i + 1, j + 1) = gp_Pnt(p.x, p.y, p.z);
				}
				pts.push_back(pt);
				if (id == 26)
					rwg.WritePoint("./model/ptsx.txt", pts);
			}


			vector<int> uM, vM;
			vector<double> uK, vK;
			int cnt = 1;
			for (int i = 0; i < suf.m_uKnots.size() - 1; i++) {
				if (suf.m_uKnots[i + 1] == suf.m_uKnots[i]) {
					cnt++;
				}
				else {
					uK.push_back(suf.m_uKnots[i]);
					uM.push_back(cnt);
					cnt = 1;
				}
			}
			uM.push_back(cnt);
			uK.push_back(suf.m_uKnots.back());
			cnt = 1;
			for (int i = 0; i < suf.m_vKnots.size() - 1; i++) {
				if (suf.m_vKnots[i + 1] == suf.m_vKnots[i]) {
					cnt++;
				}
				else {
					vK.push_back(suf.m_vKnots[i]);
					vM.push_back(cnt);
					cnt = 1;
				}
			}
			vM.push_back(cnt);
			vK.push_back(suf.m_vKnots.back());
			//重合度
			TColStd_Array1OfInteger uMults(1, uM.size());
			for (int i = 0; i < uM.size(); i++) {
				uMults(i + 1) = uM[i];
			}

			TColStd_Array1OfInteger vMults(1, vM.size());
			for (int i = 0; i < vM.size(); i++) {
				vMults(i + 1) = vM[i];
			}

			//递增节点
			TColStd_Array1OfReal uKnots(1, uK.size());
			for (int i = 0; i < uK.size(); i++) {
				uKnots(i + 1) = uK[i];
			}
			TColStd_Array1OfReal vKnots(1, vK.size());
			for (int i = 0; i < vK.size(); i++) {
				vKnots(i + 1) = vK[i];
			}

			//for (int i = 0; i < 2; i++) cout << vMults(i + 1) << endl;
			// 创建B样条曲面
			Handle(Geom_BSplineSurface) bSplineSurface = new Geom_BSplineSurface(
				controlPoints, vKnots, uKnots, vMults, uMults, suf.m_uDegree, suf.m_vDegree, Standard_False, Standard_False
			);
			Handle(Geom_Surface) geomSurface = bSplineSurface;

			// 将Geom_Surface转换为TopoDS_Shape
			TopoDS_Shape shape = BRepBuilderAPI_MakeFace(geomSurface, 1e-6);
			builder.Add(resultCompound, shape);
		}
		cout << id << endl;

	}
	cout << "  ss" << endl;
	

	IGESControl_Writer writer;
	writer.AddShape(resultCompound);
	auto result=writer.Write(outputPath.c_str());

	
}

void OccIgesTrans::outputIges(varray<SplineVolume>& svs, const string & outputPath)
{
	cout << "outputIges" << endl;
	varray<SplineSurface> surfs;
	varray<SplineVolume> tmp;
	TopoDS_Compound resultCompound;//最终的面
	BRep_Builder builder;
	builder.MakeCompound(resultCompound);

	varray<Vec4> pt;
	varray<varray<Vec4>> pts;

	//varray<SplineVolume> svs;
	if (!svs.size())
	{
		return;
	}
	int count = 0;
	for (int id = 0; id < svs.size(); id++) {
		auto vol = svs[id];
		varray<Vec4> pt;
		varray<varray<Vec4>> pts;
		varray<SplineSurface> sfs;

		SplineSurface sf;
		sf.m_uNum = vol.m_uNum;
		sf.m_uDegree = vol.m_uDegree;
		sf.m_vDegree = vol.m_vDegree;
		sf.m_vNum = vol.m_vNum;
		sf.m_uKnots = vol.m_uKnots;
		sf.m_vKnots = vol.m_vKnots;
		//顶底面
		for (int i = 0; i < vol.m_uNum*vol.m_vNum; i++) {
			sf.m_CtrlPts.push_back(vol.m_CtrlPts[i]);
		}
		sfs.push_back(sf);
		sf.m_CtrlPts.clear();
		for (int i = vol.m_uNum*vol.m_vNum*(vol.m_wNum - 1); i < vol.m_uNum*vol.m_vNum*vol.m_wNum; i++) {
			sf.m_CtrlPts.push_back(vol.m_CtrlPts[i]);
		}
		sfs.push_back(sf);

		//前后侧面
		sf.m_vNum = vol.m_wNum;
		sf.m_CtrlPts.clear();
		sf.m_vDegree = vol.m_wDegree;
		sf.m_vKnots = vol.m_wKnots;
		for (int i = 0; i < vol.m_wNum; i++) {
			for (int j = 0; j < vol.m_uNum; j++) {
				sf.m_CtrlPts.push_back(vol.m_CtrlPts[i*vol.m_uNum*vol.m_vNum + j]);
			}
		}
		sfs.push_back(sf);
		sf.m_CtrlPts.clear();

		for (int i = 0; i < vol.m_wNum; i++) {
			for (int j = 0; j < vol.m_uNum; j++) {
				sf.m_CtrlPts.push_back(vol.m_CtrlPts[i*vol.m_uNum*vol.m_vNum + j + vol.m_uNum*(vol.m_vNum - 1)]);
			}
		}
		sfs.push_back(sf);
		//左右侧面
		sf.m_uNum = vol.m_vNum;
		sf.m_CtrlPts.clear();
		sf.m_uDegree = vol.m_vDegree;
		sf.m_uKnots = vol.m_vKnots;

		for (int i = 0; i < vol.m_wNum; i++) {
			for (int j = 0; j < vol.m_vNum; j++) {
				sf.m_CtrlPts.push_back(vol.m_CtrlPts[i*vol.m_uNum*vol.m_vNum + j * vol.m_uNum]);
			}
		}
		sfs.push_back(sf);
		sf.m_CtrlPts.clear();
		for (int i = 0; i < vol.m_wNum; i++) {
			for (int j = 0; j < vol.m_vNum; j++) {
				sf.m_CtrlPts.push_back(vol.m_CtrlPts[i*vol.m_uNum*vol.m_vNum + (j + 1) * vol.m_uNum - 1]);
			}
		}
		sfs.push_back(sf);
		//}

	//pts.push_back(pt);

		/*if (id == 26)
			rwg.WriteSplineSurface("./model/testsurf.txt", sfs);*/

		for (auto &suf : sfs) {
			surfs.push_back(suf);
			int numU = suf.m_uNum; // U方向控制点数量
			int numV = suf.m_vNum; // V方向控制点数量
			TColgp_Array2OfPnt controlPoints(1, numV, 1, numU);
			TColStd_Array2OfReal weights(1, numV, 1, numU);
			for (int i = 0; i < numV; i++) {
				for (int j = 0; j < numU; j++) {
					Vec4 p;
					//if ((id == 22 || id == 26 || id == 30) && path == "bearing block") 
					p = suf.m_CtrlPts[j*numV + i];
					/*else {
						p= suf.m_CtrlPts[j + i * numV];
					}*/
					pt.push_back(p);

					controlPoints(i + 1, j + 1) = gp_Pnt(p.x, p.y, p.z);
					weights(i + 1, j + 1) = (Standard_Real)p.w;
				}
				pts.push_back(pt);
				
			}


			vector<int> uM, vM;
			vector<double> uK, vK;
			int cnt = 1;
			for (int i = 0; i < suf.m_uKnots.size() - 1; i++) {
				if (suf.m_uKnots[i + 1] == suf.m_uKnots[i]) {
					cnt++;
				}
				else {
					uK.push_back(suf.m_uKnots[i]);
					uM.push_back(cnt);
					cnt = 1;
				}
			}
			uM.push_back(cnt);
			uK.push_back(suf.m_uKnots.back());
			cnt = 1;
			for (int i = 0; i < suf.m_vKnots.size() - 1; i++) {
				if (suf.m_vKnots[i + 1] == suf.m_vKnots[i]) {
					cnt++;
				}
				else {
					vK.push_back(suf.m_vKnots[i]);
					vM.push_back(cnt);
					cnt = 1;
				}
			}
			vM.push_back(cnt);
			vK.push_back(suf.m_vKnots.back());
			//重合度
			TColStd_Array1OfInteger uMults(1, uM.size());
			for (int i = 0; i < uM.size(); i++) {
				uMults(i + 1) = uM[i];
			}

			TColStd_Array1OfInteger vMults(1, vM.size());
			for (int i = 0; i < vM.size(); i++) {
				vMults(i + 1) = vM[i];
			}

			//递增节点
			TColStd_Array1OfReal uKnots(1, uK.size());
			for (int i = 0; i < uK.size(); i++) {
				uKnots(i + 1) = uK[i];
			}
			TColStd_Array1OfReal vKnots(1, vK.size());
			for (int i = 0; i < vK.size(); i++) {
				vKnots(i + 1) = vK[i];
			}

			//for (int i = 0; i < 2; i++) cout << vMults(i + 1) << endl;
			// 创建B样条曲面
			Handle(Geom_BSplineSurface) bSplineSurface = new Geom_BSplineSurface(
				controlPoints, weights, vKnots, uKnots, vMults, uMults, suf.m_uDegree, suf.m_vDegree, Standard_False, Standard_False
			);
			Handle(Geom_Surface) geomSurface = bSplineSurface;

			// 将Geom_Surface转换为TopoDS_Shape
			TopoDS_Shape shape = BRepBuilderAPI_MakeFace(geomSurface, 1e-6);
			builder.Add(resultCompound, shape);
		}
		

	}

	IGESControl_Writer writer;
	writer.AddShape(resultCompound);
	auto result = writer.Write(outputPath.c_str());
}
