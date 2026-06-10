#pragma once
#include <fstream>
#include "varray.h"
#include "CNurbs.h"
using namespace std;
#ifndef U0sfPos
#define U0sfPos 0
#endif
#ifndef U1sfPos
#define U1sfPos 1
#endif
#ifndef V0sfPos
#define V0sfPos 2
#endif
#ifndef V1sfPos
#define V1sfPos 3
#endif
#ifndef W0sfPos
#define W0sfPos 4
#endif
#ifndef W1sfPos
#define W1sfPos 5
#endif
struct SplineVolume:NurbsVol
{
	SplineVolume() {}
	SplineVolume(NurbsVol& vol)
		:NurbsVol(vol)
	{
	}

	int GetControlPointIndex(int uid, int vid, int wid);
	point4d GetControlPoint(int idu, int idv, int idw);
	bool GetIJKIndex(int i, int& uid, int& vid, int& wid);
};

struct IGAHexCell
{
	SplineVolume m_splVol;
	varray<int> m_controlPtGlobalIDs;
	varray<int> m_6PatchIdx;//6个面编号
	varray<int> m_faceAdjacentHexIdx;//相邻六面体
	bool m_bGenerateGlobalID = false;

	IGAHexCell() 
	{
		m_6PatchIdx.resize(6, -1);
		m_faceAdjacentHexIdx.resize(6, -1);
	}
	int GetGlobalIDByLocalID(int uid, int vid, int wid);
	bool GetLocalIDByGlobalID(int& uid, int& vid, int& wid, int globalID);
	point4d GetControlPtByGlobalID(int globalID);
};

struct sidePatch
{
	int m_patchId;
	int m_twinPatchId = -1;//重合面编号
};

class CPolyParaVolume
{
public:
	varray<IGAHexCell> m_HexVolumes;
	varray<sidePatch> m_all4sidePatch;

	void CreateHCFData();
	
private:
	void  OutputParaVolumeDataTxt(const string CtrlPtsfilename, const string CtrlPtsIDfilename);
	void  OutputConstraintsAndLoadByCtrlID(ofstream& idofs);

	int  GenerateGlobalIDforOneVolume(int volumeid, int startID);

	bool  GetMirrorIJKOnShareSurface(int volume1, int volume2, int imode,
		int uid, int vid, int wid, int& muid, int& mvid, int& mwid);
	
	void  Find6ExtremEndsurfaceControlpointIDs(int surfaceID, varray<int>& CtrlptGlobalIDs);
};
