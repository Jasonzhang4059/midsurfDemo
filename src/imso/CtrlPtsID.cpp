#include "stdafx.h"
#include "CtrlPtsID.h"
#include "RWGeometric.h"

void CPolyParaVolume::CreateHCFData()
{
	RWGeometric rwg;
	varray<NurbsVol> vols;
	rwg.ReadNurbsVol("test//hcfvols.txt", vols);
	m_HexVolumes.clear();
	m_all4sidePatch.clear();
	
	for (int i = 0; i < vols.size(); ++i)
	{
		IGAHexCell hex;
		sidePatch spatch;
		hex.m_splVol = vols[i];
		for (int j = 0; j < 6; ++j)
		{
			spatch.m_patchId = 6 * i + j;
			m_all4sidePatch.push_back(spatch);
			hex.m_6PatchIdx[j] = spatch.m_patchId;
		}
		m_HexVolumes.push_back(hex);
	}
	m_HexVolumes[0].m_faceAdjacentHexIdx[0] = 0;
	m_HexVolumes[0].m_faceAdjacentHexIdx[1] = 0;
	m_HexVolumes[0].m_faceAdjacentHexIdx[3] = 1;

	m_HexVolumes[1].m_faceAdjacentHexIdx[0] = 1;
	m_HexVolumes[1].m_faceAdjacentHexIdx[1] = 1;
	m_HexVolumes[1].m_faceAdjacentHexIdx[2] = 0;

	m_all4sidePatch[0].m_twinPatchId = 1;
	m_all4sidePatch[1].m_twinPatchId = 0;

	m_all4sidePatch[3].m_twinPatchId = 8;
	m_all4sidePatch[8].m_twinPatchId = 3;

	m_all4sidePatch[6].m_twinPatchId = 7;
	m_all4sidePatch[7].m_twinPatchId = 6;

	int startID = 0;
	for (int i = 0; i < m_HexVolumes.size(); ++i)
		startID = GenerateGlobalIDforOneVolume(i, startID);
	OutputParaVolumeDataTxt("test//ctrlpts.txt", "test//ctrlptsIdx.txt");
}

void  CPolyParaVolume::OutputParaVolumeDataTxt(const string CtrlPtsfilename, const string CtrlPtsIDfilename)
{
	ofstream ofs(CtrlPtsfilename);
	ofs << "PN " << " " << m_HexVolumes.size() << "\n";  //patch number
	for (int i = 0; i<m_HexVolumes.size(); i++)
	{
		ofs << "PI" << " " << i << "\n";   //当前片的id号。

		SplineVolume& sv = m_HexVolumes.at(i).m_splVol;
		ofs << "OD" << " " << sv.m_uDegree + 1 << " " << sv.m_vDegree + 1 << " " << sv.m_wDegree + 1 << "\n"; //Order

		ofs << "UK" << " " << sv.m_uKnots.size() << "\n";
		for (int i = 0; i<sv.m_uKnots.size(); i++)
			ofs << sv.m_uKnots.at(i) << " ";   //节点向量
		ofs << "\n";
		ofs << "VK" << " " << sv.m_vKnots.size() << "\n";
		for (int i = 0; i<sv.m_vKnots.size(); i++)
			ofs << sv.m_vKnots.at(i) << " ";   //节点向量
		ofs << "\n";
		ofs << "WK" << " " << sv.m_wKnots.size() << "\n";
		for (int i = 0; i<sv.m_wKnots.size(); i++)
			ofs << sv.m_wKnots.at(i) << " ";   //节点向量
		ofs << "\n";

		ofs << "CP" << " " << sv.m_uNum << " " << sv.m_vNum << " " << sv.m_wNum << "\n"; //control point number
		for (int i = 0; i < sv.m_CtrlPts.size(); i++)
			ofs << sv.m_CtrlPts.at(i).x << " " << sv.m_CtrlPts.at(i).y << " " << sv.m_CtrlPts.at(i).z << " " << sv.m_CtrlPts.at(i).w << "\n";
	}
	ofs.close();
	//控制点编号输出
	ofstream idofs(CtrlPtsIDfilename);
	ofs << "PN " << " " << m_HexVolumes.size() << "\n";  //patch number
	for (int i = 0; i<m_HexVolumes.size(); i++)
	{
		int idxnum = m_HexVolumes.at(i).m_controlPtGlobalIDs.size();
		idofs << "PN" << " " << i << "\n";
		idofs << "PI" << " " << idxnum << "\n";
		for (int j = 0; j<idxnum; j++)
		{
			idofs << m_HexVolumes.at(i).m_controlPtGlobalIDs.at(j) << " ";
		}
		idofs << "\n";
	}
	//输出有约束或者有力的控制点编号。
	OutputConstraintsAndLoadByCtrlID(idofs);
	idofs.close();
}

//startID是当前给定的新的编号可以允许开始编号的值。 在对当前volume赋值完成以后，返回一个允许编号的值。
int  CPolyParaVolume::GenerateGlobalIDforOneVolume(int volumeid, int startID)
{
	if (startID < 0 || volumeid < 0 || volumeid >= m_HexVolumes.size())
		return 0;
	//首先初始化控制点编号赋值。'
	IGAHexCell& vol = m_HexVolumes.at(volumeid);
	int unum = vol.m_splVol.m_uNum;
	int vnum = vol.m_splVol.m_vNum;
	int wnum = vol.m_splVol.m_wNum;

	//首先赋初值。
	int maxsize = unum*vnum*wnum;
	vol.m_controlPtGlobalIDs.resize(maxsize,-1);

	int pcount = 0;
	int idx = 0;

	int twinPatchID[6], adjacenHexID[6];

	twinPatchID[U0sfPos] = m_all4sidePatch.at(vol.m_6PatchIdx.at(U0sfPos)).m_twinPatchId;
	twinPatchID[U1sfPos] = m_all4sidePatch.at(vol.m_6PatchIdx.at(U1sfPos)).m_twinPatchId;
	twinPatchID[V0sfPos] = m_all4sidePatch.at(vol.m_6PatchIdx.at(V0sfPos)).m_twinPatchId;
	twinPatchID[V1sfPos] = m_all4sidePatch.at(vol.m_6PatchIdx.at(V1sfPos)).m_twinPatchId;
	twinPatchID[W0sfPos] = m_all4sidePatch.at(vol.m_6PatchIdx.at(W0sfPos)).m_twinPatchId;
	twinPatchID[W1sfPos] = m_all4sidePatch.at(vol.m_6PatchIdx.at(W1sfPos)).m_twinPatchId;

	adjacenHexID[U0sfPos] = vol.m_faceAdjacentHexIdx.at(U0sfPos);
	adjacenHexID[U1sfPos] = vol.m_faceAdjacentHexIdx.at(U1sfPos);
	adjacenHexID[V0sfPos] = vol.m_faceAdjacentHexIdx.at(V0sfPos);
	adjacenHexID[V1sfPos] = vol.m_faceAdjacentHexIdx.at(V1sfPos);
	adjacenHexID[W0sfPos] = vol.m_faceAdjacentHexIdx.at(W0sfPos);
	adjacenHexID[W1sfPos] = vol.m_faceAdjacentHexIdx.at(W1sfPos);

	for (int k = 0; k<wnum; k++)
	{
		for (int j = 0; j<vnum; j++)
		{
			for (int i = 0; i<unum; i++)
			{
				//以下代码先将特殊情况处理掉。
				if (i == 0)
				{
					if (adjacenHexID[U0sfPos] != -1 && m_HexVolumes.at(adjacenHexID[U0sfPos]).m_bGenerateGlobalID)
					{
						IGAHexCell& adjcentBlock = m_HexVolumes.at(adjacenHexID[U0sfPos]);
						int muidx, mvidx, mwidx;
						GetMirrorIJKOnShareSurface(volumeid, adjacenHexID[U0sfPos], 0, i, j, k, muidx, mvidx, mwidx);
						int adidx = m_HexVolumes.at(adjacenHexID[U0sfPos]).GetGlobalIDByLocalID(muidx, mvidx, mwidx);
						vol.m_controlPtGlobalIDs.at(idx++) = adidx;
						continue;
					}
				}
				else if (i == unum - 1)
				{
					if (adjacenHexID[U1sfPos] != -1 && m_HexVolumes.at(adjacenHexID[U1sfPos]).m_bGenerateGlobalID)
					{
						IGAHexCell& adjcentBlock = m_HexVolumes.at(adjacenHexID[U1sfPos]);
						int muidx, mvidx, mwidx;
						GetMirrorIJKOnShareSurface(volumeid, adjacenHexID[U1sfPos], 0, i, j, k, muidx, mvidx, mwidx);
						int adidx = m_HexVolumes.at(adjacenHexID[U1sfPos]).GetGlobalIDByLocalID(muidx, mvidx, mwidx);
						vol.m_controlPtGlobalIDs.at(idx++) = adidx;
						continue;
					}
				}
				if (j == 0)
				{
					if (adjacenHexID[V0sfPos] != -1 && m_HexVolumes.at(adjacenHexID[V0sfPos]).m_bGenerateGlobalID)
					{
						IGAHexCell& adjcentBlock = m_HexVolumes.at(adjacenHexID[V0sfPos]);
						int muidx, mvidx, mwidx;
						GetMirrorIJKOnShareSurface(volumeid, adjacenHexID[V0sfPos], 1, i, j, k, muidx, mvidx, mwidx);
						int adidx = m_HexVolumes.at(adjacenHexID[V0sfPos]).GetGlobalIDByLocalID(muidx, mvidx, mwidx);
						vol.m_controlPtGlobalIDs.at(idx++) = adidx;
						continue;
					}
				}
				else if (j == vnum - 1)
				{
					if (adjacenHexID[V1sfPos] != -1 && m_HexVolumes.at(adjacenHexID[V1sfPos]).m_bGenerateGlobalID)
					{
						IGAHexCell& adjcentBlock = m_HexVolumes.at(adjacenHexID[V1sfPos]);
						int muidx, mvidx, mwidx;
						GetMirrorIJKOnShareSurface(volumeid, adjacenHexID[V1sfPos], 1, i, j, k, muidx, mvidx, mwidx);
						int adidx = m_HexVolumes.at(adjacenHexID[V1sfPos]).GetGlobalIDByLocalID(muidx, mvidx, mwidx);
						vol.m_controlPtGlobalIDs.at(idx++) = adidx;
						continue;
					}
				}
				if (k == 0)
				{
					if (adjacenHexID[W0sfPos] != -1 && m_HexVolumes.at(adjacenHexID[W0sfPos]).m_bGenerateGlobalID)
					{
						IGAHexCell& adjcentBlock = m_HexVolumes.at(adjacenHexID[W0sfPos]);
						int muidx, mvidx, mwidx;
						GetMirrorIJKOnShareSurface(volumeid, adjacenHexID[W0sfPos], 2, i, j, k, muidx, mvidx, mwidx);
						int adidx = m_HexVolumes.at(adjacenHexID[W0sfPos]).GetGlobalIDByLocalID(muidx, mvidx, mwidx);
						vol.m_controlPtGlobalIDs.at(idx++) = adidx;
						continue;
					}
				}
				else if (k == wnum - 1)
				{
					if (adjacenHexID[W1sfPos] != -1 && m_HexVolumes.at(adjacenHexID[W1sfPos]).m_bGenerateGlobalID)
					{
						IGAHexCell& adjcentBlock = m_HexVolumes.at(adjacenHexID[W1sfPos]);
						int muidx, mvidx, mwidx;
						GetMirrorIJKOnShareSurface(volumeid, adjacenHexID[W1sfPos], 2, i, j, k, muidx, mvidx, mwidx);
						int adidx = m_HexVolumes.at(adjacenHexID[W1sfPos]).GetGlobalIDByLocalID(muidx, mvidx, mwidx);
						vol.m_controlPtGlobalIDs.at(idx++) = adidx;
						continue;
					}
				}
				//特殊情况已经处理完毕，就剩正常情况的。
				vol.m_controlPtGlobalIDs.at(idx++) = pcount + startID;
				pcount++;
			}
		}
	}
	//然后搜寻周边已经赋值的hex，直接将其已赋值的编号值取出来。
	startID += pcount;
	vol.m_bGenerateGlobalID = true;

	//检查一下数据是否正确。
	bool checkflag = true;
	for (int i = 0; i<maxsize; i++)
	{
		if (vol.m_controlPtGlobalIDs.at(i) == -1)
		{
			checkflag = false;
			break;
		}
	}
	ASSERT(checkflag);   //当前编号不能存在-1.
	checkflag = false;
	for (int i = 0; i<maxsize - 1; i++)
	{
		for (int j = i + 1; j<maxsize; j++)
		{
			if (vol.m_controlPtGlobalIDs.at(i) == vol.m_controlPtGlobalIDs.at(j))
			{
				checkflag = true;
				break;
			}
		}
	}
	ASSERT(!checkflag);  //当前编号不能有重复的号码。
	int maxid = -1;
	maxsize = vol.m_controlPtGlobalIDs.size();
	for (int i = 0; i<maxsize; i++)
	{
		if (vol.m_controlPtGlobalIDs.at(i) > maxid)
			maxid = vol.m_controlPtGlobalIDs.at(i);
	}
	ASSERT(maxid + 1 == startID);   //当前最大编号作为下个体的最小编号。
									//最后返回一个值，该值用于下一个volume的编号起始ID。
	return startID;
};

bool  CPolyParaVolume::GetMirrorIJKOnShareSurface(int volume1, int volume2, int imode, int uid, int vid, int wid, int& muid, int& mvid, int& mwid)
{
	muid = mvid = mwid = -1;
	if (volume1 < 0 || volume1 >= m_HexVolumes.size() || volume2 < 0 || volume2 >= m_HexVolumes.size())
		return false;
	muid = uid;
	mvid = vid;
	mwid = wid;
	if (imode == 0)  //关于正负u向对称。
	{
		muid = m_HexVolumes.at(volume2).m_splVol.m_uNum - uid - 1;
	}
	else if (imode == 1)//关于正负v向对称。
	{
		mvid = m_HexVolumes.at(volume2).m_splVol.m_vNum - vid - 1;
	}
	else if (imode == 2)//关于正负w向对称。
	{
		mwid = m_HexVolumes.at(volume2).m_splVol.m_wNum - wid - 1;
	}

	return (muid >= 0 && mvid >= 0 && mwid >= 0 && muid < m_HexVolumes.at(volume2).m_splVol.m_uNum && mvid < m_HexVolumes.at(volume2).m_splVol.m_vNum && mwid < m_HexVolumes.at(volume2).m_splVol.m_wNum);
}

void  CPolyParaVolume::OutputConstraintsAndLoadByCtrlID(ofstream& idofs)
{
	//for sphere
	if (m_HexVolumes.size() == 1)
	{
		idofs << "WC" << " "; //有约束的。下底面
		for (int j = 0; j<m_HexVolumes.at(0).m_splVol.m_vNum; j++)
		{
			for (int i = 0; i<m_HexVolumes.at(0).m_splVol.m_uNum; i++)
			{
				int idx = m_HexVolumes.at(0).GetGlobalIDByLocalID(i, j, 0);
				idofs << idx << " ";
			}
		}
		idofs << "\n";
		idofs << "WF" << " "; //存在受力的。上底面
		for (int j = 0; j<m_HexVolumes.at(0).m_splVol.m_vNum; j++)
		{
			for (int i = 0; i<m_HexVolumes.at(0).m_splVol.m_uNum; i++)
			{
				int idx = m_HexVolumes.at(0).GetGlobalIDByLocalID(i, j, m_HexVolumes.at(0).m_splVol.m_wNum - 1);
				idofs << idx << " ";
			}
		}
		idofs << "\n";
	}


	//for L-shape
	if (m_HexVolumes.size() == 3)
	{
		idofs << "WC" << " "; //有约束的。 左端面
		for (int k = 0; k<m_HexVolumes.at(2).m_splVol.m_wNum; k++)
		{
			for (int j = 0; j<m_HexVolumes.at(2).m_splVol.m_vNum; j++)
			{
				int idx = m_HexVolumes.at(2).GetGlobalIDByLocalID(0, j, k);
				idofs << idx << " ";
			}
		}
		idofs << "\n";
		idofs << "WF" << " "; //存在受力的。  y轴上端面
		for (int k = 0; k<m_HexVolumes.at(0).m_splVol.m_wNum; k++)
		{
			for (int i = 0; i<m_HexVolumes.at(0).m_splVol.m_uNum; i++)
			{
				int idx = m_HexVolumes.at(0).GetGlobalIDByLocalID(i, m_HexVolumes.at(0).m_splVol.m_vNum - 1, k);
				idofs << idx << " ";
			}
		}
		idofs << "\n";
	}


	//for torus
	if (m_HexVolumes.size() == 8)
	{
		idofs << "WC" << " "; //有约束的。
							  //m_HexVolumes.at(1) 下底面。
		for (int j = 0; j<m_HexVolumes.at(1).m_splVol.m_vNum; j++)
		{
			for (int i = 0; i<m_HexVolumes.at(1).m_splVol.m_uNum; i++)
			{
				int idx = m_HexVolumes.at(1).GetGlobalIDByLocalID(i, j, 0);
				idofs << idx << " ";
			}
		}
		idofs << "\n";
		idofs << "WF" << " "; //存在受力的。
							  //m_HexVolumes.at(6) 上底面。
		for (int j = 0; j<m_HexVolumes.at(6).m_splVol.m_vNum; j++)
		{
			for (int i = 0; i<m_HexVolumes.at(6).m_splVol.m_uNum; i++)
			{
				int idx = m_HexVolumes.at(6).GetGlobalIDByLocalID(i, j, m_HexVolumes.at(6).m_splVol.m_wNum - 1);
				idofs << idx << " ";
			}
		}
		idofs << "\n";
	}

	//for deckel
	if (m_HexVolumes.size() == 220)
	{
		idofs << "WC" << " "; //有约束的。
							  //m_HexVolumes.at(6)  //左端面    //不好找，需要写个函数找。
		varray<int> ConstraintsIDs;
		Find6ExtremEndsurfaceControlpointIDs(0, ConstraintsIDs);
		for (int ii = 0; ii<ConstraintsIDs.size(); ii++)
		{
			idofs << ConstraintsIDs.at(ii) << " ";
		}
		idofs << "\n";

		idofs << "WF" << " "; //存在受力的。
							  //m_HexVolumes.at(126) //右中心面。
							  /*for(int k=0; k<m_HexVolumes.at(126).m_splVol.m_wNum; k++)
							  {
							  for(int j=0; j<m_HexVolumes.at(126).m_splVol.m_vNum;j++)
							  {
							  int idx = m_HexVolumes.at(126).GetGlobalIDByLocalID(m_HexVolumes.at(126).m_splVol.m_uNum-1,j,k);
							  idofs << idx << " ";
							  }
							  }*/
		varray<int> loadIDs;
		Find6ExtremEndsurfaceControlpointIDs(1, loadIDs);
		for (int jj = 0; jj<loadIDs.size(); jj++)
		{
			idofs << loadIDs.at(jj) << " ";
		}
		idofs << "\n";
	}
}

//surfaceID = 0 for xmin;
//surfaceID = 1 for xmax;
//surfaceID = 2 for ymin;
//surfaceID = 3 for ymax;
//surfaceID = 4 for zmin;
//surfaceID = 5 for zmax;
//CtrlptGlobalIDs 中无重复的控制点编号。
void  CPolyParaVolume::Find6ExtremEndsurfaceControlpointIDs(int surfaceID, varray<int>& CtrlptGlobalIDs)
{
	CtrlptGlobalIDs.clear();

	float extremVal = 0;
	int volumeNum = m_HexVolumes.size();
	int volCtrlnum;
	point4d vt;
	float valtorlerence = 5.0;
	if (surfaceID == 0 || surfaceID == 1)
	{
		if (surfaceID == 0)
		{
			extremVal = 1.0e6;
			for (int i = 0; i<volumeNum; i++)
			{
				IGAHexCell& hexcell = m_HexVolumes.at(i);
				volCtrlnum = hexcell.m_controlPtGlobalIDs.size();
				for (int j = 0; j<volCtrlnum; j++)
				{
					vt = hexcell.GetControlPtByGlobalID(hexcell.m_controlPtGlobalIDs.at(j));
					if (vt.x < extremVal)
					{
						extremVal = vt.x;
					}
				}
			}
		}
		else
		{
			extremVal = -1.0e6;
			for (int i = 0; i<volumeNum; i++)
			{
				IGAHexCell& hexcell = m_HexVolumes.at(i);
				volCtrlnum = hexcell.m_controlPtGlobalIDs.size();
				for (int j = 0; j<volCtrlnum; j++)
				{
					vt = hexcell.GetControlPtByGlobalID(hexcell.m_controlPtGlobalIDs.at(j));
					if (vt.x > extremVal)
					{
						extremVal = vt.x;
					}
				}
			}
		}
		for (int volid = 0; volid<volumeNum; volid++)
		{
			IGAHexCell& hexcell = m_HexVolumes.at(volid);
			for (int k = 0; k<hexcell.m_splVol.m_wNum; k++)
			{
				for (int j = 0; j<hexcell.m_splVol.m_vNum; j++)
				{
					for (int i = 0; i<hexcell.m_splVol.m_uNum; i++)
					{
						if (abs(hexcell.m_splVol.GetControlPoint(i, j, k).x - extremVal) < valtorlerence)
						{
							CtrlptGlobalIDs.push_back(hexcell.GetGlobalIDByLocalID(i, j, k));
						}
					}
				}
			}
		}
	}
	else if (surfaceID == 2 || surfaceID == 3)
	{
		if (surfaceID == 2)
		{
			extremVal = 1.0e6;
			for (int i = 0; i<volumeNum; i++)
			{
				IGAHexCell& hexcell = m_HexVolumes.at(i);
				volCtrlnum = hexcell.m_controlPtGlobalIDs.size();
				for (int j = 0; j<volCtrlnum; j++)
				{
					vt = hexcell.GetControlPtByGlobalID(hexcell.m_controlPtGlobalIDs.at(j));
					if (vt.y < extremVal)
					{
						extremVal = vt.y;
					}
				}
			}
		}
		else
		{
			extremVal = -1.0e6;
			for (int i = 0; i<volumeNum; i++)
			{
				IGAHexCell& hexcell = m_HexVolumes.at(i);
				volCtrlnum = hexcell.m_controlPtGlobalIDs.size();
				for (int j = 0; j<volCtrlnum; j++)
				{
					vt = hexcell.GetControlPtByGlobalID(hexcell.m_controlPtGlobalIDs.at(j));
					if (vt.y > extremVal)
					{
						extremVal = vt.y;
					}
				}
			}
		}
		for (int volid = 0; volid<volumeNum; volid++)
		{
			IGAHexCell& hexcell = m_HexVolumes.at(volid);
			for (int k = 0; k<hexcell.m_splVol.m_wNum; k++)
			{
				for (int j = 0; j<hexcell.m_splVol.m_vNum; j++)
				{
					for (int i = 0; i<hexcell.m_splVol.m_uNum; i++)
					{
						if (abs(hexcell.m_splVol.GetControlPoint(i, j, k).y - extremVal) < valtorlerence)
						{
							CtrlptGlobalIDs.push_back(hexcell.GetGlobalIDByLocalID(i, j, k));
						}
					}
				}
			}
		}
	}
	else if (surfaceID == 4 || surfaceID == 5)
	{
		if (surfaceID == 4)
		{
			extremVal = 1.0e6;
			for (int i = 0; i<volumeNum; i++)
			{
				IGAHexCell& hexcell = m_HexVolumes.at(i);
				volCtrlnum = hexcell.m_controlPtGlobalIDs.size();
				for (int j = 0; j<volCtrlnum; j++)
				{
					vt = hexcell.GetControlPtByGlobalID(hexcell.m_controlPtGlobalIDs.at(j));
					if (vt.z < extremVal)
					{
						extremVal = vt.z;
					}
				}
			}
		}
		else
		{
			extremVal = -1.0e6;
			for (int i = 0; i<volumeNum; i++)
			{
				IGAHexCell& hexcell = m_HexVolumes.at(i);
				volCtrlnum = hexcell.m_controlPtGlobalIDs.size();
				for (int j = 0; j<volCtrlnum; j++)
				{
					vt = hexcell.GetControlPtByGlobalID(hexcell.m_controlPtGlobalIDs.at(j));
					if (vt.z > extremVal)
					{
						extremVal = vt.z;
					}
				}
			}
		}
		for (int volid = 0; volid<volumeNum; volid++)
		{
			IGAHexCell& hexcell = m_HexVolumes.at(volid);
			for (int k = 0; k<hexcell.m_splVol.m_wNum; k++)
			{
				for (int j = 0; j<hexcell.m_splVol.m_vNum; j++)
				{
					for (int i = 0; i<hexcell.m_splVol.m_uNum; i++)
					{
						if (abs(hexcell.m_splVol.GetControlPoint(i, j, k).z - extremVal) < valtorlerence)
						{
							CtrlptGlobalIDs.push_back(hexcell.GetGlobalIDByLocalID(i, j, k));
						}
					}
				}
			}
		}
	}
	//删除数组中重复的数据。
	int idCount;
	idCount = CtrlptGlobalIDs.size();
	for (int i = 0; i<idCount; i++)
	{
		for (int j = i + 1; j<idCount; j++)
		{
			if (CtrlptGlobalIDs.at(i) == CtrlptGlobalIDs.at(j))
			{
				CtrlptGlobalIDs.erase(CtrlptGlobalIDs.begin() + j);
				idCount--;
			}
		}
	}
}

point4d IGAHexCell::GetControlPtByGlobalID(int globalID)
{
	int idu, idv, idw;
	GetLocalIDByGlobalID(idu, idv, idw, globalID);
	point4d vt = m_splVol.GetControlPoint(idu, idv, idw);
	return vt;
}

bool IGAHexCell::GetLocalIDByGlobalID(int& uid, int& vid, int& wid, int globalID)
{
	if (!m_bGenerateGlobalID)
		return false;
	int i;
	uid = vid = wid = -1;
	for (i = 0; i < m_controlPtGlobalIDs.size(); i++)
	{
		if (m_controlPtGlobalIDs.at(i) == globalID)
		{
			break;
		}
	}
	if (i == m_controlPtGlobalIDs.size())
		return false;

	if (!m_splVol.GetIJKIndex(i, uid, vid, wid))
	{
		return false;
	}
	return true;
}

int IGAHexCell::GetGlobalIDByLocalID(int uid, int vid, int wid)
{
	if (!m_bGenerateGlobalID)
		return -1;
	int idx = m_splVol.GetControlPointIndex(uid, vid, wid);
	int globalidx = m_controlPtGlobalIDs.at(idx);
	return globalidx;
}

int SplineVolume::GetControlPointIndex(int uid, int vid, int wid)
{
	return m_uNum*m_vNum*wid + m_uNum*vid + uid;
}

point4d SplineVolume::GetControlPoint(int idu, int idv, int idw)
{
	return m_CtrlPts[m_uNum*m_vNum*idw + m_uNum*idv + idu];
}

bool SplineVolume::GetIJKIndex(int i, int & uid, int & vid, int & wid)
{
	if (i < 0 || i >= m_CtrlPts.size())return false;

	wid = i / (m_uNum*m_vNum);
	int r = i - m_uNum*m_vNum*wid;
	vid = r / m_uNum;
	uid = r - m_uNum*vid;
	return true;
}
