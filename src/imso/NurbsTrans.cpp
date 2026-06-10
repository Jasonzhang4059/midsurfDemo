#include"NurbsTrans.h"

double* Point3dToSislPoint(const Vec3 & inputpoint, int length)
{
	double *pt = new double[length];
	pt[0] = inputpoint.x;
	pt[1] = inputpoint.y;
	if (length == 3)
		pt[2] = inputpoint.z;
	return pt;
}

SISLCurve * NurbsLineToSislLine(const Spline & inputLine)
{
	SISLCurve *curve = NULL;
	int number = inputLine.m_CtrlPts.size();
	int order = inputLine.m_Degree + 1;
	double *knots = new double[number+order];
	double *coef = new double[inputLine.m_CtrlPts.size() * 4];
	int kind = 2;
	int dim = 3;
	int copy = 1;

	double *p = knots;
	for (auto i : inputLine.m_Knots)
	{
		*(p++) = i;
	}

	p = coef;
	for (auto i : inputLine.m_CtrlPts)
	{
		*(p++) = i.x;
		*(p++) = i.y;
		*(p++) = i.z;
		*(p++) = i.w;
	}
	p = nullptr;

	////查看节点矢量与控制点是否正确
	//std::cout << "节点矢量为：" << std::endl;
	//for (int i = 0; i < (number + order); i++)
	//{
	//	std::cout << knots[i] << "   ";
	//}
	//std::cout << std::endl;

	//std::cout << "控制点为： " << std::endl;
	//for (int i = 0; i < inputLine.m_CtrlPts.size() * 4; i++)
	//{
	//	std::cout << coef[i] << "   ";
	//}
	//std::cout << std::endl;

	//构造SISL格式曲线
	curve = newCurve(number, order, knots, coef, kind, dim, copy);
	delete[] knots;
	delete[] coef;
	return curve;
}

SISLCurve * NurbsLineToSislLine(const Spline & inputLine, int dim)
{
	SISLCurve *curve = NULL;
	int number = inputLine.m_CtrlPts.size();
	int order = inputLine.m_Degree + 1;
	double *knots = new double[number + order];
	double *coef = new double[inputLine.m_CtrlPts.size() * (dim+1)];
	int kind = 2;
	int copy = 1;

	double *p = knots;
	for (auto i : inputLine.m_Knots)
	{
		*(p++) = i;
	}

	p = coef;
	for (auto i : inputLine.m_CtrlPts)
	{
		*(p++) = i.x;
		*(p++) = i.y;
		if (dim == 3)
		{
			*(p++) = i.z;
		}
		*(p++) = i.w;
	}
	p = nullptr;

	////查看维数与节点矢量与控制点是否正确
	////std::cout << "维数：" << dim << std::endl;
	////std::cout << "节点矢量为：" << std::endl;
	//for (int i = 0; i < (number + order); i++)
	//{
	//	//std::cout << knots[i] << "   ";
	//}
	//std::cout << std::endl;

	//std::cout << "控制点为： " << std::endl;
	//for (int i = 0; i < inputLine.m_CtrlPts.size() * (dim+1); i++)
	//{
	//	std::cout << coef[i] << "   ";
	//}
	//std::cout << std::endl;

	//构造SISL格式曲线
	curve = newCurve(number, order, knots, coef, kind, dim, copy);
	delete[] knots;
	delete[] coef;
	return curve;
}

Spline SislLineToNurbsLine(SISLCurve * cruve)
{
	Spline nl;
	if (!cruve) return{};
	nl.m_Degree = cruve->ik - 1;
	int num_knots = cruve->in + cruve->ik;
	nl.m_Knots.resize(num_knots);//二次三个控制点为6
	double b = cruve->et[num_knots - 1];
	for (int i = 0; i < num_knots; i++) {
		if (abs(cruve->et[i] - b) < 1e-5) {
			nl.m_Knots[i] = 1;
		}
		else {
			nl.m_Knots[i] = cruve->et[i] / b;
		}
	}

	int pos = 0;
	nl.m_CtrlPts.resize(cruve->in);
	for (auto& p : nl.m_CtrlPts) {
		p.x = cruve->ecoef[pos];
		++pos;
		p.y = cruve->ecoef[pos];
		++pos;
		p.z = cruve->ecoef[pos];
		++pos;
		p.w = 1;
	}
	return nl;
}

int CalLinePoint(SISLCurve *& curve, double u, std::vector<std::vector<double>>& mderive, int der)
{
	if (curve == nullptr)
	{
		assert(curve);
		return -1;
	}
	mderive.clear();
	//int der = 1;
	int leftknots;
	int dim = curve->idim;			//欧几里得空间维度
	double* derive = new double[(der + 1)*dim];
	int stat;
	s1227(curve, der, u, &leftknots, derive, &stat);
	if (stat < 0)
	{
		assert(0);
		return -1;
	}
	int cur = 0;
	while (cur < (der + 1)*dim) {
		//存入u处点坐标及切向量
		std::vector<double> tmp;
		for (int i = 0; i < dim; i++) {
			tmp.push_back(derive[cur++]);
		}
		mderive.push_back(tmp);
	}
	return dim;
}

double PointIntersectNurbsLine(double* &pt, SISLCurve *& pc,  int length)
{
	int idim = length;
	double aepsge = 0.001;
	int jpt = 0;
	double *gpar1 = NULL;
	int jcrv = 0;
	SISLIntcurve **wcurve = NULL;
	int jstat = 0;
	s1871(pc, pt, idim, aepsge, &jpt, &gpar1, &jcrv, &wcurve, &jstat);
	if (jpt)
	{
		//std::cout << "\n点线交点u为：  " << *gpar1 << std::endl;
		return *gpar1;
	}
	else
	{
		return -1;
	}
}

bool ISTwoNurbsLineIntersect(SISLCurve *& curve1, SISLCurve *& curve2)
{
	double epsco = 0.000001;
	double epsge = 0.001;
	double *intpar1 = nullptr;
	double *intpar2 = nullptr;
	int numintpt;
	int numintcu;
	SISLIntcurve **intcurve;
	int stat;

	s1857(curve1, curve2, epsco, epsge, &numintpt, &intpar1, &intpar2, &numintcu, &intcurve, &stat);
	if (intpar1) free(intpar1);
	if (intpar2) free(intpar2);

	if (numintpt == 1 || numintcu == 0)       //存在交点
	{
		return true;
	}
	else                                      //两曲线不存在交点
	{
		return false;
	}
}

//判断两曲线是否有交点
bool ISTwoNurbsLineIntersect(const Spline &line1, const Spline &line2, int dim)
{
	cout << "利用SISLCurve曲线判断两线是否相交" << endl;
	SISLCurve* l1 = NurbsLineToSislLine(line1, dim);
	SISLCurve* l2 = NurbsLineToSislLine(line1, dim);
	bool res = ISTwoNurbsLineIntersect(l1, l2);
	if (l1) freeCurve(l1);
	if (l2) freeCurve(l2);
	return res;
}

//两曲线相交 1：存在交点 2：两曲线存在重合段 0：不存在交点
int TwoNurbsLineIntersect(SISLCurve *& curve1, SISLCurve *& curve2, double *& intpar1, double *& intpar2)
{
	double epsco = 0.000001;
	double epsge = 0.001;
	int numintpt;
	int numintcu;
	SISLIntcurve **intcurve;
	int stat;

	s1857(curve1, curve2, epsco, epsge, &numintpt, &intpar1, &intpar2, &numintcu, &intcurve, &stat);

	if ((numintpt == 1) && (numintcu == 0))       //存在交点(后续应分析intpar1和intpar2是否都有值)
	{
		return 1;
	}
	else if ((numintpt == 0) && (numintcu != 0))   //两曲线存在重合段(后续应计算端点是否重合，以及一条线端点是否在另一条线上)
	{
		return 2;
	}
	else                                          //两曲线不存在交点
	{
		return 0;
	}
}

int TwoNurbsLineIntersectVer2(SISLCurve *& curve1, SISLCurve *& curve2, double *& intpar1, double *& intpar2, int & numintpt, int & numintcu)
{
	double epsco = 0.000001;
	double epsge = 0.001;
	//int numintpt;
	//int numintcu;
	SISLIntcurve **intcurve;
	int stat;

	s1857(curve1, curve2, epsco, epsge, &numintpt, &intpar1, &intpar2, &numintcu, &intcurve, &stat);

	if ((numintpt == 1) && (numintcu == 0))       //存在交点(后续应分析intpar1和intpar2是否都有值)
	{
		return 1;
	}
	else if ((numintpt == 0) && (numintcu != 0))   //两曲线存在重合段(后续应计算端点是否重合，以及一条线端点是否在另一条线上)
	{
		return 2;
	}
	else                                          //两曲线不存在交点
	{
		return 0;
	}
}

int StraLineIntersectNurbsLine(SISLCurve *& curve, const varray<Vec3>& strLine, double *& intpar, int & numintpt, int & numintcu)
{
	double epsco = 0.000001;
	double epsge = 0.001;
	SISLIntcurve **intcurve;
	int stat = -1;				//默认error
	int dim = 2;				//维数
	double *point = new double[strLine.size() * 2];
	double normal[2] = { 0,1 };
	if (strLine.size() < 3)
	{
		dim = 2;
		for (int i = 0; i < strLine.size(); i++)
		{
			point[i * 2] = strLine[i].x;
			point[i * 2 + 1] = strLine[i].y;
		}
	}
	else
	{
		dim = 3;
	}

	s1850(curve, point, normal, dim, epsco, epsge, &numintpt, &intpar, &numintcu, &intcurve, &stat);
	delete[] point;

	for (int i = 0; i < numintpt; i++) {
		//判断求得的两个交点是否是一个点
		for (int j = i + 1; j < numintpt;) {
			if (fabs(intpar[i] - intpar[j]) < 1e-20) {
				//两个交点为同一点
				for (int k = j + 1; k < numintpt; k++) {
					intpar[k - 1] = intpar[k];
				}
				numintpt--;
			}
			else
			{
				j++;
			}
		}
	}
	
	if ((numintpt > 0) && (numintcu == 0))			//存在交点
	{
		return 1;
	}
	else if ((numintpt == 0) && (numintcu != 0))   //存在重合段
	{
		return 2;
	}
	else                                          //两曲线不存在交点
	{
		return 0;
	}
}

bool isInterPoint(double & u, SISLCurve * curve, const Spline * nurbsCurve)
{
	std::vector<std::vector<double>> mderive;
	varray<Vec4> Der;
	CalLinePoint(curve, u, mderive);
	int val_y = mderive[0][1];//交点的y坐标

	nurbsCurve->PtsDerivs(u, 1, Der);
	if (fabs(Der[0].y - val_y) < 1e-20)
	{
		//当前点为交点
		return true;
	}

	//如果当前u求出结果与SISL求的结果不同，则向两个方向开始取值

	return true;
}

SISLCurve * FitBspline(const varray<Vec3>& p, int degree, int dim)
{
	SISLCurve *rc = nullptr;
	double *epoint = new double[dim*p.size()];
	int inbpnt = p.size();
	int* nptyp = new int[p.size()];
	int icnsta = 0;
	int icnend = 0;
	int iopen = 1;
	int ik = degree;
	double astpar = 0.0;
	double cendpar = 0.0;
	double *gpar = nullptr;
	int jnbpar = 0;
	int jstat;

	//存入插值点坐标
	int pos = 0;
	for (auto& pcoord : p) {
		epoint[pos] = pcoord.x;
		++pos;
		epoint[pos] = pcoord.y;
		++pos;
		epoint[pos] = pcoord.z;
		++pos;
	}
	for (int i = 0; i < p.size(); i++) {
		nptyp[i] = 1;
	}
	s1356(epoint, inbpnt, dim, nptyp, icnsta, icnend, iopen, ik, astpar, &cendpar, &rc, &gpar, &jnbpar, &jstat);

	return rc;
}

Spline FitBsplineCnurbs(const varray<Vec3>& p, int degree, int dim)
{
	Spline nl;
	SISLCurve* cv = nullptr;
	cv = FitBspline(p, degree, dim);
	nl = SislLineToNurbsLine(cv);
	freeCurve(cv);
	return nl;
}

Spline NurbsTrans::CnurbslineToSpline(const NurbsLine & nl)
{
	Spline sl;
	sl.m_Degree = nl.m_Degree;
	sl.m_Knots = nl.m_Knots;
	sl.m_CtrlPts.resize(nl.m_CtrlPts.size());
	for (int i = 0; i < sl.m_CtrlPts.size(); ++i) {
		sl.m_CtrlPts[i] = Vec4(nl.m_CtrlPts[i].x, nl.m_CtrlPts[i].y, nl.m_CtrlPts[i].z, nl.m_CtrlPts[i].w);
	}
	return sl;
}

NurbsLine NurbsTrans::SplineToCnurbsline(const Spline & sl)
{
	NurbsLine nl;
	nl.m_Degree = sl.m_Degree;
	nl.m_Knots = sl.m_Knots;
	nl.m_CtrlPts.resize(sl.m_CtrlPts.size());
	for (int i = 0; i < nl.m_CtrlPts.size(); ++i) {
		nl.m_CtrlPts[i] = point4d(sl.m_CtrlPts[i].x, sl.m_CtrlPts[i].y, sl.m_CtrlPts[i].z, sl.m_CtrlPts[i].w);
	}
	return nl;
}

varray<Spline> NurbsTrans::ClinesToSplines(const varray<NurbsLine>& nls)
{
	varray<Spline> sls(nls.size());
	for (int i = 0; i < sls.size(); ++i) {
		sls[i] = CnurbslineToSpline(nls[i]);
	}
	return sls;
}

varray<NurbsLine> NurbsTrans::SplinesToClines(const varray<Spline>& sls)
{
	varray<NurbsLine> nls(sls.size());
	for (int i = 0; i < nls.size(); ++i) {
		nls[i] = SplineToCnurbsline(sls[i]);
	}
	return nls;
}

SplineSurface NurbsTrans::CnurbssurfToSplinesurf(const NurbsSurface & nsf)
{
	SplineSurface sf;
	sf.m_uDegree = nsf.m_uDegree;
	sf.m_vDegree = nsf.m_vDegree;
	sf.m_uNum = nsf.m_uNum;
	sf.m_vNum = nsf.m_vNum;
	sf.m_uKnots = nsf.m_uKnots;
	sf.m_vKnots = nsf.m_vKnots;
	sf.m_CtrlPts.resize(nsf.m_CtrlPts.size());
	for (int i = 0; i < sf.m_CtrlPts.size(); ++i) {
		sf.m_CtrlPts[i] = Vec4(nsf.m_CtrlPts[i].x, nsf.m_CtrlPts[i].y, nsf.m_CtrlPts[i].z, nsf.m_CtrlPts[i].w);
	}
	return sf;
}

NurbsSurface NurbsTrans::SplinesurfToCnurbssurf(const SplineSurface & sf)
{
	NurbsSurface nsf;
	nsf.m_uDegree = sf.m_uDegree;
	nsf.m_vDegree = sf.m_vDegree;
	nsf.m_uNum = sf.m_uNum;
	nsf.m_vNum = sf.m_vNum;
	nsf.m_uKnots = sf.m_uKnots;
	nsf.m_vKnots = sf.m_vKnots;
	nsf.m_CtrlPts.resize(sf.m_CtrlPts.size());
	for (int i = 0; i < nsf.m_CtrlPts.size(); ++i) {
		nsf.m_CtrlPts[i] = point4d(sf.m_CtrlPts[i].x, sf.m_CtrlPts[i].y, sf.m_CtrlPts[i].z, sf.m_CtrlPts[i].w);
	}
	return nsf;
}

varray<SplineSurface> NurbsTrans::CsurfsToSplinesurfs(const varray<NurbsSurface>& nsfs)
{
	varray<SplineSurface> sfs(nsfs.size());
	for (int i = 0; i < sfs.size(); ++i) {
		sfs[i] = CnurbssurfToSplinesurf(nsfs[i]);
	}
	return sfs;
}

varray<NurbsSurface> NurbsTrans::SplinesurfsToCsurfs(const varray<SplineSurface>& sfs)
{
	varray<NurbsSurface> nsfs(sfs.size());
	for (int i = 0; i < nsfs.size(); ++i) {
		nsfs[i] = SplinesurfToCnurbssurf(sfs[i]);
	}
	return nsfs;
}

SplineVolume NurbsTrans::CnurbsvolToSplinevol(const NurbsVol & nvol)
{
	SplineVolume vol;
	vol.m_uDegree = nvol.m_uDegree;
	vol.m_vDegree = nvol.m_vDegree;
	vol.m_wDegree = nvol.m_wDegree;
	vol.m_uNum = nvol.m_uNum;
	vol.m_vNum = nvol.m_vNum;
	vol.m_wNum = nvol.m_wNum;
	vol.m_uKnots = nvol.m_uKnots;
	vol.m_vKnots = nvol.m_vKnots;
	vol.m_wKnots = nvol.m_wKnots;
	vol.m_CtrlPts.resize(nvol.m_CtrlPts.size());
	for (int i = 0; i < vol.m_CtrlPts.size(); ++i) {
		vol.m_CtrlPts[i] = Vec4(nvol.m_CtrlPts[i].x, nvol.m_CtrlPts[i].y, nvol.m_CtrlPts[i].z, nvol.m_CtrlPts[i].w);
	}
	return vol;
}

NurbsVol NurbsTrans::SplinevolToCnurbsvol(const SplineVolume & vol)
{
	NurbsVol nvol;
	nvol.m_uDegree = vol.m_uDegree;
	nvol.m_vDegree = vol.m_vDegree;
	nvol.m_wDegree = vol.m_wDegree;
	nvol.m_uNum = vol.m_uNum;
	nvol.m_vNum = vol.m_vNum;
	nvol.m_wNum = vol.m_wNum;
	nvol.m_uKnots = vol.m_uKnots;
	nvol.m_vKnots = vol.m_vKnots;
	nvol.m_wKnots = vol.m_wKnots;
	nvol.m_CtrlPts.resize(vol.m_CtrlPts.size());
	for (int i = 0; i < nvol.m_CtrlPts.size(); ++i) {
		nvol.m_CtrlPts[i] = point4d(vol.m_CtrlPts[i].x, vol.m_CtrlPts[i].y, vol.m_CtrlPts[i].z, vol.m_CtrlPts[i].w);
	}
	return nvol;
}

varray<SplineVolume> NurbsTrans::CvolsToSplinevols(const varray<NurbsVol>& nvols)
{
	varray<SplineVolume> vols(nvols.size());
	for (int i = 0; i < vols.size(); ++i) {
		vols[i] = CnurbsvolToSplinevol(nvols[i]);
	}
	return vols;
}

varray<NurbsVol> NurbsTrans::SplinevolsToCvols(const varray<SplineVolume>& vols)
{
	varray<NurbsVol> nvols(vols.size());
	for (int i = 0; i < nvols.size(); ++i) {
		nvols[i] = SplinevolToCnurbsvol(vols[i]);
	}
	return nvols;
}

void NurbsTrans::DimReduceNurbsLines(varray<Spline>& nl, varray<varray<Spline>>& allL, int mode)
{
	nl.clear();
	for (auto& ls : allL) {
		for (auto& l : ls) {
			nl.push_back(l);
		}
	}
	if (mode == 0) {
		allL.clear();
	}
}

void NurbsTrans::DimReduceNurbsSurfs(varray<SplineSurface>& nsf, varray<varray<SplineSurface>>& allsf, int mode)
{
	nsf.clear();
	for (auto& sf : allsf) {
		for (auto& s : sf) {
			nsf.push_back(s);
		}
	}
	if (mode == 0) {
		allsf.clear();
	}
}
