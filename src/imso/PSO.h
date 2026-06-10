//多维度PSO算法
//@ USST 何文彬 2018
#ifndef CPSO
#define CPSO
#include "varray.h"

class PSO
{
protected:
	int m_Dim;//维度
	int m_PNum;//粒子数量
	int m_iteraNum;//最大迭代次数
	double m_c1;//个体认知系数
	double m_c2;//社会知识系数
	double m_r;//速度更新约束因子
	double m_error;//限定误差

	varray<varray<double>> m_P;//粒子群位置
	varray<varray<double>> m_velocity;//粒子群速度

	varray<double> m_PGlobal;//全局最优值处的位置
	double m_DisGol;//全局最优适应度
	
	varray<double> m_Pmin;//粒子最小位置
	varray<double> m_Pmax;//粒子最大位置
	varray<double> m_Vmin;//粒子速度最小位置
	varray<double> m_Vmax;//粒子速度最大位置
	
private:
	int m_GloItNum;
	varray<double> m_DisLoc;//个体最优适应度
	varray<varray<double>> m_PLocal;//每个粒子个体最优值

public:
	//构造函数
	PSO();

	//析构函数
	virtual ~PSO() {}

	/*参数设置
	c1: 个体认知系数
	c2: 社会知识系数
	r: 速度更新约束因子
	PNum: 粒子数量
	iteraNum: 最大迭代次数
	error: 限定误差*/
	void SetPSOPara(const int PNum, const int iteraNum, const double c1, const double c2, const double r,
		const double error);

	/*设置范围
	Pmin：粒子最小位置
	Pmax：粒子最大位置
	Vmin：粒子速度最小位置
	Vmax：粒子速度最大位置
	*/
	void SetRange(const varray<double>& Pmin, const varray<double>& Pmax, const varray<double>& Vmin, const varray<double>& Vmax);

    //error: 限定误差
	void SetError(const double error);

	//获取全局最优粒子及对应的适应度
	void GetResult(varray<double>& PGlobal, double& DisGol);

	//迭代更新粒子群
	void PSOUpdate();

protected:
	//适应度函数
	//需在子类重写
	virtual double CalDis(const varray<double>& P) = 0;

	//初始化位置及速度,需完成范围限定
	virtual void InitialPV();

private:
	//初始化最优值及全局最优值
	void InitialDis();

	//更新最优值,判断是否达到精度
	bool UpdateLG();

	//计算所有个体适应度
	void CalLocDis(const varray<varray<double>>& P, varray<double>& dis);
};

#endif // !CPSO
