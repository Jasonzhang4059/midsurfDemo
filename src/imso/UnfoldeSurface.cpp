#include "UnfoldSurface.h"
#include "MyDoc.h"
#include "Mesh.h"
//两个静态成员的初始化
MyDoc::Ptr MyDoc::m_instance_ptr = nullptr;
std::mutex MyDoc::mutex;
using ABF = OpenABF::ABFPlusPlus<float>;
using LSCM1 = OpenABF::AngleBasedLSCM<float, ABF::Mesh>;

void UnfoldSurface::UnfoldSuface()
{

}

void UnfoldSurface::calculate(vector<Vec4> vs)
{

	// Make a triangular pyramid mesh
	auto mesh = ABF::Mesh::New();
	for (const auto&i : vs)
	{
		mesh->insert_vertex(i.x, i.y, i.z);
	}
	// Print original coordinates
	std::cout << "原始坐标:" << std::endl;
	for (const auto& v : mesh->vertices()) {
		std::cout << v->idx << ": " << v->pos << std::endl;
	}

	// Compute parameterized angles
	ABF::Compute(mesh);

	// Compute mesh parameterization from angles
	LSCM1::Compute(mesh);

	std::cout << "展开坐标:" << std::endl;
	// Print new coordinates
	int i = 0;
	for (const auto& v : mesh->vertices()) {
		std::cout << v->idx << ": " << v->pos << std::endl;
		vs[i].x = v->pos[0];
		vs[i].y = v->pos[1];
		vs[i].z = v->pos[2];
	}
	system("pause");
}
