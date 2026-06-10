#include <OpenABF.hpp>
#include<iostream>
// #include<Mesh.h>

// Alias algorithms for convenience
using namespace std;
using ABF = OpenABF::ABFPlusPlus<float>;
using LSCM = OpenABF::AngleBasedLSCM<float, ABF::Mesh>;


// //！展开曲面
// 	void unfoldMeshSurface(string path, varray<Vec4> &vecs) {
// 		std::cout << "--> Reading mesh..." << std::endl;
// 		MeshLib::Mesh mesh;

// 		//读取原始网格曲面
// 		mesh.read_obj(path.c_str());

// 		//FormTrait 是MeshLib库中的一个类，用于实现网格数据和表单数据之间的转换
// 		MeshLib::FormTrait traits(&mesh);

// 		//收集三角网格空间曲面上的点,用于后续曲面拟合
// 		//MeshVertexIterator可以依次访问网格中的每个顶点，并对其进行操作。
// 		for (MeshVertexIterator viter(&mesh); !viter.end(); ++viter) {
// 			Vertex * v = *viter;

// 			//判断所读取的点是否为一开始设置的固定点
// 			if (v->string().substr(0, 3) != "fix") {
// 				Point p = v->point();
// 				Vec4 vec = { p.x(), p.y(), p.z(),1 };
// 				vecs.push_back(vec);
// 			}
// 		}


// 		//计算共形映射
// 		std::cout << "--> Computing conformal map..." << std::endl;
// 		MeshLib::LSCM lscm(&mesh);
// 		lscm.project();
// 		map3d = lscm.map3d;
// 		map2d = lscm.map2d;
// 		//m = mesh;
// 		std::cout << "--> Writing mesh..." << std::endl;
// 		mesh.write_obj("obj\\ballCut1-1.obj");
		
// 	}


signed main(){
// Make a triangular pyramid mesh
auto mesh = ABF::Mesh::New();
mesh->insert_vertex(0, 0, 0);
mesh->insert_vertex(2, 0, 0);
mesh->insert_vertex(1, std::sqrt(3), 0);
mesh->insert_vertex(1, std::sqrt(3) / 3, 1);

mesh->insert_face(0, 3, 1);
mesh->insert_face(0, 2, 3);
mesh->insert_face(2, 1, 3);

// Print original coordinates
for (const auto& v : mesh->vertices()) {
    std::cout << v->idx << ": " << v->pos << std::endl;
}

// Compute parameterized angles
ABF::Compute(mesh);

// Compute mesh parameterization from angles
LSCM::Compute(mesh);

// Print new coordinates
for (const auto& v : mesh->vertices()) {
    std::cout << v->idx << ": " << v->pos << std::endl;
}
system("pause");
}