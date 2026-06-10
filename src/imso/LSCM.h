#ifndef _LSCM_H_
#define _LSCM_H_

#include <vector>
#include <list>

#include "Mesh.h"
#include "Iterators.h"
#include "FormTrait.h"
#include <unordered_map>

namespace MeshLib {

class LSCM {
public:
	std::unordered_map<int, Point> map3d;
	std::unordered_map<int, Point> map2d;
    LSCM(Mesh *mesh);
    ~LSCM();

	

    void project();
protected:
    Mesh *m_mesh;
    std::vector<Vertex*> m_fix_vertices;//¹Ì¶¨µã Á©¸ö

    void set_coefficients();
};

}

#endif
