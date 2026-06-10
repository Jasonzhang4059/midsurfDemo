#include "stdafx.h"
#include "My3MF.h"

using namespace Lib3MF;
My3MF::My3MF()
{
	
}


My3MF::~My3MF()
{
}


void My3MF::SetMeshObject(std::vector<float> vertices, std::vector<int> indices)
{
	PWrapper _wrapper = CWrapper::loadLibrary();
	PModel _model = _wrapper->CreateModel();
	PMeshObject _meshObject = _model->AddMeshObject();
	PWriter _writer = _model->QueryWriter("3mf");
	
	std::vector<sLib3MFPosition> Vertices;
	std::vector<sLib3MFTriangle> Triangles;

	for (int i = 0; i != vertices.size(); i+=3)
	{
		Vertices.push_back(fnCreateVertex(vertices[i], vertices[i + 1], vertices[i + 2]));
	}
	for (int i = 0; i != indices.size(); i += 3)
	{
		Triangles.push_back(fnCreateTriangle(indices[i], indices[i + 1], indices[i + 2]));
	}
	_meshObject->SetGeometry(Vertices, Triangles);
	_model->AddBuildItem(_meshObject.get(), _wrapper->GetIdentityTransform());

	_writer->WriteToFile("VOL.3mf");
}


void My3MF::WriteToFile(std::string fileName)
{
	
}


//***************protected***************//


sLib3MFPosition My3MF::fnCreateVertex(float x, float y, float z)
{
	sLib3MFPosition result;
	result.m_Coordinates[0] = x;
	result.m_Coordinates[1] = y;
	result.m_Coordinates[2] = z;
	return result;
}

sLib3MFTriangle My3MF::fnCreateTriangle(int v0, int v1, int v2)
{
	sLib3MFTriangle result;
	result.m_Indices[0] = v0;
	result.m_Indices[1] = v1;
	result.m_Indices[2] = v2;
	return result;
}
