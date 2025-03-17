#ifndef FEMOBJECT_H
#define FEMOBJECT_H

#include <vector>
#include "Eigen/Dense"
#include "graphics/shape.h"

using namespace Eigen;

struct Node;
struct Tetrahedron;
struct Face;

struct Properties {
    double gravity, incompressibility, rigidity, viscosity_1, viscosity_2, density;
};

class FEMObject
{
public:
    FEMObject(std::vector<Vector3d> &vertices, std::vector<Vector4i> &tets, std::vector<std::vector<Vector3i>> &tetFullFaces, Properties properties, Shape &shape);

    std::vector<Vector3d> getVertices();
    void setState(VectorXd &state);
    VectorXd getState();
    VectorXd evalDerivative();
    Shape &getShape() {return m_shape;}
    int getStateSize() {return m_state_size;}

private:
    Properties m_properties;
    Shape m_shape;
    std::vector<Node> m_nodes;
    std::vector<Tetrahedron> m_tets;
    int m_state_size;

};

struct Node {
    double mass;
    Matrix3d mass_i;
    Vector3d forceAccumulator;
    Vector3d position;
    Vector3d velocity;
};

struct Face {
    int v0, v1, v2;
};

struct Tetrahedron {
    Matrix4d beta;
    std::vector<Face> faces;
    std::vector<int> vertices;
    std::vector<Vector3d> area_weighted_normals;
};


#endif // FEMOBJECT_H
