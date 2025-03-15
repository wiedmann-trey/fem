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
    Vector3f gravity;
    double incompressibility, rigidity, viscosity_1, viscosity_2, density;
};

class FEMObject
{
public:
    FEMObject(std::vector<Vector3d> &vertices, std::vector<Vector3i> &faces, std::vector<int> &tets, Properties properties, Shape &shape);

    std::vector<Vector3d> getVertices();
    void setState(VectorXd state);
    VectorXd getState();

private:
    Properties m_properties;
    Shape m_shape;
    std::vector<Node *> m_nodes;
    std::vector<Tetrahedron> m_tets;

};

struct Node {
    double mass;
    Vector3d forceAccumulator;
    Vector3d position;
    Vector3d velocity;
};

struct Face {
    Node *v0, *v1, *v2;
};

struct Tetrahedron {
    Matrix4d beta;
    std::vector<Face> faces;
    std::vector<Node *> vertices;
};


// upon initialization
// calculate beta for each tetrahedron
// beta = each column is vertex position in homogoneous, then invert the matrix
// calculate mass for each vertex
// mass = density * volume of tet / 4

// to compute a derivative step

// set accumulator for each node to 0

// to find internal force on the vertices of a tet
// first, find dx/du dxdot/du using beta and vertex positions/velocities
// then strain and strain rate are simple calculations using these
// stress is simple calculations from strain and strain rate
// force on a node is -1/3 * F * stress * area weighted normals of 3 adjacent faces
// repeat this process for each tet, accumulating internal forces into the nodes

// for each node xdot is just velocity, vdot is M^(-1)*f, M is diagonal matrix with node mass along diagonal, f is accumulate forces


#endif // FEMOBJECT_H
