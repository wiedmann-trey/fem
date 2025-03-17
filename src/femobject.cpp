#include "femobject.h"
#include "iostream"

double calculateTetrahedronVolume(const Eigen::Vector3d& v0, const Eigen::Vector3d& v1,
                                  const Eigen::Vector3d& v2, const Eigen::Vector3d& v3) {
    Eigen::Vector3d a = v1 - v0;
    Eigen::Vector3d b = v2 - v0;
    Eigen::Vector3d c = v3 - v0;

    double volume = a.dot(b.cross(c));

    return std::abs(volume) / 6.0;
}

Vector3d calculateAreaWeightedNormal(Face f, std::vector<Node> &nodes) {
    Vector3d e1 = nodes[f.v1].position - nodes[f.v0].position;
    Vector3d e2 = nodes[f.v2].position - nodes[f.v0].position;
    return .5*e1.cross(e2);
}

Vector3d calculateAreaWeightedNormals(int node_idx, Tetrahedron tet, std::vector<Node> &nodes) {
    Vector3d total = Vector3d(0,0,0);
    for(Face &f : tet.faces) {
        if(node_idx == f.v0 || node_idx == f.v1 || node_idx == f.v2) {
            total += calculateAreaWeightedNormal(f, nodes);
        }
    }
    return total;
}

FEMObject::FEMObject(std::vector<Vector3d> &vertices, std::vector<Vector4i> &tets, std::vector<std::vector<Vector3i>> &tetFullFaces, Properties properties, Shape &shape) :
    m_shape(shape),
    m_properties(properties)
{

    m_state_size = 0;

    for(Vector3d &v : vertices) {
        Node n;
        n.position = v;
        n.velocity = Vector3d(0,3,0);
        n.mass = 0;
        n.forceAccumulator = Vector3d(0,0,0);
        m_nodes.push_back(n);
        m_state_size += 6;
    }

    int n_tets = tets.size();
    for(int i = 0; i < n_tets; i++) {
        Vector4i tet_verts = tets[i];
        Tetrahedron t;
        t.vertices.push_back(tet_verts[0]);
        t.vertices.push_back(tet_verts[1]);
        t.vertices.push_back(tet_verts[2]);
        t.vertices.push_back(tet_verts[3]);

        std::vector<Vector3i> tetFaces = tetFullFaces[i];

        for(Vector3i &face : tetFaces) {
            Face f;
            f.v0 = face[0];
            f.v1 = face[1];
            f.v2 = face[2];
            t.faces.push_back(f);
        }

        Vector3d v0, v1, v2, v3;
        v0 = m_nodes[tet_verts[0]].position;
        v1 = m_nodes[tet_verts[1]].position;
        v2 = m_nodes[tet_verts[2]].position;
        v3 = m_nodes[tet_verts[3]].position;

        // mass
        double t_mass = m_properties.density * calculateTetrahedronVolume(v0, v1, v2, v3);
        for(int i = 0; i < 4; i++) {
            int np = t.vertices[i];
            m_nodes[np].mass += t_mass/4;

            t.area_weighted_normals.push_back(calculateAreaWeightedNormals(np, t, m_nodes));
        }

        // beta
        Matrix4d m;
        m << v0.x(), v1.x(), v2.x(), v3.x(),
             v0.y(), v1.y(), v2.y(), v3.y(),
             v0.z(), v1.z(), v2.z(), v3.z(),
             1,      1,      1,      1;
        t.beta = m.inverse();

        m_tets.push_back(t);
    }

    for(Node &n : m_nodes) {
        n.mass_i = (n.mass*Matrix3d::Identity()).inverse();
    }
}

std::vector<Vector3d> FEMObject::getVertices() {
    std::vector<Vector3d> verts;

    for(Node &n : m_nodes) {
        verts.push_back(n.position);
    }

    return verts;
}

void FEMObject::setState(VectorXd &state) {
    int idx = 0;
    for(Node &n : m_nodes) {
        n.position = state.segment(idx, 3);
        idx+=3;
        n.velocity = state.segment(idx, 3);
        idx+=3;
    }
}

VectorXd FEMObject::getState() {
    VectorXd state_vector(m_state_size);

    int idx = 0;
    for(Node &n : m_nodes) {
        state_vector.segment(idx, 3) = n.position;
        idx+=3;
        state_vector.segment(idx, 3) = n.velocity;
        idx+=3;
    }

    return state_vector;
}


// to compute a derivative step

// set accumulator for each node to 0

// to find internal force on the vertices of a tet
// first, find dx/du dxdot/du using beta and vertex positions/velocities
// then strain and strain rate are simple calculations using these
// stress is simple calculations from strain and strain rate
// force on a node is -1/3 * F * stress * area weighted normals of 3 adjacent faces
// repeat this process for each tet, accumulating internal forces into the nodes

// for each node xdot is just velocity, vdot is M^(-1)*f, M is diagonal matrix with node mass along diagonal, f is accumulate forces
VectorXd FEMObject::evalDerivative() {
    VectorXd derivative_vector(m_state_size);

    for(Node &n : m_nodes) {
        n.forceAccumulator = Vector3d(0,0,0);

        if(n.position.y() < 0) {
            n.forceAccumulator += Vector3d(0, -40000*n.position.y(), 0);
        }
    }

    for(Tetrahedron &tet : m_tets) {
        Matrix<double, 3, 4> P;
        Matrix<double, 3, 4> V;
        Vector3d p0, p1, p2, p3, v0, v1, v2, v3;
        p0 = m_nodes[tet.vertices[0]].position;
        p1 = m_nodes[tet.vertices[1]].position;
        p2 = m_nodes[tet.vertices[2]].position;
        p3 = m_nodes[tet.vertices[3]].position;
        v0 = m_nodes[tet.vertices[0]].velocity;
        v1 = m_nodes[tet.vertices[1]].velocity;
        v2 = m_nodes[tet.vertices[2]].velocity;
        v3 = m_nodes[tet.vertices[3]].velocity;

        P << p0.x(), p1.x(), p2.x(), p3.x(),
             p0.y(), p1.y(), p2.y(), p3.y(),
             p0.z(), p1.z(), p2.z(), p3.z();

        V << v0.x(), v1.x(), v2.x(), v3.x(),
             v0.y(), v1.y(), v2.y(), v3.y(),
             v0.z(), v1.z(), v2.z(), v3.z();

        Matrix3d dxdu = (P*tet.beta).block<3,3>(0,0);
        Matrix3d dxdotdu = (V*tet.beta).block<3,3>(0,0);

        Matrix3d strain = dxdu.transpose()*dxdu - Matrix3d::Identity();
        Matrix3d strain_rate = dxdu.transpose()*dxdotdu + dxdotdu.transpose()*dxdu;

        Matrix3d elastic_stress = m_properties.incompressibility*Matrix3d::Identity()*strain.trace() + 2*m_properties.rigidity*strain;
        Matrix3d viscous_stress = m_properties.viscosity_1*Matrix3d::Identity()*strain_rate.trace() + 2*m_properties.viscosity_2*strain_rate;
        Matrix3d total_stress = elastic_stress+viscous_stress;

        for(int i = 0; i < 4; i++) {
            Vector3d f = (-1.0/3.0) * dxdu * total_stress * tet.area_weighted_normals[i];
            m_nodes[tet.vertices[i]].forceAccumulator += f;
        }

    }

    int idx = 0;
    for(Node &n : m_nodes) {
        derivative_vector.segment(idx, 3) = n.velocity;
        idx+=3;

        derivative_vector.segment(idx, 3) = n.mass_i*n.forceAccumulator + Vector3d(0, -m_properties.gravity, 0);
        idx+=3;
    }

    return derivative_vector;
}
