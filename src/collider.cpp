#include "collider.h"
#include <iostream>

using namespace Eigen;

Vector3d normal(const Vector3i &face, std::vector<Vector3d> &vertices) {
    Vector3d e1 = vertices[face[1]] - vertices[face[0]];
    Vector3d e2 = vertices[face[2]] - vertices[face[0]];
    return e1.cross(e2).normalized();
}

Collider::Collider() {}

Collider::Collider(const std::vector<Eigen::Vector3d> &vertices, const std::vector<Eigen::Vector3i> &faces, int id, bool is_flat) : m_id(id), m_is_flat(is_flat) {
    m_max_x = m_max_y = m_max_z = std::numeric_limits<double>::lowest();
    m_min_x = m_min_y = m_min_z = std::numeric_limits<double>::max();

    for(const Vector3d &v : vertices) {
        m_vertices.push_back(Vector3d(v));
        if(v[0] < m_min_x) m_min_x = v[0];
        if(v[1] < m_min_y) m_min_y = v[1];
        if(v[2] < m_min_z) m_min_z = v[2];

        if(v[0] > m_max_x) m_max_x = v[0];
        if(v[1] > m_max_y) m_max_y = v[1];
        if(v[2] > m_max_z) m_max_z = v[2];
    }

    for(const Vector3i &f : faces) {
        m_faces.push_back(Vector3i(f));

        m_normals.push_back(normal(f, m_vertices));
    }
}

void Collider::setVertices(const std::vector<Eigen::Vector3d> &vertices) {
    m_vertices.clear();
    m_max_x = m_max_y = m_max_z = std::numeric_limits<double>::lowest();
    m_min_x = m_min_y = m_min_z = std::numeric_limits<double>::max();
    for(const Vector3d &v : vertices) {
        m_vertices.push_back(Vector3d(v));
        if(v[0] < m_min_x) m_min_x = v[0];
        if(v[1] < m_min_y) m_min_y = v[1];
        if(v[2] < m_min_z) m_min_z = v[2];

        if(v[0] > m_max_x) m_max_x = v[0];
        if(v[1] > m_max_y) m_max_y = v[1];
        if(v[2] > m_max_z) m_max_z = v[2];
    }

    m_normals.clear();
    for(const Vector3i &f : m_faces) {
        m_normals.push_back(normal(f, m_vertices));
    }
}

Vector3d Collider::resolveCollision(Eigen::Vector3d point) {
    if(!m_is_flat && (point.x() > m_max_x || point.y() > m_max_y || point.z() > m_max_z || point.x() < m_min_x || point.y() < m_min_y || point.z() < m_min_z)) return Vector3d(0,0,0);

    for(int i = 0; i < m_faces.size(); i++) {
        Vector3i face = m_faces[i];
        Vector3d A, B, C;
        A = m_vertices[face[0]];
        B = m_vertices[face[1]];
        C = m_vertices[face[2]];
        Vector3d normal = m_normals[i];

        double d = (point - A).dot(normal);

        if(d > 0 || std::abs(d) > .005) continue;

        Vector3d v0 = C-A;
        Vector3d v1 = B-A;
        Vector3d v2 = point - A;

        double d00 = v0.dot(v0);
        double d01 = v0.dot(v1);
        double d11 = v1.dot(v1);
        double d20 = v2.dot(v0);
        double d21 = v2.dot(v1);

        double denom = d00*d11-d01*d01;

        double u = (d11*d20-d01*d21)/denom;
        double v = (d00*d21-d01*d20)/denom;

        if(u < 0 || v < 0 || u + v > 1) continue;

        Vector3d force = 80000000 * std::abs(d)*normal;
        return force;
    }

    return Vector3d(0,0,0);
}
