#ifndef COLLIDER_H
#define COLLIDER_H

#include <vector>
#include "Eigen/Dense"

class Collider
{
public:
    Collider();
    Collider(const std::vector<Eigen::Vector3d> &vertices, const std::vector<Eigen::Vector3i> &faces, int id, bool is_flat);

    Eigen::Vector3d resolveCollision(Eigen::Vector3d point);

    void setVertices(const std::vector<Eigen::Vector3d> &vertices);

    int getId() {return m_id;}
private:
    std::vector<Eigen::Vector3d> m_vertices;
    std::vector<Eigen::Vector3i> m_faces;
    std::vector<Eigen::Vector3d> m_normals;

    int m_id;
    bool m_is_flat;

    double m_min_x, m_min_y, m_min_z;
    double m_max_x, m_max_y, m_max_z;
};

#endif // COLLIDER_H
