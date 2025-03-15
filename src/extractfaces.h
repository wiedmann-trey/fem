#ifndef EXTRACTFACES_H
#define EXTRACTFACES_H
#include <vector>
#include "Eigen/Dense"

void extractFaces(std::vector<Eigen::Vector4i> &tets, std::vector<Eigen::Vector3d> &verts, std::vector<Eigen::Vector3i> &faces);

#endif // EXTRACTFACES_H
