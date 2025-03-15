#include "extractfaces.h"
#include <unordered_set>
#include <algorithm>

using namespace Eigen;

struct Vector3iEqual {
    bool operator()(const Vector3i& v1, const Vector3i& v2) const {
        Vector3i v1_sorted = v1;
        Vector3i v2_sorted = v2;
        std::sort(v1_sorted.data(), v1_sorted.data() + 3);
        std::sort(v2_sorted.data(), v2_sorted.data() + 3);
        return v1_sorted == v2_sorted;
    }
};

struct Vector3iHash {
    std::size_t operator()(const Vector3i& v) const {
        Vector3i v_sorted = v;
        std::sort(v_sorted.data(), v_sorted.data() + 3);
        std::size_t seed = 0;
        seed ^= std::hash<int>()(v_sorted.x()) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        seed ^= std::hash<int>()(v_sorted.y()) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        seed ^= std::hash<int>()(v_sorted.z()) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        return seed;
    }
};

Vector3d computeNormal(Vector3i face, std::vector<Vector3d> &verts) {
    Vector3d v0 = verts[face[0]];
    Vector3d v1 = verts[face[1]];
    Vector3d v2 = verts[face[2]];
    Vector3d e1 = v1 - v0;
    Vector3d e2 = v2 - v0;
    return e1.cross(e2);
}

bool isNormalOutward(const Vector3d& normal, int v_face, int v_opposite, std::vector<Vector3d> &verts) {
    Vector3d v = verts[v_opposite] - verts[v_face];
    return normal.dot(v) < 0;
}

std::vector<Vector3i> getTetrahedronFaces(Vector4i tet, std::vector<Vector3d> &verts) {
    std::vector<Vector3i> faces = {
        {tet[1], tet[2], tet[3]},
        {tet[0], tet[2], tet[3]},
        {tet[0], tet[1], tet[3]},
        {tet[0], tet[1], tet[2]}
    };


    for (auto& face : faces) {
        Vector3d normal = computeNormal(face, verts);
        int v_opposite;

        if (face[0] != tet[0] && face[1] != tet[0] && face[2] != tet[0]) v_opposite = tet[0];
        else if (face[0] != tet[1] && face[1] != tet[1] && face[2] != tet[1]) v_opposite = tet[1];
        else if (face[0] != tet[2] && face[1] != tet[2] && face[2] != tet[2]) v_opposite = tet[2];
        else v_opposite = tet[3];

        if (!isNormalOutward(normal, face[0], v_opposite, verts)) {
            std::reverse(face.begin(), face.end());
        }
    }

    return faces;
}

// extracts outside faces of mesh into faces
void extractFaces(std::vector<Vector4i> &tets, std::vector<Vector3d> &verts, std::vector<Vector3i> &faces) {
    std::unordered_set<Vector3i, Vector3iHash, Vector3iEqual> faceSet;

    for(Vector4i &tet : tets) {
        std::vector<Vector3i> tetFaces = getTetrahedronFaces(tet, verts);

        for(Vector3i &face : tetFaces) {
            if(faceSet.contains(face)) {
                faceSet.erase(face);
            } else {
                faceSet.insert(face);
            }
        }
    }

    for(const Vector3i &face : faceSet) {
        faces.push_back(face);
    }
}
