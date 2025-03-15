#include "simulation.h"
#include "graphics/meshloader.h"
#include "extractfaces.h"

#include <iostream>

using namespace Eigen;

Simulation::Simulation(QString config) : m_config(config) {}

void Simulation::init()
{
    QSettings settings(m_config, QSettings::IniFormat );
    QString meshFile = settings.value("IO/meshfile").toString();
    m_timestep = settings.value("Constants/timestep").toDouble();

    std::vector<Vector3d> vertices;
    std::vector<Vector4i> tets;
    if (MeshLoader::loadTetMesh(meshFile.toStdString(), vertices, tets)) {
        std::vector<Vector3i> faces;
        extractFaces(tets, vertices, faces);
        m_shape.init(vertices, faces, tets);
    }
    m_shape.setModelMatrix(Affine3f(Eigen::Translation3f(0, 2, 0)));

    initGround();
}

void Simulation::update(double seconds)
{
    // STUDENTS: This method should contain all the time-stepping logic for your simulation.
    //   Specifically, the code you write here should compute new, updated vertex positions for your
    //   simulation mesh, and it should then call m_shape.setVertices to update the display with those
    //   newly-updated vertices.

    m_seconds_since_last_step += seconds;

    int n_steps = m_seconds_since_last_step / m_timestep;
    m_seconds_since_last_step -= n_steps*m_timestep;

    for(int i = 0; i < n_steps; i++) {
        step();
    }
}

void Simulation::step()
{

}

void Simulation::draw(Shader *shader)
{
    m_shape.draw(shader);
    m_ground.draw(shader);
}

void Simulation::toggleWire()
{
    m_shape.toggleWireframe();
}

void Simulation::initGround()
{
    std::vector<Vector3d> groundVerts;
    std::vector<Vector3i> groundFaces;
    groundVerts.emplace_back(-5, 0, -5);
    groundVerts.emplace_back(-5, 0, 5);
    groundVerts.emplace_back(5, 0, 5);
    groundVerts.emplace_back(5, 0, -5);
    groundFaces.emplace_back(0, 1, 2);
    groundFaces.emplace_back(0, 2, 3);
    m_ground.init(groundVerts, groundFaces);
}
