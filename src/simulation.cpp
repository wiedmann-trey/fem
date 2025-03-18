#include "simulation.h"
#include "graphics/meshloader.h"
#include "extractfaces.h"
#include "midpoint.h"

#include <iostream>

using namespace Eigen;

Simulation::Simulation(QString config) : m_config(config) {}

void Simulation::init(Camera &camera)
{
    m_system = FEMSystem();

    QSettings settings(m_config, QSettings::IniFormat );
    if(settings.contains("Global/timestep")) {
        m_timestep = settings.value("Global/timestep").toDouble();
    } else {
        m_timestep = .0003;
    }

    double grav;
    if(settings.contains("Global/gravity")) {
        grav = settings.value("Global/gravity").toDouble();
    } else {
        grav = 1;
    }

    double collision_penalty;
    if(settings.contains("Global/collision_penalty")) {
        collision_penalty = settings.value("Global/collision_penalty").toDouble();
    } else {
        collision_penalty = 8e7;
    }

    double collision_epsilon;
    if(settings.contains("Global/collision_epsilon")) {
        collision_epsilon = settings.value("Global/collision_epsilon").toDouble();
    } else {
        collision_epsilon = .005;
    }

    for(int obj_idx = 0; settings.contains("Object"+std::to_string(obj_idx)+"/meshfile"); obj_idx++) {
        std::vector<Vector3d> vertices;
        std::vector<Vector4i> tets;
        Properties props;
        std::string current_object = "Object"+std::to_string(obj_idx);

        QString meshFile = settings.value(current_object+"/meshfile").toString();
        if (MeshLoader::loadTetMesh(meshFile.toStdString(), vertices, tets)) {

            // read transformation
            if(settings.contains(current_object+"/transform")) {
                QStringList matrixStr = settings.value(current_object+"/transform").toStringList();

                if (matrixStr.size() != 16) {
                    qWarning() << "Error: Transformation matrix must have 16 values.";
                } else {
                    Eigen::Matrix4d transformationMatrix;

                    for (int i = 0; i < 4; ++i) {
                        for (int j = 0; j < 4; ++j) {
                            transformationMatrix(i, j) = matrixStr[i * 4 + j].toDouble();
                        }
                    }

                    for(int i = 0; i < vertices.size(); i++) {
                        Vector4d h_point(vertices[i].x(), vertices[i].y(), vertices[i].z(), 1.0);
                        h_point = transformationMatrix * h_point;
                        vertices[i] = h_point.head<3>()/ h_point.w();
                    }
                }
            }

            props.gravity = grav;

            if(settings.contains(current_object+"/velocity")) {
                QStringList vectorStr = settings.value(current_object+"/velocity").toStringList();
                props.initial_velocity = Vector3d(0,0,0);
                if (vectorStr.size() != 3) {
                    qWarning() << "Error: Velocity must have 3 values.";
                } else {
                    props.initial_velocity[0] = vectorStr[0].toDouble();
                    props.initial_velocity[1] = vectorStr[1].toDouble();
                    props.initial_velocity[2] = vectorStr[2].toDouble();
                }
            } else {
                props.initial_velocity = Vector3d(0,0,0);
            }

            if(settings.contains(current_object+"/density")) {
                props.density = settings.value(current_object+"/density").toDouble();
            } else {
                props.density = 1200;
            }

            if(settings.contains(current_object+"/incompressibility")) {
                props.incompressibility = settings.value(current_object+"/incompressibility").toDouble();
            } else {
                props.incompressibility = 40000;
            }

            if(settings.contains(current_object+"/rigidity")) {
                props.rigidity = settings.value(current_object+"/rigidity").toDouble();
            } else {
                props.rigidity = 40000;
            }

            if(settings.contains(current_object+"/viscosity_1")) {
                props.viscosity_1 = settings.value(current_object+"/viscosity_1").toDouble();
            } else {
                props.viscosity_1 = 100;
            }

            if(settings.contains(current_object+"/viscosity_2")) {
                props.viscosity_2 = settings.value(current_object+"/viscosity_2").toDouble();
            } else {
                props.viscosity_2 = 100;
            }

            std::vector<Vector3i> outsideFaces;
            std::vector<std::vector<Vector3i>> tetFullFaces;
            extractFaces(tets, vertices, outsideFaces, tetFullFaces);
            Shape shape;
            shape.init(vertices, outsideFaces, tets);
            m_system.addShape(shape);

            bool use_collider = false;
            std::shared_ptr<Collider> collider;
            if(settings.contains(current_object+"/is_collider") && settings.value(current_object+"/is_collider").toBool()) {
                collider = std::make_shared<Collider>(Collider(vertices, outsideFaces, obj_idx, false, collision_penalty, collision_epsilon));

                use_collider = true;
                m_system.addCollider(collider);
            }

            if(settings.contains(current_object+"/simulate") && settings.value(current_object+"/simulate").toBool()) {
                if(use_collider) {
                    FEMObject object(vertices, tets, tetFullFaces, props, shape, collider);
                    m_system.addObject(object);
                } else {
                    FEMObject object(vertices, tets, tetFullFaces, props, shape);
                    m_system.addObject(object);
                }
            }
        }
    }

    // ground
    std::vector<Vector3d> groundVerts;
    std::vector<Vector3i> groundFaces;
    groundVerts.emplace_back(-5, 0, -5);
    groundVerts.emplace_back(-5, 0, 5);
    groundVerts.emplace_back(5, 0, 5);
    groundVerts.emplace_back(5, 0, -5);
    groundFaces.emplace_back(0, 1, 2);
    groundFaces.emplace_back(0, 2, 3);
    Shape ground;
    ground.init(groundVerts, groundFaces);
    Collider ground_collider(groundVerts, groundFaces, -1, true, collision_penalty, collision_epsilon);
    m_system.addCollider(std::make_shared<Collider>(ground_collider));
    m_system.addShape(ground);

    if(settings.contains("Global/camera_pos")) {
        QStringList vectorStr = settings.value("Global/camera_pos").toStringList();
        if (vectorStr.size() != 3) {
            qWarning() << "Error: Velocity must have 3 values.";
        } else {
            camera.setPosition(Vector3f(vectorStr[0].toFloat(), vectorStr[1].toFloat(), vectorStr[2].toFloat()));
        }
    }

    m_system.init();
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
        midpointMethod(m_system, m_timestep);
    }

    m_system.updateVertices();
}

void Simulation::draw(Shader *shader)
{
    m_system.draw(shader);
}

void Simulation::toggleWire()
{
    m_system.toggleWire();
}

// each stores vertices and faces. lets you update vertices
// vertices are updated during setstate
// after loading all objects to system, each object registers list of the other colliders
// in config, each object can set as a collider or not, and same with whether or not a FEMObject
// FEMSystem stores separate lists of colliders, objects, shapes to render
