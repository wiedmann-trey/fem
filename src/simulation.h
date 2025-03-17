#pragma once

#include "graphics/shape.h"
#include <QSettings>
#include "femsystem.h"

class Shader;

class Simulation
{
public:
    Simulation(QString config);

    void init();

    void update(double seconds);

    void draw(Shader *shader);

    void toggleWire();
private:
    QString m_config;
    double m_seconds_since_last_step;
    double m_timestep;

    FEMSystem m_system;
    Shape m_ground;
    void initGround();
};
