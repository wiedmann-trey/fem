#pragma once

#include "graphics/shape.h"
#include <QSettings>

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
    Shape m_shape;
    QString m_config;
    double m_seconds_since_last_step;
    double m_timestep;

    Shape m_ground;
    void initGround();

    void step();
};
