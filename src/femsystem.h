#ifndef FEMSYSTEM_H
#define FEMSYSTEM_H
#include "Eigen/Dense"
#include "femobject.h"
#include "graphics/shape.h"

using namespace Eigen;

class FEMSystem
{
public:
    FEMSystem();

    VectorXd getState();
    VectorXd evalDerivative();
    void setState(VectorXd newState);
    void addObject(FEMObject object);
    void renderVertices();

private:

    std::vector<FEMObject> m_objects;
};



#endif // FEMSYSTEM_H
