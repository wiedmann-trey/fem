#ifndef MIDPOINT_H
#define MIDPOINT_H

#include "femsystem.h"

void midpointMethod(FEMSystem &system, double delta_t) {
    VectorXd currentState = system.getState();
    VectorXd currentDerivative = system.evalDerivative();

    VectorXd midpointState = currentState+currentDerivative*delta_t/2;
    system.setState(midpointState);
    VectorXd midpointDerivative = system.evalDerivative();

    VectorXd finalState = currentState+midpointDerivative*delta_t;
    system.setState(finalState);
}

#endif // MIDPOINT_H
