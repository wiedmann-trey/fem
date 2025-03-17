#include "femsystem.h"

FEMSystem::FEMSystem() {
    m_state_size = 0;
}

VectorXd FEMSystem::getState() {
    Eigen::VectorXd combinedState(m_state_size);

    int idx = 0;
    for(FEMObject &o : m_objects) {
        combinedState.segment(idx, o.getStateSize()) = o.getState();
        idx += o.getStateSize();
    }

    return combinedState;
}

void FEMSystem::setState(VectorXd &newState) {
    int idx = 0;
    for(FEMObject &o : m_objects) {
        VectorXd v = newState.segment(idx, o.getStateSize());
        o.setState(v);
        idx += o.getStateSize();
    }
}

VectorXd FEMSystem::evalDerivative() {
    Eigen::VectorXd combinedDerivative(m_state_size);

    int idx = 0;
    for(FEMObject &o : m_objects) {
        combinedDerivative.segment(idx, o.getStateSize()) = o.evalDerivative();
        idx += o.getStateSize();
    }

    return combinedDerivative;
}

void FEMSystem::addObject(FEMObject &object) {
    m_objects.push_back(object);
    m_state_size += object.getStateSize();
}

void FEMSystem::updateVertices() {
    for(FEMObject &o : m_objects) {
        Shape s = o.getShape();
        s.setVertices(o.getVertices());
    }
}

void FEMSystem::toggleWire() {
    for(FEMObject &o : m_objects) {
        o.getShape().toggleWireframe();
    }
}

void FEMSystem::draw(Shader *shader) {
    for(FEMObject &o : m_objects) {
        o.getShape().draw(shader);
    }
}
