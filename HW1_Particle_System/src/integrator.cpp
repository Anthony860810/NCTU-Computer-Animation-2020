#include "integrator.hpp"

#include <algorithm>
#include<iostream>
#include "scene.hpp"

void ExplicitEulerIntegrator::Integrate(const std::vector<BodyRef>& _bodies, float deltaTime)
{
	// TODO
    //for all object in _bodies set new position by x(t+1) = x(t) + deltaTime * Velocity 
    const float2 gravity(0.0f, -9.8f);
    for each (BodyRef obj in _bodies) {
        if (obj->GetMass() != 0) {
            obj->AddPosition(obj->GetVelocity() * deltaTime );
            obj->AddVelocity((obj->GetForce()*obj->GetInvMass()+gravity)* deltaTime); 
            
        }
    }
}

void RungeKuttaFourthIntegrator::Integrate(const std::vector<BodyRef>& _bodies, float deltaTime)
{
    if (scene == nullptr)
    {
        throw std::runtime_error("RungeKuttaFourthIntegrator has no target scene.");
    }
	const float2 gravity(0.0f, -9.8f);
    std::vector<StateStep> currentstep(_bodies.size());
    std::vector<StateStep> deltaK1step(_bodies.size());
    std::vector<StateStep> deltaK2step(_bodies.size());
    std::vector<StateStep> deltaK3step(_bodies.size());
    std::vector<StateStep> deltaK4step(_bodies.size());
	// TODO
    //CurrentStep
    for (int i=0; i < _bodies.size(); i++) {
        currentstep[i].position = _bodies[i]->GetPosition();
        currentstep[i].velocity = _bodies[i]->GetVelocity();
    }
  
    //K1
    for (int i = 0; i < _bodies.size(); i++) {
        deltaK1step[i].position = _bodies[i]->GetVelocity() * deltaTime;
        deltaK1step[i].velocity = _bodies[i]->GetForce()*_bodies[i]->GetInvMass()+gravity * deltaTime;
    }
    //K2
    for (int i = 0; i < _bodies.size(); i++) {
        deltaK2step[i].position = deltaK1step[i].position/2 + _bodies[i]->GetVelocity() * deltaTime/2;
        deltaK2step[i].velocity = deltaK1step[i].velocity / 2 + _bodies[i]->GetForce() * _bodies[i]->GetInvMass() + gravity * deltaTime/2;
    }
    //K3
    for (int i = 0; i < _bodies.size(); i++) {
        deltaK3step[i].position = deltaK2step[i].position / 2 + _bodies[i]->GetVelocity() * deltaTime / 2;
        deltaK3step[i].velocity = deltaK2step[i].velocity / 2 + _bodies[i]->GetForce() * _bodies[i]->GetInvMass() + gravity * deltaTime / 2;
    }
    //K4
    for (int i = 0; i < _bodies.size(); i++) {
        deltaK4step[i].position = deltaK3step[i].position  + _bodies[i]->GetVelocity() * deltaTime;
        deltaK4step[i].velocity = deltaK3step[i].velocity + _bodies[i]->GetForce() * _bodies[i]->GetInvMass() + gravity * deltaTime;
    }
    for (int i = 0; i < _bodies.size(); i++) {
        if (_bodies[i]->GetMass() != 0) {
            _bodies[i]->AddPosition((deltaK1step[i].position + deltaK4step[i].position + 2 * (deltaK2step[i].position + deltaK4step[i].position)) / 6);
            _bodies[i]->AddVelocity((deltaK1step[i].velocity + deltaK4step[i].velocity + 2 * (deltaK2step[i].velocity + deltaK4step[i].velocity)) / 6);
        }
    }
}