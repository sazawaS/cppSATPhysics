#pragma once
#include <SFML/Graphics.hpp>
#include <iostream>
#include <cmath>

static float cross(sf::Vector2f a, sf::Vector2f b) {
    return a.x * b.y - a.y * b.x;
}

class RigidBody {
public:

    sf::Vector2f position{0,0};
    sf::Vector2f velocity{0,0};
    float angle = 0.f;            
    float angularVel = 0.f; 

    float mass = 1.f;
    float inertia = 1.f;
    float linearDamping = 0.4; //aka friction
    float angularDamping = 0.2;         
    
    bool isStatic = false;

    sf::RectangleShape shape;

    sf::Vector2f getCenterOfMass() const {
        return sf::Vector2f(shape.getSize().x / 2.f, shape.getSize().y / 2.f); //Only for rectangles
    }

    RigidBody(sf::Vector2f pos, sf::Vector2f size, sf::Color color=sf::Color::White, float density=1.f, bool makeStatic=false) {
        
        isStatic = makeStatic;
        position = pos;
        shape.setSize(size);
        shape.setFillColor(color);
        shape.setOrigin(getCenterOfMass());
        shape.setPosition(pos);

        shape.setRotation(sf::degrees(angle));
        mass = ((size.x * size.y) * density)/8000;

        if (makeStatic) {
            mass = INFINITY;
            inertia = INFINITY;
        }

        //Idk how to calculate inertia, so I just use this
        if (size.x > size.y) {
            inertia = (mass * size.x) / 2.f;
        } else {
            inertia = (mass * size.y) / 2.f;
        }
    }

    void PhysicsUpdate(float deltaTime) {

        position += velocity * deltaTime;
        angle += angularVel * deltaTime;
        velocity *= std::pow(linearDamping, deltaTime); //0.4 linear damping means keep 40% of velocity every
        angularVel *= std::pow(angularDamping, deltaTime);

        //Clamp velocity and angularvelocity
        if (std::abs(velocity.x) < 2.f) velocity.x = 0.f;
        if (std::abs(velocity.y) < 2.f) velocity.y = 0.f;
        if (std::abs(angularVel) < 2.5f) angularVel = 0.f;


        shape.setPosition(position);
        shape.setRotation(sf::degrees(angle));
    }

    void applyImpulse(sf::Vector2f impulse) {
        velocity += impulse / (mass); 
    }

    void applyImpulseAtPoint(sf::Vector2f impulse, sf::Vector2f point) {
        velocity += impulse / (mass); 

        // Torque = r x F
        sf::Vector2f r = point - (getCenterOfMass() + position);
        float torque = cross(r, impulse);

        // Torque = Inertia * angular acceleration
        // Aka angular acceleration = Torque / Inertia
        float angularAcceleration = torque / (inertia);
        //update angular velocity accounting for mass 
        angularVel += angularAcceleration / (mass);
    }

    void move(sf::Vector2f offset) {
        position += offset;
        shape.setPosition(position);
    }

};