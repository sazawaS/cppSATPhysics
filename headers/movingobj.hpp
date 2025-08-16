#pragma once
#include "object.hpp"
#include <cmath>
#include <SFML/Graphics.hpp>

class MovingObj : public Object {
public:
    float speed = 50.f;
    float frictionPerSecond = 0.4f;
    float angularDamping = 1000.0f;

    MovingObj(sf::Vector2f size, sf::Vector2f startPos, sf::Color color = sf::Color::White) 
        : Object(size, startPos, color) {}

    void PhysicsUpdate(float deltaTime) {
        previousPosition = shape.getPosition();
        
        //Gravity
        //velocity.y += 500.f * deltaTime * mass;
        shape.move(velocity * deltaTime);
        velocity *= std::pow(frictionPerSecond, deltaTime);

        angle += angularVelocity * deltaTime;
        shape.setRotation(sf::radians(angle));
        angularVelocity *= std::pow(0.5f, deltaTime * (angularDamping / inertia));

        rect.pos = shape.getPosition();
        rect.size = shape.getSize();
    }
};