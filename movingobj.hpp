#pragma once
#include "object.hpp"
#include <SFML/Graphics.hpp>

class MovingObj : public Object {
public:
    float speed = 50.f;
    float friction = 0.96f;

    MovingObj(sf::Vector2f size, sf::Vector2f startPos, sf::Color color = sf::Color::White) 
        : Object(size, startPos, color) {}

    void PhysicsUpdate(float deltaTime) {
        previousPosition = shape.getPosition();
        
        //Gravity
        //velocity.y += 500.f * deltaTime * mass;
        shape.move(velocity * deltaTime);
        velocity *= friction;

        rect.pos = shape.getPosition();
        rect.size = shape.getSize();
    }
};