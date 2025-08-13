#pragma once
#include <SFML/Graphics.hpp>
#include "AABB.hpp"

class Object {
public:
    sf::RectangleShape shape;
    sf::Vector2f velocity;
    sf::Vector2f previousPosition;
    IRect rect;
    float mass = 1.0;

    Object(sf::Vector2f size, sf::Vector2f startPos, sf::Color color = sf::Color::White) 
    {
        shape.setSize(size);
        shape.setFillColor(color);
        shape.setPosition(startPos);
        rect.pos = shape.getPosition();
        rect.size = shape.getSize();
        mass = (size.x + size.y) /100;
    }

    void PhysicsUpdate(float deltaTime) {
        previousPosition = shape.getPosition();
        rect.pos = shape.getPosition();
        rect.size = shape.getSize();
        //Gravity
        //velocity.y += 500.f * deltaTime * mass;
        shape.move(velocity * deltaTime);


    }
};