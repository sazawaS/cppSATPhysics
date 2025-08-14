#pragma once
#include "object.hpp"
#include "input.hpp"
#include <cmath>
#include <SFML/Graphics.hpp>

class Player : public Object {
public:
    Input Input;
    float speed = 50.f;
    float frictionPerSecond = 0.4f;

    sf::Vector2i initialMousePosition;
    

    Player(sf::Vector2f size, sf::Vector2f startPos, sf::Color color = sf::Color::White) 
        : Object(size, startPos, color) {}

    void PhysicsUpdate(float deltaTime) {
        Input.updateInput();
        previousPosition = shape.getPosition();

        if (Input.isKeyJustPressed("LMB")) {
            initialMousePosition = sf::Mouse::getPosition();
        }
        if (Input.isKeyJustReleased("LMB")) {
            sf::Vector2i releasedMousePosition = sf::Mouse::getPosition();
            sf::Vector2i directionVector = initialMousePosition - releasedMousePosition;
            velocity += Vector2f(directionVector.x*2, directionVector.y*2);
        }
        
        
        //Gravity
        //velocity.y += 500.f * deltaTime * mass;
        shape.move(velocity * deltaTime);
        velocity *= std::pow(frictionPerSecond, deltaTime);

        rect.pos = shape.getPosition();
        rect.size = shape.getSize();
        
    }
};