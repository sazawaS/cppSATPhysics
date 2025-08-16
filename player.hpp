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

    bool showArrow = false;
    sf::RectangleShape arrowShape;

    Player(sf::Vector2f size, sf::Vector2f startPos, sf::Color color = sf::Color::White) 
        : Object(size, startPos, color) {
            arrowShape.setSize(sf::Vector2f(200,4));
            arrowShape.setFillColor(sf::Color::Black);
        }

    void PhysicsUpdate(float deltaTime) {
        Input.updateInput();
        previousPosition = shape.getPosition();

        if (Input.isKeyJustPressed("LMB")) {
            initialMousePosition = sf::Mouse::getPosition();
            showArrow = true;
        }
        if (Input.isKeyJustReleased("LMB")) {
            sf::Vector2i releasedMousePosition = sf::Mouse::getPosition();
            sf::Vector2i directionVector = initialMousePosition - releasedMousePosition;
            velocity += Vector2f(directionVector.x*2, directionVector.y*2);
            showArrow = false;
        }
        
        if (showArrow) {
            arrowShape.setScale(sf::Vector2f(1,1));
            sf::Vector2i currentMousePos = sf::Mouse::getPosition();
            sf::Vector2i directionVector = initialMousePosition - currentMousePos;
            sf::Angle angle = sf::radians(atan2(-directionVector.y, -directionVector.x));
            float distance = std::sqrt(std::pow(directionVector.x - directionVector.y,2));
            arrowShape.setScale(sf::Vector2f(distance/200, 1));
            //sf::Vector2f(initialMousePosition.x + 2, initialMousePosition.y + 2)
            arrowShape.setPosition(shape.getPosition() + sf::Vector2f(25,25));
            arrowShape.setRotation(angle);
        } else {
            arrowShape.setScale(sf::Vector2f(0,0));
        }
        
        //Gravity
        //velocity.y += 500.f * deltaTime * mass;
        shape.move(velocity * deltaTime);
        velocity *= std::pow(frictionPerSecond, deltaTime);

        rect.pos = shape.getPosition();
        rect.size = shape.getSize();
    }
};