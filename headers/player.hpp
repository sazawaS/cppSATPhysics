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
    float angularDamping = 1000.0f;

    sf::Vector2f initialMousePosition;

    bool showArrow = false;
    sf::RectangleShape arrowShape;

    Player(sf::Vector2f size, sf::Vector2f startPos, sf::Color color = sf::Color::White) 
        : Object(size, startPos, color) {
            arrowShape.setSize(sf::Vector2f(0,4.0f));
            arrowShape.setFillColor(sf::Color::Blue);
            arrowShape.setOrigin(sf::Vector2f(0.f, 2.f));
            arrowShape.setPosition(sf::Vector2f(sf::Mouse::getPosition().x, sf::Mouse::getPosition().y));
        }

    void PhysicsUpdate(float deltaTime, sf::Vector2f mousePos) {
        Input.updateInput();
        previousPosition = shape.getPosition();

        if (Input.isKeyJustPressed("LMB")) {
            initialMousePosition = mousePos;
            showArrow = true;
        }
        if (Input.isKeyJustReleased("LMB")) {
            sf::Vector2f releasedMousePosition = mousePos;
            sf::Vector2f directionVector = initialMousePosition - releasedMousePosition;
            applyForce(sf::Vector2f(directionVector.x, directionVector.y), sf::Vector2f(initialMousePosition.x, initialMousePosition.y));
            showArrow = false;
        }
        
        if (showArrow) {
            arrowShape.setScale(sf::Vector2f(1,1));
            sf::Vector2f diff = initialMousePosition - mousePos;
            sf::Angle angle = sf::degrees(std::atan2(-diff.y, -diff.x) * 180.f / 3.14159265f);
            float length = std::sqrt(diff.x * diff.x + diff.y * diff.y);

            arrowShape.setSize(sf::Vector2f(length, 4));
            arrowShape.setPosition(initialMousePosition);
            arrowShape.setRotation(angle);

        } else {
            arrowShape.setScale(sf::Vector2f(0,0));
        }
        
        //Gravity
        //velocity.y += 500.f * deltaTime * mass;
        shape.move(velocity * deltaTime);
        velocity *= std::pow(frictionPerSecond, deltaTime);

        angle += angularVelocity * deltaTime;
        shape.setRotation(sf::degrees(angle * 180.f / 3.14159f));
        angularVelocity *= std::pow(0.5f, deltaTime * (angularDamping / inertia));

        rect.pos = shape.getPosition();
        rect.size = shape.getSize();
    }
};