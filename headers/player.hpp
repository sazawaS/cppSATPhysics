#pragma once
#include "rigidbody.hpp"
#include "input.hpp"
#include <cmath>
#include <SFML/Graphics.hpp>

class Player : public RigidBody {
public:
    Input Input;
    float speed = 50.f;

    sf::Vector2f initialMousePosition;
    bool showArrow = false;
    sf::RectangleShape arrowShape;
    sf::RectangleShape arrowShape2;

    Player(sf::Vector2f startPos, sf::Vector2f size, sf::Color color = sf::Color::White, float density=1.f) 
        : RigidBody(startPos, size, color, density) {
            arrowShape.setSize(sf::Vector2f(0,4.0f));
            arrowShape.setFillColor(sf::Color::Blue);
            arrowShape.setOrigin(sf::Vector2f(0.f, 2.f));
            arrowShape.setPosition(sf::Vector2f(sf::Mouse::getPosition().x, sf::Mouse::getPosition().y));
            
            sf::Vector2f corner = getCorners(shape)[0];
            arrowShape2.setPosition(corner);
            arrowShape2.setSize(sf::Vector2f(0,4.0f));
            arrowShape2.setFillColor(sf::Color::Red);
            arrowShape2.setOrigin(sf::Vector2f(0.f, 2.f));
        }

    void mouseUpdate(sf::Vector2f mousePos) {

        Input.updateInput();

        if (Input.isKeyJustPressed("LMB")) {
            initialMousePosition = mousePos;
            showArrow = true;
        }
        if (Input.isKeyJustReleased("LMB")) {
            sf::Vector2f releasedMousePosition = mousePos;
            sf::Vector2f directionVector = initialMousePosition - releasedMousePosition;
            applyImpulseAtPoint(sf::Vector2f(directionVector.x, directionVector.y), sf::Vector2f(initialMousePosition.x, initialMousePosition.y));
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
        
    }
};