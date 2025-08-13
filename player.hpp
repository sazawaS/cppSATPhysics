#pragma once
#include "object.hpp"
#include <SFML/Graphics.hpp>

class Player : public Object {
public:
    float speed = 50.f;
    float friction = 0.96f;

    Player(sf::Vector2f size, sf::Vector2f startPos, sf::Color color = sf::Color::White) 
        : Object(size, startPos, color) {}

    void PhysicsUpdate(float deltaTime) {
        previousPosition = shape.getPosition();

        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::W)) velocity.y -= speed;
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::A)) velocity.x -= speed;
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::S)) velocity.y += speed;
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::D)) velocity.x += speed;
        
        //Gravity
        //velocity.y += 500.f * deltaTime * mass;
        shape.move(velocity * deltaTime);
        velocity *= friction;

        rect.pos = shape.getPosition();
        rect.size = shape.getSize();
        
    }
};