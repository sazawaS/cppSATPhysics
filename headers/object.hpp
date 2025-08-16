#pragma once
#include <SFML/Graphics.hpp>
#include <cmath>
#include "AABB.hpp"

class Object {
private:
    float width = 0;
    float height = 0;

    sf::Vector2f getCenter() {
        sf::Vector2f pos = shape.getPosition();
        sf::Vector2f size = shape.getSize();
        
        return sf::Vector2f(pos.x + size.x / 2, pos.y + size.y / 2);
    }

    float getLength(sf::Vector2f vec) {
        return std::sqrt(vec.x * vec.x + vec.y * vec.y);
    }
public:
    sf::RectangleShape shape;
    sf::Vector2f velocity;
    sf::Vector2f previousPosition;
    IRect rect;
    float mass = 1.0;
    float angularVelocity = 0;
    float angle = 0; 
    float inertia;

    Object(sf::Vector2f size, sf::Vector2f startPos, sf::Color color = sf::Color::White) 
    {
        shape.setSize(size);
        shape.setFillColor(color);
        shape.setPosition(startPos);
        shape.setOrigin({size.x / 2, size.y / 2}); // Set origin to center
        rect.pos = shape.getPosition();
        rect.size = shape.getSize();
        mass = (size.x * size.y) / 2500;
        width = size.x;
        height = size.y;
        inertia = (mass * (width*width + height*height)) / 12.f;
    }

    void PhysicsUpdate(float deltaTime) {
        previousPosition = shape.getPosition();
        rect.pos = shape.getPosition();
        rect.size = shape.getSize();

        shape.move(velocity * deltaTime);
    }

    void applyForce(sf::Vector2f force, sf::Vector2f pointOfApplication) {
        velocity += force;
        
        sf::Vector2f r = (pointOfApplication - this->getCenter());
        float torque = r.x * (force.y*2) - r.y * (force.x*2); 
        angularVelocity += torque / inertia; 
    }

    sf::Vector2f getCenterOfMass() const {
        return shape.getPosition();
    }
};