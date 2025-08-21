#include <SFML/Graphics.hpp>
#include <iostream>
#include <vector>
#include <optional>
#include "headers/SAT.hpp"
#include "headers/rigidbody.hpp"
#include "headers/player.hpp"

// Collisions
void resolveCollision(RigidBody& a, RigidBody& b) {
    if (a.isStatic && b.isStatic) return;
    MTV collisionData = computeMTVSAT(a.shape, b.shape);
    if (collisionData.collision)
    {
        sf::Vector2f resolve = collisionData.axis * (collisionData.overlap);
        if (a.isStatic) {
            b.move(resolve);
        } else if (b.isStatic) {
            a.move(-resolve);
        } else {
            a.move(-resolve/2.f);
            b.move(resolve/2.f);
        }


        //Copied off the tutorial
        sf::Vector2f relativeVelocity = b.velocity - a.velocity;
        float velocityAlongNormal = relativeVelocity.x * collisionData.axis.x + relativeVelocity.y * collisionData.axis.y;

        if (velocityAlongNormal > 0) {
            return; 
        }

        float e = 0.9f;

        float j = -(1 + e) * velocityAlongNormal;
        j /= (1 / a.mass + 1 / b.mass);

        sf::Vector2f impulse = collisionData.axis * j;

        a.applyImpulseAtPoint(-(1/a.mass * impulse), collisionData.collisionPoint);
        b.applyImpulseAtPoint((1/b.mass * impulse), collisionData.collisionPoint);
    }
}   

int main()
{
    sf::RenderWindow window(sf::VideoMode({1280, 720}), "Physics In CPP");
    window.setFramerateLimit(75);


    std::vector<RigidBody> rigidBodies;
    Player player = Player(sf::Vector2f(400, 200), sf::Vector2f(200,40));    
    RigidBody box1(sf::Vector2f(500, 500), sf::Vector2f(200, 40), sf::Color::Red, 1.f, false);
    rigidBodies.push_back(box1);

    sf::Clock clock;

    while (window.isOpen())
    {
        while (const std::optional event = window.pollEvent())
        {
            if (event->is<sf::Event::Closed>())
                window.close();
        }

        //Update
        sf::Vector2f mousePos = sf::Vector2f(sf::Mouse::getPosition(window));
        float deltaTime = clock.restart().asSeconds();
        float timeStep = 1.f / 300.f; // 300 FPS physics update
        float accumulator = deltaTime;

        while (accumulator > 0.f)
        {
            float step = std::min(accumulator, timeStep);

            player.PhysicsUpdate(step);
            player.mouseUpdate(mousePos);
            for (RigidBody& obj : rigidBodies) {
                obj.PhysicsUpdate(step);
                resolveCollision(player, obj);
                for (RigidBody& other : rigidBodies) {
                    if (&obj != &other && other.isStatic == false) {
                        resolveCollision(obj, other);
                    }
                }
            }
            
            accumulator -= step;
        }




        
        window.clear(sf::Color(60, 179, 113));
        for (RigidBody& obj : rigidBodies) {
            window.draw(obj.shape);
        }
        window.draw(player.shape);
        window.draw(player.arrowShape);
        window.draw(player.arrowShape2);

        window.display();
    }
}
