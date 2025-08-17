#include <SFML/Graphics.hpp>
#include <iostream>
#include <vector>
#include <optional>
#include "headers/SAT.hpp"
#include "headers/player.hpp"
#include "headers/rigidbody.hpp"
#include "headers/AABB.hpp"

    

// Collisions
void resolveCollision(RigidBody& a, RigidBody& b) {
    if (a.isStatic && b.isStatic) return;
    MTV collisionData = computeMTVSAT(a.shape, b.shape);
    if (collisionData.collision)
    {
        sf::Vector2f resolve = collisionData.axis * (collisionData.overlap);
        float m1 = a.mass;
        float m2 = b.mass;
        if (a.isStatic) {
            b.shape.move(resolve);
        } else if (b.isStatic) {
            a.shape.move(-resolve);
        } else {
            a.shape.move(-resolve / 2.f);
            b.shape.move(resolve / 2.f);
        }
        a.applyImpulseAtPoint(-resolve, collisionData.collisionPoint);
        b.applyImpulseAtPoint(resolve, collisionData.collisionPoint);
    }
}   

int main()
{
    sf::RenderWindow window(sf::VideoMode({1280, 720}), "Physics In CPP");
    window.setFramerateLimit(120);


    std::vector<RigidBody> rigidBodies;
    Player player = Player(sf::Vector2f(400, 200), sf::Vector2f(200,40));    
    RigidBody box1(sf::Vector2f(500, 500), sf::Vector2f(200, 40), sf::Color::Red, 1.f, true);
    rigidBodies.push_back(box1);

    //Make 4 walls around the world screen 1280 720
    RigidBody topWall(sf::Vector2f(640, -10), sf::Vector2f(1280, 20), sf::Color::White, 1.f, true);
    RigidBody bottomWall(sf::Vector2f(640, 720+10), sf::Vector2f(1280, 20), sf::Color::White, 1.f, true);
    RigidBody leftWall(sf::Vector2f(-10, 360), sf::Vector2f(20, 720), sf::Color::White, 1.f, true);
    RigidBody rightWall(sf::Vector2f(1280+10, 360), sf::Vector2f(20, 720), sf::Color::White, 1.f, true);
    rigidBodies.push_back(topWall);
    rigidBodies.push_back(bottomWall);
    rigidBodies.push_back(leftWall);
    rigidBodies.push_back(rightWall);

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
        player.PhysicsUpdate(deltaTime);
        player.mouseUpdate(mousePos);

        for (RigidBody& obj : rigidBodies) {
            obj.PhysicsUpdate(deltaTime);
            resolveCollision(player, obj);
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
