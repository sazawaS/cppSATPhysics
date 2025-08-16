#include <SFML/Graphics.hpp>
#include <iostream>
#include <vector>
#include <optional>
#include "headers/SAT.hpp"
#include "headers/object.hpp"
#include "headers/player.hpp"
#include "headers/movingobj.hpp"
#include "headers/AABB.hpp"

    

// Collisions
void resolveCollisionWithPlayer(Player& player, Object& obj)
{
    MTV collisionData = computeMTVSAT(player.shape, obj.shape);
    if (collisionData.collision)
    {
        sf::Vector2f resolve = collisionData.axis * (collisionData.overlap);
        float m1 = player.mass;
        player.shape.move((-resolve));
        player.applyForce(-resolve, collisionData.collisionPoint);
    }
}
void resolveCollisionWithPlayer(Player& player, MovingObj& obj)
{
    MTV collisionData = computeMTVSAT(player.shape, obj.shape);
    if (!collisionData.collision) return;

    sf::Vector2f resolve = collisionData.axis * collisionData.overlap;

    float m1 = player.mass;
    float m2 = obj.mass;

    player.shape.move((-resolve) / 2.f);
    obj.shape.move((resolve) / 2.f);

    player.applyForce(-resolve, collisionData.collisionPoint);
    obj.applyForce(resolve, collisionData.collisionPoint);
}

void resolveCollisionWithObjects(MovingObj& a, MovingObj& b) {
    MTV collisionData = computeMTVSAT(a.shape, b.shape);
    if (collisionData.collision)
    {
        sf::Vector2f resolve = collisionData.axis * (collisionData.overlap);
        
        float m1 = a.mass;
        float m2 = b.mass;

        a.shape.move((-resolve)/2.f);
        b.shape.move((resolve)/2.f);
        
        a.applyForce(-resolve, collisionData.collisionPoint);
        b.applyForce(resolve, collisionData.collisionPoint);
    }
}
void resolveCollisionWithObjects(MovingObj& a, Object& b) {
    MTV collisionData = computeMTVSAT(a.shape, b.shape);
    if (collisionData.collision)
    {
        sf::Vector2f resolve = collisionData.axis * (collisionData.overlap);
        float m1 = a.mass;
        float m2 = b.mass;
        a.shape.move((-resolve));
        a.applyForce(-resolve, collisionData.collisionPoint);
    }
}

int main()
{
    sf::RenderWindow window(sf::VideoMode({1280, 720}), "Physics In CPP");
    window.setFramerateLimit(120);


    std::vector<Object> objects;
    std::vector<MovingObj> movingObjs;
    Player player = Player(sf::Vector2f(200,20), sf::Vector2f(1280/2, 720/2));
    player.angle = 10.0f;
    movingObjs.emplace_back(sf::Vector2f(150,30), sf::Vector2f(200, 400), sf::Color::Blue);
    movingObjs.emplace_back(sf::Vector2f(200,50), sf::Vector2f(800, 400), sf::Color::Blue);
    movingObjs.emplace_back(sf::Vector2f(250,10), sf::Vector2f(640, 600), sf::Color::Blue);
    movingObjs.emplace_back(sf::Vector2f(100,5), sf::Vector2f(640, 100), sf::Color::Blue);
    objects.emplace_back(sf::Vector2f(1280,10), sf::Vector2f(1280/2, -5), sf::Color::Red);
    objects.emplace_back(sf::Vector2f(1280,10), sf::Vector2f(1280/2, 720+5), sf::Color::Red);
    objects.emplace_back(sf::Vector2f(10,720), sf::Vector2f(-5, 720/2), sf::Color::Red);
    objects.emplace_back(sf::Vector2f(10,720), sf::Vector2f(1280+5, 720/2), sf::Color::Red);

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
        player.PhysicsUpdate(deltaTime, mousePos);
        for (MovingObj& obj : movingObjs) {
            obj.PhysicsUpdate(deltaTime);
            resolveCollisionWithPlayer(player, obj);

            for (Object& obj2 : objects) {
                resolveCollisionWithObjects(obj, obj2);
            }
            for (MovingObj& obj2 : movingObjs) {
                if (obj.shape.getPosition() != obj2.shape.getPosition())
                    resolveCollisionWithObjects(obj, obj2);
            }
        }
        for (Object& obj : objects) {
            obj.PhysicsUpdate(deltaTime);
            resolveCollisionWithPlayer(player, obj);
        }


        
        window.clear(sf::Color(60, 179, 113));
        for (auto& obj : objects) {
            window.draw(obj.shape);
        }
        for (auto& obj : movingObjs) {
            window.draw(obj.shape);
        }
        window.draw(player.shape);
        window.draw(player.arrowShape);

        window.display();
    }
}
//hi