#include <SFML/Graphics.hpp>
#include <iostream>
#include <vector>
#include "object.hpp"
#include "player.hpp"
#include "movingobj.hpp"
#include "AABB.hpp"

    

//AABB Collisions
void resolveCollisionWithPlayer(Player& player, Object& obj)
{
    if (rectInRect(player.rect, obj.rect))
    {
        Vector2f resolve = getResolve(player.rect,obj.rect);
        //Formula for velocity after collision:
        //ObjA Velocity = ((m1-m2)*v1 + (2*m2*v2))/(m1+m2)
        //ObjB Velocity = ((m2-m1)*v2 + (2*m1*v1))/(m1+m2)

        player.shape.move(resolve);
        float m1 = player.mass;
        float m2 = m1*100;
        if (resolve.y == 0) 
        {
            float v1 = player.velocity.x;
            float v2 = obj.velocity.x;
            player.velocity.x = ((m1-m2)*v1 + (2*m2*v2))/(m1+m2);
        } else { 
            float v1 = player.velocity.y;
            float v2 = obj.velocity.y;
            player.velocity.y = ((m1-m2)*v1 + (2*m2*v2))/(m1+m2);
        }
    }
}
void resolveCollisionWithPlayer(Player& player, MovingObj& obj)
{
    if (rectInRect(player.rect, obj.rect))
    {
        Vector2f resolve = getResolve(player.rect,obj.rect);
        //Formula for velocity after collision:
        //ObjA Velocity = ((m1-m2)*v1 + (2*m2*v2))/(m1+m2)
        //ObjB Velocity = ((m2-m1)*v2 + (2*m1*v1))/(m1+m2)

        player.shape.move(resolve);
        float m1 = player.mass;
        float m2 = obj.mass;
        if (resolve.y == 0) 
        {
            float v1 = player.velocity.x;
            float v2 = obj.velocity.x;
            player.velocity.x = ((m1-m2)*v1 + (2*m2*v2))/(m1+m2);
            obj.velocity.x = ((m2-m1)*v2 + (2*m1*v1))/(m1+m2);
        } else { 
            float v1 = player.velocity.y;
            float v2 = obj.velocity.y;
            player.velocity.y = ((m1-m2)*v1 + (2*m2*v2))/(m1+m2);
            obj.velocity.y = ((m2-m1)*v2 + (2*m1*v1))/(m1+m2);    
        }
    }
}
void resolveCollisionWithObjects(MovingObj& a, MovingObj& b) {
    if (rectInRect(a.rect, b.rect))
    {
        Vector2f resolve = getResolve(a.rect,b.rect);
        //Formula for velocity after collision:
        //ObjA Velocity = ((m1-m2)*v1 + (2*m2*v2))/(m1+m2)
        //ObjB Velocity = ((m2-m1)*v2 + (2*m1*v1))/(m1+m2)

        a.shape.move(resolve);
        float m1 = a.mass;
        float m2 = b.mass;
        if (resolve.y == 0) 
        {
            float v1 = a.velocity.x;
            float v2 = b.velocity.x;
            a.velocity.x = ((m1-m2)*v1 + (2*m2*v2))/(m1+m2);
            b.velocity.x = ((m2-m1)*v2 + (2*m1*v1))/(m1+m2);
        } else { 
            float v1 = a.velocity.y;
            float v2 = b.velocity.y;
            a.velocity.y = ((m1-m2)*v1 + (2*m2*v2))/(m1+m2);
            b.velocity.y = ((m2-m1)*v2 + (2*m1*v1))/(m1+m2);    
        }
    }
}
void resolveCollisionWithObjects(MovingObj& a, Object& b) {
    if (rectInRect(a.rect, b.rect))
    {
        Vector2f resolve = getResolve(a.rect,b.rect);
        //Formula for velocity after collision:
        //ObjA Velocity = ((m1-m2)*v1 + (2*m2*v2))/(m1+m2)
        //ObjB Velocity = ((m2-m1)*v2 + (2*m1*v1))/(m1+m2)

        a.shape.move(resolve);
        float m1 = a.mass;
        float m2 = m1*100;
        if (resolve.y == 0) 
        {
            float v1 = a.velocity.x;
            float v2 = b.velocity.x;
            a.velocity.x = ((m1-m2)*v1 + (2*m2*v2))/(m1+m2);
        } else { 
            float v1 = a.velocity.y;
            float v2 = b.velocity.y;
            a.velocity.y = ((m1-m2)*v1 + (2*m2*v2))/(m1+m2);
        }
    }
}

int main()
{
    sf::RenderWindow window(sf::VideoMode({1280, 720}), "Physics In CPP");
    window.setFramerateLimit(120);


    std::vector<Object> objects;
    std::vector<MovingObj> movingObjs;
    Player player = Player(sf::Vector2f(50,50), sf::Vector2f(1280/2, 720/2));
    movingObjs.emplace_back(sf::Vector2f(50,50), sf::Vector2f(90, 90), sf::Color::Blue);
    movingObjs.emplace_back(sf::Vector2f(15,15), sf::Vector2f(120, 120), sf::Color::Blue);
    objects.emplace_back(sf::Vector2f(1280,10), sf::Vector2f(0, -10), sf::Color::Red);
    objects.emplace_back(sf::Vector2f(1280,10), sf::Vector2f(0, 720), sf::Color::Red);
    objects.emplace_back(sf::Vector2f(10,720), sf::Vector2f(-10, 0), sf::Color::Red);
    objects.emplace_back(sf::Vector2f(10,720), sf::Vector2f(1280, 0), sf::Color::Red);

    sf::Clock clock;

    while (window.isOpen())
    {
        while (const std::optional event = window.pollEvent())
        {
            if (event->is<sf::Event::Closed>())
                window.close();
        }

        //Update
        float deltaTime = clock.restart().asSeconds();
        player.PhysicsUpdate(deltaTime);
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