#pragma once
#include <SFML/Graphics.hpp>
using namespace sf;

struct IRect
{
    Vector2f pos;
    Vector2f size;
};

bool pointInRect(Vector2f point, IRect rect)
{
    return (point.x >= rect.pos.x && 
            point.x <= rect.pos.x + rect.size.x &&
            point.y >= rect.pos.y &&
            point.y <= rect.pos.y + rect.size.y);
}

bool rectInRect(IRect a, IRect b)
{
    return (a.pos.x < b.pos.x + b.size.x &&
            a.pos.x + a.size.x > b.pos.x &&
            a.pos.y < b.pos.y + b.size.y &&
            a.pos.y + a.size.y > b.pos.y);
}

float min(float a, float b)
{
    if (a < b) return a;
    if (b < a) return b;
}

Vector2f getResolve(IRect a, IRect b)
{
    float overlapX1 = (a.pos.x + a.size.x) - b.pos.x; 
    float overlapX2 = (b.pos.x + b.size.x) - a.pos.x;
    float overlapY1 = (a.pos.y + a.size.y) - b.pos.y; 
    float overlapY2 = (b.pos.y + b.size.y) - a.pos.y;

    float smallestOverlapX = min(overlapX1,overlapX2);
    float smallestOverlapY = min(overlapY1,overlapY2);

    if (std::abs(smallestOverlapX) < std::abs(smallestOverlapY)) {
        float resolve;
        if (overlapX1 < overlapX2) {
            resolve = -overlapX1;
        } else {
            resolve = overlapX2;
        }
        return Vector2f(resolve,0);
    } else {
        float resolve;
        if (overlapY1 < overlapY2) {
            resolve = -overlapY1;
        } else {
            resolve = overlapY2;
        }
        return Vector2f(0,resolve);
    }
}

