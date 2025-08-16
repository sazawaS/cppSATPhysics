#pragma once
#include <SFML/Graphics.hpp>
#include <vector>
#include <cmath>
using namespace sf;

struct Projection
{
    float min;
    float max;
};

//wtf this just gives me the corners of a rectangle (WORKS ON ROTATED RECTANGLES TOO)
//        sf::Vector2f topLeft = shape.getTransform().transformPoint({0.f, 0.f});
//        sf::Vector2f topRight = shape.getTransform().transformPoint({shape.getSize().x, 0.f});
//        sf::Vector2f bottomLeft = shape.getTransform().transformPoint({0.f, shape.getSize().y});
//        sf::Vector2f bottomRight = shape.getTransform().transformPoint({shape.getSize().x, shape.getSize().y});
std::vector<sf::Vector2f> getCorners(sf::RectangleShape& shape)
{
    std::vector<sf::Vector2f> corners(4);

    sf::Vector2f topLeft = shape.getPosition() - shape.getSize() / 2.f;
    sf::Vector2f topRight = shape.getPosition() + sf::Vector2f(shape.getSize().x / 2.f, -shape.getSize().y / 2.f);
    sf::Vector2f bottomLeft = shape.getPosition() + sf::Vector2f(-shape.getSize().x / 2.f, shape.getSize().y / 2.f);
    sf::Vector2f bottomRight = shape.getPosition() + shape.getSize() / 2.f;

    //Get the corners of the rectangle minus the center
    sf::Vector2f center = topLeft + (shape.getSize() / 2.f); //Center of the rectangle
    sf::Vector2f s_topLeft = topLeft - center; 
    sf::Vector2f s_topRight = topRight - center;
    sf::Vector2f s_bottomLeft = bottomLeft - center;
    sf::Vector2f s_bottomRight = bottomRight - center;

    //Rotational matrix:
    // {cos(theta), -sin(theta)}
    // {sin(theta), cos(theta)}
    float angle = shape.getRotation().asRadians();
    float a11 = cos(angle);
    float a12 = -sin(angle);
    float a21 = sin(angle);
    float a22 = cos(angle);

    //Matrix multiplication to get the rotated corners
    // {a11 a12} * {s_topLeft.x}
    // {a21 a22}   {s_topLeft.y}
    // After matrix multiplication, we get the rotated corners
    // M = {a11 * s_topLeft.x + a12 * s_topLeft.y, a21 * s_topLeft.x + a22 * s_topLeft.y}
    // Where M is the rotated corner

    sf::Vector2f rc_topLeft = sf::Vector2f(a11 * s_topLeft.x + a12 * s_topLeft.y, a21 * s_topLeft.x + a22 * s_topLeft.y);
    sf::Vector2f rc_topRight = sf::Vector2f(a11 * s_topRight.x + a12 * s_topRight.y, a21 * s_topRight.x + a22 * s_topRight.y);
    sf::Vector2f rc_bottomLeft = sf::Vector2f(a11 * s_bottomLeft.x + a12 * s_bottomLeft.y, a21 * s_bottomLeft.x + a22 * s_bottomLeft.y);
    sf::Vector2f rc_bottomRight = sf::Vector2f(a11 * s_bottomRight.x + a12 * s_bottomRight.y, a21 * s_bottomRight.x + a22 * s_bottomRight.y);

    //Now we add the center back to the rotated corners
    sf::Vector2f r_topleft = rc_topLeft + center;
    sf::Vector2f r_topright = rc_topRight + center;
    sf::Vector2f r_bottomleft = rc_bottomLeft + center;
    sf::Vector2f r_bottomright = rc_bottomRight + center;

    corners[0] = r_topleft;     // Top Left
    corners[1] = r_topright;    // Top Right
    corners[2] = r_bottomleft;  // Bottom Left
    corners[3] = r_bottomright; // Bottom Right

    return corners;
}

//We have to check each Normal Axis of the shape to see if the shapes overlap
//But on rectangles, the Normal Axises are also the edeges of the rectangle (See readme for image)
//However, for other shapes, you'd need to get calculate the normal of the edeges to get the axes
std::vector<sf::Vector2f> getAxes(sf::RectangleShape& shape)
{
    std::vector<sf::Vector2f> corners = getCorners(shape);
    sf::Vector2f axis1 = corners[1] - corners[0]; // Top Right - Top Left
    sf::Vector2f axis2 = corners[2] - corners[0]; // Bottom Left - Top Left

    std::vector<sf::Vector2f> axes;
    axes.push_back(axis1.normalized());
    axes.push_back(axis2.normalized());
    return {axis1, axis2};
}


Projection project(const std::vector<sf::Vector2f> corners, const sf::Vector2f axis)
{

    float max = std::numeric_limits<float>::lowest();
    float min = std::numeric_limits<float>::max();
    for (int i = 0; i < corners.size(); i++)
    {

        // Let P be the point to project, and A be the axis
        // The projection of P onto A is given by:
        // Projection = (dot(P,A) / A.length()^2) * A
        // Since A is unit vector, A.length()^2 = 1
        // So we can simplify it to:
        // Projection = dot(P, A) * A
        // But we only need the length of the projection, so we can just use: dot(P, A)
        float dotProduct = corners[i].dot(axis);
        
        //Get min and max of the projections (so we can see if the rectangles overlap)
        //If you dont understand why, we're getting min and max
        //You should draw two rectangles and draw the axes, then project the corners onto the axes (look at readme)
        //The min and max of the projections will be the total distance of all the projections
        if (dotProduct > max)
            max = dotProduct;
        if (dotProduct < min)
            min = dotProduct;
    }

    return Projection{min, max};
}

bool isOverlappingOnAxis(const std::vector<sf::Vector2f>& cornersA, const std::vector<sf::Vector2f>& cornersB, const sf::Vector2f& axis)
{
    Projection p1 = project(cornersA, axis);
    Projection p2 = project(cornersB, axis);

    //Again, if you dont understand this, just look at the image in the readme
    if  (p1.max < p2.min || p2.max < p1.min) {
        return false;
    }
    return true;
}