#pragma once

#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>
#include <SFML/Graphics.hpp>
#include <stdio.h>
#include <stdlib.h>

sf::Font font;

inline float distanceBetweenVertices(sf::Vector2f pos1, sf::Vector2f pos2) {
    float& x1 = pos1.x;
    float& x2 = pos2.x;
    float& y1 = pos1.y;
    float& y2 = pos2.y;
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2) * 1.0);
}

int SGN(const float x) {
    return (x > 0) - (x < 0);
}

inline float dotProduct(const sf::Vector2f a, const sf::Vector2f b)
{
    return a.x * b.x + a.y * b.y;
}

// normalizes two vectors
// see: https://stackoverflow.com/questions/10095524/normalize-a-vector
void NormalizeVector(sf::Vector2f& vec) {
    float sum = vec.x + vec.y;
    vec.x / sum;
    vec.y / sum;
}

namespace physics {
    #define GRAVITY .2 // 9.8 M/S

    struct Polygon;
    struct Edge;

    std::vector<Polygon> bodies;

    struct Vertex {
        sf::Vector2f Position;
        sf::Vector2f OldPosition;
        sf::Vector2f Acceleration;
        sf::Vector2f Velocity;
    };

    struct Edge {
        Vertex* v1 = NULL;
        Vertex* v2 = NULL;

        float Stiffness = 1;
        float OriginalLength; // The length of the edge when it was created
        Polygon* Parent = NULL; // The physics body that it belongs to
    };

    struct Polygon {
        bool anchored = false; // If it's anchored then it will not simulate physics
        float mass = 1000; // In kg

        sf::Vector2f Center;
        std::vector<Vertex> vertices;
        std::vector<Edge> edges;
        std::vector<Edge> constrainingEdges; // edges made to stop the shape from deforming

        float PhysicsThreshold = .01; // tells physics engine not to do anything if the magnitude of objects movement is below threshold
        void project_to_axis(sf::Vector2f& Axis, float& Min, float& Max) {
            float DotP = dotProduct(Axis, vertices[0].Position);

            //Set the minimum and maximum values to the projection of the first vertex
            Min = Max = DotP;

            for (int I = 1; I < vertices.size(); I++) {
                //Project the rest of the vertices onto the axis and extend
                //the interval to the left/right if necessary
                DotP = dotProduct(Axis, vertices[I].Position);

                Min = std::min(DotP, Min);
                Max = std::max(DotP, Max);
            }
        }

        // Needs to be adjusted to go off of surface area instead.
        void inline updateCenter() {
            float avgX = 0;
            float avgY = 0;
            for (int i = 0; i < vertices.size(); i++) {
                avgX += vertices[i].Position.x;
                avgY += vertices[i].Position.y;
            }
            avgX /= vertices.size();
            avgY /= vertices.size();
            Center.x = avgX;
            Center.y = avgY;
        }
    };

    struct {
        float Depth = 0;
        sf::Vector2f Normal;

        Edge *E;
        Vertex *V;
    } CollisionInfo;


    void MakeShape(std::vector<sf::Vector2f> Points, bool anchored_ = false) {

        Polygon body_;
        bodies.push_back(body_);
        Polygon &body = bodies[bodies.size() - 1];

        body.anchored = anchored_;
        // vertices
        for (int i = 0; i < Points.size(); i++) {
            Vertex V;
            V.Position = Points[i];
            body.vertices.push_back(V);
        }

        // edges
        // the initial edge
        Edge E;
        E.v1 = &body.vertices[0];
        E.v2 = &body.vertices[1];
        E.OriginalLength = distanceBetweenVertices(E.v1->Position, E.v2->Position);
        E.Parent = &body;
        body.edges.push_back(E);

        if (Points.size() > 2) {
            for (short int i = 1; i < Points.size() - 1; i++) {
                E.v1 = &body.vertices[i];
                E.v2 = &body.vertices[i + 1];
                E.OriginalLength = distanceBetweenVertices(E.v1->Position, E.v2->Position);
                E.Parent = &body;
                body.edges.push_back(E);
            }

            // the last edge
            E.v1 = &body.vertices[Points.size() - 1];
            E.v2 = &body.vertices[0];
            E.OriginalLength = distanceBetweenVertices(E.v1->Position, E.v2->Position);
            E.Parent = &body;
            body.edges.push_back(E);

            // the last edge
            E.v1 = &body.vertices[1];
            E.v2 = &body.vertices[3];
            E.OriginalLength = distanceBetweenVertices(E.v1->Position, E.v2->Position);
            E.Parent = &body;
            body.constrainingEdges.push_back(E);

            // the last edge
            E.v1 = &body.vertices[2];
            E.v2 = &body.vertices[0];
            E.OriginalLength = distanceBetweenVertices(E.v1->Position, E.v2->Position);
            E.Parent = &body;
            body.constrainingEdges.push_back(E);
            // the last edge
            /*E.v1 = &body.vertices[1];
            E.v2 = &body.vertices[4];
            E.OriginalLength = distanceBetweenVertices(E.v1->Position, E.v2->Position);
            E.Parent = &body;
            body.constrainingEdges.push_back(E);*/
        }
    }
    
    /*sf::Vector2f calculate_normalised_projection_axis(sf::Vector2f& current_Vertex, sf::Vector2f& next_Vertex) {
        double axis_x = (next_Vertex.y - current_Vertex.y);
        double axis_y = (current_Vertex.x - next_Vertex.x);
        double magnitude = hypot(axis_x, axis_y);

        sf::Vector2f axis_normalised;
        axis_normalised.x = axis_x / magnitude;
        axis_normalised.y = axis_y / magnitude;

        return axis_normalised;
    }*/

    // Project the vertices of each polygon onto a axis
    void compute_projections(const std::vector<Vertex>& bounds_a, const std::vector<Vertex>& bounds_b, const sf::Vector2f& axis_normalised, std::vector<double>& projections_a, std::vector<double>& projections_b) {
        projections_a.clear();
        projections_b.clear();

        for (size_t i = 0; i < bounds_a.size(); i++) {
            const double projection_a = dotProduct(axis_normalised, bounds_a[i].Position);


            const double projection_b = dotProduct(axis_normalised, bounds_b[i].Position);

            projections_a.push_back(projection_a);
            projections_b.push_back(projection_b);
        }
    }

    // Check if the projections of two polygons overlap
    bool is_overlapping(const std::vector<double>& projections_a, const std::vector<double>& projections_b) {
        const double max_projection_a = *std::max_element(projections_a.begin(), projections_a.end());
        const double min_projection_a = *std::min_element(projections_a.begin(), projections_a.end());
        const double max_projection_b = *std::max_element(projections_b.begin(), projections_b.end());
        const double min_projection_b = *std::min_element(projections_b.begin(), projections_b.end());

        // True if projection overlaps but does not necessarily mean the polygons are intersecting yet
        return !(max_projection_a < min_projection_b or max_projection_b < min_projection_a);
    }

    float IntervalDistance(float MinA, float MaxA, float MinB, float MaxB) {
        if (MinA < MinB)
            return MinB - MaxA;
        else
            return MinA - MaxB;
    }

    // a vector divided by its magnitude
    sf::Vector2f normalize(sf::Vector2f vec) {
        float mag = hypot(vec.x, vec.y);
        return vec/ mag;
    }
    // Check if two convex polygons intersect
    bool separating_axis_intersect(Polygon& b1,Polygon& b2) {
        b1.updateCenter();
        b2.updateCenter();

        float MinDistance = 99999.f;

        for (int i = 0; i < b1.edges.size() + b2.edges.size(); i++) {
            Edge *E;

            if (i < b1.edges.size())
                E = &b1.edges[i];
            else
                E = &b2.edges[i - b1.edges.size()];

            sf::Vector2f Axis(E->v1->Position.y - E->v2->Position.y, E->v2->Position.x - E->v1->Position.x);
            Axis = normalize(Axis);
            
            float minA , minB, maxA , maxB; //Project both bodies onto the perpendicular axis
            b1.project_to_axis(Axis, minA, maxA);
            b2.project_to_axis(Axis, minB, maxB);

            float Distance = IntervalDistance(minA, maxA, minB, maxB);
            if (Distance > 0) {
                return false;
            }
            else {
                if (abs(Distance) < MinDistance) {
                    MinDistance = abs(Distance);
                    CollisionInfo.Normal = Axis; // invert this to switch properly colliding edges
                    CollisionInfo.E = E; //Store the edge, as it is the collision edge
                }
            }
        }

        CollisionInfo.Depth = MinDistance;

        //Ensure that the body containing the collision edge lies in
        //B2 and the one containing the collision vertex in B1
        if (CollisionInfo.E->Parent != &b2) {
            Polygon Temp = b2;
            //b2 = b1;
            //b1 = Temp;
        }


        //This is needed to make sure that the collision normal is Vertexing at B1
        sf::Vector2f &BA = b1.Center;
        sf::Vector2f &BB = b2.Center;
        int Sign = SGN(dotProduct(CollisionInfo.Normal, (BA - BB)));

        //Remember that the line equation is N*( R - R0 ). We choose B2->Center
        //as R0; the normal N is given by the collision normal

        if (Sign != 1) {
            CollisionInfo.Normal = -CollisionInfo.Normal; //Revert the collision normal if it faces away from B1
        }

        float SmallestD = 99999.0f; //Initialize the smallest distance to a high value

        for (int I = 0; I < b1.vertices.size(); I++) {
            //Measure the distance of the vertex from the line using the line equation
            //float Distance = CollisionInfo.Normal * (B1->Vertices[I]->Position - B2->Center);
            float Distance = dotProduct(CollisionInfo.Normal, (b1.vertices[I].Position - BB));

            //If the measured distance is smaller than the smallest distance reported
            //so far, set the smallest distance and the collision vertex
            if (Distance < SmallestD) {
                SmallestD = Distance;
                CollisionInfo.V = &b1.vertices[I];
            }
        }
        return true;
    }

    void collisionResponse() {
        
        sf::Vector2f CollisionVector = CollisionInfo.Normal * CollisionInfo.Depth;
        Vertex *V1 = CollisionInfo.E->v1;
        Vertex *V2 = CollisionInfo.E->v2;

        float T;
        if (abs(V1->Position.x - V2->Position.x) > abs(V1->Position.y - V2->Position.y))
            T = (CollisionInfo.V->Position.x - CollisionVector.x - V1->Position.x) / (V2->Position.x - V1->Position.x);
        else
            T = (CollisionInfo.V->Position.y - CollisionVector.x - V1->Position.x) / (V2->Position.y - V1->Position.y);


        float Lambda = 1.0f / (T * T + (1 - T) * (1 - T));

        V1->Position -= CollisionVector * (1 - T) * 0.5f * Lambda;
        V2->Position -= CollisionVector * T * 0.5f * Lambda;

        CollisionInfo.V->Position += CollisionVector * 0.5f;



        // simulates friction
        // FrictionVector = ?*m*g;
        // g = gravitational constant (9.8 on earth)
        // .01 is friction coefficient
        float Friction = 0 * 2 * GRAVITY;
        sf::Vector2f FrictionVector(V1->Velocity.x * Friction, V1->Velocity.y * Friction);
        //V1->Position += FrictionVector;
        FrictionVector.x = V2->Velocity.x * Friction;
        FrictionVector.y = V2->Velocity.y * Friction;
        //V2->Position += FrictionVector
        
    }

    void updateEdges(Polygon &body, bool inverse = false) {
        int i = 0;
        for (int a = 0; a < body.edges.size() + body.constrainingEdges.size(); a++) {
            if (inverse) {
                i = body.edges.size() + body.constrainingEdges.size() - 1 - a;
            } else {
                i = a;
            }
            
            Edge *E;
            if (i < body.edges.size()) {
                E = &body.edges[i];
            } else {
                E = &body.constrainingEdges[i-body.edges.size()];
            }

            float dx = E->v2->Position.x - E->v1->Position.x;
            float dy = E->v2->Position.y - E->v1->Position.y;
            float dist = sqrt(dx * dx + dy * dy);
            float diff = (E->OriginalLength - dist) / dist * 1;

            // gets offset of the Vertexs
            float offsetx = dx * diff * 0.5;
            float offsety = dy * diff * 0.5;

            // calculate "mass"
            float m1 = 2 + 1;
            float m2 = 2 / m1;
            m1 = 1 / m1;
            m1 = 1;
            m2 = 1;

            E->v1->Position.x -= offsetx * m1;
            E->v1->Position.y -= offsety * m1;

            E->v2->Position.x += offsetx * m2;
            E->v2->Position.y += offsety * m2;

        }
    }

    void IterateCollisions() {

        // recenters shape. Does not currently account for volume. Center is also recalculated elsewhere in program. Need to fix this.
        for (short int i = 0; i < bodies.size(); i++) {
            Polygon& body = bodies[i];
            // recalculates body center
            float totalX = 0;
            float totalY = 0;
            for (short int j = 0; j < body.vertices.size(); j++) {
                totalX += body.vertices[j].Position.x;
                totalY += body.vertices[j].Position.y;
            }
            totalX /= body.vertices.size();
            totalY /= body.vertices.size();
            body.Center = sf::Vector2f(totalX, totalY);
        }

        for (int i = 0; i < bodies.size(); i++) {
            Polygon& B1 = bodies[i];
            for (int j = 0; j < bodies.size(); j++) {
                Polygon& B2 = bodies[j];
                if (i != j && !bodies[j].anchored) {
                    if (separating_axis_intersect(B1, B2)) {
                        collisionResponse();
                    }
                }
            }
            updateEdges(B1); // updates edges should be called inside the iterate collisions funciton
        }
    }

    void UpdateForces() {
        // gravitational force
        for (int i = 0; i < bodies.size(); i++) {
            Polygon &body = bodies[i];
            if (!body.anchored) {
                float gravForce = GRAVITY;
                float gravAccel = gravForce / body.mass;
                // verts
                for (int i = 0; i < body.vertices.size(); i++) {
                    Vertex &P = body.vertices[i];
                    body.vertices[i].Acceleration.y += gravAccel * 5;
                    if (P.Acceleration.y > .2) {
                        P.Acceleration.y = .2;
                    }
                    sf::Vector2f Temp = P.Position;
                    P.Position += P.Position - P.OldPosition + P.Acceleration;
                    P.OldPosition = Temp;
                }
            }
        }
    }

    void update() {

        // call update edges multiple times to maintain shape
        // note that when an edge updates it may interfere with the length of another

        // Calculates velocity
        for (int i = 0; i < physics::bodies.size(); i++) {
            Polygon& body = physics::bodies[i];
            if (!body.anchored) {
                // verts
                for (int j = 0; j < body.vertices.size(); j++) {
                    body.vertices[j].Velocity = body.vertices[j].OldPosition - body.vertices[j].Position;
                }
            }
        }

        //UpdateForces();
        for (int i = 0; i < 100;i++) {
            IterateCollisions();
        }

    }

    // https://en.wikipedia.org/wiki/Sweep_line_algorithm
    // converts shapes to convex by splitting it into triangles
    void toConvex(Polygon& body1) {
        Polygon& body = body1;
        Edge E;
        unsigned int num = 0;
        for (unsigned int i = 0; i < body.vertices.size(); i++) {
            if (i != num) {
                E.v1 = &body.vertices[i]; // v1 cannot reference v2 or segfault will occur
                E.v2 = &body.vertices[num];
                E.OriginalLength = distanceBetweenVertices(E.v1->Position, E.v2->Position);
                E.Parent = &body;
                body.constrainingEdges.push_back(E);
            }
        }
    }

    // drags vertex when user is holding down left click on vertex
    bool debuffLeft = false;
    Vertex* closest = NULL;
    int closestDist = 500;

    void dragVertex(sf::Vector2f mousePos) {
        if (!debuffLeft) {
            closestDist = 500;
            for (unsigned int b = 0; b < bodies.size(); b++) {
                Polygon& body = bodies[b];
                for (int i = 0; i < body.vertices.size(); i++) {
                    if (distanceBetweenVertices(mousePos, body.vertices[i].Position) < closestDist) {
                        closestDist = distanceBetweenVertices(mousePos, body.vertices[i].Position);
                        closest = &body.vertices[i];
                    }
                }
            }
        }
        if (closestDist < 25) {
            closest->Position = mousePos;
        }
        debuffLeft = true;
    }

    void showVertexIndex(sf::Vector2f mousePos, sf::RenderWindow &window) {
        int closestDist = 500;
        int closestIndex;
        Vertex* closest = NULL;
        for (unsigned int b = 0; b < bodies.size(); b++) {
            Polygon& body = bodies[b];
            for (int i = 0; i < body.vertices.size(); i++) {
                if (distanceBetweenVertices(mousePos, body.vertices[i].Position) < closestDist) {
                    closestIndex = i;
                    closestDist = distanceBetweenVertices(mousePos, body.vertices[i].Position);
                    closest = &body.vertices[i];
                }
            }
        }
        if (closestDist < 50) {
            sf::Text text;
            text.setFont(font);
            text.setCharacterSize(25);
            text.setPosition(sf::Vector2f(10, 10));
            text.setFillColor(sf::Color::White);
            text.setString(std::to_string(closestIndex));
            window.draw(text);
        }
    }

    void contrainPolygons(std::vector<Polygon>& bodies, sf::RenderWindow& window) {
        int winX = window.getSize().x;
        int winY = window.getSize().y;
        
        for (unsigned int b = 0; b < bodies.size(); b++) {
            Polygon& body = bodies[b];
            for (int i = 0; i < body.vertices.size(); i++) {
                sf::Vector2f& pos = body.vertices[i].Position;
                if (pos.x < 0) { pos.x = 0; }
                if (pos.y < 0) { pos.y = 0; }
                if (pos.x > winX) { pos.x = winX; }
                if (pos.y > winY) { pos.y = winY; }
            }
        }
    }
}

void renderVertices(sf::RenderWindow& window) {
    using namespace std;
    for (int i = 0; i < physics::bodies.size(); i++) {
        physics::Polygon &body = physics::bodies[i];
        for (int j = 0; j < body.vertices.size(); j++) {
            physics::Vertex V = body.vertices[j];
            sf::CircleShape circle(4);
            circle.setPosition(V.Position);
            circle.setOrigin(circle.getGlobalBounds().width / 2, circle.getGlobalBounds().height / 2);
            circle.setFillColor(sf::Color::Red);
            window.draw(circle);
        }

        for (int j = 0; j < body.edges.size(); j++) {
            physics::Vertex V1 = *body.edges[j].v1;
            sf::CircleShape circle(3);
            circle.setPosition(V1.Position);
            circle.setOrigin(circle.getGlobalBounds().width / 2, circle.getGlobalBounds().height / 2);
            circle.setFillColor(sf::Color::Green);
            window.draw(circle);

            physics::Vertex V2 = *body.edges[j].v2;
            circle.setPosition(V2.Position);
            circle.setFillColor(sf::Color::Blue);
            window.draw(circle);

            sf::RectangleShape rect(sf::Vector2f(2, distanceBetweenVertices(V1.Position, V2.Position)));
            rect.setOrigin(rect.getGlobalBounds().width / 2, rect.getGlobalBounds().height / 2);
            float avgX = (V1.Position.x + V2.Position.x) / 2;
            float avgY = (V1.Position.y + V2.Position.y) / 2;
            float rotation = atan2(V1.Position.y - V2.Position.y, V1.Position.x - V2.Position.x);
            // converts radians to degrees
            rotation = rotation * 180 / 3.14159;
            rect.setPosition(avgX, avgY);
            rect.setRotation(rotation + 90);
            rect.setFillColor(sf::Color::Green);
            window.draw(rect);

        }

        for (int j = 0; j < body.constrainingEdges.size(); j++) {
            physics::Vertex V1 = *body.constrainingEdges[j].v1;
            sf::CircleShape circle(3);
            circle.setPosition(V1.Position);
            circle.setOrigin(circle.getGlobalBounds().width / 2, circle.getGlobalBounds().height / 2);
            circle.setFillColor(sf::Color::Green);
            window.draw(circle);

            physics::Vertex V2 = *body.constrainingEdges[j].v2;
            circle.setPosition(V2.Position);
            circle.setFillColor(sf::Color::Green);
            window.draw(circle);

            sf::RectangleShape rect(sf::Vector2f(2, distanceBetweenVertices(V1.Position, V2.Position)));
            rect.setOrigin(rect.getGlobalBounds().width / 2, rect.getGlobalBounds().height / 2);
            float avgX = (V1.Position.x + V2.Position.x) / 2;
            float avgY = (V1.Position.y + V2.Position.y) / 2;
            float rotation = atan2(V1.Position.y - V2.Position.y, V1.Position.x - V2.Position.x);
            // converts radians to degrees
            rotation = rotation * 180 / 3.14159;
            rect.setPosition(avgX, avgY);
            rect.setRotation(rotation + 90);
            rect.setFillColor(sf::Color::Green);
            window.draw(rect);
        }

        // draws a line showing the collision vector
        physics::Vertex V1;
        physics::Vertex V2;
        V1.Position = sf::Vector2f(0, 0);
        V2.Position = physics::CollisionInfo.Normal*physics::CollisionInfo.Depth;
        float temp = V2.Position.x;
        V2.Position.x = -V2.Position.y;
        V2.Position.y = temp;
        V2.Position.x *= 5;
        V2.Position.y *= 5;
        sf::RectangleShape rect(sf::Vector2f(2, distanceBetweenVertices(V1.Position, V2.Position)));
        rect.setOrigin(rect.getGlobalBounds().width / 2, rect.getGlobalBounds().height / 2);
        float avgX = (V1.Position.x + V2.Position.x) / 2;
        float avgY = (V1.Position.y + V2.Position.y) / 2;
        float rotation = atan2(V1.Position.y - V2.Position.y, V1.Position.x - V2.Position.x);
        // converts radians to degrees
        rotation = rotation * 180 / 3.14159;
        rect.setPosition(avgX+200, avgY+200);
        rect.setRotation(rotation + 90);
        rect.setFillColor(sf::Color::Blue);
        window.draw(rect);

        // draws a line showing the collision vector
        V1.Position = sf::Vector2f(0, 0);
        V2.Position = physics::CollisionInfo.Normal * physics::CollisionInfo.Depth;
        V2.Position.x *= 5;
        V2.Position.y *= 5;
        rect.setSize(sf::Vector2f(2, distanceBetweenVertices(V1.Position, V2.Position)));
        rect.setOrigin(rect.getGlobalBounds().width / 2, rect.getGlobalBounds().height / 2);
         avgX = (V1.Position.x + V2.Position.x) / 2;
         avgY = (V1.Position.y + V2.Position.y) / 2;
         rotation = atan2(V1.Position.y - V2.Position.y, V1.Position.x - V2.Position.x);
        // converts radians to degrees
        rotation = rotation * 180 / 3.14159;
        rect.setPosition(avgX + 200, avgY + 200);
        rect.setRotation(rotation + 90);
        rect.setFillColor(sf::Color::Red);
        window.draw(rect);
    }
}

#include <chrono>
#include <thread>


int main() {

    if (!font.loadFromFile("LicensePlate.ttf"))
    {
        // error...
    }

    using namespace std;
   
    physics::MakeShape({sf::Vector2f(70,10),sf::Vector2f(70,60), sf::Vector2f(120,60), sf::Vector2f(120, 10)});
    physics::MakeShape({sf::Vector2f(70,10),sf::Vector2f(70,60), sf::Vector2f(120,60), sf::Vector2f(120, 10)});

    sf::RenderWindow window(sf::VideoMode(400, 400), "Physics Test");

    while (window.isOpen()) {
        sf::Event event;
        window.clear();
        sf::Vector2i mousePos = sf::Mouse::getPosition(window);
        sf::Vector2f mousePosf = sf::Vector2f(mousePos.x, mousePos.y);
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();
            if (sf::Mouse::isButtonPressed(sf::Mouse::Left))
            {
                physics::dragVertex(mousePosf);

            }
            else {
                physics::debuffLeft = false;
            }
        }
        //sleep_for(microseconds(50000));
        physics::update();
        physics::contrainPolygons(physics::bodies, window);
        renderVertices(window);
        window.display();
    }
    return 0;
}
