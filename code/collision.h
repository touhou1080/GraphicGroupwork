#pragma once

#include <cstddef>
#include <vector>

#include "physics.h"

struct Contact {
    std::size_t bodyA = 0;
    std::size_t bodyB = 0;
    Vec2 normal{0.0f, 1.0f};
    Vec2 points[2]{{0.0f, 0.0f}, {0.0f, 0.0f}};
    int pointCount = 0;
    float penetration = 0.0f;
};

void detectCollisions(const std::vector<RigidBody2D>& bodies, std::vector<Contact>& outContacts);
void resolveCollisions(std::vector<RigidBody2D>& bodies, const std::vector<Contact>& contacts);
