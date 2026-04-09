#include "physics.h"

#include "collision.h"

namespace {
constexpr Vec2 kGravity{0.0f, -9.8f};
}

Vec2 operator+(const Vec2& a, const Vec2& b) { return Vec2{a.x + b.x, a.y + b.y}; }
Vec2 operator-(const Vec2& a, const Vec2& b) { return Vec2{a.x - b.x, a.y - b.y}; }
Vec2 operator*(const Vec2& v, float s) { return Vec2{v.x * s, v.y * s}; }
Vec2 operator/(const Vec2& v, float s) { return Vec2{v.x / s, v.y / s}; }
Vec2& operator+=(Vec2& a, const Vec2& b) {
    a.x += b.x;
    a.y += b.y;
    return a;
}
Vec2& operator-=(Vec2& a, const Vec2& b) {
    a.x -= b.x;
    a.y -= b.y;
    return a;
}

void PhysicsSystem::step(std::vector<RigidBody2D>& bodies, float dt) const {
    constexpr int kSolverIterations = 8;

    applyGlobalForces(bodies);
    integrateVelocities(bodies, dt);

    for (int i = 0; i < kSolverIterations; ++i) {
        std::vector<Contact> contacts;
        detectCollisions(bodies, contacts);
        if (contacts.empty()) {
            break;
        }
        resolveCollisions(bodies, contacts);
    }

    integratePositions(bodies, dt);
}

void PhysicsSystem::applyGlobalForces(std::vector<RigidBody2D>& bodies) const {
    for (RigidBody2D& body : bodies) {
        if (body.isStatic) {
            continue;
        }
        body.accumulatedForce += kGravity * body.mass;
    }
}

void PhysicsSystem::integrateVelocities(std::vector<RigidBody2D>& bodies, float dt) const {
    for (RigidBody2D& body : bodies) {
        if (body.isStatic) {
            continue;
        }

        const Vec2 acceleration = body.accumulatedForce * body.invMass;
        body.velocity += acceleration * dt;

        const float angularAcceleration = body.accumulatedTorque * body.invInertia;
        body.angularVelocity += angularAcceleration * dt;

        body.accumulatedForce = Vec2{0.0f, 0.0f};
        body.accumulatedTorque = 0.0f;
    }
}

void PhysicsSystem::integratePositions(std::vector<RigidBody2D>& bodies, float dt) const {
    for (RigidBody2D& body : bodies) {
        if (body.isStatic) {
            continue;
        }

        body.position += body.velocity * dt;
        body.angle += body.angularVelocity * dt;
    }
}
