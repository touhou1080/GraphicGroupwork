#include "physics.h"

#include "collision.h"

#include <algorithm>

namespace {
constexpr Vec2 kGravity{0.0f, -9.8f};

float lengthSquared(const Vec2& v) { return v.x * v.x + v.y * v.y; }
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

PhysicsStepResult PhysicsSystem::step(std::vector<RigidBody2D>& bodies, float dt) const {
    PhysicsStepResult result;

    applyGlobalForces(bodies);
    integrateVelocities(bodies, dt);

    std::vector<Contact> contacts;
    detectCollisions(bodies, contacts);
    for (const Contact& contact : contacts) {
        if (contact.bodyA == 0 || contact.bodyB == 0) {
            result.activeBirdContact = true;
            result.activeBirdCenterAtContact = bodies.front().position;
            break;
        }
    }
    resolveCollisions(bodies, contacts, dt, contactCache_);

    integratePositions(bodies, dt);
    updateSleepStates(bodies, dt);
    return result;
}

void PhysicsSystem::applyGlobalForces(std::vector<RigidBody2D>& bodies) const {
    for (RigidBody2D& body : bodies) {
        if (body.isStatic || body.isSleeping || !body.affectedByGravity) {
            continue;
        }
        body.accumulatedForce += kGravity * body.mass;
    }
}

void PhysicsSystem::integrateVelocities(std::vector<RigidBody2D>& bodies, float dt) const {
    for (RigidBody2D& body : bodies) {
        if (body.isStatic || body.isSleeping) {
            continue;
        }

        const Vec2 acceleration = body.accumulatedForce * body.invMass;
        body.velocity += acceleration * dt;

        const float angularAcceleration = body.accumulatedTorque * body.invInertia;
        body.angularVelocity += angularAcceleration * dt;

        // Mild damping keeps stacks from gaining numerical energy.
        body.velocity = body.velocity * 0.999f;
        body.angularVelocity *= 0.98f;

        constexpr float kMaxAngularSpeed = 10.0f;
        body.angularVelocity = std::max(-kMaxAngularSpeed, std::min(kMaxAngularSpeed, body.angularVelocity));

        body.accumulatedForce = Vec2{0.0f, 0.0f};
        body.accumulatedTorque = 0.0f;
    }
}

void PhysicsSystem::integratePositions(std::vector<RigidBody2D>& bodies, float dt) const {
    for (RigidBody2D& body : bodies) {
        if (body.isStatic || body.isSleeping) {
            continue;
        }

        body.position += body.velocity * dt;
        body.angle += body.angularVelocity * dt;
    }
}

void PhysicsSystem::updateSleepStates(std::vector<RigidBody2D>& bodies, float dt) const {
    constexpr float kSleepLinearSpeed = 0.06f;
    constexpr float kSleepAngularSpeed = 0.08f;
    constexpr float kSleepDelay = 0.45f;

    const float linearThresholdSq = kSleepLinearSpeed * kSleepLinearSpeed;

    for (RigidBody2D& body : bodies) {
        if (body.isStatic) {
            continue;
        }

        const bool almostStill = lengthSquared(body.velocity) < linearThresholdSq &&
                                 std::abs(body.angularVelocity) < kSleepAngularSpeed;

        if (almostStill) {
            body.sleepTimer += dt;
            if (body.sleepTimer >= kSleepDelay) {
                body.isSleeping = true;
                body.velocity = Vec2{0.0f, 0.0f};
                body.angularVelocity = 0.0f;
                body.accumulatedForce = Vec2{0.0f, 0.0f};
                body.accumulatedTorque = 0.0f;
            }
        } else {
            body.sleepTimer = 0.0f;
            body.isSleeping = false;
        }
    }
}
