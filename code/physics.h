#pragma once

#include <vector>

struct Vec2 {
    float x = 0.0f;
    float y = 0.0f;
};

Vec2 operator+(const Vec2& a, const Vec2& b);
Vec2 operator-(const Vec2& a, const Vec2& b);
Vec2 operator*(const Vec2& v, float s);
Vec2 operator/(const Vec2& v, float s);
Vec2& operator+=(Vec2& a, const Vec2& b);
Vec2& operator-=(Vec2& a, const Vec2& b);

enum class ShapeType {
    Circle,
    Box
};

struct Shape {
    ShapeType type = ShapeType::Circle;
    float radius = 0.0f;
    Vec2 halfExtents{0.0f, 0.0f};
};

struct RigidBody2D {
    Shape shape;

    Vec2 position;
    Vec2 velocity;
    Vec2 accumulatedForce;

    float angle = 0.0f;
    float angularVelocity = 0.0f;
    float accumulatedTorque = 0.0f;

    float mass = 1.0f;
    float invMass = 1.0f;
    float inertia = 1.0f;
    float invInertia = 1.0f;

    float restitution = 0.25f;
    float staticFriction = 0.7f;
    float dynamicFriction = 0.5f;
    bool affectedByGravity = true;
    bool isStatic = false;

    // Sleep state to suppress jitter when the body has effectively come to rest.
    bool isSleeping = false;
    float sleepTimer = 0.0f;
};

class PhysicsSystem {
  public:
    void step(std::vector<RigidBody2D>& bodies, float dt) const;

  private:
    void applyGlobalForces(std::vector<RigidBody2D>& bodies) const;
    void integrateVelocities(std::vector<RigidBody2D>& bodies, float dt) const;
    void integratePositions(std::vector<RigidBody2D>& bodies, float dt) const;
    void updateSleepStates(std::vector<RigidBody2D>& bodies, float dt) const;
};
