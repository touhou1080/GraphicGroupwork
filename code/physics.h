#pragma once

#include <cstddef>
#include <vector>

struct Vec2 {
    float x = 0.0f;
    float y = 0.0f;
};
struct CachedContactImpulse {
    std::size_t bodyA = 0;
    std::size_t bodyB = 0;
    Vec2 localPointA{0.0f, 0.0f};
    float normalImpulse = 0.0f;
    float tangentImpulse = 0.0f;
};

struct PhysicsStepResult {
    bool activeBirdImpact = false;
    Vec2 activeBirdCenterAtImpact{0.0f, 0.0f};
    std::vector<std::size_t> defeatedBodies;
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

enum class BodyTag {
    BirdRed = 0,
    BirdYellow = 1,
    Pig = 100,
    WoodBlock = 200,
    IceBlock = 201
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

    // Independent timer maintained by Scene::pruneStaleBirds, ticking up while
    // the body's linear speed stays below the (smaller) bird-disappearance
    // threshold. Separate from sleepTimer because the two thresholds differ.
    float quietTimer = 0.0f;

    // Application-specific marker. Bird bodies stash their BirdType here so the
    // renderer can colour each bird individually.
    int customTag = 0;
};

class PhysicsSystem {
  public:
    PhysicsStepResult step(std::vector<RigidBody2D>& bodies, float dt) const;
    void clearContactCache() const;

  private:
    void applyGlobalForces(std::vector<RigidBody2D>& bodies) const;
    void integrateVelocities(std::vector<RigidBody2D>& bodies, float dt) const;
    void integratePositions(std::vector<RigidBody2D>& bodies, float dt) const;
    void updateSleepStates(std::vector<RigidBody2D>& bodies, float dt) const;

    mutable std::vector<CachedContactImpulse> contactCache_;
};
