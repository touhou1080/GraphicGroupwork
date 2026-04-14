#include "scene.h"

namespace {
float boxInertia(float mass, const Vec2& halfExtents) {
    // Rectangle inertia around center: I = m * (w^2 + h^2) / 12.
    const float width = 2.0f * halfExtents.x;
    const float height = 2.0f * halfExtents.y;
    return mass * (width * width + height * height) / 12.0f;
}

RigidBody2D makeBird() {
    RigidBody2D bird;
    bird.shape.type = ShapeType::Circle;
    bird.shape.radius = 0.35f;
    bird.position = Vec2{-6.0f, -1.5f};
    bird.mass = 1.0f;
    bird.invMass = 1.0f / bird.mass;
    bird.inertia = 0.5f * bird.mass * bird.shape.radius * bird.shape.radius;
    bird.invInertia = 1.0f / bird.inertia;
    bird.restitution = 0.3f;
    bird.staticFriction = 0.45f;
    bird.dynamicFriction = 0.35f;
    bird.isStatic = false;
    return bird;
}

RigidBody2D makeBrick(const Vec2& position) {
    RigidBody2D brick;
    brick.shape.type = ShapeType::Box;
    brick.shape.halfExtents = Vec2{0.55f, 0.22f};
    brick.position = position;
    brick.mass = 1.4f;
    brick.invMass = 1.0f / brick.mass;
    brick.inertia = boxInertia(brick.mass, brick.shape.halfExtents);
    brick.invInertia = 1.0f / brick.inertia;
    brick.restitution = 0.05f;
    brick.staticFriction = 0.8f;
    brick.dynamicFriction = 0.62f;
    brick.isStatic = false;
    return brick;
}

RigidBody2D makeGround() {
    RigidBody2D ground;
    ground.shape.type = ShapeType::Box;
    ground.shape.halfExtents = Vec2{12.0f, 0.7f};
    ground.position = Vec2{0.0f, -4.3f};
    ground.mass = 0.0f;
    ground.invMass = 0.0f;
    ground.inertia = 0.0f;
    ground.invInertia = 0.0f;
    ground.restitution = 0.2f;
    ground.staticFriction = 0.95f;
    ground.dynamicFriction = 0.8f;
    ground.isStatic = true;
    return ground;
}
}  // namespace

Scene::Scene() { reset(); }

void Scene::reset() {
    paused_ = false;
    bodies_.clear();
    bodies_.push_back(makeBird());
    bodies_.push_back(makeGround());

    // A small stack/tower target for bird impacts.
    bodies_.push_back(makeBrick(Vec2{4.8f, -3.3f}));
    bodies_.push_back(makeBrick(Vec2{5.95f, -3.3f}));
    bodies_.push_back(makeBrick(Vec2{5.4f, -2.75f}));
    bodies_.push_back(makeBrick(Vec2{5.4f, -2.2f}));
}

std::vector<RigidBody2D>& Scene::getBodies() { return bodies_; }

const std::vector<RigidBody2D>& Scene::getBodies() const { return bodies_; }

bool Scene::isPaused() const { return paused_; }

void Scene::setPaused(bool paused) { paused_ = paused; }

void Scene::togglePause() { paused_ = !paused_; }
