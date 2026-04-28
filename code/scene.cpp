#include "scene.h"

#include <algorithm>

namespace {
float boxInertia(float mass, const Vec2& halfExtents) {
    // Rectangle inertia around center: I = m * (w^2 + h^2) / 12.
    const float width = 2.0f * halfExtents.x;
    const float height = 2.0f * halfExtents.y;
    return mass * (width * width + height * height) / 12.0f;
}

RigidBody2D makeBird(const Vec2& position) {
    RigidBody2D bird;
    bird.shape.type = ShapeType::Circle;
    bird.shape.radius = 0.35f;
    bird.position = position;
    bird.mass = 1.0f;
    bird.invMass = 1.0f / bird.mass;
    bird.inertia = 0.5f * bird.mass * bird.shape.radius * bird.shape.radius;
    bird.invInertia = 1.0f / bird.inertia;
    bird.restitution = 0.3f;
    bird.staticFriction = 0.45f;
    bird.dynamicFriction = 0.35f;
    bird.affectedByGravity = false;
    bird.isStatic = false;
    return bird;
}

RigidBody2D makeWoodBlock(const Vec2& position, const Vec2& halfExtents, float angle = 0.0f) {
    constexpr float kWoodDensity = 2.9f;
    constexpr float kMinWoodMass = 0.5f;

    RigidBody2D block;
    block.shape.type = ShapeType::Box;
    block.shape.halfExtents = halfExtents;
    block.position = position;
    block.angle = angle;

    const float width = 2.0f * halfExtents.x;
    const float height = 2.0f * halfExtents.y;
    block.mass = std::max(kMinWoodMass, width * height * kWoodDensity);
    block.invMass = 1.0f / block.mass;
    block.inertia = boxInertia(block.mass, block.shape.halfExtents);
    block.invInertia = 1.0f / block.inertia;
    block.restitution = 0.05f;
    block.staticFriction = 0.82f;
    block.dynamicFriction = 0.66f;
    block.isStatic = false;
    return block;
}

void addWoodBlock(std::vector<RigidBody2D>& bodies,
                  float x,
                  float y,
                  float halfWidth,
                  float halfHeight,
                  float angle = 0.0f) {
    bodies.push_back(makeWoodBlock(Vec2{x, y}, Vec2{halfWidth, halfHeight}, angle));
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
    isDragging_ = false;
    gameState_ = GameState::Ready;
    bodies_.clear();
    bodies_.push_back(makeBird(birdStartPosition_));
    birdStartPosition_ = bodies_.front().position;
    bodies_.push_back(makeGround());

    constexpr float groundTop = -3.6f;
    constexpr float beamHalfHeight = 0.18f;
    constexpr float shortBeamHalfWidth = 0.55f;
    constexpr float longBeamHalfWidth = 0.95f;
    constexpr float postHalfWidth = 0.16f;
    constexpr float lowerPostHalfHeight = 0.72f;
    constexpr float upperPostHalfHeight = 0.60f;
    constexpr float crateHalfExtent = 0.30f;
    constexpr float capPostHalfHeight = 0.34f;

    const float foundationY = groundTop + beamHalfHeight;
    const float lowerPostY = foundationY + beamHalfHeight + lowerPostHalfHeight;
    const float midBeamY = lowerPostY + lowerPostHalfHeight + beamHalfHeight;
    const float upperPostY = midBeamY + beamHalfHeight + upperPostHalfHeight;
    const float upperBeamY = upperPostY + upperPostHalfHeight + beamHalfHeight;
    const float crateY = upperBeamY + beamHalfHeight + crateHalfExtent;
    const float capBeamY = crateY + crateHalfExtent + beamHalfHeight;
    const float capPostY = capBeamY + beamHalfHeight + capPostHalfHeight;

    // Foundation row
    for (float x : {4.00f, 5.15f, 6.30f, 7.45f, 8.60f}) {
        addWoodBlock(bodies_, x, foundationY, shortBeamHalfWidth, beamHalfHeight);
    }

    // Lower floor posts with a central gate opening.
    for (float x : {3.75f, 4.45f, 5.45f, 6.95f, 7.95f, 8.65f}) {
        addWoodBlock(bodies_, x, lowerPostY, postHalfWidth, lowerPostHalfHeight);
    }

    // Mid-level beams: left tower, gate lintel, right tower.
    addWoodBlock(bodies_, 4.10f, midBeamY, shortBeamHalfWidth, beamHalfHeight);
    addWoodBlock(bodies_, 6.20f, midBeamY, longBeamHalfWidth, beamHalfHeight);
    addWoodBlock(bodies_, 8.30f, midBeamY, shortBeamHalfWidth, beamHalfHeight);

    // Upper floor posts.
    for (float x : {3.85f, 4.35f, 5.55f, 6.85f, 8.05f, 8.55f}) {
        addWoodBlock(bodies_, x, upperPostY, postHalfWidth, upperPostHalfHeight);
    }

    // Upper deck beams.
    addWoodBlock(bodies_, 4.10f, upperBeamY, shortBeamHalfWidth, beamHalfHeight);
    addWoodBlock(bodies_, 6.20f, upperBeamY, longBeamHalfWidth, beamHalfHeight);
    addWoodBlock(bodies_, 8.30f, upperBeamY, shortBeamHalfWidth, beamHalfHeight);

    // Interior crates create weak points and a more layered collapse.
    addWoodBlock(bodies_, 4.10f, foundationY + beamHalfHeight + crateHalfExtent, crateHalfExtent, crateHalfExtent);
    addWoodBlock(bodies_, 5.95f, foundationY + beamHalfHeight + crateHalfExtent, crateHalfExtent, crateHalfExtent);
    addWoodBlock(bodies_, 6.45f, foundationY + beamHalfHeight + crateHalfExtent, crateHalfExtent, crateHalfExtent);
    addWoodBlock(bodies_, 8.30f, foundationY + beamHalfHeight + crateHalfExtent, crateHalfExtent, crateHalfExtent);

    // Roof line and top parapet.
    addWoodBlock(bodies_, 5.15f, crateY, crateHalfExtent, crateHalfExtent);
    addWoodBlock(bodies_, 7.25f, crateY, crateHalfExtent, crateHalfExtent);
    addWoodBlock(bodies_, 6.20f, capBeamY, longBeamHalfWidth, beamHalfHeight);
    addWoodBlock(bodies_, 5.45f, capPostY, postHalfWidth, capPostHalfHeight);
    addWoodBlock(bodies_, 6.20f, capPostY, postHalfWidth, capPostHalfHeight);
    addWoodBlock(bodies_, 6.95f, capPostY, postHalfWidth, capPostHalfHeight);

    // Outer braces help the silhouette read as a heavier wooden fortress.
    addWoodBlock(bodies_, 3.25f, -2.82f, 0.70f, 0.14f, 0.42f);
    addWoodBlock(bodies_, 9.10f, -2.82f, 0.70f, 0.14f, -0.42f);
}

std::vector<RigidBody2D>& Scene::getBodies() { return bodies_; }

const std::vector<RigidBody2D>& Scene::getBodies() const { return bodies_; }

bool Scene::isPaused() const { return paused_; }

void Scene::setPaused(bool paused) { paused_ = paused; }

void Scene::togglePause() { paused_ = !paused_; }


bool Scene::isDragging() const { return isDragging_; }

void Scene::setDragging(bool dragging) { isDragging_ = dragging; }

Vec2 Scene::getBirdStartPosition() const { return birdStartPosition_; }

void Scene::setBirdStartPosition(const Vec2& position) { birdStartPosition_ = position; }

GameState Scene::getGameState() const { return gameState_; }

void Scene::setGameState(GameState state) { gameState_ = state; }
