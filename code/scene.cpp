#include "scene.h"

#include <algorithm>

namespace {
constexpr float kTrajectoryPointMinDistanceSq = 0.02f * 0.02f;

float boxInertia(float mass, const Vec2& halfExtents) {
    // Rectangle inertia around center: I = m * (w^2 + h^2) / 12.
    const float width = 2.0f * halfExtents.x;
    const float height = 2.0f * halfExtents.y;
    return mass * (width * width + height * height) / 12.0f;
}

float lengthSquared(const Vec2& v) { return v.x * v.x + v.y * v.y; }

void applyBirdTypeProperties(RigidBody2D& bird, BirdType type) {
    bird.customTag = static_cast<int>(type);
    if (type == BirdType::Yellow) {
        // Rubber-ball yellow bird: bouncy, slick surface so it skips off blocks.
        bird.restitution = 0.55f;
        bird.staticFriction = 0.20f;
        bird.dynamicFriction = 0.15f;
    } else {
        // Heavy red bird: minimal bounce, grippy.
        bird.restitution = 0.30f;
        bird.staticFriction = 0.45f;
        bird.dynamicFriction = 0.35f;
    }
}

RigidBody2D makeBird(const Vec2& position, BirdType type) {
    RigidBody2D bird;
    bird.shape.type = ShapeType::Circle;
    bird.shape.radius = 0.35f;
    bird.position = position;
    bird.mass = 1.0f;
    bird.invMass = 1.0f / bird.mass;
    bird.inertia = 0.5f * bird.mass * bird.shape.radius * bird.shape.radius;
    bird.invInertia = 1.0f / bird.inertia;
    bird.affectedByGravity = false;
    bird.isStatic = false;
    applyBirdTypeProperties(bird, type);
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

void buildFortressScene(std::vector<RigidBody2D>& bodies) {
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

    for (float x : {4.00f, 5.15f, 6.30f, 7.45f, 8.60f}) {
        addWoodBlock(bodies, x, foundationY, shortBeamHalfWidth, beamHalfHeight);
    }

    for (float x : {3.75f, 4.45f, 5.45f, 6.95f, 7.95f, 8.65f}) {
        addWoodBlock(bodies, x, lowerPostY, postHalfWidth, lowerPostHalfHeight);
    }

    addWoodBlock(bodies, 4.10f, midBeamY, shortBeamHalfWidth, beamHalfHeight);
    addWoodBlock(bodies, 6.20f, midBeamY, longBeamHalfWidth, beamHalfHeight);
    addWoodBlock(bodies, 8.30f, midBeamY, shortBeamHalfWidth, beamHalfHeight);

    for (float x : {3.85f, 4.35f, 5.55f, 6.85f, 8.05f, 8.55f}) {
        addWoodBlock(bodies, x, upperPostY, postHalfWidth, upperPostHalfHeight);
    }

    addWoodBlock(bodies, 4.10f, upperBeamY, shortBeamHalfWidth, beamHalfHeight);
    addWoodBlock(bodies, 6.20f, upperBeamY, longBeamHalfWidth, beamHalfHeight);
    addWoodBlock(bodies, 8.30f, upperBeamY, shortBeamHalfWidth, beamHalfHeight);

    addWoodBlock(bodies, 4.10f, foundationY + beamHalfHeight + crateHalfExtent, crateHalfExtent, crateHalfExtent);
    addWoodBlock(bodies, 5.95f, foundationY + beamHalfHeight + crateHalfExtent, crateHalfExtent, crateHalfExtent);
    addWoodBlock(bodies, 6.45f, foundationY + beamHalfHeight + crateHalfExtent, crateHalfExtent, crateHalfExtent);
    addWoodBlock(bodies, 8.30f, foundationY + beamHalfHeight + crateHalfExtent, crateHalfExtent, crateHalfExtent);

    addWoodBlock(bodies, 5.15f, crateY, crateHalfExtent, crateHalfExtent);
    addWoodBlock(bodies, 7.25f, crateY, crateHalfExtent, crateHalfExtent);
    addWoodBlock(bodies, 6.20f, capBeamY, longBeamHalfWidth, beamHalfHeight);
    addWoodBlock(bodies, 5.45f, capPostY, postHalfWidth, capPostHalfHeight);
    addWoodBlock(bodies, 6.20f, capPostY, postHalfWidth, capPostHalfHeight);
    addWoodBlock(bodies, 6.95f, capPostY, postHalfWidth, capPostHalfHeight);

    addWoodBlock(bodies, 3.25f, -2.82f, 0.70f, 0.14f, 0.42f);
    addWoodBlock(bodies, 9.10f, -2.82f, 0.70f, 0.14f, -0.42f);
}

void buildPyramidScene(std::vector<RigidBody2D>& bodies) {
    // Stable 4-3-2-1 cube pyramid. Each upper cube straddles the seam
    // between two lower cubes, so contacts are vertical and the stack
    // self-supports under gravity.
    constexpr float groundTop = -3.6f;
    constexpr float halfExtent = 0.30f;
    constexpr float spacing = 2.0f * halfExtent;
    constexpr float centerX = 6.0f;
    constexpr int rows = 4;

    for (int row = 0; row < rows; ++row) {
        const int count = rows - row;
        const float y = groundTop + halfExtent + row * spacing;
        for (int i = 0; i < count; ++i) {
            const float x = centerX + (i - (count - 1) * 0.5f) * spacing;
            addWoodBlock(bodies, x, y, halfExtent, halfExtent);
        }
    }
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
    clearBirdTrajectory();
    bodies_.clear();
    bodies_.push_back(makeBird(birdStartPosition_, currentBird_));
    birdStartPosition_ = bodies_.front().position;
    bodies_.push_back(makeGround());

    switch (currentScene_) {
        case SceneType::Fortress:
            buildFortressScene(bodies_);
            break;
        case SceneType::Pyramid:
            buildPyramidScene(bodies_);
            break;
        case SceneType::Count:
            break;
    }
}

void Scene::nextScene() {
    const int next = (static_cast<int>(currentScene_) + 1) %
                     static_cast<int>(SceneType::Count);
    currentScene_ = static_cast<SceneType>(next);
    reset();
}

SceneType Scene::getCurrentSceneType() const { return currentScene_; }

void Scene::nextBird() {
    const int next = (static_cast<int>(currentBird_) + 1) %
                     static_cast<int>(BirdType::Count);
    currentBird_ = static_cast<BirdType>(next);

    // Re-skin the active bird (bodies[0]) in place — keep its position/velocity,
    // just swap its restitution / friction / colour tag. Other bodies (parked
    // older birds, bricks, ground) are not touched, so the scene is preserved.
    if (!bodies_.empty() && bodies_.front().shape.type == ShapeType::Circle) {
        applyBirdTypeProperties(bodies_.front(), currentBird_);
        bodies_.front().isSleeping = false;
        bodies_.front().sleepTimer = 0.0f;
    }
}

void Scene::pruneStaleBirds(float dt) {
    // A bird is "done" if EITHER:
    //   - Its linear speed has stayed below kQuietSpeed for kBirdLifetime
    //     seconds (settled). The threshold is deliberately smaller than the
    //     physics sleep threshold (0.06), so a bird only counts as quiet when
    //     it is essentially frozen, not just rolling slowly.
    //   - Its position has crossed the visible-world margin (flew off-screen).
    constexpr float kQuietSpeed = 0.05f;
    constexpr float kQuietSpeedSq = kQuietSpeed * kQuietSpeed;
    constexpr float kBirdLifetime = 1.0f;
    constexpr float kBoundaryX = 15.0f;
    constexpr float kBoundaryY = 10.0f;

    auto outOfBounds = [&](const RigidBody2D& body) {
        return std::abs(body.position.x) > kBoundaryX ||
               std::abs(body.position.y) > kBoundaryY;
    };

    auto tickQuietTimer = [&](RigidBody2D& body) {
        const float vx = body.velocity.x;
        const float vy = body.velocity.y;
        if (vx * vx + vy * vy < kQuietSpeedSq) {
            body.quietTimer += dt;
        } else {
            body.quietTimer = 0.0f;
        }
        return body.quietTimer >= kBirdLifetime;
    };

    // Parked birds (idx > 0): erase when stale. Walk back-to-front so indices
    // we still want to inspect are not shifted by the erase.
    for (std::size_t i = bodies_.size(); i > 1; --i) {
        const std::size_t idx = i - 1;
        RigidBody2D& body = bodies_[idx];
        if (body.shape.type != ShapeType::Circle || body.isStatic) {
            continue;
        }
        if (outOfBounds(body) || tickQuietTimer(body)) {
            bodies_.erase(bodies_.begin() + idx);
        }
    }

    // Active bird (bodies[0]): only consider it stale once it has been
    // launched. Ready / Dragging keep the bird parked at the start with zero
    // velocity, which would otherwise grow quietTimer and self-trigger.
    // Replace in place rather than erase, because input.cpp expects bodies[0]
    // to always be the player's bird.
    if (gameState_ == GameState::Launched && !bodies_.empty() &&
        bodies_.front().shape.type == ShapeType::Circle && !bodies_.front().isStatic) {
        RigidBody2D& bird = bodies_.front();
        if (outOfBounds(bird) || tickQuietTimer(bird)) {
            bird = makeBird(birdStartPosition_, currentBird_);
            isDragging_ = false;
            gameState_ = GameState::Ready;
        }
    }
}

BirdType Scene::getCurrentBirdType() const { return currentBird_; }

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

void Scene::beginBirdTrajectory(const Vec2& launchPosition) {
    birdTrajectorySegments_.push_back({});
    birdTrajectoryRecording_ = true;
    appendBirdTrajectoryPoint(launchPosition);
}

void Scene::updateBirdTrajectory(const PhysicsStepResult& stepResult) {
    if (!birdTrajectoryRecording_) {
        return;
    }

    if (stepResult.activeBirdContact) {
        appendBirdTrajectoryPoint(stepResult.activeBirdCenterAtContact);
        birdTrajectoryRecording_ = false;
        return;
    }

    if (!bodies_.empty() && gameState_ == GameState::Launched) {
        appendBirdTrajectoryPoint(bodies_.front().position);
    }
}

const std::vector<std::vector<Vec2>>& Scene::getBirdTrajectorySegments() const {
    return birdTrajectorySegments_;
}

void Scene::clearBirdTrajectory() {
    birdTrajectorySegments_.clear();
    birdTrajectoryRecording_ = false;
}

void Scene::appendBirdTrajectoryPoint(const Vec2& point) {
    if (birdTrajectorySegments_.empty()) {
        birdTrajectorySegments_.push_back({});
    }

    std::vector<Vec2>& currentSegment = birdTrajectorySegments_.back();
    if (!currentSegment.empty() &&
        lengthSquared(point - currentSegment.back()) < kTrajectoryPointMinDistanceSq) {
        return;
    }
    currentSegment.push_back(point);
}
