#pragma once

#include <vector>

#include "physics.h"

enum class GameState {
    Ready,
    Dragging,
    Launched
};

enum class SceneType {
    Fortress,
    Pyramid,
    Count
};

enum class BirdType {
    Red,
    Yellow,
    Count
};

class Scene {
  public:
    Scene();

    void reset();
    void nextScene();
    void nextBird();
    void pruneStaleBirds(float dt);
    SceneType getCurrentSceneType() const;
    BirdType getCurrentBirdType() const;

    std::vector<RigidBody2D>& getBodies();
    const std::vector<RigidBody2D>& getBodies() const;

    bool isPaused() const;
    void setPaused(bool paused);
    void togglePause();
    
    //getter and setter
    bool isDragging() const;
    void setDragging(bool dragging);

    Vec2 getBirdStartPosition() const;
    void setBirdStartPosition(const Vec2& position);

    GameState getGameState() const;
    void setGameState(GameState state);

    void beginBirdTrajectory(const Vec2& launchPosition);
    void updateBirdTrajectory(const PhysicsStepResult& stepResult);
    const std::vector<std::vector<Vec2>>& getBirdTrajectorySegments() const;

  private:
    void clearBirdTrajectory();
    void appendBirdTrajectoryPoint(const Vec2& point);

    std::vector<RigidBody2D> bodies_;
    std::vector<std::vector<Vec2>> birdTrajectorySegments_;
    bool paused_ = false;
    bool isDragging_ = false;
    bool birdTrajectoryRecording_ = false;
    Vec2 birdStartPosition_{-6.0f, -1.5f};
    GameState gameState_ = GameState::Ready;
    SceneType currentScene_ = SceneType::Fortress;
    BirdType currentBird_ = BirdType::Red;
};
