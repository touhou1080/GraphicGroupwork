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

class Scene {
  public:
    Scene();

    void reset();
    void nextScene();
    SceneType getCurrentSceneType() const;

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

  private:
    std::vector<RigidBody2D> bodies_;
    bool paused_ = false;
    bool isDragging_ = false;
    Vec2 birdStartPosition_{-6.0f, -1.5f};
    GameState gameState_ = GameState::Ready;
    SceneType currentScene_ = SceneType::Fortress;
};
