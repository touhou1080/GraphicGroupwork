#pragma once

#include <vector>

#include "physics.h"

class Scene {
  public:
    Scene();

    void reset();

    std::vector<RigidBody2D>& getBodies();
    const std::vector<RigidBody2D>& getBodies() const;

    bool isPaused() const;
    void setPaused(bool paused);
    void togglePause();

  private:
    std::vector<RigidBody2D> bodies_;
    bool paused_ = false;
};
