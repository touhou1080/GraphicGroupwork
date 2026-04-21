#pragma once

struct GLFWwindow;
class Scene;

class InputController {
  public:
    void fixedUpdate(GLFWwindow* window, Scene& scene);

  private:
    bool previousResetPressed_ = false;
    bool previousPausePressed_ = false;
    bool previousMousePressed_ = false;
};
