#include "input.h"

#include <GLFW/glfw3.h>

#include "scene.h"

void InputController::fixedUpdate(GLFWwindow* window, Scene& scene) {
    const bool resetPressed = glfwGetKey(window, GLFW_KEY_R) == GLFW_PRESS;
    if (resetPressed && !previousResetPressed_) {
        scene.reset();
    }
    previousResetPressed_ = resetPressed;

    const bool pausePressed = glfwGetKey(window, GLFW_KEY_P) == GLFW_PRESS;
    if (pausePressed && !previousPausePressed_) {
        scene.togglePause();
    }
    previousPausePressed_ = pausePressed;
}
