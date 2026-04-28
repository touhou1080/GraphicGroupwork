#include "input.h"

#include <GLFW/glfw3.h>

#include <cmath>

#include "physics.h"
#include "scene.h"

namespace {
constexpr float kWorldHalfHeight = 6.0f;
constexpr float kLaunchSpeedScale = 6.5f;
constexpr float kMaxDragDistance = 2.2f;

float lengthSquared(const Vec2& v) { return v.x * v.x + v.y * v.y; }

float length(const Vec2& v) { return std::sqrt(lengthSquared(v)); }

Vec2 screenToWorld(GLFWwindow* window, double mouseX, double mouseY) {
    int width = 1;
    int height = 1;
    glfwGetWindowSize(window, &width, &height);
    if (height <= 0) {
        height = 1;
    }

    const float aspect = static_cast<float>(width) / static_cast<float>(height);
    const float halfWidth = kWorldHalfHeight * aspect;

    const float x = static_cast<float>((mouseX / static_cast<double>(width)) * 2.0 - 1.0) * halfWidth;
    const float y = static_cast<float>(1.0 - (mouseY / static_cast<double>(height)) * 2.0) * kWorldHalfHeight;
    return Vec2{x, y};
}

Vec2 clampDragOffset(const Vec2& offset) {
    const float currentLength = length(offset);
    if (currentLength <= kMaxDragDistance || currentLength <= 1e-6f) {
        return offset;
    }
    return offset * (kMaxDragDistance / currentLength);
}
}  // namespace

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

    const bool nextScenePressed = glfwGetKey(window, GLFW_KEY_N) == GLFW_PRESS;
    if (nextScenePressed && !previousNextScenePressed_) {
        scene.nextScene();
    }
    previousNextScenePressed_ = nextScenePressed;

    const bool nextBirdPressed = glfwGetKey(window, GLFW_KEY_B) == GLFW_PRESS;
    if (nextBirdPressed && !previousNextBirdPressed_) {
        scene.nextBird();
    }
    previousNextBirdPressed_ = nextBirdPressed;

    std::vector<RigidBody2D>& bodies = scene.getBodies();
    if (bodies.empty()) {
        previousMousePressed_ = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
        return;
    }

    RigidBody2D& bird = bodies.front();
    const Vec2 startPosition = scene.getBirdStartPosition();

    if (scene.getGameState() == GameState::Ready) {
        bird.position = startPosition;
        bird.velocity = Vec2{0.0f, 0.0f};
        bird.angularVelocity = 0.0f;
        bird.accumulatedForce = Vec2{0.0f, 0.0f};
        bird.accumulatedTorque = 0.0f;
        bird.angle = 0.0f;
        bird.affectedByGravity = false;
        bird.isSleeping = false;
        bird.sleepTimer = 0.0f;
    }

    const bool mousePressed = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
    double mouseX = 0.0;
    double mouseY = 0.0;
    glfwGetCursorPos(window, &mouseX, &mouseY);
    const Vec2 mouseWorld = screenToWorld(window, mouseX, mouseY);

    if (scene.getGameState() == GameState::Ready && mousePressed && !previousMousePressed_) {
        const Vec2 toMouse = mouseWorld - bird.position;
        const float pickRadius = bird.shape.radius * bird.shape.radius;
        if (lengthSquared(toMouse) <= pickRadius) {
            scene.setDragging(true);
            scene.setGameState(GameState::Dragging);
        }
    }

    if (scene.getGameState() == GameState::Dragging) {
        bird.affectedByGravity = false;
        bird.velocity = Vec2{0.0f, 0.0f};
        bird.angularVelocity = 0.0f;
        bird.accumulatedForce = Vec2{0.0f, 0.0f};
        bird.accumulatedTorque = 0.0f;
        bird.isSleeping = false;
        bird.sleepTimer = 0.0f;

        if (mousePressed) {
            const Vec2 offset = clampDragOffset(mouseWorld - startPosition);
            bird.position = startPosition + offset;
            bird.angle = 0.0f;
        } else {
            const Vec2 launchVelocity = (startPosition - bird.position) * kLaunchSpeedScale;
            bird.velocity = launchVelocity;
            bird.angularVelocity = 0.0f;
            bird.affectedByGravity = true;
            scene.setDragging(false);
            scene.setGameState(GameState::Launched);
        }
    }

    previousMousePressed_ = mousePressed;
}
