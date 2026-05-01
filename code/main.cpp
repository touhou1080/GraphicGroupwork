#include <GLFW/glfw3.h>

#include <chrono>
#include <iostream>

#include "input.h"
#include "physics.h"
#include "render.h"
#include "scene.h"

namespace {
constexpr int kWindowWidth = 1280;
constexpr int kWindowHeight = 720;
constexpr float kFixedDt = 1.0f / 60.0f;
}  // namespace

int main() {
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW.\n";
        return 1;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);

    GLFWwindow* window =
        glfwCreateWindow(kWindowWidth, kWindowHeight, "2D Angry Bird Simulation", nullptr, nullptr);
    if (!window) {
        std::cerr << "Failed to create GLFW window.\n";
        glfwTerminate();
        return 1;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    Scene scene;
    PhysicsSystem physicsSystem;
    Renderer renderer;
    InputController inputController;

    auto lastTime = std::chrono::steady_clock::now();
    double accumulator = 0.0;

    while (!glfwWindowShouldClose(window)) {
        auto now = std::chrono::steady_clock::now();
        double frameTime = std::chrono::duration<double>(now - lastTime).count();
        lastTime = now;

        if (frameTime > 0.25) {
            frameTime = 0.25;
        }

        accumulator += frameTime;

        while (accumulator >= kFixedDt) {
            inputController.fixedUpdate(window, scene);
            if (!scene.isPaused()) {
                const PhysicsStepResult stepResult = physicsSystem.step(scene.getBodies(), kFixedDt);
                scene.updateBirdTrajectory(stepResult);
                bool removedBodies = scene.applyPhysicsStepResult(stepResult);
                removedBodies = scene.pruneStaleBirds(kFixedDt) || removedBodies;
                if (removedBodies) {
                    physicsSystem.clearContactCache();
                }
            }
            accumulator -= kFixedDt;
        }

        int fbw = 0;
        int fbh = 0;
        glfwGetFramebufferSize(window, &fbw, &fbh);
        renderer.render(scene, fbw, fbh);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
