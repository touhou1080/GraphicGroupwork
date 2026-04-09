#include "render.h"

#include <GLFW/glfw3.h>

#include <cmath>
#include <vector>

#include "physics.h"
#include "scene.h"

namespace {
void setupProjection(int framebufferWidth, int framebufferHeight) {
    if (framebufferHeight <= 0) {
        framebufferHeight = 1;
    }
    glViewport(0, 0, framebufferWidth, framebufferHeight);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    const float aspect = static_cast<float>(framebufferWidth) / static_cast<float>(framebufferHeight);
    const float halfHeight = 6.0f;
    const float halfWidth = halfHeight * aspect;
    glOrtho(-halfWidth, halfWidth, -halfHeight, halfHeight, -1.0, 1.0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void drawCircle(const Vec2& center, float radius) {
    glBegin(GL_TRIANGLE_FAN);
    glVertex2f(center.x, center.y);
    constexpr int kSegments = 24;
    for (int i = 0; i <= kSegments; ++i) {
        const float t = static_cast<float>(i) / static_cast<float>(kSegments);
        const float angle = 2.0f * 3.14159265f * t;
        glVertex2f(center.x + radius * std::cos(angle), center.y + radius * std::sin(angle));
    }
    glEnd();
}

void drawBox(const Vec2& center, const Vec2& halfExtents, float angle) {
    glPushMatrix();
    glTranslatef(center.x, center.y, 0.0f);
    glRotatef(angle * 57.2957795f, 0.0f, 0.0f, 1.0f);
    glBegin(GL_QUADS);
    glVertex2f(-halfExtents.x, -halfExtents.y);
    glVertex2f(halfExtents.x, -halfExtents.y);
    glVertex2f(halfExtents.x, halfExtents.y);
    glVertex2f(-halfExtents.x, halfExtents.y);
    glEnd();
    glPopMatrix();
}
}  // namespace

void Renderer::render(const Scene& scene, int framebufferWidth, int framebufferHeight) const {
    setupProjection(framebufferWidth, framebufferHeight);

    glClearColor(0.52f, 0.78f, 0.96f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    const std::vector<RigidBody2D>& bodies = scene.getBodies();
    for (const RigidBody2D& body : bodies) {
        if (body.shape.type == ShapeType::Circle) {
            glColor3f(0.85f, 0.2f, 0.2f);
            drawCircle(body.position, body.shape.radius);
        } else {
            if (body.isStatic) {
                glColor3f(0.25f, 0.55f, 0.25f);
            } else {
                glColor3f(0.7f, 0.5f, 0.3f);
            }
            drawBox(body.position, body.shape.halfExtents, body.angle);
        }
    }
}
