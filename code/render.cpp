#include "render.h"

#include <GLFW/glfw3.h>

#include <cmath>
#include <cstdint>
#include <vector>

#include "physics.h"
#include "scene.h"

namespace {
struct Glyph {
    char c;
    std::uint8_t rows[7];
};

// Minimal 5x7 bitmap font. Each row uses the low 5 bits, MSB = leftmost pixel.
constexpr Glyph kFont[] = {
    {'A', {0x0E, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11}},
    {'B', {0x1E, 0x11, 0x11, 0x1E, 0x11, 0x11, 0x1E}},
    {'C', {0x0F, 0x10, 0x10, 0x10, 0x10, 0x10, 0x0F}},
    {'D', {0x1E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1E}},
    {'E', {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x1F}},
    {'G', {0x0F, 0x10, 0x10, 0x17, 0x11, 0x11, 0x0E}},
    {'H', {0x11, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11}},
    {'I', {0x1F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x1F}},
    {'L', {0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1F}},
    {'N', {0x11, 0x19, 0x15, 0x15, 0x15, 0x13, 0x11}},
    {'P', {0x1E, 0x11, 0x11, 0x1E, 0x10, 0x10, 0x10}},
    {'R', {0x1E, 0x11, 0x11, 0x1E, 0x14, 0x12, 0x11}},
    {'S', {0x0F, 0x10, 0x10, 0x0E, 0x01, 0x01, 0x1E}},
    {'T', {0x1F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04}},
    {'U', {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E}},
    {'X', {0x11, 0x11, 0x0A, 0x04, 0x0A, 0x11, 0x11}},
    {':', {0x00, 0x04, 0x04, 0x00, 0x04, 0x04, 0x00}},
};

const std::uint8_t* findGlyph(char c) {
    for (const Glyph& g : kFont) {
        if (g.c == c) {
            return g.rows;
        }
    }
    return nullptr;
}

void drawChar(char c, float x, float y, float pixelSize) {
    const std::uint8_t* rows = findGlyph(c);
    if (rows == nullptr) {
        return;
    }
    for (int r = 0; r < 7; ++r) {
        for (int col = 0; col < 5; ++col) {
            if (rows[r] & (1 << (4 - col))) {
                const float px = x + col * pixelSize;
                const float pyTop = y - r * pixelSize;
                glRectf(px, pyTop - pixelSize, px + pixelSize, pyTop);
            }
        }
    }
}

void drawText(const char* text, float x, float y, float pixelSize) {
    float cx = x;
    for (const char* p = text; *p != '\0'; ++p) {
        if (*p != ' ') {
            drawChar(*p, cx, y, pixelSize);
        }
        cx += 6.0f * pixelSize;
    }
}

void drawHUD(int framebufferWidth, int framebufferHeight) {
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(0.0, framebufferWidth, 0.0, framebufferHeight, -1.0, 1.0);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    constexpr float kPixel = 4.0f;
    constexpr float kLineGap = 4.0f * kPixel;
    constexpr float kLineHeight = 7.0f * kPixel + kLineGap;

    float y = framebufferHeight - 2.0f * kPixel;
    glColor3f(0.08f, 0.10f, 0.20f);
    drawText("DRAG: LAUNCH BIRD", 4.0f * kPixel, y, kPixel);
    y -= kLineHeight;
    drawText("N: NEXT SCENE", 4.0f * kPixel, y, kPixel);
    y -= kLineHeight;
    drawText("B: NEXT BIRD", 4.0f * kPixel, y, kPixel);
    y -= kLineHeight;
    drawText("R: RESET", 4.0f * kPixel, y, kPixel);
    y -= kLineHeight;
    drawText("P: PAUSE", 4.0f * kPixel, y, kPixel);

    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
}

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
    const BirdType birdType = scene.getCurrentBirdType();
    for (const RigidBody2D& body : bodies) {
        if (body.shape.type == ShapeType::Circle) {
            if (birdType == BirdType::Yellow) {
                glColor3f(0.98f, 0.85f, 0.15f);
            } else {
                glColor3f(0.85f, 0.2f, 0.2f);
            }
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

    drawHUD(framebufferWidth, framebufferHeight);
}
