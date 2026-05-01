#include "render.h"

#include <algorithm>
#include <GLFW/glfw3.h>

#define STB_IMAGE_IMPLEMENTATION
#include "third_party/stb_image.h"

#include <cmath>
#include <cstdint>
#include <iostream>
#include <vector>

#include "physics.h"
#include "scene.h"

namespace {
constexpr float kPi = 3.14159265f;

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
    {'Y', {0x11, 0x11, 0x0A, 0x04, 0x04, 0x04, 0x04}},
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
    drawText("B: BIRD TYPE", 4.0f * kPixel, y, kPixel);
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
        const float angle = 2.0f * kPi * t;
        glVertex2f(center.x + radius * std::cos(angle), center.y + radius * std::sin(angle));
    }
    glEnd();
}

void keyOutWhiteBackground(unsigned char* pixels, int width, int height) {
    const int totalPixels = width * height;
    for (int i = 0; i < totalPixels; ++i) {
        unsigned char* pixel = pixels + i * 4;
        const int r = pixel[0];
        const int g = pixel[1];
        const int b = pixel[2];

        if (r > 246 && g > 246 && b > 246) {
            pixel[3] = 0;
            continue;
        }

        if (r > 220 && g > 220 && b > 220) {
            const int minChannel = std::min({r, g, b});
            const int whiteness = 255 - minChannel;
            pixel[3] = static_cast<unsigned char>(std::clamp(whiteness * 8, 0, 255));
        }
    }
}

unsigned int loadTexture(const char* path, bool keyOutWhite = false) {
    int width = 0;
    int height = 0;
    int channels = 0;
    stbi_set_flip_vertically_on_load(true);
    unsigned char* pixels = stbi_load(path, &width, &height, &channels, 4);
    if (pixels == nullptr) {
        std::cerr << "Failed to load texture: " << path << "\n";
        return 0;
    }

    if (keyOutWhite) {
        keyOutWhiteBackground(pixels, width, height);
    }

    unsigned int texture = 0;
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
    glTexImage2D(GL_TEXTURE_2D,
                 0,
                 GL_RGBA,
                 width,
                 height,
                 0,
                 GL_RGBA,
                 GL_UNSIGNED_BYTE,
                 pixels);

    stbi_image_free(pixels);
    glBindTexture(GL_TEXTURE_2D, 0);
    return texture;
}

void drawTexturedQuad(const Vec2& center, float width, float height, float angle, unsigned int texture) {
    if (texture == 0) {
        return;
    }

    const float halfWidth = 0.5f * width;
    const float halfHeight = 0.5f * height;

    glEnable(GL_TEXTURE_2D);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glBindTexture(GL_TEXTURE_2D, texture);
    glColor3f(1.0f, 1.0f, 1.0f);

    glPushMatrix();
    glTranslatef(center.x, center.y, 0.0f);
    glRotatef(angle * 57.2957795f, 0.0f, 0.0f, 1.0f);
    glBegin(GL_QUADS);
    glTexCoord2f(0.0f, 0.0f);
    glVertex2f(-halfWidth, -halfHeight);
    glTexCoord2f(1.0f, 0.0f);
    glVertex2f(halfWidth, -halfHeight);
    glTexCoord2f(1.0f, 1.0f);
    glVertex2f(halfWidth, halfHeight);
    glTexCoord2f(0.0f, 1.0f);
    glVertex2f(-halfWidth, halfHeight);
    glEnd();
    glPopMatrix();

    glBindTexture(GL_TEXTURE_2D, 0);
    glDisable(GL_TEXTURE_2D);
    glDisable(GL_BLEND);
}

void drawTexturedCircleSprite(const Vec2& center, float radius, float angle, unsigned int texture) {
    if (texture == 0) {
        drawCircle(center, radius);
        return;
    }
    constexpr float kSpriteScale = 1.85f;
    const float spriteRadius = radius * kSpriteScale;
    drawTexturedQuad(center, spriteRadius * 2.0f, spriteRadius * 2.0f, angle, texture);
}

void drawBackground(int framebufferWidth, int framebufferHeight, unsigned int texture) {
    if (texture == 0) {
        return;
    }

    if (framebufferHeight <= 0) {
        framebufferHeight = 1;
    }

    constexpr float kWorldHalfHeight = 6.0f;
    const float aspect = static_cast<float>(framebufferWidth) / static_cast<float>(framebufferHeight);
    const float halfWidth = kWorldHalfHeight * aspect;

    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, texture);
    glColor3f(1.0f, 1.0f, 1.0f);

    glBegin(GL_QUADS);
    glTexCoord2f(0.0f, 0.0f);
    glVertex2f(-halfWidth, -kWorldHalfHeight);
    glTexCoord2f(1.0f, 0.0f);
    glVertex2f(halfWidth, -kWorldHalfHeight);
    glTexCoord2f(1.0f, 1.0f);
    glVertex2f(halfWidth, kWorldHalfHeight);
    glTexCoord2f(0.0f, 1.0f);
    glVertex2f(-halfWidth, kWorldHalfHeight);
    glEnd();

    glBindTexture(GL_TEXTURE_2D, 0);
    glDisable(GL_TEXTURE_2D);
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

void drawBirdTrajectory(const std::vector<std::vector<Vec2>>& segments) {
    glColor3f(1.0f, 1.0f, 1.0f);
    glLineWidth(2.0f);
    glEnable(GL_LINE_STIPPLE);
    glLineStipple(1, 0x00FF);

    for (const std::vector<Vec2>& segment : segments) {
        if (segment.size() < 2) {
            continue;
        }
        glBegin(GL_LINE_STRIP);
        for (const Vec2& point : segment) {
            glVertex2f(point.x, point.y);
        }
        glEnd();
    }

    glDisable(GL_LINE_STIPPLE);
    glLineWidth(1.0f);
}

float lengthSquared(const Vec2& v) { return v.x * v.x + v.y * v.y; }

Vec2 normalized(const Vec2& v) {
    const float lenSq = lengthSquared(v);
    if (lenSq <= 1e-8f) {
        return Vec2{0.0f, 0.0f};
    }
    return v / std::sqrt(lenSq);
}

void drawBandSegment(const Vec2& start, const Vec2& end, float thickness) {
    const Vec2 delta = end - start;
    const float lenSq = lengthSquared(delta);
    if (lenSq <= 1e-8f) {
        return;
    }

    const Vec2 tangent = delta / std::sqrt(lenSq);
    const Vec2 normal = Vec2{-tangent.y, tangent.x} * (0.5f * thickness);

    glBegin(GL_QUADS);
    glVertex2f(start.x - normal.x, start.y - normal.y);
    glVertex2f(start.x + normal.x, start.y + normal.y);
    glVertex2f(end.x + normal.x, end.y + normal.y);
    glVertex2f(end.x - normal.x, end.y - normal.y);
    glEnd();
}

struct SlingshotBandGeometry {
    Vec2 leftFork;
    Vec2 rightFork;
    Vec2 pouchLeft;
    Vec2 pouchRight;
};

SlingshotBandGeometry makeSlingshotBandGeometry(const Scene& scene) {
    constexpr Vec2 kSlingshotVisualOffset{0.0f, -1.28f};
    constexpr Vec2 kLeftForkOffset{0.10f, 1.74f};
    constexpr Vec2 kRightForkOffset{1.08f, 1.74f};
    constexpr float kPouchHalfWidth = 0.12f;
    constexpr Vec2 kPouchOffset{0.0f, 0.02f};
    const float kBirdRadius = 0.35f;

    const Vec2 launchPoint = scene.getBirdStartPosition() + kSlingshotVisualOffset;
    const std::vector<RigidBody2D>& bodies = scene.getBodies();
    const Vec2 birdCenter = bodies.empty() ? launchPoint : bodies.front().position;
    const Vec2 pullDirection = normalized(birdCenter - launchPoint);
    const Vec2 pouchCenter = birdCenter + kPouchOffset - pullDirection * (0.2f * kBirdRadius);

    return SlingshotBandGeometry{
        launchPoint + kLeftForkOffset,
        launchPoint + kRightForkOffset,
        pouchCenter + Vec2{-kPouchHalfWidth, 0.0f},
        pouchCenter + Vec2{kPouchHalfWidth, 0.0f},
    };
}

void drawSlingshotBase(const Scene& scene, unsigned int texture) {
    constexpr Vec2 kSlingshotVisualOffset{0.0f, -1.28f};
    constexpr float kSpriteWidth = 3.7f;
    constexpr float kSpriteHeight = 3.7f;
    constexpr Vec2 kSpriteAnchorUv{0.51f, 0.31f};

    const Vec2 launchPoint = scene.getBirdStartPosition() + kSlingshotVisualOffset;
    const Vec2 spriteCenter = launchPoint +
                              Vec2{(0.5f - kSpriteAnchorUv.x) * kSpriteWidth,
                                   (0.5f - kSpriteAnchorUv.y) * kSpriteHeight};
    drawTexturedQuad(spriteCenter, kSpriteWidth, kSpriteHeight, 0.0f, texture);
}

void drawSlingshotBands(const Scene& scene, bool frontLayer) {
    constexpr float kBandThickness = 0.09f;
    if (scene.getGameState() == GameState::Launched) {
        return;
    }

    const SlingshotBandGeometry geometry = makeSlingshotBandGeometry(scene);

    if (frontLayer) {
        glColor3f(0.18f, 0.10f, 0.06f);
        drawBandSegment(geometry.rightFork, geometry.pouchRight, kBandThickness);
        drawBandSegment(geometry.pouchLeft, geometry.pouchRight, kBandThickness * 0.9f);
        return;
    }

    glColor3f(0.30f, 0.18f, 0.10f);
    drawBandSegment(geometry.leftFork, geometry.pouchLeft, kBandThickness);
}
}  // namespace

Renderer::Renderer()
    : redBirdTexture_(loadTexture("assets/red.png")),
      yellowBirdTexture_(loadTexture("assets/yellow.png")),
      pigTexture_(loadTexture("assets/pig.png")),
      backgroundTexture_(loadTexture("assets/background_grassland.png")),
      slingshotTexture_(loadTexture("assets/slingshot.png", true)) {}

void Renderer::render(const Scene& scene, int framebufferWidth, int framebufferHeight) const {
    setupProjection(framebufferWidth, framebufferHeight);

    glClearColor(0.52f, 0.78f, 0.96f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    drawBackground(framebufferWidth, framebufferHeight, backgroundTexture_);

    drawBirdTrajectory(scene.getBirdTrajectorySegments());
    drawSlingshotBase(scene, slingshotTexture_);
    drawSlingshotBands(scene, false);

    const std::vector<RigidBody2D>& bodies = scene.getBodies();
    for (const RigidBody2D& body : bodies) {
        if (body.shape.type == ShapeType::Circle) {
            if (body.customTag == static_cast<int>(BodyTag::Pig)) {
                glColor3f(0.35f, 0.82f, 0.22f);
                drawTexturedCircleSprite(body.position, body.shape.radius, body.angle, pigTexture_);
            } else if (body.customTag == static_cast<int>(BodyTag::BirdYellow)) {
                // Each bird carries its own type tag, so parked birds keep
                // the colour they had when they were launched.
                glColor3f(0.98f, 0.85f, 0.15f);
                drawTexturedCircleSprite(body.position, body.shape.radius, body.angle, yellowBirdTexture_);
            } else {
                glColor3f(0.85f, 0.2f, 0.2f);
                drawTexturedCircleSprite(body.position, body.shape.radius, body.angle, redBirdTexture_);
            }
        } else {
            if (body.isStatic) {
                continue;
            }
            glColor3f(0.7f, 0.5f, 0.3f);
            drawBox(body.position, body.shape.halfExtents, body.angle);
        }
    }

    drawSlingshotBands(scene, true);
    drawHUD(framebufferWidth, framebufferHeight);
}
