#pragma once

class Scene;

class Renderer {
  public:
    void render(const Scene& scene, int framebufferWidth, int framebufferHeight) const;
};
