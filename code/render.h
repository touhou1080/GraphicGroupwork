#pragma once

class Scene;

class Renderer {
  public:
    Renderer();
    void render(const Scene& scene, int framebufferWidth, int framebufferHeight) const;

  private:
    unsigned int redBirdTexture_ = 0;
    unsigned int yellowBirdTexture_ = 0;
    unsigned int pigTexture_ = 0;
    unsigned int grasslandBackgroundTexture_ = 0;
    unsigned int winterBackgroundTexture_ = 0;
    unsigned int slingshotTexture_ = 0;
};
