#include "application.h"
#include <imgui.h>

#include <iostream>
#include <math.h>
#include <deque>
#include <chrono>

#include "../boids/boids.h"

#define T float
#define dim 2

class TestApp : public Application
{
#define COLOR_OUT    nvgRGBA(220,50,50,255)
#define COLOR_IN     nvgRGBA(50,50,220,255)
#define COLOR_SOLVED nvgRGBA(50,220,50,255)
#define RED_COLOR nvgRGBA(220,10,10,255)
#define BLUE_COLOR nvgRGBA(10,10,220,255)

typedef Matrix<T, Eigen::Dynamic, 1> VectorXT;
typedef Matrix<T, dim, Eigen::Dynamic> TVStack;
typedef Vector<T, dim> TV;

public:

    TestApp(int w, int h, const char * title) : Application(title, w, h) {

        ImGui::StyleColorsClassic();

        const char* name = IMGUI_FONT_FOLDER"/Cousine-Regular.ttf";
        nvgCreateFont(vg, "sans", name);

        
        
    }

    void process() override {
        std::chrono::high_resolution_clock::time_point now = std::chrono::high_resolution_clock::now();
        if(std::chrono::duration_cast<std::chrono::microseconds>(now-lastFrame).count() >= 10./120. * 1.e6)
        {
            if(keyDown[GLFW_KEY_R])
                boids.initializePositions();
            if(keyDown[GLFW_KEY_SPACE])
                boids.pause();
            if(keyDown[GLFW_KEY_ESCAPE])
                exit(0);
            lastFrame = now;
        }
    }

    void drawImGui() override {

        using namespace ImGui;
        Begin("Menu");
       const char* names[] = {"FreeFall", "Separation", "Alignment", "Cohesion", "Leading", "ALL"};
       Combo("Boids Behavior", (int*)&currentMethod, names, 6);

       const char* int_names[] = { "explicit Euler", "sympletic Euler", "Midpoint"};
       Combo("Integration Scheme", (int*)&currentMethodIntegration, int_names, 3);

       ImGui::Text("Boids %d", boids.getParticleNumber());
       ImGui::Text("Red %d", boids.m_redBoids);
       ImGui::Text("Blue %d", boids.m_blueBoids);

       ImGui::SliderFloat("Step Size", &boids.h, 0.0f, 0.1f);
       ImGui::SliderFloat("new Boids", &boids.m_newBoidDistance, 0.0f, 0.8f);
 
       if (currentMethod == COHESION) {
           ImGui::SliderFloat("Cohesion Radius", &boids.m_radius, 0.0f, 1.0f);
           ImGui::SliderFloat("Cohesion Strength", &boids.m_CohesionStrength, 0.0f, 4.0f);
       }

       if (currentMethod == ALIGNMENT) {
           ImGui::SliderFloat("Alignment Radius", &boids.m_radius, 0.0f, 1.0f);
           ImGui::SliderFloat("Alignment Strength", &boids.m_AlignmentStrength, 0.0f, 4.0f);
       }
       
       if (currentMethod == SEPARATION) {
           ImGui::SliderFloat("Separation distance", &boids.m_sepDis, 0.0f, 1.0f);
           ImGui::SliderFloat("Separation Strength", &boids.m_sepStr, 0.0f, 4.0f);
       }

       if (currentMethod == ALL) {
           ImGui::SliderFloat("Radius", &boids.m_radius, 0.0f, 1.0f);
           ImGui::SliderFloat("Cohesion Strength", &boids.m_CohesionStrength, 0.0f, 4.0f);
           ImGui::SliderFloat("Alignment Strength", &boids.m_AlignmentStrength, 0.0f, 4.0f);
           ImGui::SliderFloat("Separation distance", &boids.m_sepDis, 0.0f, 1.0f);
           ImGui::SliderFloat("Separation Strength", &boids.m_sepStr, 0.0f, 4.0f);
           ImGui::SliderFloat("Control Strength", &boids.m_controlStrength, 0.0f, 4.0f);
       }

       if (currentMethod == LEADER) {
           ImGui::SliderFloat("Radius", &boids.m_radius, 0.0f, 1.0f);
           ImGui::SliderFloat("Cohesion Strength", &boids.m_CohesionStrength, 0.0f, 4.0f);
           ImGui::SliderFloat("Alignment Strength", &boids.m_AlignmentStrength, 0.0f, 4.0f);
           ImGui::SliderFloat("Separation distance", &boids.m_sepDis, 0.0f, 1.0f);
           ImGui::SliderFloat("Separation Strength", &boids.m_sepStr, 0.0f, 4.0f);
           ImGui::SliderFloat("leader strength", &boids.m_leaderStrength, 0.0f, 4.0f);
       }

       End();
    }

    void drawNanoVG() override {
        
        boids.updateBehavior(currentMethod, currentMethodIntegration);
        
        TVStack boids_pos = boids.getPositions();

        auto shift_01_to_screen = [](TV pos_01, T scale, T width, T height)
        {
            return TV(0.5 * (0.5 - scale) * width + scale * pos_01[0] * width, 0.5 * (0.5 - scale) * height + scale * pos_01[1] * height);
        };

        auto shift_screen_to_01 = [](TV pos_01, T scale, T width, T height)
        {
            return TV((pos_01[0] - 0.5 * (0.5 - scale) * width) / (scale * width), (pos_01[1] - 0.5 * (0.5 - scale) * height) / (scale * height));
        };

        for(int i = 0; i < boids.getParticleNumber(); i++)
        {
            TV pos = boids_pos.col(i);
            nvgBeginPath(vg);
        
            // just map position from 01 simulation space to scree space
            // feel free to make changes
            // the only thing that matters is you have pos computed correctly from your simulation
            T scale = 0.06;
            TV screen_pos = shift_01_to_screen(TV(pos[0], pos[1]), scale, width, height);
            nvgCircle(vg, screen_pos[0], screen_pos[1], 2.f);

            nvgCircle(vg, mouseState.lastMouseX, mouseState.lastMouseY, 5.f);

            if (i == 0 && false) {
                nvgFillColor(vg, nvgRGBA(50, 220, 10, 255));
                nvgCircle(vg, screen_pos[0], screen_pos[1], 7.f);
            } 
            else if(boids.boidGroups.col(i)[0] == BoidGroups::RED)
                nvgFillColor(vg, RED_COLOR);
            else if(boids.boidGroups.col(i)[0] == BoidGroups::BLUE)
                nvgFillColor(vg, BLUE_COLOR);
            nvgFill(vg);
        

        }

        nvgBeginPath(vg);

        nvgCircle(vg, mouseState.lastMouseX, mouseState.lastMouseY, 3.f);
        TV non_screen_pos = shift_screen_to_01(TV(mouseState.lastMouseX, mouseState.lastMouseY), 0.06, width, height);
        boids.leaderPos = non_screen_pos;
        //std::cout << non_screen_pos[0] << " " << non_screen_pos[1] << std::endl;
        nvgFillColor(vg, nvgRGBA(50, 220, 10, 255));
        nvgFill(vg);

        //draw circle
        /*
        nvgBeginPath(vg);
        int circlePosX = 400;
        int circlePosY = 300;

        nvgCircle(vg, circlePosX, circlePosY, 30.f);
        

        boids.circlePos = shift_screen_to_01(TV(circlePosX, circlePosY), 0.06, width, height);

        nvgFillColor(vg, nvgRGBA(50, 0, 220, 255));
        nvgFill(vg);
        */
        

    }

protected:
    void mouseButtonPressed(int button, int mods) override {

    }

    void mouseButtonReleased(int button, int mods) override {
        
    }

private:
    int loadFonts(NVGcontext* vg)
    {
        int font;
        font = nvgCreateFont(vg, "sans", "../example/Roboto-Regular.ttf");
        if (font == -1) {
            printf("Could not add font regular.\n");
            return -1;
        }
        font = nvgCreateFont(vg, "sans-bold", "../example/Roboto-Bold.ttf");
        if (font == -1) {
            printf("Could not add font bold.\n");
            return -1;
        }
        return 0;
    }

private:

    MethodTypes currentMethod = LEADER;

    IntegrationMethodTypes currentMethodIntegration = EXPLICIT;

    Boids<T, dim> boids = Boids<T, dim>(400);
    std::chrono::high_resolution_clock::time_point lastFrame;
};

int main(int, char**)
{
    int width = 720;
    int height = 720;
    TestApp app(width, height, "Assignment 3 Boids");
    app.run();

    

    return 0;
}
