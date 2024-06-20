#include "planar_quadrotor_visualizer.h"
#include <cmath>
#pragma once

#include <memory>
#include <SDL.h>
#include <matplot/matplot.h>
#include <SDL2_gfx/SDL2_gfxPrimitives.h>

PlanarQuadrotorVisualizer::PlanarQuadrotorVisualizer(PlanarQuadrotor* quadrotor_ptr) : quadrotor_ptr(quadrotor_ptr) {}

// Utility function to convert quadrotor coordinates to screen coordinates
std::pair<int, int> PlanarQuadrotorVisualizer::convertToScreenCoordinates(float x, float y, int screenWidth, int screenHeight) {
    int screenX = static_cast<int>(screenWidth / 2 + x);
    int screenY = static_cast<int>(screenHeight / 2 - y);  // Invert y-axis for screen coordinates
    return std::make_pair(screenX, screenY);
}

void PlanarQuadrotorVisualizer::render(std::shared_ptr<SDL_Renderer>& gRenderer) {
    Eigen::VectorXf state = quadrotor_ptr->GetState();
    float q_x, q_y, q_theta;

    // Extract x, y, theta coordinates
    q_x = state[0];
    q_y = state[1];
    q_theta = state[2];

    int screenWidth, screenHeight;
    SDL_GetRendererOutputSize(gRenderer.get(), &screenWidth, &screenHeight);

    auto [screenX, screenY] = convertToScreenCoordinates(q_x, q_y, screenWidth, screenHeight);

    struct Point {
        float x, y;
    };

    struct Rotor {
        Point offset;
        Point position;
        Point large_left, small_left, large_right, small_right;
        int width;
    };

    struct Quadrotor {
        Point left_top, right_bottom;
        Rotor rotor;
        float length, height;
    };

    Quadrotor quadrotor;
    quadrotor.length = 100.0f; // Quadrotor body length
    quadrotor.height = 10.0f;  // Quadrotor body height
    quadrotor.rotor.width = 5; // Rotor width

    static int last_tick = 0;
    static int multiplier = 1;

    if (SDL_GetTicks() - last_tick > 200) {
        last_tick = SDL_GetTicks();
        multiplier = multiplier == -1 ? 1 : -1;
    }

    // Calculate rotor and body positions
    quadrotor.left_top = { screenX - std::cos(q_theta) * (quadrotor.length / 2), screenY + std::sin(q_theta) * (quadrotor.length / 2) };
    quadrotor.right_bottom = { screenX + (quadrotor.length / 2) * std::cos(q_theta), screenY - std::sin(q_theta) * (quadrotor.length / 2) };

    // Rotor positions relative to the body
    quadrotor.rotor.offset = { quadrotor.length * std::cos(q_theta), quadrotor.length * std::sin(q_theta) };
    quadrotor.rotor.position = { quadrotor.height * std::sin(q_theta), quadrotor.height * std::cos(q_theta) };

    quadrotor.rotor.large_left = { quadrotor.left_top.x + (15.0f * multiplier), quadrotor.left_top.y };
    quadrotor.rotor.small_left = { quadrotor.left_top.x - (10.0f * multiplier), quadrotor.left_top.y };
    quadrotor.rotor.large_right = { quadrotor.right_bottom.x - (15.0f * multiplier), quadrotor.right_bottom.y };
    quadrotor.rotor.small_right = { quadrotor.right_bottom.x + (10.0f * multiplier), quadrotor.right_bottom.y };

    int quadrotor_color = 0xFF133D86;
    int rotor_mount_color = 0xFF85C335;
    int rotor_color1 = 0xFFFFB340;
    int rotor_color2 = 0xFFD89735;

    // Draw the quadrotor body
    //thickLineColor(gRenderer.get(), quadrotor.left_top.x, quadrotor.left_top.y, quadrotor.right_bottom.x, quadrotor.right_bottom.y, quadrotor.height, quadrotor_color);

    // Draw rotor mountings
    thickLineColor(gRenderer.get(), quadrotor.left_top.x, quadrotor.left_top.y, quadrotor.left_top.x - quadrotor.rotor.offset.x, quadrotor.left_top.y - quadrotor.rotor.offset.y, quadrotor.rotor.width, rotor_mount_color);
    thickLineColor(gRenderer.get(), quadrotor.right_bottom.x, quadrotor.right_bottom.y, quadrotor.right_bottom.x - quadrotor.rotor.offset.x, quadrotor.right_bottom.y - quadrotor.rotor.offset.y, quadrotor.rotor.width, rotor_mount_color);

    // Draw rotors
    filledEllipseColor(gRenderer.get(), quadrotor.rotor.large_left.x, quadrotor.rotor.large_left.y, 15, 5, rotor_color1);
    filledEllipseColor(gRenderer.get(), quadrotor.rotor.small_left.x, quadrotor.rotor.small_left.y, 10, 3, rotor_color2);
    filledEllipseColor(gRenderer.get(), quadrotor.rotor.large_right.x, quadrotor.rotor.large_right.y, 15, 5, rotor_color1);
    filledEllipseColor(gRenderer.get(), quadrotor.rotor.small_right.x, quadrotor.rotor.small_right.y, 10, 3, rotor_color2);

    // Calculate and draw the quadrotor body on the other end of rotor mountings
    float body_offset_x = quadrotor.length * std::cos(q_theta);
    float body_offset_y = quadrotor.length * std::sin(q_theta);
    Point body_left_top = { quadrotor.left_top.x - body_offset_x, quadrotor.left_top.y - body_offset_y };
    Point body_right_bottom = { quadrotor.right_bottom.x - body_offset_x, quadrotor.right_bottom.y - body_offset_y };

    thickLineColor(gRenderer.get(), body_left_top.x, body_left_top.y, body_right_bottom.x, body_right_bottom.y, quadrotor.height, quadrotor_color);
}
