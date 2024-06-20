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
        Point position;
        float width;
    };

    struct Quadrotor {
        Point body_start, body_end;
        Rotor left_rotor1, left_rotor2, right_rotor1, right_rotor2;
        float length, height;
    };

    Quadrotor quadrotor;
    quadrotor.length = 100.0f; // Quadrotor body length
    quadrotor.height = 10.0f;  // Quadrotor body height
    quadrotor.left_rotor1.width = 5.0f; // Rotor width
    quadrotor.left_rotor2.width = 5.0f; // Rotor width
    quadrotor.right_rotor1.width = 5.0f; // Rotor width
    quadrotor.right_rotor2.width = 5.0f; // Rotor width

    static int last_tick = 0;
    static int multiplier = 1;

    if (SDL_GetTicks() - last_tick > 200) {
        last_tick = SDL_GetTicks();
        multiplier = multiplier == -1 ? 1 : -1;
    }

    // Calculate body positions
    quadrotor.body_start = { screenX - (quadrotor.length / 2) * std::cos(q_theta), screenY + (quadrotor.length / 2) * std::sin(q_theta) };
    quadrotor.body_end = { screenX + (quadrotor.length / 2) * std::cos(q_theta), screenY - (quadrotor.length / 2) * std::sin(q_theta) };

    // Rotor positions relative to the body
    quadrotor.left_rotor1.position = { quadrotor.body_start.x - 15.0f * multiplier, quadrotor.body_start.y - 15.0f };
    quadrotor.left_rotor2.position = { quadrotor.body_start.x - 15.0f * multiplier, quadrotor.body_start.y + 15.0f };
    quadrotor.right_rotor1.position = { quadrotor.body_end.x + 15.0f * multiplier, quadrotor.body_end.y - 15.0f };
    quadrotor.right_rotor2.position = { quadrotor.body_end.x + 15.0f * multiplier, quadrotor.body_end.y + 15.0f };

    // Updated colors
    int quadrotor_color = 0xFF00FF00;       // Green color for the quadrotor body
    int rotor_mount_color = 0xFF0000FF;     // Blue color for the rotor mountings
    int rotor_color1 = 0xFFFF0000;          // Red color for the large rotors
    int rotor_color2 = 0xFFFFFF00;          // Yellow color for the small rotors

    // Draw the quadrotor body
    thickLineColor(gRenderer.get(), quadrotor.body_start.x, quadrotor.body_start.y, quadrotor.body_end.x, quadrotor.body_end.y, quadrotor.height, quadrotor_color);

    // Draw rotor mountings for the first left rotor
    thickLineColor(gRenderer.get(), quadrotor.body_start.x, quadrotor.body_start.y, quadrotor.left_rotor1.position.x, quadrotor.left_rotor1.position.y, quadrotor.left_rotor1.width, rotor_mount_color);
    // Draw rotor mountings for the second left rotor
    thickLineColor(gRenderer.get(), quadrotor.body_start.x, quadrotor.body_start.y, quadrotor.left_rotor2.position.x, quadrotor.left_rotor2.position.y, quadrotor.left_rotor2.width, rotor_mount_color);
    // Draw rotor mountings for the first right rotor
    thickLineColor(gRenderer.get(), quadrotor.body_end.x, quadrotor.body_end.y, quadrotor.right_rotor1.position.x, quadrotor.right_rotor1.position.y, quadrotor.right_rotor1.width, rotor_mount_color);
    // Draw rotor mountings for the second right rotor
    thickLineColor(gRenderer.get(), quadrotor.body_end.x, quadrotor.body_end.y, quadrotor.right_rotor2.position.x, quadrotor.right_rotor2.position.y, quadrotor.right_rotor2.width, rotor_mount_color);

    // Draw first pair of left rotors
    filledEllipseColor(gRenderer.get(), quadrotor.left_rotor1.position.x, quadrotor.left_rotor1.position.y, 15, 5, rotor_color1);
    filledEllipseColor(gRenderer.get(), quadrotor.left_rotor1.position.x, quadrotor.left_rotor1.position.y, 10, 3, rotor_color2);
    // Draw second pair of left rotors with different sizes
    filledEllipseColor(gRenderer.get(), quadrotor.left_rotor2.position.x, quadrotor.left_rotor2.position.y, 20, 7, rotor_color1);
    filledEllipseColor(gRenderer.get(), quadrotor.left_rotor2.position.x, quadrotor.left_rotor2.position.y, 12, 4, rotor_color2);

    // Draw first pair of right rotors
    filledEllipseColor(gRenderer.get(), quadrotor.right_rotor1.position.x, quadrotor.right_rotor1.position.y, 15, 5, rotor_color1);
    filledEllipseColor(gRenderer.get(), quadrotor.right_rotor1.position.x, quadrotor.right_rotor1.position.y, 10, 3, rotor_color2);
    // Draw second pair of right rotors with different sizes
    filledEllipseColor(gRenderer.get(), quadrotor.right_rotor2.position.x, quadrotor.right_rotor2.position.y, 20, 7, rotor_color1);
    filledEllipseColor(gRenderer.get(), quadrotor.right_rotor2.position.x, quadrotor.right_rotor2.position.y, 12, 4, rotor_color2);
}
