/**
 * SDL window creation adapted from https://github.com/isJuhn/DoublePendulum
*/
#include "simulate.h"
#include <matplot/matplot.h>
#include <thread>

Eigen::MatrixXf LQR(PlanarQuadrotor& quadrotor, float dt) {
    /* Calculate LQR gain matrix */
    Eigen::MatrixXf Eye = Eigen::MatrixXf::Identity(6, 6);
    Eigen::MatrixXf A = Eigen::MatrixXf::Zero(6, 6);
    Eigen::MatrixXf A_discrete = Eigen::MatrixXf::Zero(6, 6);
    Eigen::MatrixXf B(6, 2);
    Eigen::MatrixXf B_discrete(6, 2);
    Eigen::MatrixXf Q = Eigen::MatrixXf::Identity(6, 6);
    Eigen::MatrixXf R = Eigen::MatrixXf::Identity(2, 2);
    Eigen::MatrixXf K = Eigen::MatrixXf::Zero(6, 6);
    Eigen::Vector2f input = quadrotor.GravityCompInput();

    Q.diagonal() << 4e-3, 4e-3, 4e2, 8e-3, 4.5e-2, 2 / 2 / M_PI;
    R.row(0) << 3e1, 7;
    R.row(1) << 7, 3e1;

    std::tie(A, B) = quadrotor.Linearize();
    A_discrete = Eye + dt * A;
    B_discrete = dt * B;

    return LQR(A_discrete, B_discrete, Q, R);
}

void rysuj(std::vector<float> x_history, std::vector<float> y_history, std::vector<float> theta_history, std::vector<float> time)
{
    matplot::subplot(4, 1, 1);
    matplot::title("x");
    matplot::plot(time, x_history, "r");

    matplot::subplot(4, 1, 2);
    matplot::title("y");
    matplot::plot(time, y_history, "g");

    matplot::subplot(4, 1, 3);
    matplot::title("theta");
    matplot::plot(time, theta_history, "b");

    matplot::subplot(4, 1, 4);
    matplot::title("x-y");
    matplot::plot(x_history, y_history, "y");

    matplot::show();
}

void control(PlanarQuadrotor& quadrotor, const Eigen::MatrixXf& K) {
    Eigen::Vector2f input = quadrotor.GravityCompInput();
    quadrotor.SetInput(input - K * quadrotor.GetControlState());
}

int main(int argc, char* args[])
{
    std::shared_ptr<SDL_Window> gWindow = nullptr;
    std::shared_ptr<SDL_Renderer> gRenderer = nullptr;
    const int SCREEN_WIDTH = 1280;
    const int SCREEN_HEIGHT = 720;

    /**
     * TODO: Extend simulation
     * 1. Set goal state of the mouse when clicking left mouse button (transform the coordinates to the quadrotor world! see visualizer TODO list)
     *    [x, y, 0, 0, 0, 0]
     * 2. Update PlanarQuadrotor from simulation when goal is changed
    */
    Eigen::VectorXf initial_state = Eigen::VectorXf::Zero(6);
    PlanarQuadrotor quadrotor(initial_state);
    PlanarQuadrotorVisualizer quadrotor_visualizer(&quadrotor);
    /**
     * Goal pose for the quadrotor
     * [x, y, theta, x_dot, y_dot, theta_dot]
     * For implemented LQR controller, it has to be [x, y, 0, 0, 0, 0]
    */
    Eigen::VectorXf goal_state = Eigen::VectorXf::Zero(6);
    goal_state << 0, 0, 0, 0, 0, 0;
    quadrotor.SetGoal(goal_state);
    /* Timestep for the simulation */
    const float dt = 0.001;
    Eigen::MatrixXf K = LQR(quadrotor, dt);
    Eigen::Vector2f input = Eigen::Vector2f::Zero(2);

    /**
     * TODO: Plot x, y, theta over time
     * 1. Update x, y, theta history vectors to store trajectory of the quadrotor
     * 2. Plot trajectory using matplot++ when key 'p' is clicked
    */
    std::vector<float> x_history;
    std::vector<float> y_history;
    std::vector<float> theta_history;
    std::vector<float> time_history;
    float time = 0;
    float previous_time = 0;

    if (init(gWindow, gRenderer, SCREEN_WIDTH, SCREEN_HEIGHT) >= 0)
    {
        SDL_Event e;
        bool quit = false;
        float delay;
        int x, y;
        Eigen::VectorXf state = Eigen::VectorXf::Zero(6);

        while (!quit)
        {
            //events
            while (SDL_PollEvent(&e) != 0)
            {
                if (e.type == SDL_QUIT)
                {
                    quit = true;
                }
                else if (e.type == SDL_MOUSEMOTION)
                {
                    SDL_GetMouseState(&x, &y);
                    //std::cout << "Mouse position: (" << x << ", " << y << ")" << std::endl;
                }
                else if (e.type == SDL_MOUSEBUTTONDOWN)
                {
                    SDL_GetMouseState(&x, &y);
                    std::cout << "Mouse position: (" << x << ", " << y << ")" << std::endl;

                    int screenWidth, screenHeight;
                    SDL_GetRendererOutputSize(gRenderer.get(), &screenWidth, &screenHeight);

                    x = static_cast<int>(x - screenWidth / 2);
                    y = static_cast<int>(y - screenHeight / 2);  // Invert y-axis for screen coordinates
                    std::cout << "Q position: (" << x << ", " << y << ")" << std::endl;
                    goal_state << x, y, 0, 0, 0, 0;
                    quadrotor.SetGoal(goal_state);
                }

                else if (e.type == SDL_KEYDOWN && e.key.keysym.sym == SDL_KeyCode::SDLK_p)
                {
                    //Narysuj trajektorie
                    std::cout << "rysuje";
                    std::thread ploting_thread(rysuj, x_history, y_history, theta_history, time_history);
                    ploting_thread.detach();

                }

            }

            SDL_Delay((int)dt * 1000);

            SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0xFF, 0xFF, 0xFF);
            SDL_RenderClear(gRenderer.get());

            /* Quadrotor rendering step */
            quadrotor_visualizer.render(gRenderer);

            SDL_RenderPresent(gRenderer.get());

            /* Simulate quadrotor forward in time */
            control(quadrotor, K);
            quadrotor.Update(dt);
            time += dt;
            //std::cout << "time: (" << time << ")" << std::endl;
            if (std::abs(time - previous_time) >= 1) {
                //Ograniczamy liczbe punktow
                //std::cout << "Hello" << std::endl;
                previous_time = time;
                Eigen::VectorXf current_state = quadrotor.GetState();
                x_history.push_back(current_state[0]);
                y_history.push_back(current_state[1]);
                theta_history.push_back(current_state[2]);
                time_history.push_back(time);
            }

        }
    }
    SDL_Quit();
    return 0;
}

int init(std::shared_ptr<SDL_Window>& gWindow, std::shared_ptr<SDL_Renderer>& gRenderer, const int SCREEN_WIDTH, const int SCREEN_HEIGHT)
{
    if (SDL_Init(SDL_INIT_VIDEO) >= 0)
    {
        SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "1");
        gWindow = std::shared_ptr<SDL_Window>(SDL_CreateWindow("Planar Quadrotor", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN), SDL_DestroyWindow);
        gRenderer = std::shared_ptr<SDL_Renderer>(SDL_CreateRenderer(gWindow.get(), -1, SDL_RENDERER_ACCELERATED), SDL_DestroyRenderer);
        SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0xFF, 0xFF, 0xFF);
    }
    else
    {
        std::cout << "SDL_ERROR: " << SDL_GetError() << std::endl;
        return -1;
    }
    return 0;
}