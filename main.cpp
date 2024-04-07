#define SDL_MAIN_HANDLED

#include <SDL.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <SDL2/SDL.h>
#include <boost/numeric/odeint.hpp>

const int SCREEN_WIDTH = 800;
const int SCREEN_HEIGHT = 600;

struct Body {
    double x, y; // Position
    double vx, vy; // Velocity
    double mass;
};

typedef std::vector<Body> State;

const double G = 6.67430e-11; // Gravitational constant

void gravity(const State &state, State &dstate_dt, const double t) {
    for (size_t i = 0; i < state.size(); ++i) {
        dstate_dt[i].x = state[i].vx;
        dstate_dt[i].y = state[i].vy;
        dstate_dt[i].vx = 0;
        dstate_dt[i].vy = 0;
        for (size_t j = 0; j < state.size(); ++j) {
            if (i != j) {
                double dx = state[j].x - state[i].x;
                double dy = state[j].y - state[i].y;
                double r = sqrt(dx * dx + dy * dy);
                double force = G * state[i].mass * state[j].mass / (r * r);
                dstate_dt[i].vx += force * dx / (state[i].mass * r);
                dstate_dt[i].vy += force * dy / (state[i].mass * r);
            }
        }
    }
}

void draw(SDL_Renderer *renderer, const State &state) {
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
    SDL_RenderClear(renderer);
    for (const auto &body : state) {
        SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
        SDL_Rect rect = {static_cast<int>(body.x), static_cast<int>(body.y), 5, 5};
        SDL_RenderFillRect(renderer, &rect);
    }
    SDL_RenderPresent(renderer);
}

int main() {
    SDL_Init(SDL_INIT_VIDEO);
    SDL_Window *window;
    SDL_Renderer *renderer;
    SDL_CreateWindowAndRenderer(SCREEN_WIDTH, SCREEN_HEIGHT, 0, &window, &renderer);
    SDL_SetWindowTitle(window, "Three-Body Problem");

    State state = {
        {200, 200, 0, 0, 1e12},  // Body 1
        {400, 200, 0, -5, 1e12}, // Body 2
        {600, 200, 0, 5, 1e12}   // Body 3
    };

    const double dt = 0.01; // Time step

    boost::numeric::odeint::runge_kutta4<State> stepper;
    while (true) {
        draw(renderer, state);
        State next_state(state.size());
        boost::numeric::odeint::integrate_const(stepper, gravity, state, 0.0, dt, dt);
        SDL_Event event;
        if (SDL_PollEvent(&event) && event.type == SDL_QUIT)
            break;
    }

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
