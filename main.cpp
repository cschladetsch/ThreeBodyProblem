#define SDL_MAIN_HANDLED

#include <SDL.h>
#include <iostream>
#include <vector>
#include <cmath>

const int SCREEN_WIDTH = 800;
const int SCREEN_HEIGHT = 800;

struct Body {
    int number;
    double x, y; // Position
    double vx, vy; // Velocity
    double mass;
};

std::ostream& operator<<(std::ostream& out, const Body& body) {
    return out << body.number << ": " << body.x << ", " << body.y << std::endl;
}

typedef std::vector<Body> State;

const double G = 1;

void calculateForces(const State &state, std::vector<double> &fx, std::vector<double> &fy) {
    size_t n = state.size();
    fx.assign(n, 0.0);
    fy.assign(n, 0.0);

    for (size_t i = 0; i < n; ++i) {
        for (size_t j = 0; j < n; ++j) {
            if (i != j) {
                double dx = state[j].x - state[i].x;
                double dy = state[j].y - state[i].y;
                double r = sqrt(dx * dx + dy * dy);
                double force = G * state[i].mass * state[j].mass / (r * r);
                fx[i] += force * dx / r;
                fy[i] += force * dy / r;
            }
        }
    }
}

template <class T>
T clip(T low, T high, T value) {
    if (value <= low) return value = high;
    return value >= high ? low : value;
}

void updateState(State& state, double dt) {
    std::vector<double> fx, fy;
    calculateForces(state, fx, fy);

    size_t i = 0;
    for (auto& body : state) {
        double ax = fx[i] / body.mass;
        double ay = fy[i] / body.mass;
        body.vx += ax * dt;
        body.vy += ay * dt;
        body.x += body.vx * dt;
        body.y += body.vy * dt;

        body.x = clip(body.x, 0.0, static_cast<double>(SCREEN_WIDTH));
        body.y = clip(body.y, 0.0, static_cast<double>(SCREEN_HEIGHT));

        ++i;
    }
}
void draw(SDL_Renderer *renderer, const State &state) {
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
    SDL_RenderClear(renderer);
    for (const auto &body : state) {
        SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
        SDL_Rect rect = {static_cast<int>(body.x), static_cast<int>(body.y), 5*body.mass/100, 5*body.mass/100};
        SDL_RenderFillRect(renderer, &rect);
    }
    SDL_RenderPresent(renderer);
}

void drawState(std::vector<Body> const& bodies) {
    for (auto const& body : bodies) {
        std::cout << body << std::endl;
    }
}

int main() {
    SDL_Init(SDL_INIT_VIDEO);
    SDL_Window *window;
    SDL_Renderer *renderer;
    SDL_CreateWindowAndRenderer(SCREEN_WIDTH, SCREEN_HEIGHT, 0, &window, &renderer);
    SDL_SetWindowTitle(window, "Three-Body Problem");

    State state = {
        {0, 200, 200, 0, 0, 320},  // Body 1
        {1, 400, 200, -0.5, 0.3, 120}, // Body 2
        {2, 600, 200, 0.5, 00.2, 80}   // Body 3
    };

    const double dt = 0.01; // Time step

    while (true) {
        draw(renderer, state);
        updateState(state, dt);
        SDL_Event event;
        if (SDL_PollEvent(&event) && event.type == SDL_QUIT)
            break;
    }

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
