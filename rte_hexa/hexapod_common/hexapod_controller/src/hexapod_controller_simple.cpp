#include <iostream>
#include <hexapod_controller/hexapod_controller_simple.hpp>

using namespace hexapod_controller;

int main()
{
    HexapodControllerSimple controller({1, 0, 0.5, 0.25, 0.25, 0.5, 1, 0.5, 0.5, 0.25, 0.75, 0.5, 1, 0, 0.5, 0.25, 0.25, 0.5, 1, 0, 0.5, 0.25, 0.75, 0.5, 1, 0.5, 0.5, 0.25, 0.25, 0.5, 1, 0, 0.5, 0.25, 0.75, 0.5}, {});

    for (double t = 0.0; t <= 5.0; t += 0.1)
        auto angles = controller.pos(t);
    return 0;
}
