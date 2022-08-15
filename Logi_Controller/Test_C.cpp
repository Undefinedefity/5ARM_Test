#include "joystick.h"
#include <unistd.h>
#include <memory>

int main(int argc, char **argv)
{
    // init
    GamepadCommand gameCmd;
    JoystickEvent event;
    std::shared_ptr<Joystick> joystick; // why?
    int iter = 0;                       // iteration counter

    // necessary param
    // b  -- mins, plus
    // J1 -- LB, RB
    // J2 -- LT, RT
    // J3 -- BACK, START
    // J4 -- A, B
    // J5 -- X, Y
    double J1 = 0, J2 = 0, J3 = 0, J4 = 0, J5 = 0;

    // Create an instance of Joystick
    joystick = std::make_shared<Joystick>("/dev/input/js0");

    // Ensure that it was found and that we can use it
    if (!joystick->isFound())
    {
        printf("joystick not found!\n");
        exit(1);
    }
    gameCmd.zero();

    while (true)
    {
        // Restrict rate
        usleep(2000); // <unistd.h>

        if (iter % 10 == 0)
            joystick->updateCommand(&event, gameCmd);

        J1 -= gameCmd.LB ? 1 : 0;
        J1 += gameCmd.RB ? 1 : 0;
        J2 -= gameCmd.LT ? 1 : 0;
        J2 += gameCmd.RT ? 1 : 0;
        J3 -= gameCmd.BACK ? 1 : 0;
        J3 += gameCmd.START ? 1 : 0;
        J4 -= gameCmd.A ? 1 : 0;
        J4 += gameCmd.B ? 1 : 0;
        J5 -= gameCmd.X ? 1 : 0;
        J5 += gameCmd.Y ? 1 : 0;

        std::cout << "\n\n";
        std::cout << "J1: " << J1 << std::endl
                  << "J2: " << J2 << std::endl
                  << "J3: " << J3 << std::endl
                  << "J4: " << J4 << std::endl
                  << "J5: " << J5 << std::endl
                  << std::endl;
        iter++;
    }
}