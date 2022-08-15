#include <iostream>
#include "logitech_controller.h"

int main(int argc, char *argv[])
{

    char path[] = "/dev/input/js0";
    Logitech gamepad(path);

    gamepad.init();
    int delta_angle = 0;
    std::cout << gamepad.Keystate_map[JSKEY_A] << endl;

    return 0;
}