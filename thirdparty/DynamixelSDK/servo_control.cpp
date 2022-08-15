// ServoControll.cpp
// Two XM540 serial connected with a USB2RDXL
// Using simple Protocol2PacketHandler::write4ByteTxRx to control

#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0

#include <stdlib.h>
#include <stdio.h>
#include <iostream>

#include "dynamixel_sdk.h" // Uses DYNAMIXEL SDK library

#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_POSITION 116
#define ADDR_PRESENT_POSITION 132
#define MINIMUM_POSITION_LIMIT 0        // Refer to the Minimum Position Limit of product eManual
#define MAXIMUM_POSITION_LIMIT_ID1 4095 // ID 1, 360 degree
#define MAXIMUM_POSITION_LIMIT_ID2 1000 // ID 2,  88 degree
#define BAUDRATE 57600

// https://emanual.robotis.com/docs/en/dxl/protocol2/
#define PROTOCOL_VERSION 2.0

// Factory default ID of all DYNAMIXEL is 1
#define DXL1_ID 1 // J4
#define DXL2_ID 2 // J5 gripper

// Use the actual port assigned to the U2D2.
// Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
#define DEVICENAME "/dev/ttyUSB0"

#define TORQUE_ENABLE 1
#define TORQUE_DISABLE 0
#define DXL_MOVING_STATUS_THRESHOLD 20 // DYNAMIXEL moving status threshold
#define ESC_ASCII_VALUE 0x1b           // KEY ESC

#define J4Step 50
#define J5Step 10

// For Logi_Controller
#include "joystick.h"
#include <memory>
#include <unistd.h>

int getch(); //

int main(int argc, char const *argv[])
{
    // Dynamxiel initialization
    // Initialize PortHandler instance
    // Set the port path
    // Get methods and members of PortHandlerLinux or PortHandlerWindows
    dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

    // Initialize PacketHandler instance
    // Set the protocol version
    // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
    dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    int dxl_comm_result = COMM_RX_FAIL;                           // Communication result
    int dxl1_goal_position = 0;                                   // For ID 1 J4
    int dxl2_goal_position = 0;                                   // For ID 2 gripper
    uint8_t dxl_error = 0;                                        // DYNAMIXEL error
    int32_t dxl1_present_position = 0, dxl2_present_position = 0; // Read 4 byte Position data

    // Open port
    if (portHandler->openPort())
    {
        printf("Succeeded to open the port!\n");
    }
    else
    {
        printf("Failed to open the port!\n");
        printf("Press any key to terminate...\n");
        getch();
        return 0;
    }

    // Set port baudrate
    if (portHandler->setBaudRate(BAUDRATE))
    {
        printf("Succeeded to change the baudrate!\n");
    }
    else
    {
        printf("Failed to change the baudrate!\n");
        printf("Press any key to terminate...\n");
        getch();
        return 0;
    }

    // Enable DYNAMIXEL#1 Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }
    else
    {
        printf("Dynamixel#%d has been successfully connected \n", DXL1_ID);
    }

    // Enable Dynamixel#2 Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }
    else
    {
        printf("Dynamixel#%d has been successfully connected \n", DXL2_ID);
    }

    // Logi game controller initialization
    GamepadCommand gameCmd;
    JoystickEvent event;
    std::shared_ptr<Joystick> joystick;
    int iter = 0; // iteration counter

    // necessary param
    // b  -- mins, plus
    // J1 -- LB, RB
    // J2 -- LT, RT
    // J3 -- BACK, START
    // J4 -- A, B
    // J5 -- X, Y
    double J1 = 0, J2 = 0, J3 = 0;
    int J4 = MAXIMUM_POSITION_LIMIT_ID1/2, J5 = MAXIMUM_POSITION_LIMIT_ID2;
    dxl1_goal_position = J4;
    dxl2_goal_position = J5;

    // Create an instance of Joystick
    joystick = std::make_shared<Joystick>("/dev/input/js0");

    // Ensure that it was found and that we can use it
    if (!joystick->isFound())
    {
        printf("joystick not found!\n");
        exit(1);
    }
    gameCmd.zero();

    printf("Press any key to continue. (Press [ESC] to exit)\n");
    if (getch() == ESC_ASCII_VALUE)
        return 0;

    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL1_ID, ADDR_GOAL_POSITION, dxl1_goal_position, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }

    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL2_ID, ADDR_GOAL_POSITION, dxl2_goal_position, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }

    while (true)
    {
        // usleep(1000); // <unistd.h>
        if (iter % 5 == 0)
            joystick->updateCommand(&event, gameCmd);

        J1 -= 0.001 * (gameCmd.LB ? 1 : 0);
        J1 += 0.001 * (gameCmd.RB ? 1 : 0);
        J2 -= 0.001 * (gameCmd.LT ? 1 : 0);
        J2 += 0.001 * (gameCmd.RT ? 1 : 0);
        J3 -= 0.001 * (gameCmd.BACK ? 1 : 0);
        J3 += 0.001 * (gameCmd.START ? 1 : 0);
        J4 -= J4Step * (gameCmd.A ? 1 : 0);
        J4 += J4Step * (gameCmd.B ? 1 : 0);
        J5 -= J5Step * (gameCmd.X ? 1 : 0);
        J5 += J5Step * (gameCmd.Y ? 1 : 0);
        iter++;

        if (J4 >= MAXIMUM_POSITION_LIMIT_ID1)
            dxl1_goal_position = MAXIMUM_POSITION_LIMIT_ID1;
        else if (J4 <= MINIMUM_POSITION_LIMIT)
            dxl1_goal_position = MINIMUM_POSITION_LIMIT;
        else
            dxl1_goal_position = J4;

        if (J5 >= MAXIMUM_POSITION_LIMIT_ID2)
            dxl2_goal_position = MAXIMUM_POSITION_LIMIT_ID2;
        else if (J5 <= MINIMUM_POSITION_LIMIT)
            dxl2_goal_position = MINIMUM_POSITION_LIMIT;
        else
            dxl2_goal_position = J5;

        dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL1_ID, ADDR_GOAL_POSITION, dxl1_goal_position, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        }

        dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL2_ID, ADDR_GOAL_POSITION, dxl2_goal_position, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        }

        if ((abs(dxl1_goal_position - dxl1_present_position) > J4Step) || (abs(dxl2_goal_position - dxl2_present_position) > J5Step))
        {
            // Get Dynamixel#1 present position value
            dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL1_ID, ADDR_PRESENT_POSITION, (uint32_t *)&dxl1_present_position, &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS)
            {
                printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
            }
            else if (dxl_error != 0)
            {
                printf("%s\n", packetHandler->getRxPacketError(dxl_error));
            }

            // Get Dynamixel#2 present position value
            dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL2_ID, ADDR_PRESENT_POSITION, (uint32_t *)&dxl2_present_position, &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS)
            {
                printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
            }
            else if (dxl_error != 0)
            {
                printf("%s\n", packetHandler->getRxPacketError(dxl_error));
            }

            std::cout << "\n\n";
            std::cout << "J1: " << J1 << std::endl
                      << "J2: " << J2 << std::endl
                      << "J3: " << J3 << std::endl
                      << "J4: " << J4 << std::endl
                      << "dxl1_present_position: "
                      << dxl1_goal_position << std::endl
                      << "J5: " << J5 << std::endl
                      << "dxl2_present_position: "
                      << dxl2_goal_position << std::endl;
        }
    }

    return 0;
}

int getch()
{
#if defined(__linux__) || defined(__APPLE__)
    struct termios oldt, newt;
    int ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
#elif defined(_WIN32) || defined(_WIN64)
    return _getch();
#endif
}