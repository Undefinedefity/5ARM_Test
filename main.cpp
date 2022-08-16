#include "main.h"

int main(int argc, char *argv[])
{
    // dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    // dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    // int dxl_comm_result = COMM_RX_FAIL;                           // Communication result
    // int dxl1_goal_position = 0;                                   // For ID 1 J4
    // int dxl2_goal_position = 0;                                   // For ID 2 gripper
    // uint8_t dxl_error = 0;                                        // DYNAMIXEL error
    // int32_t dxl1_present_position = 0, dxl2_present_position = 0; // Read 4 byte Position data

    // // Open port
    // if (portHandler->openPort())
    // {
    //     printf("Succeeded to open the port!\n");
    // }
    // else
    // {
    //     printf("Failed to open the port!\n");
    //     printf("Press any key to terminate...\n");
    //     getch();
    //     return 0;
    // }

    // // Set port baudrate
    // if (portHandler->setBaudRate(BAUDRATE))
    // {
    //     printf("Succeeded to change the baudrate!\n");
    // }
    // else
    // {
    //     printf("Failed to change the baudrate!\n");
    //     printf("Press any key to terminate...\n");
    //     getch();
    //     return 0;
    // }

    // // Enable DYNAMIXEL#1 Torque
    // dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    // if (dxl_comm_result != COMM_SUCCESS)
    // {
    //     printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    // }
    // else if (dxl_error != 0)
    // {
    //     printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    // }
    // else
    // {
    //     printf("Dynamixel#%d has been successfully connected \n", DXL1_ID);
    // }

    // // Enable Dynamixel#2 Torque
    // dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    // if (dxl_comm_result != COMM_SUCCESS)
    // {
    //     printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    // }
    // else if (dxl_error != 0)
    // {
    //     printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    // }
    // else
    // {
    //     printf("Dynamixel#%d has been successfully connected \n", DXL2_ID);
    // }

    // // Logi game controller initialization
    // GamepadCommand gameCmd;
    // JoystickEvent event;
    // std::shared_ptr<Joystick> joystick;
    // int iter = 0; // iteration counter

    // // necessary param
    // // button -- mins, plus
    // // J1 -- LB, RB
    // // J2 -- LT, RT
    // // J3 -- BACK, START
    // // J4 -- A, B
    // // J5 -- X, Y
    // double J1 = 0, J2 = 0, J3 = 0;
    // int J4 = MAXIMUM_POSITION_LIMIT_ID1 / 2, J5 = MAXIMUM_POSITION_LIMIT_ID2;
    // dxl1_goal_position = J4;
    // dxl2_goal_position = J5;

    // // Create an instance of Joystick
    // joystick = std::make_shared<Joystick>("/dev/input/js0");

    // // Ensure that it was found and that we can use it
    // if (!joystick->isFound())
    // {
    //     printf("joystick not found!\n");
    //     exit(1);
    // }
    // gameCmd.zero();

    // printf("Press any key to continue. (Press [ESC] to exit)\n");
    // if (getch() == ESC_ASCII_VALUE)
    //     return 0;

    // dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL1_ID, ADDR_GOAL_POSITION, dxl1_goal_position, &dxl_error);
    // if (dxl_comm_result != COMM_SUCCESS)
    // {
    //     printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    // }
    // else if (dxl_error != 0)
    // {
    //     printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    // }

    // dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL2_ID, ADDR_GOAL_POSITION, dxl2_goal_position, &dxl_error);
    // if (dxl_comm_result != COMM_SUCCESS)
    // {
    //     printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    // }
    // else if (dxl_error != 0)
    // {
    //     printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    // }

    int uart = uart_init(BAUD, UART_PATH);
    int key;

    initMsg(&armJointMsg);
    std::thread recv_thread(uart_recv, uart, &armJointState);
    recv_thread.detach();
    std::thread send_thread(uart_send, uart, &armJointMsg);
    send_thread.detach();
    std::thread detect_keyboard(keyBoardDetector, &key);
    detect_keyboard.detach();

    std::cout << "Press c to continue or e to terminate" << std::endl;
    pressKeytoContinue(&key, 99);

    int value = 0;

    while (key != 101)
    {
        std::cout << "arm: " << std::endl;
        arm_can_state_mut.lock();
        showArmState(&armJointState);
        arm_can_state_mut.unlock();
        
        std::cout << "input value: ";
        std::cin >> value;
        armJointControl.j2.p_des = armJointState.j2.motorMultiAngle + value; 
        std::cout << "armJointControl.j2.p_des is " << armJointControl.j2.p_des << std::endl;
        // packCmd(&(armJointMsg.j2), armJointControl.j2);
        // armJointState.switchCmd = true;
    }

    return 0;
}