#include "main.h"

int main(int argc, char *argv[])
{
    dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
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
    // button -- mins, plus
    // J1 -- LB, RB
    // J2 -- LT, RT
    // J3 -- BACK, START
    // J4 -- A, B
    // J5 -- X, Y
    double J1 = 0, J2 = 0, J3 = 0;
    int J4 = MAXIMUM_POSITION_LIMIT_ID1 / 2, J5 = MAXIMUM_POSITION_LIMIT_ID2;
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

    shut_motorPackAll();
    usleep(2000);
    int uart = uart_init(BAUD, UART_PATH);
    int key;

    initMsg(&armJointMsg);
    std::thread recv_thread(uart_recv, uart, &armJointState);
    recv_thread.detach();
    std::thread send_thread(uart_send, uart, &armJointMsg);
    send_thread.detach();
    std::thread detect_keyboard(keyBoardDetector, &key);
    detect_keyboard.detach();

    printf("Press any key to continue. (Press [ESC] to exit)\n");
    if (getch() == ESC_ASCII_VALUE)
        return 0;

    cmdReadPos1(&(armJointMsg.j1));
    cmdReadPos1(&(armJointMsg.j2));
    cmdReadPos1(&(armJointMsg.j3));
    sleep(1);

    const int J2_init = 25000;
    const int J3_init = 10000;

    armJointControl.j2.p_des = armJointState.j2.motorMultiAngle + J2_init;
    packCmd(&(armJointMsg.j2), armJointControl.j2);
    armJointState.switchCmd = true;
    sleep(2);

    armJointControl.j3.p_des = armJointState.j3.motorMultiAngle - J3_init;
    packCmd(&(armJointMsg.j3), armJointControl.j3);
    armJointState.switchCmd = true;
    sleep(2);

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

    std::cout << "\nPress c to continue" << std::endl;
    pressKeytoContinue(&key, 99);

    while (key != 101)
    {
        if (iter % 5 == 0)
        {
            std::cout << "\033[2J\033[1;1H";
            std::cout << "***Press e to exit ***" << std::endl;
            std::cout << "arm: " << std::endl;
            arm_can_state_mut.lock();
            showArmState(&armJointState);
            std::cout << "\n";
            std::cout << "J1 input: " << J1 << std::endl
                      << "J2 input: " << J2 << std::endl
                      << "J3 input: " << J3 << std::endl
                      << "J4 input: " << J4 << std::endl
                      << "J5 input: " << J5 << std::endl
                      << std::endl;
            arm_can_state_mut.unlock();

            joystick->updateCommand(&event, gameCmd);
        }

            J1 -= J1Step * (gameCmd.LB ? 1 : 0);
            J1 += J1Step * (gameCmd.RB ? 1 : 0);
            J2 -= J2Step * (gameCmd.LT ? 1 : 0);
            J2 += J2Step * (gameCmd.RT ? 1 : 0);
            J3 -= J3Step * (gameCmd.BACK ? 1 : 0);
            J3 += J3Step * (gameCmd.START ? 1 : 0);
            J4 -= J4Step * (gameCmd.A ? 1 : 0);
            J4 += J4Step * (gameCmd.B ? 1 : 0);
            J5 -= J5Step * (gameCmd.X ? 1 : 0);
            J5 += J5Step * (gameCmd.Y ? 1 : 0);
            iter++;

            armJointControl.j1.p_des = armJointState.j1.motorMultiAngle + J1;
            packCmd(&(armJointMsg.j1), armJointControl.j1);

            armJointControl.j2.p_des = armJointState.j2.motorMultiAngle + J2_init + J2;
            packCmd(&(armJointMsg.j2), armJointControl.j2);

            armJointControl.j3.p_des = armJointState.j3.motorMultiAngle - J3_init - J3;
            packCmd(&(armJointMsg.j3), armJointControl.j3);

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
            usleep(2000);
    }

    // Disable DYNAMIXEL#1 Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);

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
        printf("Succeeded disabling DYNAMIXEL Torque.\n");
    }

    // Disable DYNAMIXEL#2 Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);

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
        printf("Succeeded disabling DYNAMIXEL Torque.\n");
    }

    // Close port
    portHandler->closePort();

    shut_motorPackAll();
    std::cout << "!!! The Motor Shut Off !!!" << std::endl;
    usleep(2000);

    system("stty echo");
    system("stty icanon");
    sleep(1);

    exit(0);

    return 0;
}