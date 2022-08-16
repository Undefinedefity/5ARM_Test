#include "uart.hpp"
#include "sys/time.h"

using namespace std;

// initial control message format
uint8_t txMsg[19] = {0x55,                   //frame head
                     0x00,                   //cmd
                     0x00,                   //subcmd
                     0x13,                   //frame length
                     0x00, 0x00, 0x00, 0x00, //can id
                     0x00,                   //can port
                     0x08,
                     0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, //DATA
                     0xaa};                                          //frame tail

// can id for one leg
uint32_t armMotorJ1 = 0x02 + 0x140;
uint32_t armMotorJ2 = 0x01 + 0x140;
uint32_t armMotorJ3 = 0x03 + 0x140;

void initMsg(ArmJointMsg *armCan)
{
    for (int i = 0; i < 19; i++)
    {
        armCan->j1.data[i] = txMsg[i];
        armCan->j2.data[i] = txMsg[i];
        armCan->j3.data[i] = txMsg[i];
    }
    // set CAN id
    for (int i = 0; i < 4; i++)
    {
        armCan->j1.data[i + 4] = (uint8_t) (armMotorJ1 >> (i * 8));
        armCan->j2.data[i + 4] = (uint8_t) (armMotorJ2 >> (i * 8));
        armCan->j3.data[i + 4] = (uint8_t) (armMotorJ3 >> (i * 8));
    }
    // set CAN port to 0 for armMotor
    armCan->j1.data[8] = 0x00;
    armCan->j2.data[8] = 0x00;
    armCan->j3.data[8] = 0x00;
}

void readOneCirclePose(uint8_t *msg, ArmJointState *armJointState)
{
    uint32_t id = (uint32_t) msg[4] + ((uint32_t) msg[5] << 8) + ((uint32_t) msg[6] << 16) + ((uint32_t) msg[7] << 24);
}

void unpackReply(uint8_t *msg, ArmJointState *armJointState)
{
    uint32_t id = (uint32_t) msg[4] + ((uint32_t) msg[5] << 8) + ((uint32_t) msg[6] << 16) + ((uint32_t) msg[7] << 24);
    int16_t i_int = (uint16_t) msg[12] + ((uint16_t) msg[13] << 8);
    int16_t v_int = (uint16_t) msg[14] + ((uint16_t) msg[15] << 8);
    uint16_t p_int = (uint16_t) msg[16] + ((uint16_t) msg[17] << 8);

    // can port 0
    if (msg[8] == 0x00)
    {
        arm_can_state_mut.lock();
        if (id == 0x142)
        {
            armJointState->j1.p = p_int;
            armJointState->j1.v = v_int;
            armJointState->j1.t = i_int;
        } else if (id == 0x141)
        {
            armJointState->j2.p = p_int;
            armJointState->j2.v = v_int;
            armJointState->j2.t = i_int;
        } else if (id == 0x143)
        {
            armJointState->j3.p = p_int;
            armJointState->j3.v = v_int;
            armJointState->j3.t = i_int;
        }
        arm_can_state_mut.unlock();
    }
}

void readMultiCirclePos(uint8_t *msg, ArmJointState *armJointState)
{
    uint32_t id = (uint32_t) msg[4]
                  + ((uint32_t) msg[5] << 8)
                  + ((uint32_t) msg[6] << 16)
                  + ((uint32_t) msg[7] << 24);
//    std::cout << "id is: " << id << std::endl;
//    printf("id %x", id);

    int64_t motorMultiAngle;
    if (msg[17] >> 7 == 1)
    {
//        cout << "-----" << endl;
        motorMultiAngle =
                ((uint64_t) (uint8_t) ~msg[11])
                + ((uint64_t) (uint8_t) ~msg[12] << 8)
                + ((uint64_t) (uint8_t) ~msg[13] << 16)
                + ((uint64_t) (uint8_t) ~msg[14] << 24)
                + ((uint64_t) (uint8_t) ~msg[15] << 32)
                + ((uint64_t) (uint8_t) ~msg[16] << 40)
                + ((uint64_t) (uint8_t) ~msg[17] << 48);
        motorMultiAngle = -motorMultiAngle;
//        unsigned char j;
//        for (int ix = 7; ix >= 0; ix--)
//        {
//            j = ((msg[11] >> ix) & (01));
//            printf("%d", j);
//        }
//        cout << endl;

    } else
    {
        cout << "++++" << endl;
        motorMultiAngle =
                (uint64_t) msg[11]
                + ((uint64_t) msg[12] << 8)
                + ((uint64_t) msg[13] << 16)
                + ((uint64_t) msg[14] << 24)
                + ((uint64_t) msg[15] << 32)
                + ((uint64_t) msg[16] << 40)
                + ((uint64_t) msg[17] << 48);
    }


    if (msg[8] == 0x00)
    {
//        std::cout << "receive msg and the can port is port1!" << std::endl;
        if (id == 0x142)
        {
//            std::cout << "This is joint1!" << std::endl;
            armJointState->j1.motorMultiAngle = motorMultiAngle;
        } else if (id == 0x141)
        {
//            std::cout << "This is joint2!" << std::endl;
            armJointState->j2.motorMultiAngle = motorMultiAngle;
        } else if (id == 0x143)
        {
//            std::cout << "This is joint3!" << std::endl;
            armJointState->j3.motorMultiAngle = motorMultiAngle;
        } else
        {
            std::cout << "id wrong!" << std::endl;
        }
    } else
    {
        std::cout << "can port is 2, fix this issue!" << std::endl;
    }
}

#ifdef TAU_CTRL
void unpackReply(uint8_t *msg, ArmJointState *arm_can)
{
    uint32_t id = (uint32_t) msg[4] + ((uint32_t) msg[5] << 8) + ((uint32_t) msg[6] << 16) + ((uint32_t) msg[7] << 24);
    int16_t i_int = (uint16_t) msg[12] + ((uint16_t) msg[13] << 8);
    int16_t v_int = (uint16_t) msg[14] + ((uint16_t) msg[15] << 8);
    uint16_t p_int = (uint16_t) msg[16] + ((uint16_t) msg[17] << 8);

    // can port 0
    if (msg[8] == 0x00)
    {
        arm_can_state_mut.lock();
        if (id == 0x142)
        {
            arm_can->j1.p = p_int;
            arm_can->j1.v = v_int;
            arm_can->j1.t = i_int;
        } else if (id == 0x141)
        {
            arm_can->j2.p = p_int;
            arm_can->j2.v = v_int;
            arm_can->j2.t = i_int;
        } else if (id == 0x143)
        {
            arm_can->j3.p = p_int;
            arm_can->j3.v = v_int;
            arm_can->j3.t = i_int;
        }
        arm_can_state_mut.unlock();
    }
}

void packCmd(JointMsg *msg, JointControl joint)
{
    // pack ints into the joint_msg //
    int32_t tau = joint.tau;
    msg->data[10] = 0xA1;
    msg->data[11] = 0x00;
    msg->data[12] = 0x00;
    msg->data[13] = 0x00;
    msg->data[14] = (uint8_t) tau;
    msg->data[15] = (uint8_t) (tau >> 8);
    msg->data[16] = 0x00;
    msg->data[17] = 0x00;
}
#endif

void uart_recv(int uartHandle, ArmJointState *arm_can)
{
    std::cout << "step into uart_recv!" << std::endl;
    //TODO: size need to adjust?
    uint8_t r_buf[200];
    uint8_t temp_buf[19];
    int r_len;
    while (1)
    {
        usleep(1);
        memset(r_buf, 0, sizeof(r_buf));
        uart_mut.lock();
        r_len = read(uartHandle, r_buf, sizeof(r_buf));
        uart_mut.unlock();

        if ((r_len > 0) && (r_len % 19 == 0) && (r_buf[0] == 0x55))
        {
            int num_msg = r_len / 19;
            for (int i = 0; i < num_msg; i++)
            {
                for (int j = 0; j < 19; j++)
                {
                    temp_buf[j] = r_buf[j + 19 * i];
                }
                if ((temp_buf[0] == 0x55) && (temp_buf[1] == 0x02) && (temp_buf[18] == 0xaa))
                {
#ifdef READ_ANGLE
                    readMultiCirclePos(temp_buf, arm_can);
#endif

#ifdef SEND_CTRL_CMD
                    if (arm_can->switchCmd == true)
                    {
//                        cout << "switch command, sending arm tau now!" << endl;
                        unpackReply(temp_buf, arm_can);
                    } else
                    {
                        readMultiCirclePos(temp_buf, arm_can);
                    }
#endif
                }
            }
        }
    }
}

void packCmd(JointMsg *msg, JointControl joint)
{
    // pack ints into the joint_msg //
    int32_t p_des = joint.p_des;
    msg->data[10] = 0xA3;
    msg->data[11] = 0x00;
    msg->data[12] = 0x00;
    msg->data[13] = 0x00;
    msg->data[14] = (uint8_t) p_des;
    msg->data[15] = (uint8_t) (p_des >> 8);
    msg->data[16] = (uint8_t) (p_des >> 16);
    msg->data[17] = (uint8_t) (p_des >> 24);
}

void cmdReadPos1(JointMsg *msg)
{
    msg->data[10] = 0x92;
    msg->data[11] = 0x00;
    msg->data[12] = 0x00;
    msg->data[13] = 0x00;
    msg->data[14] = 0x00;
    msg->data[15] = 0x00;
    msg->data[16] = 0x00;
    msg->data[17] = 0x00;
}

void cmdReadPos2(JointMsg *msg)
{
    msg->data[10] = 0x94;
    msg->data[11] = 0x00;
    msg->data[12] = 0x00;
    msg->data[13] = 0x00;
    msg->data[14] = 0x00;
    msg->data[15] = 0x00;
    msg->data[16] = 0x00;
    msg->data[17] = 0x00;
}


void packAll(ArmJointMsg *msg_can, ArmJointControl ctrl_can)
{
    arm_can_msg_mut.lock();
    packCmd(&(msg_can->j1), ctrl_can.j1);
    packCmd(&(msg_can->j2), ctrl_can.j2);
    packCmd(&(msg_can->j3), ctrl_can.j3);
    arm_can_msg_mut.unlock();
}


void uart_send(int uartHandle, ArmJointMsg *msg_can)
{
    int wait_time = 150;
    while (1)
    {
        usleep(wait_time);
        uart_mut.lock();
        arm_can_msg_mut.lock();
        write(uartHandle, &(msg_can->j1), 19);
        usleep(wait_time);
        write(uartHandle, &(msg_can->j2), 19);
        usleep(wait_time);
        write(uartHandle, &(msg_can->j3), 19);
        arm_can_msg_mut.unlock();
        uart_mut.unlock();
    }

}

void shut_motor(JointMsg *msg)
{
    msg->data[10] = 0x80;
    msg->data[11] = 0x00;
    msg->data[12] = 0x00;
    msg->data[13] = 0x00;
    msg->data[14] = 0x00;
    msg->data[15] = 0x00;
    msg->data[16] = 0x00;
    msg->data[17] = 0x00;
}

int uart_open(int fd, const char *pathname)
{
    fd = open(pathname, O_RDWR | O_NOCTTY);
    if (-1 == fd)
    {
        perror("Can't Open Serial Port");
        return (-1);
    }
    if (isatty(STDIN_FILENO) == 0)
        printf("standard input is not a terminal device\n");
    return fd;
}

int uart_set(int fd, int nSpeed, int nBits, char nEvent, int nStop)
{
    struct termios newtio, oldtio;
    if (tcgetattr(fd, &oldtio) != 0)
    {
        perror("SetupSerial 1");
        printf("tcgetattr( fd,&oldtio) -> %d\n", tcgetattr(fd, &oldtio));
        return -1;
    }
    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag |= CLOCAL | CREAD;
    newtio.c_cflag &= ~CSIZE;
    switch (nBits)
    {
        case 7:
            newtio.c_cflag |= CS7;
            break;
        case 8:
            newtio.c_cflag |= CS8;
            break;
    }
    switch (nEvent)
    {
        case 'o':
        case 'O':
            newtio.c_cflag |= PARENB;
            newtio.c_cflag |= PARODD;
            newtio.c_iflag |= (INPCK | ISTRIP);
            break;
        case 'e':
        case 'E':
            newtio.c_iflag |= (INPCK | ISTRIP);
            newtio.c_cflag |= PARENB;
            newtio.c_cflag &= ~PARODD;
            break;
        case 'n':
        case 'N':
            newtio.c_cflag &= ~PARENB;
            break;
        default:
            break;
    }
    switch (nSpeed)
    {
        case 2400:
            cfsetispeed(&newtio, B2400);
            cfsetospeed(&newtio, B2400);
            break;
        case 4800:
            cfsetispeed(&newtio, B4800);
            cfsetospeed(&newtio, B4800);
            break;
        case 9600:
            cfsetispeed(&newtio, B9600);
            cfsetospeed(&newtio, B9600);
            break;
        case 115200:
            cfsetispeed(&newtio, B115200);
            cfsetospeed(&newtio, B115200);
            break;
        case 460800:
            cfsetispeed(&newtio, B460800);
            cfsetospeed(&newtio, B460800);
            break;
        default:
            cfsetispeed(&newtio, B115200);
            cfsetospeed(&newtio, B115200);
            break;
    }
    if (nStop == 1)
        newtio.c_cflag &= ~CSTOPB;
    else if (nStop == 2)
        newtio.c_cflag |= CSTOPB;
    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 0;
    tcflush(fd, TCIFLUSH);
    if ((tcsetattr(fd, TCSANOW, &newtio)) != 0)
    {
        perror("com set error");
        return -1;
    }
    return 0;
}

int uart_close(int fd)
{
    assert(fd);
    close(fd);
    return 0;
}

int uart_init(int baud, char *uart_path)
{
    int fd = uart_open(fd, uart_path);
    if (fd == -1)
    {
        fprintf(stderr, "uart_open error\n");
        exit(EXIT_FAILURE);
    }
    if (uart_set(fd, baud, 8, 'N', 1) == -1)
    {
        fprintf(stderr, "uart set failed!\n");
        exit(EXIT_FAILURE);
    }
    return fd;
}