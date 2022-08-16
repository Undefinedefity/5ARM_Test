#ifndef _UART_HPP_
#define _UART_HPP_
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <assert.h>
#include <termios.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <sys/types.h>
#include <errno.h>
#include <thread>
#include <mutex>
#include <iostream>

#include "armMsg.hpp"

#define BAUD 115200
#define POS_CTRL
//#define TAU_CTRL
#define SEND_CTRL_CMD
//#define READ_ANGLE

static std::mutex uart_mut;
static std::mutex arm_can_state_mut, arm_can_msg_mut;
static std::mutex leg_can1_state_mut, leg_can2_state_mut;
static std::mutex leg_can1_msg_mut, leg_can2_msg_mut;


void initMsg(ArmJointMsg *armCan);

void unpackReply(uint8_t *msg, ArmJointState *arm_can);
//void uart_recv(int uartHandle, leg_state *leg_can1, leg_state *leg_can2);
void uart_recv(int uartHandle, ArmJointState *arm_can);
//void uart_send(int uartHandle, leg_msg *msg_can1, leg_msg *msg_can2);
void uart_send(int uartHandle, ArmJointMsg *msg_can);


void packCmd(JointMsg *msg, JointControl joint);
void cmdReadPos1(JointMsg *msg);
void cmdReadPos2(JointMsg *msg);
void packAll(ArmJointMsg *msg_can, ArmJointControl ctrl_can);
void shut_motor(JointMsg *msg);

int uart_init(int baud, char *uart_path);
int uart_open(int fd, const char *pathname);
int uart_set(int fd, int nSpeed, int nBits, char nEvent, int nStop);
int uart_close(int fd);


#endif