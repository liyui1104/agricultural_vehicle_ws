#ifndef _SERIAL_PORT_COMMUNICATION_H
#define _SERIAL_PORT_COMMUNICATION_H

#include <iostream>
#include "std_msgs/String.h"
#include <sstream>
#include "ros/ros.h"
#include "serial/serial.h"
#include "geometry_msgs/Twist.h"
using namespace std;

/********************************************************
Create some serial port sends and receives related variables and objects
********************************************************/

// Serial port related objects
serial::Serial sp;

int LEFT_ROTATE_SPEED_TARGET = 0; // The target for left wheel rotate speed
int RIGHT_ROTATE_SPEED_TARGET = 0; // The target for right wheel rotate speed

int LEFT_ROTATE_SPEED_NOW = 0; // The current left wheel rotate speed
int RIGHT_ROTATE_SPEED_NOW = 0; // The current right wheel rotate speed

/********************************************************
Function: Initialize the serial port using the ros package serial
input: empty
output: empty
********************************************************/
void serialInit();

/********************************************************
Function: transfer float(rotate_speed) to hex(uint8_t)
input: decimal data
output: Hexadecimal uint8_t indicating the rotate speed

rotate speed -> rotate speed / rated rotate speed = persentage -> persentage *  max rotate speed = decimal data -> Hexadecimal uint8_t indicating the rotate speed
********************************************************/
uint8_t float_to_hex(float dec_result);

/********************************************************
Function: transfer hex(uint8_t rotate speed) to int(rotate speed)
input: uint8_t rotate speed
output: int rotate speed
********************************************************/
int hex_to_int(uint8_t hex_rotate_speed1,uint8_t hex_rotate_speed2);

/********************************************************
Function: Calculate the positive rotate speed after converison
input: forwared rotate speed
output: forwared rotate speed after converison
********************************************************/
float transformed_positive_rotate_speed(float rotate_speed);

/********************************************************
Function: Calculate the negative rotate speed after converison
input: backward rotate speed
output: backward rotate speed after converison
********************************************************/
float transformed_negative_rotate_speed(float rotate_speed);

/********************************************************
Function: Send the left and right wheel rotate speeds to the lower machine
input: left wheel rotate speed, right wheel rotate speed
output: empty
********************************************************/
void sendRotateSpeed(float left_rotate_speed, float right_rotate_speed);

/********************************************************
Function: Obtain the current speed through the heartbeat data continuously feedbacking by the lower machine
input: empty
output: If gets rotate speed successfully
********************************************************/
bool receiveRotateSpeed();

#endif