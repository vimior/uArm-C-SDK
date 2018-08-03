/******************************************************************
 > Software License Agreement (BSD License)
 > Copyright (c) 2018, UFACTORY, Inc.
 > All rights reserved.
 > Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>
 ******************************************************************/
#include "uarm.h"

extern "C" {
#include "uarm.c"
}

SwiftAPI::SwiftAPI(char *port)
{
    this->serial.fd = -1;
    this->serial.port = port;
    this->serial.nBits = 8;
    this->serial.nSpeed = 115200;
    this->serial.nStop = 1;
    this->serial.nEvent = 'N';
    this->serial.connected = false;
    this->serial.cnt = 1;
    this->connect(port);
}
SwiftAPI::~SwiftAPI(){
    this->disconnect();
}

int SwiftAPI::connect(char *port)
{
    return ::connect_uarm(&this->serial, port);
}

void SwiftAPI::disconnect()
{
    ::disconnect_uarm(&this->serial);
}

unsigned char* SwiftAPI::send_cmd_sync(const void *cmd, float timeout)
{
    return ::send_cmd_sync(&this->serial, cmd, timeout);
}

unsigned char* SwiftAPI::send_cmd_async(const void *cmd, void (*callback)(unsigned char *ret))
{
    return ::send_cmd_async(&this->serial, cmd, callback);
}

bool SwiftAPI::register_power_callback(void (*callback)(bool power))
{
    return ::register_power_callback(&this->serial, callback);
}
void SwiftAPI::release_power_callback()
{
    ::release_power_callback(&this->serial);
}
bool SwiftAPI::register_report_position_callback(void (*callback)(float pos[]))
{
    return ::register_report_position_callback(&this->serial, callback);
}
void SwiftAPI::release_report_position_callback()
{
    ::release_report_position_callback(&this->serial);
}
bool SwiftAPI::register_key0_callback(void (*callback)(int value))
{
    return ::register_key0_callback(&this->serial, callback);
}
void SwiftAPI::release_key0_callback()
{
    ::release_key0_callback(&this->serial);
}
bool SwiftAPI::register_key1_callback(void (*callback)(int value))
{
    return ::register_key1_callback(&this->serial, callback);
}
void SwiftAPI::release_key1_callback()
{
    ::release_key1_callback(&this->serial);
}
bool SwiftAPI::register_limit_switch_callback(void (*callback)(bool status))
{
    return ::register_limit_switch_callback(&this->serial, callback);
}
void SwiftAPI::release_limit_switch_callback()
{
    ::release_limit_switch_callback(&this->serial);
}

unsigned char* SwiftAPI::set_report_position(int interval)
{
    return ::set_report_position(&this->serial, interval);
}

unsigned char*  SwiftAPI::set_report_keys(bool on)
{
    return ::set_report_keys(&this->serial, on);
}

unsigned char* SwiftAPI::set_mode(int mode, bool wait, float timeout, void (*callback)(unsigned char *ret))
{
    return ::set_mode(&this->serial, 3, mode, (int)wait, (double)timeout, callback);
}

unsigned char* SwiftAPI::set_position(float x, float y, float z, long speed, bool relative, bool wait, float timeout, void (*callback)(unsigned char *ret))
{
    return ::set_position(&this->serial, 5, x, y, z, (long)speed, (int)relative, (int)wait, (double)timeout, callback);
}

unsigned char* SwiftAPI::set_polar(float stretch, float rotation, float height, long speed, bool relative, bool wait, float timeout, void (*callback)(unsigned char *ret))
{
    return ::set_polar(&this->serial, 5, stretch, rotation, height, (long)speed, (int)relative, (int)wait, (double)timeout, callback);
}

unsigned char* SwiftAPI::set_servo_angle(int servo_id, float angle, long speed, bool wait, float timeout, void (*callback)(unsigned char *ret))
{
    return ::set_servo_angle(&this->serial, 4, servo_id, angle, (long)speed, (int)wait, (double)timeout, callback);
}

unsigned char* SwiftAPI::set_servo_attach(int servo_id, bool wait, float timeout, void (*callback)(unsigned char *ret))
{
    return ::set_servo_attach(&this->serial, 3, servo_id, (int)wait, (double)timeout, callback);
}

unsigned char* SwiftAPI::set_servo_detach(int servo_id, bool wait, float timeout, void (*callback)(unsigned char *ret))
{
    return ::set_servo_detach(&this->serial, 3, servo_id, (int)wait, (double)timeout, callback);
}
