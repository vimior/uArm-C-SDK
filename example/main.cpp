/******************************************************************
 > Software License Agreement (BSD License)
 > Copyright (c) 2018, UFACTORY, Inc.
 > All rights reserved.
 > Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>
 ******************************************************************/
#include "../uarm/uarm.h"

void test_callback(unsigned char *ret)
{
    printf("callback: %s\n", ret);
}

void power_callback(bool power)
{
    printf("power: %d\n", power);
}
void pos_report_callback(float pos[])
{
    printf("pos: X=%f, Y=%f, Z=%f, R=%f\n", pos[0], pos[1], pos[2], pos[3]);
}
void key0_report_callback(int value)
{
    printf("key0: %d\n", value);
}
void key1_report_callback(int value)
{
    printf("key1: %d\n", value);
}
void limit_switch_callback(bool status)
{
    printf("limit switch: %d\n", status);
}

int main()
{
    SwiftAPI swift((char *)"/dev/ttyACM0");
    if (!swift.serial.connected)
    {
        printf("connect failed\n");
        exit(-1);
    }

    sleep(3);
    swift.register_power_callback(power_callback);
    swift.register_report_position_callback(pos_report_callback);
    swift.register_key0_callback(key0_report_callback);
    swift.register_key1_callback(key1_report_callback);
    swift.register_limit_switch_callback(limit_switch_callback);

    unsigned char *ret;
    ret = swift.set_report_position(1);
    printf("set_report_position: %s\n", ret);
    ret = swift.set_report_keys(true);
    printf("set_report_keys: %s\n", ret);
    ret = swift.send_cmd_sync("G0 X200 Y0 Z90 F10000");
    printf("ret: %s\n", ret);
    ret = swift.send_cmd_sync("P2220");
    printf("ret: %s\n", ret);
    ret = swift.send_cmd_sync("P2221");
    printf("ret: %s\n", ret);

    while(true){
        sleep(2);
        swift.send_cmd_async("P2220", test_callback);
        // printf("ret: %s\n", ret);
        swift.send_cmd_async("P2221", test_callback);
        // printf("ret: %s\n", ret);
    }
    return 0;
}




