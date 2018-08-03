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

void pos_report_callback(float pos[])
{
    printf("pos: X=%f, Y=%f, Z=%f, R=%f\n", pos[0], pos[1], pos[2], pos[3]);
}

int main()
{
    serial_t ser;
    serial_t *serial = &ser;
    Swift_t swift;
    if (uarm_init(&swift, serial, "/dev/ttyACM0") != 0) {
        printf("connect failed\n");
        exit(-1);
    }

    sleep(3);
    unsigned char *ret;

    swift.register_report_position_callback(serial, pos_report_callback);
    ret = swift.set_report_position(serial, 1);
    printf("set_report_position: %s\n", ret);
    ret = swift.set_position(serial, 0, 180, 0, 90);
    printf("set_position-0: %s\n", ret);
    ret = swift.set_position(serial, 5, 180, 80, 90, (long)100000, false, false, (double)0.0, test_callback);
    printf("set_position-1: %s\n", ret);
    ret = swift.set_position(serial, 3, 180, -80, 90, 100000, false, true);
    printf("set_position-2: %s\n", ret);

    sleep(3);
    ret = swift.set_servo_detach(serial, 0, 0);
    printf("set_servo_detach-0: %s\n", ret);
    sleep(2);
    ret = swift.set_servo_detach(serial, 0, 1);
    printf("set_servo_detach-1: %s\n", ret);
    sleep(2);
    ret = swift.set_servo_detach(serial, 0, 2);
    printf("set_servo_detach-2: %s\n", ret);
    sleep(2);

    ret = swift.set_servo_attach(serial, 0, 0);
    printf("set_servo_attach-0: %s\n", ret);
    sleep(2);
    ret = swift.set_servo_attach(serial, 0, 1);
    printf("set_servo_attach-1: %s\n", ret);
    sleep(2);
    ret = swift.set_servo_attach(serial, 0, 2);
    printf("set_servo_attach-2: %s\n", ret);
    sleep(2);

    ret = swift.set_servo_detach(serial, 0, -1);
    printf("set_servo_detach-null: %s\n", ret);
    sleep(2);
    ret = swift.set_servo_attach(serial, 0, -1);
    printf("set_servo_attach-null: %s\n", ret);

    return 0;
}




