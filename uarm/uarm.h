/******************************************************************
 > Software License Agreement (BSD License)
 > Copyright (c) 2018, UFACTORY, Inc.
 > All rights reserved.
 > Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>
 ******************************************************************/
#ifndef _SERIAL_H
#define _SERIAL_H

#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>
#include <stdbool.h>
#include <errno.h>
#include <stdbool.h>
#include <pthread.h>
#include <time.h>
#include <unistd.h>
#include <stdarg.h>
#include <sys/timeb.h>

typedef struct {
    char *port;
    unsigned int nBits;
    int nSpeed;
    unsigned int nStop;
    unsigned char nEvent;
    int fd;
    bool connected;
    int cnt;
    pthread_t r_thread;
    pthread_mutex_t mutex;
    void (*power_callback)(bool);
    void (*pos_report_callback)(float *);
    void (*key0_report_callback)(int);
    void (*key1_report_callback)(int);
    void (*limit_switch_callback)(bool);
    unsigned char *result[10000];
    void (*callbacks[10000])(unsigned char *);
} serial_t;

typedef struct Swift {
    int (*connect)(serial_t *, char *);
    void (*disconnect)(serial_t *);
    unsigned char* (*send_cmd_sync)(serial_t *, const void *, float);
    unsigned char* (*send_cmd_async)(serial_t *, const void *, void (*)(unsigned char *));

    bool (*register_power_callback)(serial_t *, void (*)(bool));
    void (*release_power_callback)(serial_t *);

    bool (*register_report_position_callback)(serial_t *, void (*)(float *));
    void (*release_report_position_callback)(serial_t *);

    bool (*register_key0_callback)(serial_t *, void (*)(int));
    void (*release_key0_callback)(serial_t *);

    bool (*register_key1_callback)(serial_t *, void (*)(int));
    void (*release_key1_callback)(serial_t *);

    bool (*register_limit_switch_callback)(serial_t *, void (*)(bool));
    void (*release_limit_switch_callback)(serial_t *);

    unsigned char* (*set_report_position)(serial_t *, int);
    unsigned char* (*set_report_keys)(serial_t *, bool);

    unsigned char* (*set_mode)(serial_t *serial, int args, int mode, ...);
    unsigned char* (*set_position)(serial_t *serial, int args, float x, float y, float z, ...);
    unsigned char* (*set_polar)(serial_t *serial, int args, float stretch, float rotation, float height, ...);
    unsigned char* (*set_servo_angle)(serial_t *serial, int args, int servo_id, float angle, ...);
    unsigned char* (*set_servo_attach)(serial_t *serial, int args, int servo_id, ...);
    unsigned char* (*set_servo_detach)(serial_t *serial, int args, int servo_id, ...);
}Swift_t;


#ifdef __cplusplus
extern "C"
{
#endif

int uarm_init(Swift_t *swift, serial_t *serial, char *port);

int connect_uarm(serial_t *serial, char *port);
void disconnect_uarm(serial_t *serial);

unsigned char* send_cmd_sync(serial_t *serial, const void *cmd, float timeout);
unsigned char* send_cmd_async(serial_t *serial, const void *cmd, void (*callback)(unsigned char *));

bool register_power_callback(serial_t *serial, void (*callback)(bool));
void release_power_callback(serial_t *serial);
bool register_report_position_callback(serial_t *serial, void (*callback)(float *));
void release_report_position_callback(serial_t *serial);
bool register_key0_callback(serial_t *serial, void (*callback)(int));
void release_key0_callback(serial_t *serial);
bool register_key1_callback(serial_t *serial, void (*callback)(int));
void release_key1_callback(serial_t *serial);
bool register_limit_switch_callback(serial_t *serial, void (*callback)(bool));
void release_limit_switch_callback(serial_t *serial);

unsigned char* set_report_position(serial_t *serial, int interval);
unsigned char* set_report_keys(serial_t *serial, bool on);

unsigned char* set_mode(serial_t *serial, int args, int mode, ...);
unsigned char* set_position(serial_t *serial, int args, float x, float y, float z, ...);
unsigned char* set_polar(serial_t *serial, int args, float stretch, float rotation, float height, ...);
unsigned char* set_servo_angle(serial_t *serial, int args, int servo_id, float angle, ...);
unsigned char* set_servo_attach(serial_t *serial, int args, int servo_id, ...);
unsigned char* set_servo_detach(serial_t *serial, int args, int servo_id, ...);

#ifdef __cplusplus
}
#endif


#ifdef __cplusplus
class SwiftAPI
{
public:
    SwiftAPI(char *port);
    ~SwiftAPI();
public:
    serial_t serial;
public:
    int connect(char *port=NULL);
    void disconnect();
    unsigned char* send_cmd_sync(const void *cmd, float timeout=2);
    unsigned char* send_cmd_async(const void *cmd, void (*callback)(unsigned char *ret)=NULL);

    bool register_power_callback(void (*callback)(bool power)=NULL);
    void release_power_callback();
    bool register_report_position_callback(void (*callback)(float *pos)=NULL);
    void release_report_position_callback();
    bool register_key0_callback(void (*callback)(int value)=NULL);
    void release_key0_callback();
    bool register_key1_callback(void (*callback)(int value)=NULL);
    void release_key1_callback();
    bool register_limit_switch_callback(void (*callback)(bool status)=NULL);
    void release_limit_switch_callback();

    unsigned char* set_report_position(int interval=0);
    unsigned char* set_report_keys(bool on=true);

    unsigned char* set_mode(int mode, bool wait=true, float timeout=0, void (*callback)(unsigned char *ret)=NULL);
    unsigned char* set_position(float x, float y, float z, long speed=0, bool relative=false, bool wait=false, float timeout=0, void (*callback)(unsigned char *ret)=NULL);
    unsigned char* set_polar(float stretch, float rotation, float height, long speed=0, bool relative=false, bool wait=false, float timeout=0, void (*callback)(unsigned char *ret)=NULL);
    unsigned char* set_servo_angle(int servo_id, float angle, long speed=0, bool wait=false, float timeout=0, void (*callback)(unsigned char *ret)=NULL);
    unsigned char* set_servo_attach(int servo_id, bool wait=true, float timeout=0, void (*callback)(unsigned char *ret)=NULL);
    unsigned char* set_servo_detach(int servo_id, bool wait=true, float timeout=0, void (*callback)(unsigned char *ret)=NULL);
};
#endif

#endif // _SERIAL_H
