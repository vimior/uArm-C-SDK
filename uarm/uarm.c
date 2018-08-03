/******************************************************************
 > Software License Agreement (BSD License)
 > Copyright (c) 2018, UFACTORY, Inc.
 > All rights reserved.
 > Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>
 ******************************************************************/
#include "uarm.h"

#define TIMEOUT (unsigned char *)(void *)"TIMEOUT"
#define NotConnect (unsigned char *)(void *)"NotConnect"
#define OK (unsigned char *)(void *)"ok"

#define DEFAULT_TIMEOUT_2 2
#define DEFAULT_TIMEOUT_10 10
#define DEFAULT_SPEED 6000

long long get_system_time()
{
    struct timeb t;
    ftime(&t);
    return 1000 * t.time + t.millitm;
}

inline unsigned char* send_cmd_sync(serial_t *serial, const void *cmd, float timeout)
{
    if (serial->fd != -1 && serial->connected) {
        pthread_mutex_lock(&serial->mutex);
        if (serial->cnt >= 10000)
            serial->cnt = 1;
        int cnt = serial->cnt;
        int ret = dprintf(serial->fd, "#%d%s\n", serial->cnt, (unsigned char *)cmd);
        // printf("send: %s, %d, %f\n", (unsigned char *)cmd, ret, timeout);
        serial->cnt += 1;
        pthread_mutex_unlock(&serial->mutex);

        long long start_time = get_system_time();
        while(true)
        {
            if (serial->result[cnt] != NULL && serial->result[cnt] != 0)
            {
                unsigned char* result = serial->result[cnt];
                serial->result[cnt] = NULL;
                return result;
            }
            else
            {
                long long now = get_system_time();
                if (now - start_time > timeout * 1000) {
                    return TIMEOUT;
                }
            }
            sleep(0.01);
        }
    }
    else {
        return NotConnect;
    }
}

inline unsigned char* send_cmd_async(serial_t *serial, const void *cmd, void (*callback)(unsigned char *ret))
{
    if (serial->fd != -1 && serial->connected) {
        pthread_mutex_lock(&serial->mutex);
        if (serial->cnt >= 10000)
            serial->cnt = 1;
        serial->callbacks[serial->cnt] = callback;
        int ret = dprintf(serial->fd, "#%d%s\n", serial->cnt, (unsigned char *)cmd);
        serial->cnt += 1;
        pthread_mutex_unlock(&serial->mutex);
        return OK;
    }
    return NotConnect;
}

static void *receive_thread(void *args) {
    serial_t *serial = (serial_t *)args;
    unsigned char buf;
    int len = 0;
    bool flag = false;
    unsigned char line[1024];
    while(serial->connected)
    {
        len += read(serial->fd, &buf, 1);
        if (buf != '\n')
        {
            line[len-1] = buf;
            flag = true;
        }
        else if (flag)
        {
            // printf("recv: %s\n", line);
            if (line[0] == '$') // response
            {
                char *delims = (char *)" ";
                int length = strlen((char *)line);
                char *result = strtok((char *)line, delims);
                if (result != NULL)
                {
                    int cnt = atoi(result + 1);
                    unsigned char data[length - len];
                    int l = strlen(result) + 1;
                    memcpy(data, line + l, length - l);
                    if (serial->callbacks[cnt] != NULL && serial->callbacks[cnt] != 0) {
                        serial->callbacks[cnt](data);
                        serial->callbacks[cnt] = NULL;
                    }
                    else {
                        serial->result[cnt] = data;
                    }
                }
            }
            else if (line[0] == '@') // report
            {
                char *delims = (char *)" ";
                int length = strlen((char *)line);
                char *result = strtok((char *)line, delims);
                if (result != NULL)
                {
                    if (strcmp(result, "@3") == 0) {
                        // report pos
                        if (serial->pos_report_callback != NULL) {
                            float pos[4];
                            result = strtok(NULL, delims);
                            int i = 0;
                            while (result != NULL && i < sizeof(pos) / sizeof(pos[0])) {
                                pos[i] = atof(result + 1);
                                result = strtok(NULL, delims);
                                i += 1;
                            }
                            serial->pos_report_callback(pos);
                        }
                    }
                    else if (strcmp(result, "@4") == 0) {
                        // report keys
                        if (serial->key0_report_callback != NULL || serial->key1_report_callback ) {
                            result = strtok(NULL, delims);
                            if (result != NULL) {
                                if (strcmp(result, "B0") == 0 && serial->key0_report_callback != NULL) {
                                    result = strtok(NULL, delims);
                                    if (result != NULL) {
                                        serial->key0_report_callback(atoi(result + 1));
                                    }
                                }
                                else if (strcmp(result, "B1") == 0 && serial->key1_report_callback != NULL) {
                                    result = strtok(NULL, delims);
                                    if (result != NULL) {
                                        serial->key1_report_callback(atoi(result + 1));
                                    }
                                }
                            }
                        }
                    }
                    else if (strcmp(result, "@5") == 0) {
                        // report power
                        if (serial->power_callback != NULL) {
                            result = strtok(NULL, delims);
                            if (result != NULL && strlen(result) >= 2) {
                                if (result[1] == '0') {
                                    serial->power_callback(false);
                                }
                                else if (result[1] == '1') {
                                    serial->power_callback(true);
                                }
                            }
                        }
                    }
                    else if (strcmp(result, "@6") == 0) {
                        // report limit switch
                        if (serial->limit_switch_callback != NULL) {
                            result = strtok(NULL, delims);
                            if (result != NULL) {
                                result = strtok(NULL, delims);
                                if (result != NULL && strlen(result) >= 2) {
                                    if (result[1] == '0') {
                                        serial->limit_switch_callback(false);
                                    }
                                    else if (result[1] == '1') {
                                        serial->limit_switch_callback(true);
                                    }
                                }
                            }
                        }
                    }
                }
            }
            len = 0;
            bzero(line, sizeof(line));
            flag = false;
        }
    }
}

static int open_port(serial_t *serial)
{
    serial->fd = open(serial->port, O_RDWR | O_NOCTTY | O_SYNC);
    if (-1 == serial->fd) {
        perror("[serial] can't open port");
        return -1;
    }
    if (fcntl(serial->fd, F_SETFL, 0) < 0)
        perror("[serial] fcntl");
    if (isatty(STDIN_FILENO) == 0)
        printf("[serial] standrad input is not a terminal device\n");
    return 0;
}

static int set_opt(serial_t *serial)
{
    struct termios tty;
    if (tcgetattr(serial->fd, &tty) < 0) {
        perror("[serial] tcgetattr");
        return -1;
    }
    bzero(&tty, sizeof(tty));
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;

    switch(serial->nBits)
    {
        case 7:
            tty.c_cflag |= CS7;
            break;
        case 8:
            tty.c_cflag |= CS8;
            break;
    }

    switch(serial->nEvent)
    {
        case 'O':
            tty.c_cflag |= PARENB;
            tty.c_cflag |= PARODD;
            tty.c_iflag |= (INPCK | ISTRIP);
            break;
        case 'E':
            tty.c_iflag |= (INPCK | ISTRIP);
            tty.c_cflag |= PARENB;
            tty.c_cflag &= ~PARODD;
            break;
        case 'N':
            tty.c_cflag &= ~PARENB;
            break;
    }

    switch(serial->nSpeed)
    {
        case 2400:
            cfsetispeed(&tty, B2400);
            cfsetospeed(&tty, B2400);
            break;
        case 4800:
            cfsetispeed(&tty, B4800);
            cfsetospeed(&tty, B4800);
            break;
        case 9600:
            cfsetispeed(&tty, B9600);
            cfsetospeed(&tty, B9600);
            break;
        case 115200:
            cfsetispeed(&tty, B115200);
            cfsetospeed(&tty, B115200);
            break;
        case 921600:
            cfsetispeed(&tty, B921600);
            cfsetospeed(&tty, B921600);
            break;
        default:
            cfsetispeed(&tty, B115200);
            cfsetospeed(&tty, B115200);
            break;
    }

    if(serial->nStop == 1)
        tty.c_cflag &= ~CSTOPB;
    else if(serial->nStop == 2)
        tty.c_cflag |= CSTOPB;

    /* setup for non-canonical mode */
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;

    tty.c_cc[VTIME] = 1;
    tty.c_cc[VMIN] = 0;

    // tcflush(serial->fd, TCIFLUSH);
    if (tcsetattr(serial->fd, TCSANOW, &tty) != 0) {
        perror("[serial] tcgetattr");
        return -1;
    }
    if (!serial->connected)
    {
        serial->connected = true;
        pthread_create(&serial->r_thread, NULL, receive_thread, serial);
    }
    return 0;
}

int connect_uarm(serial_t *serial, char *port)
{
    if (serial->connected) {
        if (serial->port != port && port != NULL) {
            close(serial->fd);
            serial->connected = false;
            serial->port = port;
            serial->cnt = 1;
            if (open_port(serial) == 0)
            {
                if (set_opt(serial) == 0)
                {
                    printf("[SwiftAPI] connect %s success\n", serial->port);
                    serial->connected = true;
                    return 0;
                }
            }
        }
        else if (serial->port == port && port != NULL)
        {
            return 0;
        }
    }
    else
    {
        serial->port = port;
        serial->nBits = 8;
        serial->nSpeed = 115200;
        serial->nStop = 1;
        serial->nEvent = 'N';
        serial->cnt = 1;
        if (port != NULL)
        {
            serial->port = port;
        }
        if (serial->port != NULL)
        {
            if (open_port(serial) == 0)
            {
                if (set_opt(serial) == 0)
                {
                    printf("[SwiftAPI] connect %s success\n", serial->port);
                    serial->connected = true;
                    pthread_mutex_init(&serial->mutex, NULL);
                    return 0;
                }
            }
        }
        else
        {
            printf("open falied, port is NULL\n");
        }
    }
    return -1;
}

void disconnect_uarm(serial_t *serial)
{
    if (serial != NULL && serial->connected) {
        close(serial->fd);
        serial->fd = -1;
        serial->connected = false;
        pthread_mutex_destroy(&serial->mutex);
    }
}

bool register_power_callback(serial_t *serial, void (*callback)(bool power))
{
    if (callback != NULL)
    {
        serial->power_callback = callback;
        return true;
    }
    return false;
}
void release_power_callback(serial_t *serial)
{
    serial->power_callback = NULL;
}

bool register_report_position_callback(serial_t *serial, void (*callback)(float *pos))
{
    if (callback != NULL)
    {
        serial->pos_report_callback = callback;
        return true;
    }
    return false;
}
void release_report_position_callback(serial_t *serial)
{
    serial->pos_report_callback = NULL;
}

bool register_key0_callback(serial_t *serial, void (*callback)(int value))
{
    if (callback != NULL)
    {
        serial->key0_report_callback = callback;
        return true;
    }
    return false;
}
void release_key0_callback(serial_t *serial)
{
    serial->key0_report_callback = NULL;
}

bool register_key1_callback(serial_t *serial, void (*callback)(int value))
{
    if (callback != NULL)
    {
        serial->key1_report_callback = callback;
        return true;
    }
    return false;
}
void release_key1_callback(serial_t *serial)
{
    serial->key1_report_callback = NULL;
}

bool register_limit_switch_callback(serial_t *serial, void (*callback)(bool status))
{
    if (callback != NULL)
    {
        serial->limit_switch_callback = callback;
        return true;
    }
    return false;
}
void release_limit_switch_callback(serial_t *serial)
{
    serial->limit_switch_callback = NULL;
}

unsigned char* set_report_position(serial_t *serial, int interval)
{
    char cmd[11];
    sprintf(cmd, "M2120 V%d", interval);
    return send_cmd_sync(serial, cmd, DEFAULT_TIMEOUT_2);
}
unsigned char* set_report_keys(serial_t *serial, bool on)
{
    if (on) {
        return send_cmd_sync(serial, "M2213 V0", DEFAULT_TIMEOUT_2);
    }
    else {
        return send_cmd_sync(serial, "M2213 V1", DEFAULT_TIMEOUT_2);
    }
}

// unsigned char* reset(serial_t *serial, int args, ...)
// {
//     float speed = DEFAULT_SPEED;
//     bool wait = true;
//     float timeout = DEFAULT_TIMEOUT_10;
//     va_list v;
//     va_start(v, args);

//     if (args > 0)
//     {
//         speed = (float)va_arg(v, double);
//         if (args > 1)
//         {
//             wait = (bool)va_arg(v, int);
//             if (args > 2)
//             {
//                 timeout = (float)va_arg(v, double);
//             }
//         }
//     }
//     va_end(v);
//     if (wait)
//     {

//     }
//     else
//     {
//         return NULL;
//     }
// }


unsigned char* set_mode(serial_t *serial, int args, int mode, ...)
{
    char cmd[11];
    sprintf(cmd, "M2400 S%d", mode);
    bool wait = true;
    double timeout = DEFAULT_TIMEOUT_2;
    void (*callback)(unsigned char *) = NULL;
    va_list v;
    va_start(v, mode);

    if (args > 0)
    {
        wait = (bool)va_arg(v, int);
        if (args > 1)
        {
            timeout = (double)va_arg(v, double);
            if (timeout <= 0)
            {
                timeout = DEFAULT_TIMEOUT_2;
            }
            if (args > 2)
            {
                callback = va_arg(v, void (*)(unsigned char *));
            }
        }
    }
    va_end(v);
    if (wait)
    {
        return send_cmd_sync(serial, cmd, (float)timeout);
    }
    else
    {
        return send_cmd_async(serial, cmd, callback);
    }
}

unsigned char* set_position(serial_t *serial, int args, float x, float y, float z, ...)
{
    long speed = DEFAULT_SPEED;
    bool relative = false;
    bool wait = false;
    double timeout = DEFAULT_TIMEOUT_10;
    void (*callback)(unsigned char *) = NULL;
    va_list v;
    va_start(v, z);

    if (args > 0)
    {
        speed = va_arg(v, long);
        if (speed <= 0)
        {
            speed = DEFAULT_SPEED;
        }
        if (args > 1)
        {
            relative = (bool)va_arg(v, int);
            if (args > 2)
            {
                wait = (bool)va_arg(v, int);
                if (args > 3)
                {
                    timeout = (double)va_arg(v, double);
                    if (timeout <= 0)
                    {
                        timeout = DEFAULT_TIMEOUT_10;
                    }
                    if (args > 4)
                    {
                        callback = va_arg(v, void (*)(unsigned char *));
                    }
                }
            }
        }
    }
    va_end(v);
    if (relative)
    {
        char cmd[64];
        sprintf(cmd, "G2204 X%f Y%f Z%f F%ld", x, y, z, speed);
        if (wait) {
            return send_cmd_sync(serial, cmd, (float)timeout);
        }
        else
        {
            return send_cmd_async(serial, cmd, callback);
        }
    }
    else
    {
        char cmd[64];
        sprintf(cmd, "G0 X%f Y%f Z%f F%ld", x, y, z, speed);
        if (wait) {
            return send_cmd_sync(serial, cmd, (float)timeout);
        }
        else
        {
            return send_cmd_async(serial, cmd, callback);
        }
    }
}

unsigned char* set_polar(serial_t *serial, int args, float stretch, float rotation, float height, ...)
{
    long speed = DEFAULT_SPEED;
    bool relative = false;
    bool wait = false;
    double timeout = DEFAULT_TIMEOUT_10;
    void (*callback)(unsigned char *) = NULL;
    va_list v;
    va_start(v, height);

    if (args > 0)
    {
        speed = va_arg(v, long);
        if (speed <= 0)
        {
            speed = DEFAULT_SPEED;
        }
        if (args > 1)
        {
            relative = (bool)va_arg(v, int);
            if (args > 2)
            {
                wait = (bool)va_arg(v, int);
                if (args > 3)
                {
                    timeout = (double)va_arg(v, double);
                    if (timeout <= 0)
                    {
                        timeout = DEFAULT_TIMEOUT_10;
                    }
                    if (args > 4)
                    {
                        callback = va_arg(v, void (*)(unsigned char *));
                    }
                }
            }
        }
    }
    va_end(v);
    if (relative)
    {
        char cmd[64];
        sprintf(cmd, "G2205 S%f R%f H%f F%ld", stretch, rotation, height, speed);
        if (wait) {
            return send_cmd_sync(serial, cmd, (float)timeout);
        }
        else
        {
            return send_cmd_async(serial, cmd, callback);
        }
    }
    else
    {
        char cmd[64];
        sprintf(cmd, "G2201 S%f R%f H%f F%ld", stretch, rotation, height, speed);
        if (wait) {
            return send_cmd_sync(serial, cmd, (float)timeout);
        }
        else
        {
            return send_cmd_async(serial, cmd, callback);
        }
    }
}

unsigned char* set_servo_angle(serial_t *serial, int args, int servo_id, float angle, ...)
{
    long speed = DEFAULT_SPEED;
    bool wait = false;
    double timeout = DEFAULT_TIMEOUT_10;
    void (*callback)(unsigned char *) = NULL;
    va_list v;
    va_start(v, angle);

    if (args > 0)
    {
        speed = va_arg(v, long);
        if (speed <= 0)
        {
            speed = DEFAULT_SPEED;
        }
        if (args > 1)
        {
            wait = (bool)va_arg(v, int);
            if (args > 2)
            {
                timeout = (double)va_arg(v, double);
                if (timeout <= 0)
                {
                    timeout = DEFAULT_TIMEOUT_10;
                }
                if (args > 3)
                {
                    callback = va_arg(v, void (*)(unsigned char *));
                }
            }
        }
    }
    va_end(v);

    char cmd[64];
    sprintf(cmd, "G2202 N%d V%f F%ld", servo_id, angle, speed);
    if (wait) {
        return send_cmd_sync(serial, cmd, (float)timeout);
    }
    else
    {
        return send_cmd_async(serial, cmd, callback);
    }
}

unsigned char* set_servo_attach(serial_t *serial, int args, int servo_id, ...)
{
    bool wait = true;
    double timeout = DEFAULT_TIMEOUT_2;
    void (*callback)(unsigned char *) = NULL;
    va_list v;
    va_start(v, servo_id);

    if (args > 0)
    {
        wait = (bool)va_arg(v, int);
        if (args > 2)
        {
            timeout = (double)va_arg(v, double);
            if (timeout <= 0)
            {
                timeout = DEFAULT_TIMEOUT_2;
            }
            if (args > 3)
            {
                callback = va_arg(v, void (*)(unsigned char *));
            }
        }
    }
    va_end(v);

    char cmd[30];
    if (servo_id < 0)
    {
        sprintf(cmd, "M17");
    }
    else
    {
        sprintf(cmd, "M2201 N%d", servo_id);
    }
    if (wait)
    {
        return send_cmd_sync(serial, cmd, (float)timeout);
    }
    else
    {
        return send_cmd_async(serial, cmd, callback);
    }
}

unsigned char* set_servo_detach(serial_t *serial, int args, int servo_id, ...)
{
    bool wait = true;
    double timeout = DEFAULT_TIMEOUT_2;
    void (*callback)(unsigned char *) = NULL;
    va_list v;
    va_start(v, servo_id);

    if (args > 0)
    {
        wait = (bool)va_arg(v, int);
        if (args > 2)
        {
            timeout = (double)va_arg(v, double);
            if (timeout <= 0)
            {
                timeout = DEFAULT_TIMEOUT_2;
            }
            if (args > 3)
            {
                callback = va_arg(v, void (*)(unsigned char *));
            }
        }
    }
    va_end(v);

    char cmd[30];
    if (servo_id < 0)
    {
        sprintf(cmd, "M2019");
    }
    else
    {
        sprintf(cmd, "M2202 N%d", servo_id);
    }
    if (wait)
    {
        return send_cmd_sync(serial, cmd, (float)timeout);
    }
    else
    {
        return send_cmd_async(serial, cmd, callback);
    }
}

int uarm_init(Swift_t *swift, serial_t *serial, char *port)
{
    swift->connect = connect_uarm;
    swift->disconnect = disconnect_uarm;
    swift->send_cmd_sync = send_cmd_sync;
    swift->send_cmd_async = send_cmd_async;

    swift->register_power_callback = register_power_callback;
    swift->release_power_callback = release_power_callback;
    swift->register_report_position_callback = register_report_position_callback;
    swift->release_report_position_callback = release_report_position_callback;
    swift->register_key0_callback = register_key0_callback;
    swift->release_key0_callback = release_key0_callback;
    swift->register_key1_callback = register_key1_callback;
    swift->release_key1_callback = release_key1_callback;
    swift->register_limit_switch_callback = register_limit_switch_callback;
    swift->release_limit_switch_callback = release_limit_switch_callback;

    swift->set_report_position = set_report_position;
    swift->set_report_keys = set_report_keys;

    swift->set_mode = set_mode;
    swift->set_position = set_position;
    swift->set_polar = set_polar;
    swift->set_servo_angle = set_servo_angle;
    swift->set_servo_attach = set_servo_attach;
    swift->set_servo_detach = set_servo_detach;

    serial->port = port;
    return swift->connect(serial, serial->port);
}
