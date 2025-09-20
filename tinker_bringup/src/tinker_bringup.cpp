#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <math.h>
#include <time.h>
#include "comm.h"
#include "spi_node.h"
#include "spi.h"
#include "sys_time.h"
#include <pthread.h>
#include <signal.h>
#include <errno.h>

// ROS2 includes
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/vector3.hpp"

using namespace std;

// ROS2 message types
using Float32MultiArray = std_msgs::msg::Float32MultiArray;
using BoolMsg = std_msgs::msg::Bool;
using Float32Msg = std_msgs::msg::Float32;
using ImuMsg = sensor_msgs::msg::Imu;
using Vector3Msg = geometry_msgs::msg::Vector3;

// Глобальные переменные
_MEMS mems;
_SPI_RX spi_rx;
_SPI_TX spi_tx;

uint8_t spi_tx_buf[SPI_BUF_SIZE] = {0};
uint8_t spi_rx_buf[SPI_BUF_SIZE] = {0};
int spi_tx_cnt_show = 0;
int spi_tx_cnt = 0;

uint8_t tx[SPI_BUF_SIZE] = {};
uint8_t rx[SPI_BUF_SIZE] = {};

// Глобальные переменные для многопоточности
pthread_mutex_t lock;
volatile int running = 1;

// ROS2 глобальные переменные
rclcpp::Node::SharedPtr ros_node;
rclcpp::Publisher<Float32MultiArray>::SharedPtr motors_state_pub;
rclcpp::Publisher<ImuMsg>::SharedPtr imu_pub;
rclcpp::Publisher<Vector3Msg>::SharedPtr attitude_pub;

#define SPI_SEND_MAX 120+20+20
static uint32_t speed = 20000000;
#define DELAY_SPI 250

float spi_loss_cnt = 0;
int spi_connect = 0;

// Прототипы функций
static void setDataInt_spi(int i);
static void setDataFloat_spi(float f);
static void setDataFloat_spi_int(float f, float size);
static float floatFromData_spi(unsigned char *data, int *anal_cnt);
static float floatFromData_spi_int(unsigned char *data, int *anal_cnt, float size);
static char charFromData_spi(unsigned char *data, int *anal_cnt);
static int intFromData_spi(unsigned char *data, int *anal_cnt);
int slave_rx(uint8_t *data_buf, int num);
void can_board_send(char sel);
void transfer(int fd, int sel);
void* Thread_SPI(void* arg);
void signal_handler(int sig);

// Вспомогательные функции
static void setDataInt_spi(int i) {
    spi_tx_buf[spi_tx_cnt++] = ((i << 24) >> 24);
    spi_tx_buf[spi_tx_cnt++] = ((i << 16) >> 24);
    spi_tx_buf[spi_tx_cnt++] = ((i << 8) >> 24);
    spi_tx_buf[spi_tx_cnt++] = (i >> 24);
}

static void setDataFloat_spi(float f) {
    int i = *(int *)&f;
    spi_tx_buf[spi_tx_cnt++] = ((i << 24) >> 24);
    spi_tx_buf[spi_tx_cnt++] = ((i << 16) >> 24);
    spi_tx_buf[spi_tx_cnt++] = ((i << 8) >> 24);
    spi_tx_buf[spi_tx_cnt++] = (i >> 24);
}

static void setDataFloat_spi_int(float f, float size) {
    int16_t _temp = f * size;
    spi_tx_buf[spi_tx_cnt++] = ((_temp >> 8) & 0xFF);
    spi_tx_buf[spi_tx_cnt++] = (_temp & 0xFF);
}

static float floatFromData_spi(unsigned char *data, int *anal_cnt) {
    int i = 0x00;
    i |= (*(data + *anal_cnt + 3) << 24);
    i |= (*(data + *anal_cnt + 2) << 16);
    i |= (*(data + *anal_cnt + 1) << 8);
    i |= (*(data + *anal_cnt + 0));
    *anal_cnt += 4;
    return *(float *)&i;
}

static float floatFromData_spi_int(unsigned char *data, int *anal_cnt, float size) {
    int16_t temp = (*(data + *anal_cnt + 0) << 8) | *(data + *anal_cnt + 1);
    *anal_cnt += 2;
    return (float)temp / size;
}

static char charFromData_spi(unsigned char *data, int *anal_cnt) {
    char temp = *(data + *anal_cnt);
    *anal_cnt += 1;
    return temp;
}

static int intFromData_spi(unsigned char *data, int *anal_cnt) {
    int i = 0x00;
    i |= (*(data + *anal_cnt + 3) << 24);
    i |= (*(data + *anal_cnt + 2) << 16);
    i |= (*(data + *anal_cnt + 1) << 8);
    i |= (*(data + *anal_cnt + 0));
    *anal_cnt += 4;
    return i;
}

int slave_rx(uint8_t *data_buf, int num) {
    static int cnt_err_sum = 0;
    uint8_t sum = 0;
    int anal_cnt = 4;
    for (int i = 0; i < (num - 1); i++) {
        sum += *(data_buf + i);
    }

    if (!(sum == *(data_buf + num - 1))) {
        cnt_err_sum++;
        printf("SPI ERROR: sum err=%d sum_cal=0x%X sum=0x%X\n",
               cnt_err_sum, sum, *(data_buf + num - 1));
        return 0;
    }

    if (!(*(data_buf) == 0xFF && *(data_buf + 1) == 0xFB)) {
        printf("SPI ERROR: Invalid header! Expected 0xFF 0xFB, got 0x%02X 0x%02X\n", 
               *(data_buf), *(data_buf + 1));
        return 0;
    }

    if (*(data_buf + 2) == 26) {
        spi_loss_cnt = 0;
        if (spi_connect == 0) {
            printf("Hardware::Hardware SPI-STM32 Link3-Sbus Yunzhuo!!!=%d!!!\n", spi_connect);
            spi_connect = 1;
        }

        pthread_mutex_lock(&lock);
        
        // Чтение IMU данных
        spi_rx.att[0] = floatFromData_spi(data_buf, &anal_cnt); // Roll
        spi_rx.att[1] = floatFromData_spi(data_buf, &anal_cnt); // Pitch
        spi_rx.att[2] = floatFromData_spi(data_buf, &anal_cnt); // Yaw

        spi_rx.att_rate[0] = floatFromData_spi(data_buf, &anal_cnt);
        spi_rx.att_rate[1] = floatFromData_spi(data_buf, &anal_cnt);
        spi_rx.att_rate[2] = floatFromData_spi(data_buf, &anal_cnt);

        spi_rx.acc_b[0] = floatFromData_spi(data_buf, &anal_cnt);
        spi_rx.acc_b[1] = floatFromData_spi(data_buf, &anal_cnt);
        spi_rx.acc_b[2] = floatFromData_spi(data_buf, &anal_cnt);

        // Чтение данных моторов
        for (int i = 0; i < 10; i++) {
            if (anal_cnt + 6 > num) {
                printf("SPI ERROR: Buffer overflow in motor data parsing! anal_cnt=%d, num=%d, motor=%d\n", 
                       anal_cnt, num, i);
                break;
            }
            
            spi_rx.q[i] = floatFromData_spi_int(data_buf, &anal_cnt, CAN_POS_DIV);
            spi_rx.dq[i] = floatFromData_spi_int(data_buf, &anal_cnt, CAN_POS_DIV);
            spi_rx.tau[i] = floatFromData_spi_int(data_buf, &anal_cnt, CAN_T_DIV);

            uint8_t temp = charFromData_spi(data_buf, &anal_cnt);
            spi_rx.connect_motor[i] = (temp % 100) / 10;
            spi_rx.ready[i] = temp % 10;
        }
        pthread_mutex_unlock(&lock);

        // Публикация данных в ROS2
        publish_ros2_data();
        
    } else {
        return 0;
    }
    return 1;
}

void publish_ros2_data() {
    // Публикация состояния моторов
    auto motors_msg = Float32MultiArray();
    motors_msg.data.resize(30); // 10 моторов * 3 параметра
    
    pthread_mutex_lock(&lock);
    for (int i = 0; i < 10; i++) {
        motors_msg.data[i] = spi_rx.q[i];          // current_pos
        motors_msg.data[i + 10] = spi_rx.dq[i];    // current_vel
        motors_msg.data[i + 20] = spi_rx.tau[i];   // current_trg
    }
    pthread_mutex_unlock(&lock);
    
    motors_state_pub->publish(motors_msg);

    // Публикация IMU данных
    auto imu_msg = ImuMsg();
    auto attitude_msg = Vector3Msg();
    
    pthread_mutex_lock(&lock);
    // Заполнение IMU сообщения
    imu_msg.angular_velocity.x = spi_rx.att_rate[0];
    imu_msg.angular_velocity.y = spi_rx.att_rate[1];
    imu_msg.angular_velocity.z = spi_rx.att_rate[2];
    
    imu_msg.linear_acceleration.x = spi_rx.acc_b[0];
    imu_msg.linear_acceleration.y = spi_rx.acc_b[1];
    imu_msg.linear_acceleration.z = spi_rx.acc_b[2];
    
    // Заполнение attitude сообщения
    attitude_msg.x = spi_rx.att[0]; // Roll
    attitude_msg.y = spi_rx.att[1]; // Pitch
    attitude_msg.z = spi_rx.att[2]; // Yaw
    pthread_mutex_unlock(&lock);
    
    imu_pub->publish(imu_msg);
    attitude_pub->publish(attitude_msg);
}

void can_board_send(char sel) {
    spi_tx_cnt = 0;
    spi_tx_buf[spi_tx_cnt++] = 0xFE;
    spi_tx_buf[spi_tx_cnt++] = 0xFC;
    spi_tx_buf[spi_tx_cnt++] = sel;
    spi_tx_buf[spi_tx_cnt++] = 0;

    pthread_mutex_lock(&lock);
    

    if (sel == 45) {
        spi_tx_buf[spi_tx_cnt++] = spi_tx.en_motor * 100 + spi_tx.reset_q * 10 + spi_tx.reset_err;
        spi_tx_buf[spi_tx_cnt++] = mems.Acc_CALIBRATE * 100 + mems.Gyro_CALIBRATE * 10 + mems.Mag_CALIBRATE;
        spi_tx_buf[spi_tx_cnt++] = spi_tx.beep_state;

        for (int id = 0; id < 10; id++) {
            setDataFloat_spi_int(spi_tx.q_set[id], CAN_POS_DIV);
            setDataFloat_spi_int(spi_tx.dq_set[id], CAN_DPOS_DIV);
            setDataFloat_spi_int(spi_tx.tau_ff[id], CAN_T_DIV);
            setDataFloat_spi_int(spi_tx.kp, CAN_GAIN_DIV_P);
            setDataFloat_spi_int(spi_tx.kd, CAN_GAIN_DIV_D);
        }
    } else {
        for (int id = 0; id < 10; id++) {
            setDataFloat_spi(0);
            setDataFloat_spi(0);
            setDataFloat_spi(0);
        }
    }

    spi_tx_buf[3] = (spi_tx_cnt) - 4;
    uint8_t sum_t = 0;
    for (int i = 0; i < spi_tx_cnt; i++) {
        sum_t += spi_tx_buf[i];
    }
    spi_tx_buf[spi_tx_cnt++] = sum_t;

    if (spi_tx_cnt > SPI_SEND_MAX) {
        printf("spi_tx_cnt=%d over flow!!!\n", spi_tx_cnt);
    }
    spi_tx_cnt_show = spi_tx_cnt;
    pthread_mutex_unlock(&lock);
}

void transfer(int fd, int sel) {
    static uint8_t state, rx_cnt;
    static uint8_t _data_len2 = 0, _data_cnt2 = 0;
    static int parser_timeout = 0;
    int ret;
    uint8_t data = 0;

    can_board_send(sel);
    ret = SPIDataRW(0, spi_tx_buf, rx, SPI_SEND_MAX);

    if (ret < 1) {
        printf("SPI ERROR: Reopen! ret=%d\n", ret);
        SPISetup(0, speed);
    } else {
        for (int i = 0; i < SPI_SEND_MAX; i++) {
            data = rx[i];
            parser_timeout++;
            
            if (parser_timeout > 1000) {
                state = 0;
                parser_timeout = 0;
            }
            
            if (state == 0 && data == 0xFF) {
                state = 1;
                spi_rx_buf[0] = data;
                parser_timeout = 0;
            } else if (state == 1 && data == 0xFB) {
                state = 2;
                spi_rx_buf[1] = data;
                parser_timeout = 0;
            } else if (state == 1 && data == 0xFF) {
                spi_rx_buf[0] = data;
                parser_timeout = 0;
            } else if (state == 2 && data > 0 && data < 0xF1) {
                state = 3;
                spi_rx_buf[2] = data;
                parser_timeout = 0;
            } else if (state == 3 && data < SPI_BUF_SIZE) {
                if (data < 50 || data > 150) {
                    state = 0;
                    parser_timeout = 0;
                    continue;
                }
                
                state = 4;
                spi_rx_buf[3] = data;
                _data_len2 = data;
                _data_cnt2 = 0;
                parser_timeout = 0;
            } else if (state == 4 && _data_len2 > 0) {
                _data_len2--;
                spi_rx_buf[4 + _data_cnt2++] = data;
                if (_data_len2 == 0) {
                    state = 5;
                    parser_timeout = 0;
                }
            } else if (state == 5) {
                state = 0;
                spi_rx_buf[4 + _data_cnt2] = data;
                parser_timeout = 0;
                slave_rx(spi_rx_buf, _data_cnt2 + 5);
            } else {
                if (data == 0xFF) {
                    state = 1;
                    spi_rx_buf[0] = data;
                } else {
                    state = 0;
                }
            }
        }
    }
}

// Callback для команд моторов
void motors_command_callback(const Float32MultiArray::SharedPtr msg) {
    pthread_mutex_lock(&lock);
    
    // target_pos
    for (int i = 0; i < 10 && i < msg->data.size() / 3; i++) {
        spi_tx.q_set[i] = msg->data[i];
    }
    
    // target_vel
    for (int i = 0; i < 10 && i < (msg->data.size() - 10) / 2; i++) {
        spi_tx.dq_set[i] = msg->data[i + 10];
    }
    
    // target_trg
    for (int i = 0; i < 10 && i < (msg->data.size() - 20); i++) {
        spi_tx.tau_ff[i] = msg->data[i + 20];
    }
    
    pthread_mutex_unlock(&lock);
}

// Callback для параметров моторов
void motor_params_callback(const Float32MultiArray::SharedPtr msg) {
    if (msg->data.size() >= 2) {
        pthread_mutex_lock(&lock);
        spi_tx.kp = msg->data[0];
        spi_tx.kd = msg->data[1];
        pthread_mutex_unlock(&lock);
    }
}

// Callback для управления мотором
void motor_control_callback(const BoolMsg::SharedPtr msg, int motor_id, int control_type) {
    pthread_mutex_lock(&lock);
    
    switch (control_type) {
        case 0: // enable
            spi_tx.en_motor = msg->data;
            break;
        case 1: // reset_zero
            spi_tx.reset_q = msg->data;
            break;
        case 2: // reset_error
            spi_tx.reset_err = msg->data;
            break;
    }
    
    pthread_mutex_unlock(&lock);
}

// Callback для параметров IMU
void imu_params_callback(const BoolMsg::SharedPtr msg, int param_type) {
    pthread_mutex_lock(&lock);
    
    switch (param_type) {
        case 0: // acc_calibrate
            mems.Acc_CALIBRATE = msg->data;
            break;
        case 1: // mag_calibrate
            mems.Mag_CALIBRATE = msg->data;
            break;
        case 2: // gyro_calibrate
            mems.Gyro_CALIBRATE = msg->data;
            break;
    }
    
    pthread_mutex_unlock(&lock);
}

// Callback для состояния пищалки
void beep_state_callback(const Float32Msg::SharedPtr msg) {
    pthread_mutex_lock(&lock);
    spi_tx.beep_state = msg->data;
    pthread_mutex_unlock(&lock);
}

void* Thread_SPI(void* arg) {
    static float timer_spi1 = 0, timer_spi2 = 0;
    static int timer_1s = 0, timer_1m = 0, timer_1h = 0;
    static int consecutive_errors = 0;
    static const int max_consecutive_errors = 10;
    static float timer_cnt = 0;
    float sys_dt = 0;
    int fd = 0;

    printf("Hardware::Thread_SPI started\n");
    Cycle_Time_Init();
    fd = SPISetup(0, speed);
    
    if (fd == -1) {
        printf("init spi failed!\n");
        return nullptr;
    }

    while (running) {
        sys_dt = Get_Cycle_T(15);
        timer_cnt += sys_dt;
        
        if (timer_cnt > 1) {
            timer_cnt = 0;
            timer_1s++;
            if (timer_1s > 60) {
                timer_1s = 0;
                timer_1m++;
            }
            if (timer_1m > 60) {
                timer_1m = 0;
                timer_1h++;
            }
            printf("Hardware::SPI Still Online at hour-%d min-%d sec-%d spi_cnt=%d\n", 
                   timer_1h, timer_1m, timer_1s, spi_tx_cnt_show);
        }
        
        spi_loss_cnt += sys_dt;
        if (spi_loss_cnt > 1.5 && spi_connect == 1) {
            spi_loss_cnt = 0;
            spi_connect = 0;
            consecutive_errors++;
            printf("Hardware::Hardware SPI-STM32 Loss!!! consecutive_errors=%d\n", consecutive_errors);
            
            if (consecutive_errors >= max_consecutive_errors) {
                printf("Hardware::Too many consecutive errors, reinitializing SPI...\n");
                close(fd);
                usleep(100000);
                fd = SPISetup(0, speed);
                if (fd == -1) {
                    printf("Hardware::SPI reinitialization failed!\n");
                } else {
                    printf("Hardware::SPI reinitialized successfully\n");
                    consecutive_errors = 0;
                }
            }
        } else if (spi_connect == 1) {
            consecutive_errors = 0;
        }
        
        timer_spi1 += sys_dt;
        timer_spi2 += sys_dt;

        transfer(fd, 45);
        usleep(DELAY_SPI);
    }
    
    close(fd);
    return nullptr;
}

void signal_handler(int sig) {
    printf("Hardware::Received signal %d, shutting down...\n", sig);
    running = 0;
}

int main(int argc, char *argv[]) {
    pthread_t tidb;
    int ret;
    
    // Инициализация ROS2
    rclcpp::init(argc, argv);
    ros_node = rclcpp::Node::make_shared("spi2can_bridge");
    
    // Создание publishers
    motors_state_pub = ros_node->create_publisher<Float32MultiArray>("/motors_states", 10);
    imu_pub = ros_node->create_publisher<ImuMsg>("/imu_data", 10);
    attitude_pub = ros_node->create_publisher<Vector3Msg>("/attitude", 10);
    
    // Создание subscribers
    auto motors_cmd_sub = ros_node->create_subscription<Float32MultiArray>(
        "/motors_commands", 10, motors_command_callback);
    
    auto motor_params_sub = ros_node->create_subscription<Float32MultiArray>(
        "/motor_parameters", 10, motor_params_callback);
    
    auto beep_state_sub = ros_node->create_subscription<Float32Msg>(
        "/beep_state", 10, beep_state_callback);
    
    // Установка обработчика сигналов
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    // Инициализация мьютекса
    pthread_mutex_init(&lock, NULL);
    
    // Создание SPI потока
    pthread_create(&tidb, NULL, Thread_SPI, NULL);
    printf("Hardware::SPI thread created successfully\n");
    
    // Запуск ROS2 spinning
    rclcpp::spin(ros_node);
    
    // Ожидание завершения потока
    running = 0;
    pthread_join(tidb, NULL);
    
    // Очистка ресурсов
    pthread_mutex_destroy(&lock);
    rclcpp::shutdown();
    
    printf("Hardware::All threads finished, program exiting\n");
    return 0;
}
