#include "unitreeA1_cmd.h"
#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <cerrno>
#include <chrono>
#include "serial_port.h"

motor_send_t cmd; 
motor_recv_t data; 
//int fd = serial_init("/dev/ttyUSB0");  ///dev/ttyCH343USB2
void my_pos(float p);

int main() {
    int fd = serial_init("/dev/ttyUSB0");  ///dev/ttyCH343USB2
    // int fd = serial_init("/dev/ttyUSB0");  ///dev/ttyCH343USB2
    if(fd<0)
    {
        std::cout <<"serial init error!" <<std::endl;
        return 1;
    }
    else
    {
        std::cout <<"serial init success!" <<fd<<std::endl;
    }

    
    cmd.id=0;
    cmd.mode=10;
    cmd.K_P=0.0;
    cmd.K_W=0.0;
    cmd.Pos=0;
    cmd.T=0.06;
    cmd.W=0;
    
    // // 获取起始时间点
    // auto start = std::chrono::high_resolution_clock::now();
    // for(int i = 0; i<1000; i++)
    // {
    //     serial_SendRecv(fd, cmd, data);
    //     //usleep(200);
    // }
    // // 获取结束时间点
    // auto end = std::chrono::high_resolution_clock::now(    // // 计算耗时（以毫秒为单位）
    // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    // // 打印结果
    // std::cout << "程序运行时间: " << duration.count() << " us" << std::endl;
    

    tcflush(fd, TCIOFLUSH);
    usleep(10000);
    serial_SendRecv(fd, cmd, data);
    usleep(10000);
    int i = 0, j = 0;       
    while(1) 
    {   
        
        //位置模式测试
        // i++;
        // if(i>2000)
        // {
        //     i =0;
        //     j++;//每2s目标位置加1rad
        //     if(j>3)j=0; 
        // }
        //cmd.Pos = (j*9.1); //对应到输出轴的位置（rad）
        // cmd.id=1;

        // int state = serial_SendRecv(fd, cmd, data);
        // printf("id=1   T: %.4f \tw: %.4f \tPos: %.4f \tTemp: %d \tAcc: %.4f \tMError: %d", data.T, data.W, data.Pos, data.Temp, data.Acc, data.MError);
        // usleep(1000*2);
        
        //cmd.id=2;
        serial_SendRecv(fd, cmd, data);
        usleep(1000*1);
        //my_pos(20);


        //打印接收到的数据
        printf("id=2   T: %.4f \tw: %.4f \tPos: %.4f \tTemp: %d \tAcc: %.4f \tMError: %d", data.T, data.W, data.Pos, data.Temp, data.Acc, data.MError);
    
    }

    close(fd);
    return 0;

    
}

// void my_pos(float p)
// {
//     while(!(data.Pos>=p))
//     {
//     cmd.Pos+=0.05;
//     serial_SendRecv(fd, cmd, data);
//     usleep(1000*20);
//     }
    
// }


