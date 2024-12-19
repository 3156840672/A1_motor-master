#include "unitreeA1_cmd.h"
#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <cerrno>
#include <unistd.h>
#include "uart.hpp"


void clearReceiveBuffer(int fd) {
    const size_t bufferSize = 1024; // 创建一个足够大的缓冲区
    char buffer[bufferSize];
    ssize_t bytes_read;

    // 循环读取，直到没有更多数据可读
    while ((bytes_read = read(fd, buffer, bufferSize)) > 0) {
        // 这里可以打印或丢弃读取的数据
        // 例如，简单地丢弃数据：
        // 注意：这里没有对读取的数据做任何处理
    }

    // 检查是否因为错误而退出循环
    if (bytes_read < 0 && errno != EAGAIN) {
        // EAGAIN表示没有数据可读，这是正常的
        // 其他错误应该被处理
        std::cerr << "Error reading from serial port: " << strerror(errno) << std::endl;
        // 这里可以添加错误处理代码
    }
}


int set_serial_port(int fd, int baud_rate) {
    struct termios tty;
    memset(&tty, 0, sizeof tty);
 
    if (tcgetattr(fd, &tty) != 0) {
        std::cerr << "Error from tcgetattr: " << strerror(errno) << std::endl;
        return -1;
    }
    
    if(cfsetospeed(&tty, baud_rate) || cfsetispeed(&tty, baud_rate))
    {
        std::cerr << "baud set error: " << strerror(errno) << std::endl;
    }


    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_lflag = 0;                // no signaling chars, no echo,
                                    // no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;            // read doesn't block
    tty.c_cc[VTIME] = 1;           // 0.1 second read timeout (deciseconds)
 
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
    tty.c_cflag |= (CLOCAL | CREAD);    // ignore modem controls,
                                       // enable reading
    tty.c_cflag &= ~PARENB;             // shut off parity
    tty.c_cflag &= ~CSTOPB;             // 1 stop bit
    tty.c_cflag &= ~CRTSCTS;            // no hardware flowcontrol
 
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cerr << "Error from tcsetattr: " << strerror(errno) << std::endl;
        return -1;
    }
    return 0;
}
 
//初始化串口
//参数：端口号
int serial_init(const char* portname)
{
    int fd = open(portname, O_RDWR | O_NOCTTY | O_NDELAY);//open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        std::cerr << "Error opening " << portname << ": " << strerror(errno) << std::endl;
        return -1;
    }
    //宇树A1电机固定波特率4.8Mbps
    // if (set_serial_port(fd, B4800000) != 0) {
    //     close(fd);
    //     return -1;
    // }
    //通过以下方式设置波特率4.8M才能稳定发送
    Uart uart;
    uart.SerialInit(fd, portname, 4800000);
    return fd;
}


/***************************************************************
 * @brief     串口发送命令接收数据
 * @param     fd:文件描述符
 * @return    
 *  0:发送成功、接收成功； 
 * -1:发送失败； 
 * -2：发送成功，接收读取错误； 
 * -3：发送成功，接收0字节（未回应）； 
 * -4：发送成功，接收未通过CRC校验
 * @note      
 * 发送和接收之间加上适当延时；
 * 连续发送serial_SendRecv时建议控制频率在1000Hz；
 * 要读取数据，建议加上判断，返回值为0才读取，否则接收到的数据可能没有更新
 * @example   
**************************************************************/
int serial_SendRecv(int fd, motor_send_t& cmd, motor_recv_t& data)
{
    // tcflush(fd, TCIFLUSH);
    // tcflush(fd, TCOFLUSH);
    // usleep(200);
    send_cmd(cmd);
    //发送34字节命令
    ssize_t n_written = write(fd, &cmd.motor_send_data, 34);
    if (n_written < 0) {
        std::cerr << "Error writing to: " << strerror(errno) << std::endl;
        return -1;
    }
    printf("\nTx: %d Byte\t", n_written);
    
    tcdrain(fd);
    usleep(200);//可改--不建议去除

    //收78字节数据
    ssize_t n_read = read(fd, &data.motor_recv_data, 78);
     //clearReceiveBuffer(fd);
     //usleep(500);
    if (n_read < 0) {
        std::cerr << " Error reading from: " << strerror(errno);
        return -2;
    }
    printf(" Rx: %d Byte\t" ,n_read );
    if(n_read==0){
        return -3;
    }
    if(receive(data))
    {
        clearReceiveBuffer(fd);
        printf("CRC err! ");
        return -4;
    }
   // tcflush(fd, TCIFLUSH);
    return 0;
}