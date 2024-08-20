#include "Serial.h"

#include <stdio.h>
#include <stdlib.h>    
#include <string.h>
#include <unistd.h>    
#include <sys/types.h>  
#include <sys/stat.h>   
#include <fcntl.h>      
#include <termios.h>  
#include <errno.h>

WzSerialPort::WzSerialPort()
{
}

WzSerialPort::~WzSerialPort()
{

}

void float2u8Arry(uint8_t *u8Arry, float *floatdata, bool key)
{
    uint8_t farray[4];
    *(float *)farray = *floatdata;
    if (key == true)
    {
        u8Arry[3] = farray[0];
        u8Arry[2] = farray[1];
        u8Arry[1] = farray[2];
        u8Arry[0] = farray[3];
    }
    else
    {
        u8Arry[0] = farray[0];
        u8Arry[1] = farray[1];
        u8Arry[2] = farray[2];
        u8Arry[3] = farray[3];
    }
}

void float2u8Arry_real(uint8_t *u8Arry, float floatdata, bool key)
{
    uint8_t farray[4];
    *(float *)farray = floatdata;
    if (key == true)
    {
        u8Arry[3] = farray[0];
        u8Arry[2] = farray[1];
        u8Arry[1] = farray[2];
        u8Arry[0] = farray[3];
    }
    else
    {
        u8Arry[0] = farray[0];
        u8Arry[1] = farray[1];
        u8Arry[2] = farray[2];
        u8Arry[3] = farray[3];
    }
}


float u8Arry2float(uint8_t *data, bool key)
{
    float fa = 0;
    uint8_t uc[4];
    if (key == true)
    {
        uc[3] = data[0];
        uc[2] = data[1];
        uc[1] = data[2];
        uc[0] = data[3];
    }
    else
    {
        uc[0] = data[0];
        uc[1] = data[1];
        uc[2] = data[2];
        uc[3] = data[3];
    }

    memcpy(&fa, uc, 4);
    return fa;
}

bool WzSerialPort::open(const char* portname, int baudrate, char parity, char databit, char stopbit, char synchronizeflag)
{
    // 打开串口
    pHandle[0] = -1;
    // 以 读写、不阻塞 方式打开
    pHandle[0] = ::open(portname,O_RDWR|O_NOCTTY|O_NONBLOCK);
    
    // 打开失败，则打印失败信息，返回false
    if(pHandle[0] == -1)
    {
        // std::cout << portname << " open failed , may be you need 'sudo' permission." << std::endl;
        return false;
    }

    // 设置串口参数
    // 创建串口参数对象
    struct termios options;
    // 先获得串口的当前参数
    if(tcgetattr(pHandle[0],&options) < 0)
    {
        std::cout << portname << " open failed , get serial port attributes failed." << std::endl;
        return false;
    }

    // 设置波特率
    switch(baudrate)
    {
        case 4800:
            cfsetispeed(&options,B4800);
            cfsetospeed(&options,B4800);
            break;
        case 9600:
            cfsetispeed(&options,B9600);
            cfsetospeed(&options,B9600);
            break;   
        case 19200:
            cfsetispeed(&options,B19200);
            cfsetospeed(&options,B19200);
            break;
        case 38400:
            cfsetispeed(&options,B38400);
            cfsetospeed(&options,B38400);
            break;
        case 57600:
            cfsetispeed(&options,B57600);
            cfsetospeed(&options,B57600);
            break;
        case 115200:
            cfsetispeed(&options,B115200);
            cfsetospeed(&options,B115200);
            break;
        default:
            std::cout << portname << " open failed , unkown baudrate , only support 4800,9600,19200,38400,57600,115200." << std::endl;
            return false;

    }

    // 设置校验位
    switch(parity)
    {
        // 无校验
        case 0:
            options.c_cflag &= ~PARENB;//PARENB：产生奇偶位，执行奇偶校验
            options.c_cflag &= ~INPCK;//INPCK：使奇偶校验起作用
            break;
        // 设置奇校验
        case 1:
            options.c_cflag |= PARENB;//PARENB：产生奇偶位，执行奇偶校验
            options.c_cflag |= PARODD;//PARODD：若设置则为奇校验,否则为偶校验
            options.c_cflag |= INPCK;//INPCK：使奇偶校验起作用
            options.c_cflag |= ISTRIP;//ISTRIP：若设置则有效输入数字被剥离7个字节，否则保留全部8位
            break;
        // 设置偶校验
        case 2:
            options.c_cflag |= PARENB;//PARENB：产生奇偶位，执行奇偶校验
            options.c_cflag &= ~PARODD;//PARODD：若设置则为奇校验,否则为偶校验
            options.c_cflag |= INPCK;//INPCK：使奇偶校验起作用
            options.c_cflag |= ISTRIP;//ISTRIP：若设置则有效输入数字被剥离7个字节，否则保留全部8位
            break;
        default:
            std::cout << portname << " open failed , unkown parity ." << std::endl;
            return false;

    }

    // 设置数据位
    switch(databit)
    {
        case 5:
            options.c_cflag &= ~CSIZE;//屏蔽其它标志位
            options.c_cflag |= CS5;
            break;
        case 6:
            options.c_cflag &= ~CSIZE;//屏蔽其它标志位
            options.c_cflag |= CS6;
            break;
        case 7:
            options.c_cflag &= ~CSIZE;//屏蔽其它标志位
            options.c_cflag |= CS7;
            break;
        case 8:
            options.c_cflag &= ~CSIZE;//屏蔽其它标志位
            options.c_cflag |= CS8;
            break;
        default:
            std::cout << portname << " open failed , unkown databit ." << std::endl;
            return false;
    }

    // 设置停止位
    switch(stopbit)
    {
        case 1:
            options.c_cflag &= ~CSTOPB;//CSTOPB：使用1位停止位
            break;
        case 2:
            options.c_cflag |= CSTOPB;//CSTOPB：使用2位停止位
            break;
        default:
            std::cout << portname << " open failed , unkown stopbit ." << std::endl;
            return false;
    }

//流控
//    newtio.c_cflag |=CRTSCTS;
    //设置等待时间和最小接收字符   
    options.c_cc[VTIME] = 1;
    options.c_cc[VMIN] = 0;
    //处理未接收字符 清空输入缓冲区
	tcflush ( pHandle[0], TCIFLUSH);
	//需要注意的是: 如果不是开发终端之类的，只是串口传输数据，而不需要串口来处理，那么使用原始模式(Raw Mode)方式来通讯，设置方式如下： 
    options.c_lflag  &= ~(ICANON | ECHO | ECHOE | ISIG);  /*Input*/
    options.c_oflag  &= ~OPOST;   /*Output*/
    
    // set serial port to binary mode
    options.c_iflag &= ~(INLCR | ICRNL | IGNCR);
    options.c_oflag &= ~(ONLCR | OCRNL | ONOCR | ONLRET);

    // //设置串口模式为直接接受模式
    cfmakeraw(&options);
        // 激活新配置
    if((tcsetattr(pHandle[0],TCSANOW,&options))!=0) 
    { 
        std::cout << portname << " open failed , can not complete set attributes ." << std::endl;
        return false;
    } 



    return true;
}

void WzSerialPort::close()
{
    if(pHandle[0] != -1)
    {
        ::close(pHandle[0]);
    }
}

int WzSerialPort::send(const void *buf,int len)
{
    int sendCount = 0;
    if(pHandle[0] != -1)
    {   
        // 将 buf 和 len 转换成api要求的格式
        const char *buffer = (char*)buf;
        size_t length = len;
        // 已写入的数据个数
        ssize_t tmp;

        while(length > 0)
        {
            if((tmp = write(pHandle[0], buffer, length)) <= 0)
            {
                if(tmp < 0&&errno == EINTR)
                {
                    tmp = 0;
                }
                else
                {
                    break;
                }
            }
            length -= tmp;
            buffer += tmp;
        }

        sendCount = len - length;
    }
   
    return sendCount;
}

int WzSerialPort::receive(void *buf,int maxlen)
{
    int receiveCount = ::read(pHandle[0],buf,maxlen);
    if(receiveCount < 0)
    {
        receiveCount = 0;
    }
    return receiveCount;
}
