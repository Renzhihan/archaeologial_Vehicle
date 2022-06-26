#include     "../Serial/Serial.h"
#include  <cstring>
using namespace std;
Serial::Serial()	
{
}

Serial::~Serial()
{
}


int Serial::serialMode()
{
    int fd = -1;           //文件描述符，先定义一个与程序无关的值，防止fd为任意值导致程序出bug
    int err;               //返回调用函数的状态
    int len;
    int flag=0;

    bool send=false;
    
    //串口接收的数据.
    uint8_t packages[31]={0};

    const char *dev[]  = {"/dev/ttyUSB1"};
    fd = open(dev[0],O_RDWR | O_NOCTTY | O_NDELAY); //打开串口，返回文件描述符
    if(-1 == fd)
    {
        perror("Can't Open Serial Port");
        return (0);
    }else{
        flag=1;//Seral open permit to trans
    }

    serial_pub = nh_.advertise<yhs_can_msgs::serial>("uwb_pose",10);
    tf::TransformBroadcaster br;
    tf::Transform transform;

    uint8_t uart0_send_buf[11];
    uart0_send_buf[0] = (uint8_t)0x01;
    uart0_send_buf[1] = (uint8_t) 0x10;
    uart0_send_buf[2] = (uint8_t) 0x00;
    uart0_send_buf[3] = (uint8_t) 0x28;
    uart0_send_buf[4] = (uint8_t) 0x00;
    uart0_send_buf[5] = (uint8_t) 0x01;
    uart0_send_buf[6] = (uint8_t)0x02;
    uart0_send_buf[7] = (uint8_t) 0x00;
    uart0_send_buf[8] = (uint8_t) 0x04;
    uart0_send_buf[9] = (uint8_t) 0xA1; //crc
    uart0_send_buf[10] = (uint8_t) 0xBB; //crc

    yhs_can_msgs::serial uwb_pose_msg;

    do{
        err = UART0_Init(fd,57600,0,8,2,'N');
    }while(FALSE == err || FALSE == fd);

    while(ros::ok())//serial fasong shuju
    {
        if (flag)
        {
            if(send==false)
            {
                //***************发送**************//
                len = UART0_Send(fd, uart0_send_buf, 11);
                if(len > 0)
                {
                    printf("time send %d data successful\n",len);
                }else{
                    printf("send data failed!\n");
                }

                flag = fcntl(fd,F_GETFL, 0);
                flag |= O_NONBLOCK;
                fcntl(fd,F_SETFL,flag);
                send=true;
            }
            //***************接收**************//
            int len_receive = UART0_Recv(fd, (char *)packages, sizeof(packages));

            if (len_receive > 0)
            {
                printf("\n receive %d data successful\n", len_receive);
            }else {
                printf("\n receive data failed!\n");
            }
            // int x=0;
            // int y=0;
            // vector<int> temp1;
            // vector<int> temp2;
            // bool first=true;
            //提取数据包
            printf("new frame  \n");
            char src[30]={0};
    for(int i=0;i<sizeof(packages);i++)
    {
        src[i]=packages[i];
        printf("%c ",src[i]);
    }

    int len = strlen(src);
    int k=0;
    int num[5];
    int x = 0,y = 0;
    int flag1 = 0;
    for (int i = 0; i < len; i++)
    {
        if(src[i] >= '0' && src[i] <= '9'){
        	k++;
        	num[k] = src[i] - '0';
        	if(!(src[i+1] >= '0' && src[i+1] <= '9')){
        		if(flag1 == 0){
        			for(int s=1 ; s<k+1 ;s++)  x = 10*x + num[s];
        			flag1 = 1;
				}
        		else{
        			for(int s=1 ; s<k+1 ;s++)  y = 10*y + num[s];
        			flag1 = 0;
				}
				k = 0;
			}   	
		}
    }
    cout<<"x="<<x<<endl;
    cout<<"y="<<y<<endl;




        //    if((uint8_t)packages[0]==0x52)
        //     {
        //         for(int i=0;i<sizeof(packages);i++){
        //             if(packages[i]=='=')
        //             {
        //                 for(int j=i+2;packages[j]>='0'&&packages[j]<='9';j++){
        //                     if(first){
        //                         temp1.push_back(packages[j]-'0');
        //                     }
        //                     else
        //                         temp2.push_back(packages[j]-'0');
        //                 }
        //                 first=false;
        //             }   
        //              printf("%c ",packages[i]);
        //             cout<<temp1[0];//测试容器是否存储到数值
        //         }
        //     }
        //     int x_len=temp1.size();
        //     for(int i=0;i<=x_len-1;i++){//转化为数字
        //             x=x*10+temp1[i];
        //     }
        //     int y_len=temp2.size();
        //     for(int i=0;i<=y_len-1;i++){//同上
        //             y=y*10+temp2[i];
        //     }
        //     printf("XXXXXXXx :%d YYYYYY:%d",x,y);
            // for (int i=0;i<sizeof(packages);i++)
            // {
            //     if(packages[i]=='=')
            //     {
            //         printf("aaaaaaaa");
            //         for(int j=i+2;packages[j]>='0'&&packages[j]<='9';j++)
            //         {
            //             printf("bbbbbbbb");
            //             if(first) {tmp1[j-i-2]=(int)(packages[j]-48);}
            //             else tmp2[j-i-2]=(int)(packages[j]-48);
            //         }
            //         first=false;
            //     }
            //     printf("%c ",packages[i]);
            // }
            //     int x_len=tmp1.size();
            //     for(int i=x_len;i>=1;i--)
            //     {
            //         x+=tmp1[i-1]*10^(x_len-i);
            //     }
            //     int y_len=tmp2.size();
            //     for(int i=y_len;i>=1;i--)
            //     {
            //        y+=tmp2[i-1]*10^(y_len-i);
            //     }

            //     printf("XXXXXXXx :%d YYYYYY:%d",x,y);
            // }
                // if(packages[i]>='0'&&packages[i]<='9')

                // {
                //     for(;;i++)
                //     {
                //         if(first)
                //         {
                            
                //         }
                //     }
                
                 //printf("%c",packages[i]);
                
            

            //if ((uint8_t)packages[1]==0x03)
            //{
            //    uint8_t  x_high=(uint8_t)packages[8];
            //    uint8_t x_low=(uint8_t)packages[7];
            //    uint8_t  y_high=(uint8_t)packages[10];
            //    uint8_t  y_low=(uint8_t)packages[9];
            //     printf("X: %x %x ",x_high,x_low);
            //     printf("Y: %x %x ",y_high,y_low);

            //     float x=(((x_high<<8)&0xFF00)|((x_low)&0x00FF))*0.01;
            //     float y=(((y_high<<8)&0xFF00)|((y_low)&0x00FF))*0.01;

            //     printf("XXXX: %f YYYY: %f",x,y);
                // 



                
                uwb_pose_msg.uwb_x = x;
                uwb_pose_msg.uwb_y = y;
                serial_pub.publish(uwb_pose_msg);

                transform.setOrigin( tf::Vector3(x, y, 0.0) );
                tf::Quaternion q;
                q.setRPY(0.0, 0.0, 0.0);
                transform.setRotation(q);
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));

            
            //break;
        }
    }
    close(fd);

}


int Serial::UART0_Open(int fd,char*port)
{
    fd = open( port, O_RDWR|O_NOCTTY|O_NDELAY);
    if (fd<0)
    {
        perror("Can't Open Serial Port");
        return(FALSE);
    }
    //恢复串口为阻塞状态
    if(fcntl(fd, F_SETFL, FNDELAY) < 0)
    {
        printf("fcntl failed!\n");
        return(FALSE);
    }
    else
    {
        printf("fcntl=%d\n",fcntl(fd, F_SETFL,0));
    }
    //测试是否为终端设备
    if(0 == isatty(STDIN_FILENO))
    {
        printf("standard input is not a terminal device\n");
        return(FALSE);
    }
    else
    {
        printf("isatty success!\n");
    }
    printf("fd->open=%d\n",fd);
    return fd;
}

int Serial::UART0_Set(int fd,int speed,int flow_ctrl,int databits,int stopbits,int parity)
{

    int   i;
    int   status;
    int   speed_arr[] = { B115200,B57600, B38400,B19200,B9600, B4800, B2400, B1200, B300};
    int   name_arr[] = {115200, 57600,38400, 19200,  9600,  4800,  2400,  1200,  300};

    struct termios options;

    /*  tcgetattr(fd,&options)得到与fd指向对象的相关参数，并将它们保存于options,该函数还可以测试配置是否正确，
        该串口是否可用等。若调用成功，函数返回值为0，若调用失败，函数返回值为1.  */
    if( tcgetattr( fd,&options)  !=  0)
    {
        perror("SetupSerial 1");
        return(FALSE);
    }

    //设置串口输入波特率和输出波特率
    for ( i= 0;  i < sizeof(speed_arr) / sizeof(int);  i++)
    {
        if  (speed == name_arr[i])
        {
            cfsetispeed(&options, speed_arr[i]);
            cfsetospeed(&options, speed_arr[i]);
        }
    }

    //修改控制模式，保证程序不会占用串口
    options.c_cflag |= CLOCAL;
    //修改控制模式，使得能够从串口中读取输入数据
    options.c_cflag |= CREAD;

    //设置数据流控制
    switch(flow_ctrl)
    {

    case 0 ://不使用流控制
        options.c_cflag &= ~CRTSCTS;
        break;

    case 1 ://使用硬件流控制
        options.c_cflag |= CRTSCTS;
        break;
    case 2 ://使用软件流控制
        options.c_cflag |= IXON | IXOFF | IXANY;
        break;
    }
    //设置数据位
    //屏蔽其他标志位
    options.c_cflag &= ~CSIZE;
    switch (databits)
    {
    case 5    :
        options.c_cflag |= CS5;
        break;
    case 6    :
        options.c_cflag |= CS6;
        break;
    case 7    :
        options.c_cflag |= CS7;
        break;
    case 8:
        options.c_cflag |= CS8;
        break;
    default:
        fprintf(stderr,"Unsupported data size\n");
        return (FALSE);
    }
    //设置校验位
    switch (parity)
    {
    case 'n':
    case 'N': //无奇偶校验位。
        options.c_cflag &= ~PARENB;
        options.c_iflag &= ~INPCK;
        break;
    case 'o':
    case 'O'://设置为奇校验
        options.c_cflag |= (PARODD | PARENB);
        options.c_iflag |= INPCK;
        break;
    case 'e':
    case 'E'://设置为偶校验
        options.c_cflag |= PARENB;
        options.c_cflag &= ~PARODD;
        options.c_iflag |= INPCK;
        break;
    case 's':
    case 'S': //设置为空格
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        break;
    default:
        fprintf(stderr,"Unsupported parity\n");
        return (FALSE);
    }
    // 设置停止位
    switch (stopbits)
    {
    case 1:
        options.c_cflag &= ~CSTOPB; break;
    case 2:
        options.c_cflag |= CSTOPB; break;
    default:
        fprintf(stderr,"Unsupported stop bits\n");
        return (FALSE);
    }

    //修改输出模式，原始数据输出
    options.c_oflag &= ~OPOST;

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    //options.c_lflag &= ~(ISIG | ICANON);

    //设置等待时间和最小接收字符
    options.c_cc[VTIME] = 1; /* 读取一个字符等待1*(1/10)s */
    options.c_cc[VMIN] = 1; /* 读取字符的最少个数为1 */

    //如果发生数据溢出，接收数据，但是不再读取 刷新收到的数据但是不读
    tcflush(fd,TCIFLUSH);

    //激活配置 (将修改后的termios数据设置到串口中）
    if (tcsetattr(fd,TCSANOW,&options) != 0)
    {
        perror("com set error!\n");
        return (FALSE);
    }
    return (TRUE);
}

int Serial::UART0_Init(int fd, int speed,int flow_ctrl,int databits,int stopbits,int parity)
{
    int err=0;
    //设置串口数据帧格式
    if (UART0_Set(fd,115200,0,8,1,'N') == FALSE)
    {
        return FALSE;
    }
    else
    {
        return  TRUE;
    }
}

int Serial::UART0_Recv(int fd, char *rcv_buf,int data_len)
{
    int len,fs_sel;
    fd_set fs_read;

    struct timeval time;

    FD_ZERO(&fs_read);
    FD_SET(fd,&fs_read);    //如果fd == -1, FD_SET将在此阻塞

    time.tv_sec = 1;
    time.tv_usec = 0;

    //串口的多路通信
    fs_sel = select(fd+1,&fs_read,NULL,NULL,&time);
    printf("fs_sel = %d\n",fs_sel);
    if(fs_sel)
    {
        len = read(fd,rcv_buf,data_len);
        return len;
    }
    else
    {
        return FALSE;
    }
}

int Serial::UART0_Send(int fd, uint8_t *send_buf,int data_len)
{
    int len = 0;

    len = write(fd,send_buf,data_len);
    if (len == data_len )
    {
        //printf("send data is %d\n",send_buf);
        return len;
    }
    else
    {
        tcflush(fd,TCOFLUSH);
        return FALSE;
    }
}



int main(int argc,char **argv)
{
    ros::init(argc,argv,"serial_node");
    Serial test;

    test.serialMode();

}

