#include <stdio.h>
#include <iostream>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <string.h>
#include <unistd.h>
#include<ros/ros.h>
#include "roborts_msgs/appdata.h"
#include "roborts_msgs/ifused.h"

int main(int argc, char *argv[])
{
	int server_sockfd;//服务器端套接字
	int client_sockfd;//客户端套接字
	int len;
	struct sockaddr_in my_addr;   //服务器网络地址结构体
	struct sockaddr_in remote_addr; //客户端网络地址结构体
	socklen_t sin_size;
	char buf[BUFSIZ];  //数据传送的缓冲区
	int  data[6];
	memset(&my_addr,0,sizeof(my_addr)); //数据初始化--清零
	my_addr.sin_family=AF_INET; //设置为IP通信
	my_addr.sin_addr.s_addr=inet_addr("192.168.1.111");//服务器IP地址--允许连接到所有本地地址上
	my_addr.sin_port=htons(3000); //服务器端口号
	
	/*创建服务器端套接字--IPv4协议，面向连接通信，TCP协议*/
	if((server_sockfd=socket(PF_INET,SOCK_STREAM,0))<0)
	{  
		perror("socket error");
		return 1;
	}
 
 
	/*将套接字绑定到服务器的网络地址上*/
	if(bind(server_sockfd,(struct sockaddr *)&my_addr,sizeof(struct sockaddr))<0)
	{
		perror("bind error");
		return 1;
	}
	
	/*监听连接请求--监听队列长度为5*/
	if(listen(server_sockfd,5)<0)
	{
		perror("listen error");
		return 1;
	};
	
	sin_size=sizeof(struct sockaddr_in);
	
	/*等待客户端连接请求到达*/
	if((client_sockfd=accept(server_sockfd,(struct sockaddr *)&remote_addr,&sin_size))<0)
	{
		perror("accept error");
		return 1;
	}
	ROS_INFO("accept client %s\n",inet_ntoa(remote_addr.sin_addr));
	len=send(client_sockfd,"6;121;20;\n",21,0);//发送
	ros::init(argc, argv, "socket_server");
    ros::NodeHandle n;

	ros::Publisher appdata_pub = n.advertise<roborts_msgs::appdata>("appdata",10);
    ros::Publisher sys_pub = n.advertise<roborts_msgs::ifused>("ifused",10);
    ros::Rate loop_rate(10);

    roborts_msgs::appdata app_msg;
	roborts_msgs::ifused  system_be_used;

	
	if((len=recv(client_sockfd,buf,BUFSIZ,0))>0)
	system_be_used.system_used =1;
	else system_be_used.system_used = 0;
	sys_pub.publish(system_be_used);
	ros::spinOnce();
	/*接收客户端的数据并将其发送给客户端--recv返回接收到的字节数，send返回发送的字节数*/
	while((len=recv(client_sockfd,buf,BUFSIZ,0))>0)
	{
		ROS_INFO("%s\n",buf);
		int index = 0;
		const char *d = ";";
		char *p;
		p = strtok(buf,d);
		while(p)
		{
			data[index] = std::stod(p);
			index++;
			p=strtok(NULL,d);
		}
        app_msg.task_number = data[0];
		app_msg.position_x = data[1];
		app_msg.position_y = data[2];
		app_msg.vehicle_number = data[3];
		app_msg.hole_number = data[4];
		app_msg.soil_number=data[5];
        ROS_INFO("task_number :%d",app_msg.task_number);
		ROS_INFO("position_x :%d",app_msg.position_x);
        ROS_INFO("position_y :%d",app_msg.position_y);
		ROS_INFO("vehicle_number: %d",app_msg.vehicle_number);
		ROS_INFO("hole_number:%d",app_msg.hole_number);
		ROS_INFO("soil_number:%d",app_msg.soil_number);
		appdata_pub.publish(app_msg);

        ros::spinOnce();
        loop_rate.sleep();
	
		if(send(client_sockfd,buf,len,0)<0)
		{
			perror("write error");
			return 1;
		}
	}
 
	
	/*关闭套接字*/
	close(client_sockfd);
	close(server_sockfd);
    
	return 0;
}
