#include <fcntl.h>
#include <dirent.h>
#include <linux/input.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <pthread.h>
#include <time.h>
#include<mutex>

#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include "sensor_msgs/Imu.h"
#include "yhs_can_control.h"


namespace yhs_tool {

double yaw=0.0;
double pitch=0.0;
double roll=0.0;
double uwb_x=0.0;
double uwb_y=0.0;

std::mutex mtx;
std::mutex uwb_mtx;

CanControl::CanControl()
{
	ros::NodeHandle private_node("~");
	
}


CanControl::~CanControl()
{

}

Serial::Serial()
{

	
}

Serial::~Serial()
{}
//io控制回调函数
void CanControl::io_cmdCallBack(const yhs_can_msgs::io_cmd msg)
{
	static unsigned char count_1 = 0;

	cmd_mutex_.lock();

	memset(sendData_u_io_,0,8);

	sendData_u_io_[0] = msg.io_cmd_enable;

	sendData_u_io_[1] = 0xff;
	if(msg.io_cmd_upper_beam_headlamp)
		sendData_u_io_[1] &= 0xff;
	else sendData_u_io_[1] &= 0xfd;

	if(msg.io_cmd_turn_lamp == 0)
		sendData_u_io_[1] &= 0xf3;
	if(msg.io_cmd_turn_lamp == 1)
		sendData_u_io_[1] &= 0xf7;
	if(msg.io_cmd_turn_lamp == 2)
		sendData_u_io_[1] &= 0xfb;

	sendData_u_io_[2] = msg.io_cmd_speaker;

	sendData_u_io_[3] = 0;
	sendData_u_io_[4] = 0;
	sendData_u_io_[5] = 0;

	count_1 ++;
	if(count_1 == 16)	count_1 = 0;

	sendData_u_io_[6] =  count_1 << 4;

	sendData_u_io_[7] = sendData_u_io_[0] ^ sendData_u_io_[1] ^ sendData_u_io_[2] ^ sendData_u_io_[3] ^ sendData_u_io_[4] ^ sendData_u_io_[5] ^ sendData_u_io_[6];

	send[0].ID = 0x18C4D7D0;

	memcpy(send[0].Data,sendData_u_io_,8);

	if(VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1)
	{
	}

	cmd_mutex_.unlock();
}

//速度控制回调函数
void CanControl::ctrl_cmdCallBack(const yhs_can_msgs::ctrl_cmd msg)
{
	unsigned short vel = 0;
	short angular = msg.ctrl_cmd_steering * 100;
	static unsigned char count = 0;

	cmd_mutex_.lock();

	memset(sendData_u_vel_,0,8);

	if(msg.ctrl_cmd_velocity < 0) vel = 0;
	else
	{
		vel = msg.ctrl_cmd_velocity * 1000;
	}

	sendData_u_vel_[0] = sendData_u_vel_[0] | (0x0f & msg.ctrl_cmd_gear);
	
	sendData_u_vel_[0] = sendData_u_vel_[0] | (0xf0 & ((vel & 0x0f) << 4));

	sendData_u_vel_[1] = (vel >> 4) & 0xff;

	sendData_u_vel_[2] = sendData_u_vel_[2] | (0x0f & (vel >> 12));

	sendData_u_vel_[2] = sendData_u_vel_[2] | (0xf0 & ((angular & 0x0f) << 4));

	sendData_u_vel_[3] = (angular >> 4) & 0xff;

	sendData_u_vel_[4] = sendData_u_vel_[4] | (0xf0 & ((msg.ctrl_cmd_Brake & 0x0f) << 4));

	sendData_u_vel_[4] = sendData_u_vel_[4] | (0x0f & (angular >> 12));

	sendData_u_vel_[5] = 0;

	count ++;

	if(count == 16)	count = 0;

	sendData_u_vel_[6] =  count << 4;
	

	sendData_u_vel_[7] = sendData_u_vel_[0] ^ sendData_u_vel_[1] ^ sendData_u_vel_[2] ^ sendData_u_vel_[3] ^ sendData_u_vel_[4] ^ sendData_u_vel_[5] ^ sendData_u_vel_[6];

	send[0].ID = 0x18C4D2D0;

	memcpy(send[0].Data,sendData_u_vel_,8);

	if(VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1)
	{
	}

	cmd_mutex_.unlock();
}

//数据接收解析线程
void CanControl::recvData()
{
	ros::Rate loop(100);

	int reclen=0;
	int i,j;
	int ind=0;
	double x = 0.0;
	double y = 0.0;
    double th = 0.0;			
	double vel_right= 0.0;
	double vel_left= 0.0;
	double vel_line= 0.0;
    double vx= 0.0,vy= 0.0,r,theta= 0.0;
	double omega= 0.0;
	double current_time,last_time,dt;
	ros::Time current_time1;
	double R;
	
	VCI_CAN_OBJ rec[3000];//接收缓存，设为3000为佳。
	tf::TransformBroadcaster odom_broadcaster;
	last_time = ros::Time::now().toSec();
	while(ros::ok())
	{
		
		if((reclen=VCI_Receive(VCI_USBCAN2,0,ind,rec,3000,100))>0)//调用接收函数，如果有数据，进行数据处理显示。
		{

			
			for(j=0;j<reclen;j++)
			{
				
				switch (rec[j].ID)
				{
					//速度控制反馈
					case 0x18C4D2EF:
					{
						yhs_can_msgs::ctrl_fb msg;
						msg.ctrl_fb_gear = 0x0f & rec[j].Data[0];
						
						msg.ctrl_fb_velocity = (float)((unsigned short)((rec[j].Data[2] & 0x0f) << 12 | rec[j].Data[1] << 4 | (rec[j].Data[0] & 0xf0) >> 4)) / 1000;
						
						msg.ctrl_fb_steering = (float)((short)((rec[j].Data[4] & 0x0f) << 12 | rec[j].Data[3] << 4 | (rec[j].Data[2] & 0xf0) >> 4)) / 100;

						msg.ctrl_fb_Brake = (rec[j].Data[4] & 0x30) >> 4;
						
						msg.ctrl_fb_mode = (rec[j].Data[4] & 0xc0) >> 6;

						unsigned char crc = rec[j].Data[0] ^ rec[j].Data[1] ^ rec[j].Data[2] ^ rec[j].Data[3] ^ rec[j].Data[4] ^ rec[j].Data[5] ^ rec[j].Data[6];

						if(crc == rec[j].Data[7])
						{
								
							ctrl_fb_pub_.publish(msg);
						}

						break;
					}

					//左轮反馈
					case 0x18C4D7EF:
					{
						yhs_can_msgs::lr_wheel_fb msg;
						msg.lr_wheel_fb_velocity = (float)((short)(rec[j].Data[1] << 8 | rec[j].Data[0])) / 1000;
						vel_left = msg.lr_wheel_fb_velocity;
						msg.lr_wheel_fb_pulse = (int)(rec[j].Data[5] << 24 | rec[j].Data[4] << 16 | rec[j].Data[3] << 8 | rec[j].Data[2]);

						unsigned char crc = rec[j].Data[0] ^ rec[j].Data[1] ^ rec[j].Data[2] ^ rec[j].Data[3] ^ rec[j].Data[4] ^ rec[j].Data[5] ^ rec[j].Data[6];

						if(crc == rec[j].Data[7])
						{
								
							lr_wheel_fb_pub_.publish(msg);
						}

						break;
					}

					//右轮反馈
					case 0x18C4D8EF:
					{
						yhs_can_msgs::rr_wheel_fb msg;
						msg.rr_wheel_fb_velocity = (float)((short)(rec[j].Data[1] << 8 | rec[j].Data[0])) / 1000;
						vel_right = msg.rr_wheel_fb_velocity;
						msg.rr_wheel_fb_pulse = (int)(rec[j].Data[5] << 24 | rec[j].Data[4] << 16 | rec[j].Data[3] << 8 | rec[j].Data[2]);

						unsigned char crc = rec[j].Data[0] ^ rec[j].Data[1] ^ rec[j].Data[2] ^ rec[j].Data[3] ^ rec[j].Data[4] ^ rec[j].Data[5] ^ rec[j].Data[6];

						if(crc == rec[j].Data[7])
						{
								
							rr_wheel_fb_pub_.publish(msg);
						}

						break;
					}

					//io反馈
					case 0x18C4DAEF:
					{
						yhs_can_msgs::io_fb msg;
						if(0x01 & rec[j].Data[0]) msg.io_fb_enable = true;	else msg.io_fb_enable = false;
	
						if(0x02 & rec[j].Data[1]) msg.io_fb_upper_beam_headlamp = true;	else msg.io_fb_upper_beam_headlamp = false;

						msg.io_fb_turn_lamp = (0xc & rec[j].Data[1]) >> 2;

						if(0x10 & rec[j].Data[1]) msg.io_fb_braking_lamp = true;	else msg.io_fb_braking_lamp = false;

						if(0x01 & rec[j].Data[2]) msg.io_fb_speaker = true;	else msg.io_fb_speaker = false;

						if(0x02 & rec[j].Data[3]) msg.io_fb_fm_impact_sensor = true;	else msg.io_fb_fm_impact_sensor = false;

						if(0x10 & rec[j].Data[3]) msg.io_fb_rm_impact_sensor = true;	else msg.io_fb_rm_impact_sensor = false;

						unsigned char crc = rec[j].Data[0] ^ rec[j].Data[1] ^ rec[j].Data[2] ^ rec[j].Data[3] ^ rec[j].Data[4] ^ rec[j].Data[5] ^ rec[j].Data[6];

						if(crc == rec[j].Data[7])
						{
								
							io_fb_pub_.publish(msg);
						}

						break;
					}

					//里程计反馈
					case 0x18C4DEEF:
					{
						yhs_can_msgs::odo_fb msg;
						msg.odo_fb_accumulative_mileage = (float)((int)(rec[j].Data[3] << 24 | rec[j].Data[2] << 16 | rec[j].Data[1] << 8 | rec[j].Data[0])) / 1000;

						msg.odo_fb_accumulative_angular = (float)((int)(rec[j].Data[7] << 24 | rec[j].Data[6] << 16 | rec[j].Data[5] << 8 | rec[j].Data[4])) / 1000;

						odo_fb_pub_.publish(msg);

						break;
					}

					//bms_Infor反馈
					case 0x18C4E1EF:
					{
						yhs_can_msgs::bms_Infor_fb msg;
						msg.bms_Infor_voltage = (float)((unsigned short)(rec[j].Data[1] << 8 | rec[j].Data[0])) / 100;

						msg.bms_Infor_current = (float)((short)(rec[j].Data[3] << 8 | rec[j].Data[2])) / 100;

						msg.bms_Infor_remaining_capacity = (float)((unsigned short)(rec[j].Data[5] << 8 | rec[j].Data[4])) / 100;

						unsigned char crc = rec[j].Data[0] ^ rec[j].Data[1] ^ rec[j].Data[2] ^ rec[j].Data[3] ^ rec[j].Data[4] ^ rec[j].Data[5] ^ rec[j].Data[6];

						if(crc == rec[j].Data[7])
						{
								
							bms_Infor_fb_pub_.publish(msg);
						}

						break;
					}

					//bms_flag_Infor反馈
					case 0x18C4E2EF:
					{
						yhs_can_msgs::bms_flag_Infor_fb msg;
						msg.bms_flag_Infor_soc = rec[j].Data[0];

						if(0x01 & rec[j].Data[1]) msg.bms_flag_Infor_single_ov = true;	else msg.bms_flag_Infor_single_ov = false;

						if(0x02 & rec[j].Data[1]) msg.bms_flag_Infor_single_uv = true;	else msg.bms_flag_Infor_single_uv = false;

						if(0x04 & rec[j].Data[1]) msg.bms_flag_Infor_ov = true;	else msg.bms_flag_Infor_ov = false;

						if(0x08 & rec[j].Data[1]) msg.bms_flag_Infor_uv = true;	else msg.bms_flag_Infor_uv = false;

						if(0x10 & rec[j].Data[1]) msg.bms_flag_Infor_charge_ot = true;	else msg.bms_flag_Infor_charge_ot = false;

						if(0x20 & rec[j].Data[1]) msg.bms_flag_Infor_charge_ut = true;	else msg.bms_flag_Infor_charge_ut = false;

						if(0x40 & rec[j].Data[1]) msg.bms_flag_Infor_discharge_ot = true;	else msg.bms_flag_Infor_discharge_ot = false;

						if(0x80 & rec[j].Data[1]) msg.bms_flag_Infor_discharge_ut = true;	else msg.bms_flag_Infor_discharge_ut = false;

						if(0x01 & rec[j].Data[2]) msg.bms_flag_Infor_charge_oc = true;	else msg.bms_flag_Infor_charge_oc = false;

						if(0x02 & rec[j].Data[2]) msg.bms_flag_Infor_discharge_oc = true;	else msg.bms_flag_Infor_discharge_oc = false;

						if(0x04 & rec[j].Data[2]) msg.bms_flag_Infor_short = true;	else msg.bms_flag_Infor_short = false;

						if(0x08 & rec[j].Data[2]) msg.bms_flag_Infor_ic_error = true;	else msg.bms_flag_Infor_ic_error = false;

						if(0x10 & rec[j].Data[2]) msg.bms_flag_Infor_lock_mos = true;	else msg.bms_flag_Infor_lock_mos = false;

						if(0x20 & rec[j].Data[2]) msg.bms_flag_Infor_charge_flag = true;	else msg.bms_flag_Infor_charge_flag = false;

						msg.bms_flag_Infor_hight_temperature = (float)((short)(rec[j].Data[4] << 4 | rec[j].Data[3] >> 4)) / 10;

						msg.bms_flag_Infor_low_temperature = (float)((short)((rec[j].Data[6] & 0x0f) << 8 | rec[j].Data[5])) / 10;

						unsigned char crc = rec[j].Data[0] ^ rec[j].Data[1] ^ rec[j].Data[2] ^ rec[j].Data[3] ^ rec[j].Data[4] ^ rec[j].Data[5] ^ rec[j].Data[6];

						if(crc == rec[j].Data[7])
						{
								
							bms_flag_Infor_fb_pub_.publish(msg);
						}

						break;
					}

					//Drive_fb_MCUEcoder反馈
					case 0x18C4DCEF:
					{
						yhs_can_msgs::Drive_MCUEcoder_fb msg;
						msg.Drive_fb_MCUEcoder = (int)(rec[j].Data[3] << 24 | rec[j].Data[2] << 16 | rec[j].Data[1] << 8 | rec[j].Data[0]); 

						unsigned char crc = rec[j].Data[0] ^ rec[j].Data[1] ^ rec[j].Data[2] ^ rec[j].Data[3] ^ rec[j].Data[4] ^ rec[j].Data[5] ^ rec[j].Data[6];

						if(crc == rec[j].Data[7])
						{
								
							Drive_MCUEcoder_fb_pub_.publish(msg);
						}

						break;
					}

					//Veh_fb_Diag反馈
					case 0x18C4EAEF:
					{
						yhs_can_msgs::Veh_Diag_fb msg;
						msg.Veh_fb_FaultLevel = 0x0f & rec[j].Data[0];

						if(0x10 & rec[j].Data[0]) msg.Veh_fb_AutoCANCtrlCmd = true;	else msg.Veh_fb_AutoCANCtrlCmd = false;

						if(0x20 & rec[j].Data[0]) msg.Veh_fb_AutoIOCANCmd = true;	else msg.Veh_fb_AutoIOCANCmd = false;

						if(0x01 & rec[j].Data[1]) msg.Veh_fb_EPSDisOnline = true;	else msg.Veh_fb_EPSDisOnline = false;

						if(0x02 & rec[j].Data[1]) msg.Veh_fb_EPSfault = true;	else msg.Veh_fb_EPSfault = false;

						if(0x04 & rec[j].Data[1]) msg.Veh_fb_EPSMosfetOT = true;	else msg.Veh_fb_EPSMosfetOT = false;

						if(0x08 & rec[j].Data[1]) msg.Veh_fb_EPSWarning = true;	else msg.Veh_fb_EPSWarning = false;

						if(0x10 & rec[j].Data[1]) msg.Veh_fb_EPSDisWork = true;	else msg.Veh_fb_EPSDisWork = false;

						if(0x20 & rec[j].Data[1]) msg.Veh_fb_EPSOverCurrent = true;	else msg.Veh_fb_EPSOverCurrent = false;

						
						
						if(0x10 & rec[j].Data[2]) msg.Veh_fb_EHBecuFault = true;	else msg.Veh_fb_EHBecuFault = false;

						if(0x20 & rec[j].Data[2]) msg.Veh_fb_EHBDisOnline = true;	else msg.Veh_fb_EHBDisOnline = false;

						if(0x40 & rec[j].Data[2]) msg.Veh_fb_EHBWorkModelFault = true;	else msg.Veh_fb_EHBWorkModelFault = false;

						if(0x80 & rec[j].Data[2]) msg.Veh_fb_EHBDisEn = true;	else msg.Veh_fb_EHBDisEn = false;


						if(0x01 & rec[j].Data[3]) msg.Veh_fb_EHBAnguleFault = true;	else msg.Veh_fb_EHBAnguleFault = false;

						if(0x02 & rec[j].Data[3]) msg.Veh_fb_EHBOT = true;	else msg.Veh_fb_EHBOT = false;

						if(0x04 & rec[j].Data[3]) msg.Veh_fb_EHBPowerFault = true;	else msg.Veh_fb_EHBPowerFault = false;

						if(0x08 & rec[j].Data[3]) msg.Veh_fb_EHBsensorAbnomal = true;	else msg.Veh_fb_EHBsensorAbnomal = false;

						if(0x10 & rec[j].Data[3]) msg.Veh_fb_EHBMotorFault = true;	else msg.Veh_fb_EHBMotorFault = false;

						if(0x20 & rec[j].Data[3]) msg.Veh_fb_EHBOilPressSensorFault = true;	else msg.Veh_fb_EHBOilPressSensorFault = false;

						if(0x40 & rec[j].Data[3]) msg.Veh_fb_EHBOilFault = true;	else msg.Veh_fb_EHBOilFault = false;




						if(0x01 & rec[j].Data[4]) msg.Veh_fb_DrvMCUDisOnline = true;	else msg.Veh_fb_DrvMCUDisOnline = false;

						if(0x02 & rec[j].Data[4]) msg.Veh_fb_DrvMCUOT = true;	else msg.Veh_fb_DrvMCUOT = false;

						if(0x04 & rec[j].Data[4]) msg.Veh_fb_DrvMCUOV = true;	else msg.Veh_fb_DrvMCUOV = false;

						if(0x08 & rec[j].Data[4]) msg.Veh_fb_DrvMCUUV = true;	else msg.Veh_fb_DrvMCUUV = false;

						if(0x10 & rec[j].Data[4]) msg.Veh_fb_DrvMCUShort = true;	else msg.Veh_fb_DrvMCUShort = false;

						if(0x20 & rec[j].Data[4]) msg.Veh_fb_DrvMCUScram = true;	else msg.Veh_fb_DrvMCUScram = false;

						if(0x40 & rec[j].Data[4]) msg.Veh_fb_DrvMCUHall = true;	else msg.Veh_fb_DrvMCUHall = false;

						if(0x80 & rec[j].Data[4]) msg.Veh_fb_DrvMCUMOSFEF = true;	else msg.Veh_fb_DrvMCUMOSFEF = false;


						if(0x10 & rec[j].Data[5]) msg.Veh_fb_AUXBMSDisOnline = true;	else msg.Veh_fb_AUXBMSDisOnline = false;

						if(0x20 & rec[j].Data[5]) msg.Veh_fb_AuxScram = true;	else msg.Veh_fb_AuxScram = false;

						if(0x40 & rec[j].Data[5]) msg.Veh_fb_AuxRemoteClose = true;	else msg.Veh_fb_AuxRemoteClose = false;

						if(0x80 & rec[j].Data[5]) msg.Veh_fb_AuxRemoteDisOnline = true;	else msg.Veh_fb_AuxRemoteDisOnline = false;

						unsigned char crc = rec[j].Data[0] ^ rec[j].Data[1] ^ rec[j].Data[2] ^ rec[j].Data[3] ^ rec[j].Data[4] ^ rec[j].Data[5] ^ rec[j].Data[6];

						if(crc == rec[j].Data[7])
						{
								
							Veh_Diag_fb_pub_.publish(msg);
						}


						break;
					}

					default:
						break;
				}
			
			}

				current_time=ros::Time::now().toSec();
				current_time1=ros::Time::now();
				dt=current_time-last_time;

			    vel_line = (vel_left+vel_right)/2.0;
				//vel_left/R = vel_line/(R+0.3125);
				if ((vel_line-vel_left)>1e-4)
					R	 = 0.3125*vel_line/(vel_line-vel_left);
				else
					R=99999;
				double tan_delta = 0.66/R;

				omega = vel_line*tan_delta/0.66;
				vx = vel_line*cos(th);
				vy = vel_line*sin(th);

				th = yaw;
				x = uwb_x;
				y = uwb_y;
				//th +=omega*dt;
				// x += vx*dt;
				// y += vy*dt;




				//omega = (vel_right-vel_left)/0.645;            //轮距645mm

                

				// float vx = vel_line;
				// float vy = 0.0;
				//double vth = omega;
				
                




                // double delta_th = vth * dt;
                // if((vel_right-vel_left)<1e-6)
                // {
                //     vx=vel_line;
                //     vy = 0.0;
                // }
                // else
                // {
                //     r=vel_line/omega;
                //     theta=atan(0.66/r);
                //     vx=vel_line*cos(theta);
                //     vy=vel_line*sin(theta);

                // }

                // ROS_ERROR("vx: %.14lf",vx);
                // ROS_ERROR("vy: %.14lf",vy);

                

				//compute odometry in a typical way given the velocities of the robot
				// double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
				// double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
               // ROS_ERROR("dy: %.14lf",delta_y);
				// float delta_th = vth * dt;
				// x += delta_x;
				// y += delta_y;
				// th += delta_th;
				//th = yaw;
				//since all odometry is 6DOF we'll need a quaternion created from yaw
				geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

				//first, we'll publish the transform over tf
				// geometry_msgs::TransformStamped odom_trans;
				// odom_trans.header.stamp = current_time1;
				// odom_trans.header.frame_id = "odom";
				// odom_trans.child_frame_id = "odom_combined";

				// odom_trans.transform.translation.x = x;
				// odom_trans.transform.translation.y = y;
				// odom_trans.transform.translation.z = 0.0;
				// odom_trans.transform.rotation = odom_quat;

				//send the transform
				//odom_broadcaster.sendTransform(odom_trans);

				//next, we'll publish the odometry message over ROS
				nav_msgs::Odometry odom;
				odom.header.stamp = current_time1;
				odom.header.frame_id = "map";
		
				//set the position
				odom.pose.pose.position.x = x;
				odom.pose.pose.position.y = y;
				odom.pose.pose.position.z = 0.0;
				odom.pose.pose.orientation = odom_quat;
				odom.pose.covariance={0};
				//set the velocity
				odom.child_frame_id = "odom_combined";
				odom.twist.twist.linear.x = vx;
				odom.twist.twist.linear.y = vy;
				odom.twist.twist.angular.z = omega;

				//publish the message
				odom_pub_.publish(odom);

				last_time = current_time;					
		}

		loop.sleep();
	}
}

//数据发送线程
void CanControl::sendData()
{
	ros::Rate loop(100);


	while(ros::ok())
	{

		loop.sleep();
	}

	//复位CAN1通道。
	VCI_ResetCAN(VCI_USBCAN2, 0, 0);
	usleep(100000);

	//关闭设备。
	VCI_CloseDevice(VCI_USBCAN2,0);	
}


void CanControl::run()
{

	ctrl_cmd_sub_ = nh_.subscribe<yhs_can_msgs::ctrl_cmd>("ctrl_cmd", 5, &CanControl::ctrl_cmdCallBack, this);
	io_cmd_sub_ = nh_.subscribe<yhs_can_msgs::io_cmd>("io_cmd", 5, &CanControl::io_cmdCallBack, this);
	odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 50);
	ctrl_fb_pub_ = nh_.advertise<yhs_can_msgs::ctrl_fb>("ctrl_fb",5);
	lr_wheel_fb_pub_ = nh_.advertise<yhs_can_msgs::lr_wheel_fb>("lr_wheel_fb",5);
	rr_wheel_fb_pub_ = nh_.advertise<yhs_can_msgs::rr_wheel_fb>("rr_wheel_fb",5);
	io_fb_pub_ = nh_.advertise<yhs_can_msgs::io_fb>("io_fb",5);
	odo_fb_pub_ = nh_.advertise<yhs_can_msgs::odo_fb>("odo_fb",5);
	bms_Infor_fb_pub_ = nh_.advertise<yhs_can_msgs::bms_Infor_fb>("bms_Infor_fb",5);
	bms_flag_Infor_fb_pub_ = nh_.advertise<yhs_can_msgs::bms_flag_Infor_fb>("bms_flag_Infor_fb",5);
	Drive_MCUEcoder_fb_pub_ = nh_.advertise<yhs_can_msgs::Drive_MCUEcoder_fb>("Drive_MCUEcoder_fb",5);
	Veh_Diag_fb_pub_ = nh_.advertise<yhs_can_msgs::Veh_Diag_fb>("Veh_Diag_fb",5);

	// car_state_sub=nh_.subscribe<yhs_can_msgs::state>("state",5,&Serial::);

	//打开设备
	if(VCI_OpenDevice(VCI_USBCAN2,0,0)==1)
	{
		ROS_INFO(">>open can deivce success!");
	}
	else
	{
		ROS_ERROR(">>open can deivce error!");
		return;
	}


	//初始化参数，严格参数二次开发函数库说明书。
	VCI_INIT_CONFIG config;
	config.AccCode=0;
	config.AccMask=0xFFFFFFFF;
	//接收所有帧
	config.Filter=1;
	/*波特率500 Kbps  0x00  0x1C*/
	config.Timing0=0x00;
	config.Timing1=0x1C;
	//正常模式	
	config.Mode=0;	

	send[0].SendType=1;
	send[0].RemoteFlag=0;
	send[0].ExternFlag=1;
	send[0].DataLen=8;
	
	bool error = false;
	if(VCI_InitCAN(VCI_USBCAN2,0,0,&config)!=1)
	{
		ROS_ERROR(">>Init CAN1 error!\n");
		error = true;
	}
	if(VCI_StartCAN(VCI_USBCAN2,0,0)!=1)
	{
		ROS_ERROR(">>Start CAN1 error!\n");
		error = true;
	}


	if(VCI_InitCAN(VCI_USBCAN2,0,1,&config)!=1)
	{
		ROS_ERROR(">>Init can2 error!\n");
		error = true;
	}
	if(VCI_StartCAN(VCI_USBCAN2,0,1)!=1)
	{
		ROS_ERROR(">>Start can2 error!\n");
		error = true;
	}

	if(error)
	{
		VCI_CloseDevice(VCI_USBCAN2,0);
		return;
	}
	yhs_tool::Serial S,T,W,Q;
	//创建接收发送数据线程




	boost::thread recvdata_thread(boost::bind(&CanControl::recvData, this));
	//boost::thread senddata_thread(boost::bind(&CanControl::sendData, this));
	//boost::thread imu_thread(boost::bind(&Serial::serialMode,this));
	boost::thread  imu_thread(boost::bind(&Serial::imu_serialMode,&S));
	boost::thread  uwb_thread(boost::bind(&Serial::uwb_serialMode,&T));
	boost::thread  hw_thread(boost::bind(&Serial::hw_serialMode,&W));
	//boost::thread  laser_thread(boost::bind(&Serial::laser_serialMode,&Q));


	ros::spin();
}



double Serial::calc_distance(double x1,double y1,double x2,double y2)
{
	return sqrt(abs((x2-x1)*(x2-x1))+abs((y2-y1)*(y2-y1)));
}

void Serial::uwb_serialMode()
{
    int fd = -1;           //文件描述符，先定义一个与程序无关的值，防止fd为任意值导致程序出bug
    int err;               //返回调用函数的状态
    int len;
    int flag=0;

    bool send=false;
    
    //串口接收的数据.
    uint8_t packages[31]={0};

    const char *dev[]  = {"/dev/uwb"};
    fd = open(dev[0],O_RDWR | O_NOCTTY | O_NDELAY); //打开串口，返回文件描述符
    if(-1 == fd)
    {
        perror("Can't uwb Open Serial Port");
        return ;
    }else{
        flag=1;//Seral open permit to trans
    }

    serial_pub = nh_.advertise<nav_msgs::Odometry>("gps",1);
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

    nav_msgs::Odometry uwb_pose_msg;

    do{
        err = UART0_Init(fd,57600,0,8,2,'N');
    }while(FALSE == err || FALSE == fd);

	double uwb_last_x;
	double uwb_last_y;
	double tolerence=1.0;
	bool uwb_init=false;
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
                    printf("uwb time send %d data successful\n",len);
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
                //printf("\n receive %d data successful\n", len_receive);
            }else {
                ROS_WARN("uwb receive data failed!\n");
            }


            // int x=0;
            // int y=0;
            // vector<int> temp1;
            // vector<int> temp2;
            // bool first=true;
            //提取数据包
            //printf("new frame  \n");

			//uwb_mtx.lock();

            char src[30]={0};
			for(int i=0;i<sizeof(packages);i++)
			{
				src[i]=packages[i];
				//printf("%c ",src[i]);
			}

			int len = strlen(src);
			int k=0;
			int num[5];
			int x = 0,y = 0;
			int flag1 = 0;
			int x_fu=0,y_fu=0;
			for (int i = 0; i < len; i++)
			{
				//printf("%c",src[i]);
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
				else if((src[i] == '-')&& (flag1==0))
				{
					//ROS_ERROR("X_FU");	
					x_fu=1;
				}
				else if((src[i] == '-') && (flag1==1))
				{
					//ROS_ERROR("Y_FU");
					y_fu=1;
				}
			}
			//ROS_ERROR("xxxxxxx:%d",x);
			//ROS_ERROR("yyyyyyy:%d",y);
			if(x_fu==1) x=0-x;
		
			if(y_fu==1) y=0-y;
			
			//ROS_ERROR("x:%f",x);
			//ROS_ERROR("y:%f",y);
			uwb_x = (double)x/100;
			uwb_y = (double)y/100;
			if(!uwb_init)
			{
				uwb_last_x=uwb_x;
				uwb_last_y=uwb_y;
				uwb_init=true;
			}
			else{
				double distance=calc_distance(uwb_x,uwb_y,uwb_last_x,uwb_last_y);
				if(distance<tolerence)
				{
					uwb_last_x=uwb_x;
					uwb_last_y=uwb_y;
					tolerence=1.0;
				}
				else{
					uwb_x=uwb_last_x;
					uwb_y=uwb_last_y;
					tolerence+=0.1;
					ROS_ERROR("UWB DISTANCE ERROR");
					continue ;
				}
			}

			//uwb_mtx.unlock();


			// std::cout<<"x="<<x<<std::endl;
			// std::cout<<"y="<<y<<std::endl;
			ROS_INFO("uwb_x:%f",uwb_x);
			ROS_INFO("uwb_y:%f",uwb_y);

			
			uwb_pose_msg.header.frame_id = "map";
			uwb_pose_msg.header.stamp = ros::Time::now();
			//uwb_pose_msg.child_frame_id = "base_link";
			uwb_pose_msg.pose.pose.position.x = x;
			uwb_pose_msg.pose.pose.position.y = y;
			uwb_pose_msg.pose.pose.position.z = 0;
			uwb_pose_msg.pose.covariance={0};
			serial_pub.publish(uwb_pose_msg);

			transform.setOrigin( tf::Vector3(x, y, 0.0) );
			tf::Quaternion q;
			q.setRPY(0.0, 0.0, 0.0);
			transform.setRotation(q);
			//br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "odom"));


            
            //break;
        }
    }
    close(fd);
}


void Serial::imu_serialMode()
{
    int fd = -1;           //文件描述符，先定义一个与程序无关的值，防止fd为任意值导致程序出bug
    int err;               //返回调用函数的状态
    int len;
    int flag=0;

    bool send=false;
    	tf::TransformBroadcaster imu_broadcaster;
    //串口接收的数据.
    uint8_t packages[11]={0};

    const char *dev[]  = {"/dev/imu"};
    fd = open(dev[0],O_RDWR | O_NOCTTY | O_NDELAY); //打开串口，返回文件描述符
    if(-1 == fd)
    {
        perror("Can't Open imu Serial Port");
    }else{
        flag=1;//Seral open permit to trans
    }

    //serial_pub = nh_.advertise<yhs_can_msgs::serial>("uwb_pose",10);
    imu_pub = nh_.advertise<sensor_msgs::Imu> ("imu_data",10);
	ros::Publisher imu_yaw_pub_ = nh_.advertise<yhs_can_msgs::imu>("imu_yaw",10);
    tf::TransformBroadcaster br;
    tf::Transform transform;

    uint8_t uart0_send_buf[5];
    uart0_send_buf[0] = (uint8_t)0xFF;
    uart0_send_buf[1] = (uint8_t) 0xAA;
    uart0_send_buf[2] = (uint8_t) 0x01;
    uart0_send_buf[3] = (uint8_t) 0x04;
    uart0_send_buf[4] = (uint8_t) 0x00;
 
	geometry_msgs::Quaternion q;	
    yhs_can_msgs::serial uwb_pose_msg;
    yhs_can_msgs::imu imu_yaw;
	sensor_msgs::Imu imu_msg;
    do{
        err = UART0_Init(fd,57600,0,8,2,'N');
    }while(FALSE == err || FALSE == fd);

	double roll_zero = 0.0;
	double pitch_zero = 0.0;
	bool flag_imu = true;
    while(ros::ok())//serial fasong shuju
    {
        if (flag)
        {
            if(send==false)
            {
                //***************发送**************//
                len = UART0_Send(fd, uart0_send_buf, 5);
                if(len > 0)
                {
                    printf("imu time send %d data successful\n",len);
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
                //printf("\n receive %d data successful\n", len_receive);
            }else {
                printf("\n receive data failed!\n");
            }
            //提取数据包
    //         printf("new frame  \n");
    // for(int i=0;i<sizeof(packages);i++)
    // {
    //     printf("%02x  ",packages[i]);
    // }
    // printf("\n %02x %02x \n",packages[7],packages[6]);

    //float yaw=0.0;
	mtx.lock();

	roll=(((packages[3]<<8)&0xFF00)|((packages[2])&0x00FF))/32768.0*180.0;
	pitch=(((packages[5]<<8)&0xFF00)|((packages[4])&0x00FF))/32768.0*180.0;
    yaw =(((packages[7]<<8)&0xFF00)|((packages[6])&0x00FF))/32768.0*180.0;

    //printf("YYYYYYYYYY %f \n",yaw);
	if (flag_imu){
		roll_zero = roll;
		pitch_zero = pitch;
		flag_imu = false;
	}
	else{
		roll = (roll-roll_zero)/180*3.14;
		pitch = (pitch - pitch_zero)/180*3.14;
	}

    if(yaw>180.0)
    {
        yaw -= 360.0;
    }
	yaw = yaw/180.0*3.14159;
    q=tf::createQuaternionMsgFromRollPitchYaw(roll,pitch,yaw);
	imu_msg.orientation=q;
	imu_msg.header.frame_id="base_imu_link";
	imu_msg.header.stamp=ros::Time::now();
	imu_msg.orientation_covariance={0};
	imu_yaw.imu_yaw = yaw;
	imu_yaw.imu_pitch = pitch;
	imu_yaw.imu_roll = roll;
	imu_yaw_pub_.publish(imu_yaw);
    imu_pub.publish(imu_msg);

	mtx.unlock();

	tf::Transform imutransform;
	imutransform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
	tf::Quaternion qt;
	qt.setRPY(0.0, 0.0, 0.0);
	imutransform.setRotation(qt);
	imu_broadcaster.sendTransform(tf::StampedTransform(imutransform, ros::Time::now(), "base_footprint", "base_imu_link"));

usleep(10000);
            
        }
    }
    close(fd);

}


const unsigned char auchCRCHi[] = /* CRC��λ�ֽڱ�*/
{ 	 
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40 
} ; 

const unsigned char auchCRCLo[] = /* CRC��λ�ֽڱ�*/ 
{ 
	0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC,
	0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8, 0xD8, 
	0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 
	0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3, 0x11, 0xD1, 0xD0, 0x10, 0xF0, 
	0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 
	0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 
	0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 
	0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26, 0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 
	0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 
	0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 
	0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 
	0x74, 0x75, 0xB5, 0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 
	0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 
	0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88, 
	0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C, 0x44, 
	0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80, 0x40
} ;	 

/******************************************************************************
												    CRCУ��
*******************************************************************************/
unsigned int CRC_Calculate(unsigned char *pdata,unsigned char num)
{
  unsigned char uchCRCHi = 0xFF ;               
	unsigned char  uchCRCLo = 0xFF ;               
	unsigned char uIndex ;                
	while(num --)                    
	{
		uIndex = uchCRCHi^*pdata++ ;           
		uchCRCHi = uchCRCLo^auchCRCHi[uIndex];
		uchCRCLo = auchCRCLo[uIndex];
	}
	return (uchCRCHi << 8 | uchCRCLo) ;
}



void Serial::car_state_callback(const yhs_can_msgs::state msg)
{
	car_state=msg.state;

}



void Serial::hw_serialMode()
{
    int fd_1 = -1;           //文件描述符，先定义一个与程序无关的值，防止fd_1为任意值导致程序出bug
    int err_1;               //返回调用函数的状态
    int len_1;
    int flag_1=0;

    bool send_1=false;
    tf::TransformBroadcaster imu_broadcaster;
    //串口接收的数据.
    uint8_t packages_1[11]={0};

    const char *dev_1[]  = {"/dev/imu"};
    fd_1 = open(dev_1[0],O_RDWR | O_NOCTTY | O_NDELAY); //打开串口，返回文件描述符
    if(-1 == fd_1)
    {
        perror("Can't Open imu Serial Port");
    }else{
        flag_1=1;//Seral open permit to trans
    }

    //serial_pub = nh_.advertise<yhs_can_msgs::serial>("uwb_pose",10);
    imu_pub = nh_.advertise<sensor_msgs::Imu> ("imu_data",10);
    ros::Publisher imu_yaw_pub_ = nh_.advertise<yhs_can_msgs::imu>("imu_yaw",10);
    tf::TransformBroadcaster br;
    tf::Transform transform;

    uint8_t imu_buf[5];
    imu_buf[0] = (uint8_t)0xFF;
    imu_buf[1] = (uint8_t) 0xAA;
    imu_buf[2] = (uint8_t) 0x01;
    imu_buf[3] = (uint8_t) 0x04;
    imu_buf[4] = (uint8_t) 0x00;
 
    geometry_msgs::Quaternion q;    
    yhs_can_msgs::serial uwb_pose_msg;
    yhs_can_msgs::imu imu_yaw;
    sensor_msgs::Imu imu_msg;
    do{
        err_1 = UART0_Init(fd_1,57600,0,8,2,'N');
    }while(FALSE == err_1 || FALSE == fd_1);

    double roll_zero = 0.0;
    double pitch_zero = 0.0;
    bool flag_1_imu = true;









    int fd = -1;           //文件描述符，先定义一个与程序无关的值，防止fd为任意值导致程序出bug
    int err;               //返回调用函数的状态
    int len;
    int flag=0;

    bool in_pit_send=false,in_pit_check=false;
	bool out_pit_send=false,out_pit_check=false;
	bool pour_send=false,pour_check=false;
	bool data_init=false,init_check=false;
	bool inquire=false;
	bool pour_init=false,pour_init_check=false;
    uint8_t packages[5]={0};
    const char *dev[]  = {"/dev/hw"};
    fd = open(dev[0],O_RDWR | O_NOCTTY | O_NDELAY); //打开串口，返回文件描述符
    if(-1 == fd)
    {
        perror("Can't Open hw Serial Port");
    }else{
        flag=1;//Seral open permit to trans
		ROS_WARN("hw open success");
    }

    ros::Publisher hw_pub = nh_.advertise<yhs_can_msgs::hw>("hw_data",10);
	car_state_sub=nh_.subscribe<yhs_can_msgs::state>("state",5,&Serial::car_state_callback,this);

    uint8_t init[7];  // 总初始化
    init[0] = (uint8_t) 0x55;
    init[1] = (uint8_t) 0x04;
    init[2] = (uint8_t) 0xB0; //1200m
    init[3] = (uint8_t) 0x27; //10000ms
    init[4] = (uint8_t) 0x10; 
    init[5] = (uint8_t) 0x32; //crc h
	init[6] = (uint8_t) 0xE7; //crc l


    uint8_t uart0_send_buf[4];  // 查询
    uart0_send_buf[0] = (uint8_t) 0x50  ;
    uart0_send_buf[1] = (uint8_t) 0x01;
    uart0_send_buf[2] = (uint8_t) 0xfc;  //crc h
    uart0_send_buf[3] = (uint8_t) 0x70; //crc l

 
    uint8_t uart1_send_buf[4];  // 倒土
    uart1_send_buf[0] = (uint8_t) 0x50;
    uart1_send_buf[1] = (uint8_t) 0x02;
    uart1_send_buf[2] = (uint8_t) 0xbc; //crc h
    uart1_send_buf[3] = (uint8_t) 0x71; //crc l



	uint8_t uart2_send_buf[4];  // 
    uart2_send_buf[0] = (uint8_t) 0x53;
    uart2_send_buf[1] = (uint8_t) 0x00;  // ms
    uart2_send_buf[2] = (uint8_t) 0x00;  // ms
    uart2_send_buf[3] = (uint8_t) 0x81; //crc h
    uart2_send_buf[4] = (uint8_t) 0xD1; //crc l
	int last_state=0;

	yhs_can_msgs::hw hw_msg;

    do{
        err = UART0_Init(fd,57600,0,8,2,'N');
    }while(FALSE == err || FALSE == fd);

    while(ros::ok())//serial fasong shuju
    {
		uint8_t packages[5]={0};
        uint8_t imu_packages[5]={0};

        int imu_len = UART0_Send(fd, imu, 7);
        if(imu_len>0)
        {
            int imu_len_receive = UART0_Recv(fd, (char *)imu_packages, sizeof(imu_packages));

            if(imu_len_receive>0)
            {
                mtx.lock();

                roll=(((packages[3]<<8)&0xFF00)|((packages[2])&0x00FF))/32768.0*180.0;
                pitch=(((packages[5]<<8)&0xFF00)|((packages[4])&0x00FF))/32768.0*180.0;
                yaw =(((packages[7]<<8)&0xFF00)|((packages[6])&0x00FF))/32768.0*180.0;

                if (flag_imu){
                    roll_zero = roll;
                    pitch_zero = pitch;
                    flag_imu = false;
                }
                else{
                    roll = (roll-roll_zero)/180*3.14;
                    pitch = (pitch - pitch_zero)/180*3.14;
                }

                if(yaw>180.0)
                {
                    yaw -= 360.0;
                }
                yaw = yaw/180.0*3.14159;
                q=tf::createQuaternionMsgFromRollPitchYaw(roll,pitch,yaw);
                imu_msg.orientation=q;
                imu_msg.header.frame_id="base_imu_link";
                imu_msg.header.stamp=ros::Time::now();
                imu_msg.orientation_covariance={0};
                imu_yaw.imu_yaw = yaw;
                imu_yaw.imu_pitch = pitch;
                imu_yaw.imu_roll = roll;
                imu_yaw_pub_.publish(imu_yaw);
                imu_pub.publish(imu_msg);

                mtx.unlock();

                tf::Transform imutransform;
                imutransform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
                tf::Quaternion qt;
                qt.setRPY(0.0, 0.0, 0.0);
                imutransform.setRotation(qt);
                imu_broadcaster.sendTransform(tf::StampedTransform(imutransform, ros::Time::now(), "base_footprint", "base_imu_link"));
            }

        }

        
		if(!data_init)
		{
			len = UART0_Send(fd, init, 7);
			if(len > 0)
			{
				printf("hw time send %d data successful\n",len);
			}else{
				printf("hw send data failed!\n");
			}
			flag = fcntl(fd,F_GETFL, 0);
			flag |= O_NONBLOCK;
			fcntl(fd,F_SETFL,flag);

			usleep(20);

				int len_receive = UART0_Recv(fd, (char *)packages, sizeof(packages));
			for(int i=0;i<sizeof(packages);i++)
			{
				printf("%2x",packages[i]);
			}

				if(((uint16_t)packages[0]==0xaa) && ((uint16_t)packages[1]==0x3f) && ((uint16_t)packages[2]==0x3f) )
				{
					data_init=true;
					init_check=true;
					ROS_ERROR("A init done");
				}	
				else
				{
					ROS_ERROR("in check error");
				}
				continue;
		}

		if(car_state==0)
		{
			len = UART0_Send(fd, uart0_send_buf, 4);
			if(len > 0)
			{
				printf("hw time send %d data successful\n",len);
			}else{
				printf("hw send data failed!\n");
			}
			flag = fcntl(fd,F_GETFL, 0);
			flag |= O_NONBLOCK;
			fcntl(fd,F_SETFL,flag);

			

			int len_receive = UART0_Recv(fd, (char *)packages, sizeof(packages));

			if(len_receive)
			{
				unsigned char check[3]={0};
				for (int i=0;i<3;i++)
				{
					check[i]=(uint16_t)packages[i];
				}
				unsigned int  crc=( (((int16_t)packages[3]<<8) & 0xFF00 ) | ((uint16_t)packages[4] & 0x00FF ) );
				unsigned int  calc_crc=CRC_Calculate(check,3);
				for (int i=0;i<5;i++)
				{
					printf("%2x",packages[i]);
				}				
				if(CRC_Calculate(packages,5)==0)
				{
					int left_bias = (uint16_t)packages[1];
					int right_bias = (uint16_t)packages[2];
					hw_msg.v1=left_bias;
					hw_msg.v2=right_bias;
					hw_pub.publish(hw_msg);
				}
				else
				{
					ROS_ERROR("hw receive data error");
				}

				
			}
			usleep(20000);
			continue;
		}


		if(car_state==2)
		{
			if(!pour_init)
			{
				len = UART0_Send(fd, uart1_send_buf, 4);
				if(len > 0)
				{
					printf("hw time send %d data successful\n",len);
				}else{
					printf("hw send data failed!\n");
				}
				flag = fcntl(fd,F_GETFL, 0);
				flag |= O_NONBLOCK;
				fcntl(fd,F_SETFL,flag);

				
				int len_receive = UART0_Recv(fd, (char *)packages, sizeof(packages));

				usleep(20000);
				if(((uint16_t)packages[0]==0xaa) && ((uint16_t)packages[1]==0x3f) && ((uint16_t)packages[2]==0x3f) )
				{
					pour_init=true;
					pour_init_check=true;
					ROS_ERROR("pour init done");
				}	
				else
				{
					ROS_ERROR("pour check error");
				}
				continue;
			}
			else{

				int len_receive = UART0_Recv(fd, (char *)packages, sizeof(packages));

				if(len_receive!=0)
				{
					if(CRC_Calculate(packages,5)==0)
					{
						
                        if((uint16_t)packages[1]==1)
						{
							ROS_WARN("pour 1");

						}
                        if((uint16_t)packages[1]==2)
						{
							ROS_WARN("pour 2");

						}
						if((uint16_t)packages[1]==3)
						{
							ROS_WARN("pour done");
							sleep(5);
							pour_init_check=false;
							pour_init=false;
						}
					}
					else
					{
						ROS_ERROR("hw receive data check error");
					}
				}
                else{
                    ROS_ERROR("hw receive data error");
                }
				usleep(20000);
			}

			
		}





        if (flag_1)
        {
            if(send_1==false)
            {
                //***************发送**************//
                len_1 = UART0_Send(fd_1, imu_buf, 5);
                if(len_1 > 0)
                {
                    printf("imu time send_1 %d data successful\n",len_1);
                }else{
                    printf("send_1 data failed!\n");
                }

                flag_1 = fcntl(fd_1,F_GETFL, 0);
                flag_1 |= O_NONBLOCK;
                fcntl(fd_1,F_SETFL,flag_1);
                send_1=true;
            }
            //***************接收**************//
            int len_1_receive = UART0_Recv(fd_1, (char *)packages_1, sizeof(packages_1));

            if (len_1_receive > 0)
            {
                //printf("\n receive %d data successful\n", len_1_receive);
            }else {
                printf("\n receive data failed!\n");
            }
            mtx.lock();

            roll=(((packages_1[3]<<8)&0xFF00)|((packages_1[2])&0x00FF))/32768.0*180.0;
            pitch=(((packages_1[5]<<8)&0xFF00)|((packages_1[4])&0x00FF))/32768.0*180.0;
            yaw =(((packages_1[7]<<8)&0xFF00)|((packages_1[6])&0x00FF))/32768.0*180.0;
            if (flag_1_imu){
                roll_zero = roll;
                pitch_zero = pitch;
                flag_1_imu = false;
            }
            else{
                roll = (roll-roll_zero)/180*3.14;
                pitch = (pitch - pitch_zero)/180*3.14;
            }

            if(yaw>180.0)
            {
                yaw -= 360.0;
            }
            yaw = yaw/180.0*3.14159;
            q=tf::createQuaternionMsgFromRollPitchYaw(roll,pitch,yaw);
            imu_msg.orientation=q;
            imu_msg.header.frame_id="base_imu_link";
            imu_msg.header.stamp=ros::Time::now();
            imu_msg.orientation_covariance={0};
            imu_yaw.imu_yaw = yaw;
            imu_yaw.imu_pitch = pitch;
            imu_yaw.imu_roll = roll;
            imu_yaw_pub_.publish(imu_yaw);
            imu_pub.publish(imu_msg);

            mtx.unlock();

            tf::Transform imutransform;
            imutransform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
            tf::Quaternion qt;
            qt.setRPY(0.0, 0.0, 0.0);
            imutransform.setRotation(qt);
            imu_broadcaster.send_1Transform(tf::StampedTransform(imutransform, ros::Time::now(), "base_footprint", "base_imu_link"));            
        }
            








        }
    
		
	
    close(fd);
    close(fd_1);
}


void Serial::laser_serialMode()
{
    int fd = -1;           //文件描述符，先定义一个与程序无关的值，防止fd为任意值导致程序出bug
    int err;               //返回调用函数的状态
    int len;
    int flag=0;

    bool send=false,read=false;
    	tf::TransformBroadcaster imu_broadcaster;
    //串口接收的数据.
    

    const char *dev[]  = {"/dev/laser_distance"};
    fd = open(dev[0],O_RDWR | O_NOCTTY | O_NDELAY); //打开串口，返回文件描述符
    if(-1 == fd)
    {
        perror("Can't Open laser Serial Port");
    }else{
        flag=1;//Seral open permit to trans
		ROS_WARN("laser open success");
    }

    ros::Publisher hw_pub = nh_.advertise<yhs_can_msgs::hw>("hw_data",10);
    // imu_pub = nh_.advertise<sensor_msgs::Imu> ("imu_data",10);
	// ros::Publisher imu_yaw_pub_ = nh_.advertise<yhs_can_msgs::imu>("imu_yaw",10);
    // tf::TransformBroadcaster br;
    // tf::Transform transform;

    uint8_t uart0_send_buf[8];
    uart0_send_buf[0] = (uint8_t)0x50;
    uart0_send_buf[1] = (uint8_t) 0x06;
    uart0_send_buf[2] = (uint8_t) 0x00;
    uart0_send_buf[3] = (uint8_t) 0x38;
    uart0_send_buf[4] = (uint8_t) 0x01;
	uart0_send_buf[4] = (uint8_t) 0xC4;
	uart0_send_buf[4] = (uint8_t) 0x46;

	uint8_t uart_send_buf[8];
	uart_send_buf[0]=(uint8_t)0x50;
	uart_send_buf[1]=(uint8_t)0x03;
	uart_send_buf[2]=(uint8_t)0x00;
	uart_send_buf[3]=(uint8_t)0x34;
	uart_send_buf[4]=(uint8_t)0x00;
	uart_send_buf[5]=(uint8_t)0x01;
	uart_send_buf[6]=(uint8_t)0xC8;
	uart_send_buf[7]=(uint8_t)0x45;

	yhs_can_msgs::hw hw_msg;
    // yhs_can_msgs::serial uwb_pose_msg;
    // yhs_can_msgs::imu imu_yaw;
	// sensor_msgs::Imu imu_msg;
    do{
        err = UART0_Init(fd,57600,0,8,2,'N');
    }while(FALSE == err || FALSE == fd);

    while(ros::ok())//serial fasong shuju
    {
        if (flag)
        {

			//***************发送**************//

			len = UART0_Send(fd, uart_send_buf, 8);
			if(len > 0)
			{
				printf("laser time send %d data successful\n",len);
			}else{
				printf("laser send data failed!\n");
			}

			flag = fcntl(fd,F_GETFL, 0);
			flag |= O_NONBLOCK;
			fcntl(fd,F_SETFL,flag);
			uint8_t packages[7]={0};
            //***************接收**************//
            int len_receive = UART0_Recv(fd, (char *)packages, sizeof(packages));
 
            char src[7]={0};
			for(int i=0;i<sizeof(packages);i++)
			{
				//src[i]=packages[i];
				printf("%2x ",packages[i]);
			}
			printf("\n");

			if((uint16_t)packages[0]==0x50)
			{
				int distance1=( (((int16_t)packages[3]<<8) & 0xFF00 ) | ((uint16_t)packages[4] & 0x00FF ) ) ;

				printf("dddddddd: %d\n",distance1);

			}
			usleep(10);
			// int len = strlen(src);		3
			// int k=0;
			// int num[5];
			// int x = 0;
			// for (int i = 0; i < len; i++)
			// 		{
			// 			if(src[i] >= '0' && src[i] <= '9'){
			// 				k++;
			// 				num[k] = src[i] - '0'; 
			// 			}
			// 		}
			// for(int s=1 ; s<k ;s++){
			// 	x = 10*x + num[s];
			// }
			// k = 0;
			// printf("laser::::%d \n",x);



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
    //ROS_INFO("fs_sel = %d\n",fs_sel);
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

}



//主函数
int main(int argc, char ** argv)
{
	ros::init(argc, argv, "yhs_can_control_node");

	yhs_tool::CanControl cancontrol;
	cancontrol.run();

	return 0;
}
