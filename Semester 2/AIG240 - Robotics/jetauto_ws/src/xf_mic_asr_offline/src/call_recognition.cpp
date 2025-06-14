/**************************************************************************
功能：录音调用控制器，包含休眠功能(function: recording call controller, including sleep function)
**************************************************************************/
#include <user_interface.h>
#include <string>
#include <locale>
#include <codecvt>
#include <ctime>
#include <ros/ros.h>
#include <signal.h>
#include <std_msgs/String.h>
#include <xf_mic_asr_offline/Get_Offline_Result_srv.h>
#include <xf_mic_asr_offline/Pcm_Msg.h>
#include <xf_mic_asr_offline/Start_Record_srv.h>
#include <xf_mic_asr_offline/Set_Awake_Word_srv.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int32.h>
#include <sys/stat.h>
#include <iostream>
#include <geometry_msgs/Twist.h>
 
using namespace std;
int awake_flag = 0;    //唤醒标志位(awake flag)
int confidence_threshold ;
int seconds_per_order ;
int recognize_fail_count = 0;  //被动休眠相关变量(passive sleep related variables)
/**************************************************************************
函数功能：唤醒标志sub回调函数(function feature: awake flag sub callback function)
入口参数：唤醒标志位awake_flag_msg  voice_control.cpp(Entry parameter: wake up flag bit awake_flag_msg)
返回  值：无(return value: none)
**************************************************************************/
void awake_flag_Callback(std_msgs::Int8 msg)
{
	awake_flag = msg.data;
	//printf("awake_flag=%d\n",awake_flag);
	recognize_fail_count = 0;
}

void shutdown(int sig)
{
  // Do some custom action.
  // For example, publish a stop message to some other nodes.
  
  // All the default sigint handler does is call shutdown()
  printf("close...");
  ros::shutdown();
}

/**************************************************************************
函数功能：主函数(function feature: main function)
入口参数：无(Entry parameter: none)
返回  值：无(Return value: none)
**************************************************************************/
int main(int argc, char *argv[])
{
	//int temp=0;
	   
	int recognize_fail_count_threshold = 15;    //识别失败次数阈值(recognition failure threshold)
	
	string str1 = "ok";				            //语音识别相关字符串(voice recognition relative string)
	string str2 = "fail";				        //语音识别相关字符串(voice recognition relative string)
	string str3 = "休眠(Sleep)";				//语音识别相关字符串(voice recognition relative string)
	string str4 = "失败5次(Fail-5-times)";		//语音识别相关字符串(voice recognition relative string)
	string str5 = "失败10次(Fail-10-times)";	//语音识别相关字符串(voice recognition relative string)

	ros::init(argc, argv, "call_rec", ros::init_options::NoSigintHandler);    //初始化ROS节点(initialize ROS node)
	ros::NodeHandle nh("~");    //创建句柄(create handle)

    signal(SIGINT, shutdown);

	/***离线命令词识别服务客服端创建(create offline voice command recognition service client)***/
	ros::ServiceClient get_offline_recognise_result_client = 
	nh.serviceClient<xf_mic_asr_offline::Get_Offline_Result_srv>("/xf_asr_offline_node/get_offline_recognise_result_srv");
	
	/***唤醒标志位话题订阅者创建(awake flag topic subscriber creating)***/
	ros::Subscriber awake_flag_sub = nh.subscribe("/awake_flag", 1, awake_flag_Callback);

	/***唤醒标志位话题发布者创建(awake flag topic publisher creating)***/
	ros::Publisher awake_flag_pub = nh.advertise<std_msgs::Int8>("/awake_flag", 1);

	/***离线命令词识别结果话题发布者创建(offline voice command recognition result topic publisher creating)***/
	ros::Publisher control = nh.advertise<std_msgs::String>("/voice_words", 1);

	ros::Rate loop_rate(10);   //循环频率10Hz(circulation frequency 10Hz)

	nh.param("confidence", confidence_threshold, 18);
	nh.param("seconds_per_order", seconds_per_order, 3);

    /***离线命令词识别服务参数设置(offline voice command service parameter setting)***/
	xf_mic_asr_offline::Get_Offline_Result_srv GetOfflineResult_srv;
	GetOfflineResult_srv.request.offline_recognise_start = 1;
	GetOfflineResult_srv.request.confidence_threshold = confidence_threshold;
	GetOfflineResult_srv.request.time_per_order = seconds_per_order;

	while(ros::ok())
	{
		if(awake_flag)    //判断休眠状态还是唤醒状态(judge sleep status or awake status)
		{
		    //请求离线命令词识别服务并返回应答为调用成功(request for offline voice command recognition service and return answer, call successfully.)
			if(get_offline_recognise_result_client.call(GetOfflineResult_srv))
			{
				//ROS_INFO("succeed to call service \"get_offline_recognise_result_srv\"!");
				//打印识别结果、置信度、识别命令词等信息(print recognition result, confidence, recognition voice command and other information)
				//std::cout << "result: " << GetOfflineResult_srv.response.result << endl;
				//std::cout << "fail reason: " << GetOfflineResult_srv.response.fail_reason << endl;
				//std::cout << "text: " << GetOfflineResult_srv.response.text << endl;

				if(str3 == GetOfflineResult_srv.response.text)    //主动休眠(active sleep)
				{ 
					awake_flag=0;
					recognize_fail_count = 0;
				}
				else if(str1 == GetOfflineResult_srv.response.result)    //清零被动休眠相关变量(clear passive sleep relative variable)
				{
					awake_flag=0;
					recognize_fail_count = 0;
                }
				else if(str2 == GetOfflineResult_srv.response.result)    //记录识别失败次数(record the number of recognition failures)
				{
					recognize_fail_count++;
					//printf("recognize_fail_count=%d\n",recognize_fail_count);
					if(recognize_fail_count==5)  //连续识别失败5次，用户界面显示提醒信息(fail to recognize for consecutive 5 times. Warning occurs on user interface)
					{
						std_msgs::String count_msg;
						count_msg.data = str4;
						control.publish(count_msg);
					}
					else if(recognize_fail_count==10)  //连续识别失败10次，用户界面显示提醒信息(fail to recognize for consecutive 10 times. Warning occurs on user interface)
					{
						std_msgs::String count_msg;
						count_msg.data = str5;
						control.publish(count_msg);
					}
					else if(recognize_fail_count >= recognize_fail_count_threshold)    //被动休眠(passive sleep)
					{
						awake_flag=0;
						std_msgs::String controloff_msg;
						controloff_msg.data = str3;
						control.publish(controloff_msg);
						recognize_fail_count = 0;
					}
				}
			}
			else   //请求离线命令词识别服务并返回应答为调用失败(request for offline voice command recognition service and return answer, call failed)
			{
				ROS_INFO("failed to call service \"get_offline_recognise_result_srv\"!");
				//awake_flag=0;
				continue;
			}
		}		
		//printf("awake_flag=%d\n",awake_flag);
		//printf("-----confidence_threshold =%d\n",confidence_threshold);
		//printf("seconds_per_order =%d\n",seconds_per_order); 
		ros::spinOnce();    
		loop_rate.sleep();   //10Hz循环(10Hz loop)
	}
}
