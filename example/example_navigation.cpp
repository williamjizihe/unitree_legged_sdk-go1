/**
  * @file example_navigation.cpp
  * @brief This file is an example of controlling the robot to move forward and turn
  * @details This example that how to get depth frame
  * @author Zihe Ji
  * @date  2024.10.16
  */

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <UnitreeCameraSDK.hpp>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <iostream>
#include <unistd.h>
#include <string.h>

using namespace UNITREE_LEGGED_SDK;

class Custom
{
public:
  Custom(uint8_t level) : safe(LeggedType::Go1),
                          udp(level, 8090, "192.168.123.161", 8082)
  {
    udp.InitCmdData(cmd);
  }
  void UDPRecv();
  void UDPSend();
  void RobotControl();
  void MoveForward(float speed, float height, int gaitType);
  void Turn(float yawSpeed);

  Safety safe;
  UDP udp;
  HighCmd cmd = {0};
  HighState state = {0};
  int motiontime = 0;
  float dt = 0.002; // 0.001~0.01
};

// ����UDP����
void Custom::UDPRecv()
{
  udp.Recv();
}

// ����UDP����
void Custom::UDPSend()
{
  udp.Send();
}

// ���ƻ�����ǰ���ĺ���
void Custom::MoveForward(float speed, float height, int gaitType)
{
  cmd.mode = 2;               // ����ģʽ
  cmd.gaitType = gaitType;     // ��̬����
  cmd.velocity[0] = speed;     // ����ǰ���ٶ�
  cmd.bodyHeight = height;     // ��������߶�
  cmd.footRaiseHeight = 0.1;   // ���ò����߶�
  cmd.yawSpeed = 0;            // ��ת��
}

// ���ƻ�����ת��ĺ���
void Custom::Turn(float yawSpeed)
{
  cmd.mode = 2;               // ���ֲ���ģʽ
  cmd.gaitType = 2;           // ���ò�̬���ͣ�����ֵ���������в�̬��
  cmd.velocity[0] = 0.0f;     // ������ǰ��
  cmd.yawSpeed = yawSpeed;    // ����ת���ٶ�
}

// ��Ҫ�����߼�������ǰ����ת�亯��
void Custom::RobotControl()
{
  motiontime += 2;
  udp.GetRecv(state); // ��ȡ�����˵�ǰ״̬

  // ��ʼ������Ϊվ��ģʽ
  cmd.mode = 0;
  cmd.gaitType = 0;
  cmd.speedLevel = 0;
  cmd.footRaiseHeight = 0;
  cmd.bodyHeight = 0;
  cmd.euler[0] = 0;
  cmd.euler[1] = 0;
  cmd.euler[2] = 0;
  cmd.velocity[0] = 0.0f;
  cmd.velocity[1] = 0.0f;
  cmd.yawSpeed = 0.0f;
  cmd.reserve = 0;

  if (motiontime > 16000 && motiontime < 20000)
  {
    MoveForward(0.4f, 0.0f, 2); // ��ָ��ʱ�����ǰ��
  }
  if (motiontime > 20000 && motiontime < 22000)
  {
    Turn(2); // ��ָ��ʱ�������ת
  }
  if (motiontime > 22000 && motiontime < 26000)
  {
    MoveForward(0.2f, 0.1f, 1); // ��ָ��ʱ����ڻ���ǰ��������̧��
  }
  if (motiontime > 26000 && motiontime < 27000)
  {
    cmd.mode = 1; // ��������ģʽ
  }

  udp.SetSend(cmd); // ���Ϳ�������
}

// ��ȡͼ����ʾ������ saveToDisk ���������Ƿ񱣴浽����
void getAndDisplayFrame(UnitreeCamera& cam, bool saveToDisk = false, const std::string& savePath = "./frame.jpg") {
    while (cam.isOpened()) {
        cv::Mat left, right;
        if (!cam.getRectStereoFrame(left, right)) { ///< ��ȡ����У��֡
            usleep(1000);
            continue;
        }

        cv::Mat stereo;
        cv::hconcat(left, right, stereo);   ///< ������ͼ��ƴ��Ϊһ��ͼ
        cv::flip(stereo, stereo, -1);       ///< ��תͼ��
        cv::imshow("Longlat_Rect", stereo); ///< ��ʾͼ��

        if (saveToDisk) {                   ///< �����Ҫ����ͼ�񵽱���
            cv::imwrite(savePath, stereo);  ///< ����ͼ��ָ��·��
            std::cout << "Frame saved to: " << savePath << std::endl;
        }

        char key = cv::waitKey(10);         ///< ��������
        if (key == 27)                      ///< ���� ESC ���˳�
            break;
    }
}

int main(void)
{
  std::cout << "Communication level is set to HIGH-level." << std::endl
            << "WARNING: Make sure the robot is standing on the ground." << std::endl
            << "Press Enter to continue..." << std::endl;
  std::cin.ignore();

  Custom custom(HIGHLEVEL);
  LoopFunc loop_control("control_loop", custom.dt, boost::bind(&Custom::RobotControl, &custom));
  LoopFunc loop_udpSend("udp_send", custom.dt, 3, boost::bind(&Custom::UDPSend, &custom));
  LoopFunc loop_udpRecv("udp_recv", custom.dt, 3, boost::bind(&Custom::UDPRecv, &custom));

  loop_udpSend.start();
  loop_udpRecv.start();
  loop_control.start();

  while (1)
  {
    sleep(10);
  };

  return 0;
}
