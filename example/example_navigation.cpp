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

// 接收UDP数据
void Custom::UDPRecv()
{
  udp.Recv();
}

// 发送UDP数据
void Custom::UDPSend()
{
  udp.Send();
}

// 控制机器人前进的函数
void Custom::MoveForward(float speed, float height, int gaitType)
{
  cmd.mode = 2;               // 步行模式
  cmd.gaitType = gaitType;     // 步态类型
  cmd.velocity[0] = speed;     // 设置前进速度
  cmd.bodyHeight = height;     // 设置身体高度
  cmd.footRaiseHeight = 0.1;   // 设置步幅高度
  cmd.yawSpeed = 0;            // 不转弯
}

// 控制机器人转弯的函数
void Custom::Turn(float yawSpeed)
{
  cmd.mode = 2;               // 保持步行模式
  cmd.gaitType = 2;           // 设置步态类型（任意值，保持现有步态）
  cmd.velocity[0] = 0.0f;     // 不进行前进
  cmd.yawSpeed = yawSpeed;    // 设置转弯速度
}

// 主要控制逻辑，调用前进和转弯函数
void Custom::RobotControl()
{
  motiontime += 2;
  udp.GetRecv(state); // 获取机器人当前状态

  // 初始化命令为站立模式
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
    MoveForward(0.4f, 0.0f, 2); // 在指定时间段内前进
  }
  if (motiontime > 20000 && motiontime < 22000)
  {
    Turn(2); // 在指定时间段内右转
  }
  if (motiontime > 22000 && motiontime < 26000)
  {
    MoveForward(0.2f, 0.1f, 1); // 在指定时间段内缓慢前进，身体抬高
  }
  if (motiontime > 26000 && motiontime < 27000)
  {
    cmd.mode = 1; // 进入特殊模式
  }

  udp.SetSend(cmd); // 发送控制命令
}

// 获取图像并显示，根据 saveToDisk 参数决定是否保存到本地
void getAndDisplayFrame(UnitreeCamera& cam, bool saveToDisk = false, const std::string& savePath = "./frame.jpg") {
    while (cam.isOpened()) {
        cv::Mat left, right;
        if (!cam.getRectStereoFrame(left, right)) { ///< 获取左、右校正帧
            usleep(1000);
            continue;
        }

        cv::Mat stereo;
        cv::hconcat(left, right, stereo);   ///< 将左、右图像拼接为一张图
        cv::flip(stereo, stereo, -1);       ///< 翻转图像
        cv::imshow("Longlat_Rect", stereo); ///< 显示图像

        if (saveToDisk) {                   ///< 如果需要保存图像到本地
            cv::imwrite(savePath, stereo);  ///< 保存图像到指定路径
            std::cout << "Frame saved to: " << savePath << std::endl;
        }

        char key = cv::waitKey(10);         ///< 监听按键
        if (key == 27)                      ///< 按下 ESC 键退出
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
