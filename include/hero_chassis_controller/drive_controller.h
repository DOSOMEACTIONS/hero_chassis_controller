// Copyright 2026 heifeiyang. All rights reserved.

#ifndef HERO_CHASSIS_CONTROLLER_DRIVE_CONTROLLER_H
#define HERO_CHASSIS_CONTROLLER_DRIVE_CONTROLLER_H

// ===== 1. 用到的类型，各自住在哪个头文件里 =====
#include <control_toolbox/pid.h>  // ⭐ 官方 PID 实现（作业要求用它）
#include <controller_interface/controller.h>  // Controller 基类
#include <geometry_msgs/Twist.h>              // /cmd_vel 的消息类型
#include <hardware_interface/joint_command_interface.h>  // EffortJointInterface / JointHandle
#include <nav_msgs/Odometry.h>           // /odom 的消息类型
#include <ros/ros.h>                     // ros::Time / ros::NodeHandle
#include <std_msgs/Float64MultiArray.h>  // 给 rqt_plot 画的曲线数据
#include <tf/transform_broadcaster.h>    // 广播 odom→base_link（TF）
#include <tf/transform_listener.h>  // ⭐ 查变换（世界系速度模式要用）

#include <string>  // std::string
#include <vector>

namespace hero_chassis_controller
{
// ===== 2. 类的形状：继承一个"模板化的基类"，然后填 4 个函数 =====
// <hardware_interface::EffortJointInterface> 这个尖括号里写什么，
//   决定了 init() 收到的 hw 是什么类型，也决定了 setCommand() 的单位（力矩）
class DriveController : public controller_interface::Controller<
                            hardware_interface::EffortJointInterface>
{
 public:
  // 这 4 个函数由框架来调（不是你去调它们）
  // override 的意思是："我在重写基类里的同名虚函数，编译器帮我检查签名"
  bool init(hardware_interface::EffortJointInterface* hw,
            ros::NodeHandle& nh) override;
  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void stopping(const ros::Time& time) override;

  // 收到 /cmd_vel 时被调用（这就是"键盘控制"的入口）
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);

 private:
  // ===== 3. 类成员：整个生命周期里跨函数共享的数据 =====
  std::vector<hardware_interface::JointHandle> joints_;  // 4 个轮子的"把手"
  double effort_{0.0};      // 开多大劲（N·m），从参数读
  double duration_{3.0};    // 开多少秒，从参数读
  ros::Time start_time_;    // starting() 里记下的起点
  double left_sign_{1.0};   // 左轮方向系数（+1 / -1）
  double right_sign_{1.0};  // 右轮方向系数
  // ——— 下面四个是「逆运动学 + 比例控制」需要的 ———
  double wheel_radius_{0.07625};  // 轮子半径（m），从 URDF 读出来
  double k_kin_{0.4};  // (轮距 + 轮基) / 2，麦轮转向项系数（m）
  // ⭐ 四个轮子【各一个】PID（作业要求：一个 PID 对应一个轮子）
  control_toolbox::Pid pid_[4];

  // ——— 给 rqt_plot / rqt_reconfigure 用的可视化通道 ———
  ros::Publisher speed_pub_;  // /drive/wheel_speed   [fl, fr, rl, rr] 实际转速
  ros::Publisher target_pub_;  // /drive/wheel_target  [fl, fr, rl, rr] 目标转速
  int vis_cnt_{0};             // 降频计数器（1kHz → 50Hz）

  // ——— TF 广播降频（1kHz → 50Hz）———
  //   ⚠️ TF 不需要 1kHz：正常频率 30-50Hz 就够
  //   ⚠️ 1kHz 时同一毫秒内会广播 2-3 次，时间戳重复 → tf2 报 TF_REPEATED_DATA
  int tf_cnt_{0};
  ros::Time last_tf_stamp_;  // 上一帧广播用的时间戳（去重用）

  // ——— 里程计（odom）———
  ros::Publisher odom_pub_;  // /odom 的发布者
  double odom_x_{0.0};       // 车在世界坐标里的 x（米）
  double odom_y_{0.0};       // 车在世界坐标里的 y（米）
  double odom_yaw_{0.0};     // 车头朝向（弧度，逆时针为正）

  ros::Subscriber cmd_sub_;  // /cmd_vel 的订阅者
  double cmd_vx_{0.0};       // 最近一次收到的"前进速度 m/s"（x 轴）
  double cmd_vy_{0.0};       // ⭐ 横向速度 m/s（麦轮专属，正 = 向左平移）
  double cmd_wz_{0.0};       // 最近一次收到的"转向角速度 rad/s"

  // ——— ⭐ 特色功能：加速度限制 ———
  //   max_accel_* = 0 表示不限制（保持原来的"瞬间响应"）
  //   物理意义：车体速度每秒最多变化多少
  double max_accel_linear_{0.0};   // m/s²，线速度最大变化率
  double max_accel_angular_{0.0};  // rad/s²，角速度最大变化率
  double smooth_vx_{0.0};          // 斜坡限制后的 vx（真正的执行值）
  double smooth_vy_{0.0};          // 斜坡限制后的 vy
  double smooth_wz_{0.0};          // 斜坡限制后的 wz

  // ⭐ 辅助：把 cur 朝 target 移动，一步最多 max_step
  //    这就是"速率限幅"（slew rate limit）的全部逻辑
  static double MoveToward(double cur, double target, double max_step);

  // ——— ⭐ 世界坐标系速度模式（选做 7）———
  //    frame_mode = "base"  /cmd_vel 直接当作【车体系】速度（默认，最简单）
  //    frame_mode = "world" /cmd_vel 当作【世界系】速度，先用 tf 转到车体系
  std::string frame_mode_{"base"};
  std::string world_frame_{"odom"};  // 世界坐标系的名字（"odom" 或 "map"）
  bool world_mode_{false};           // frame_mode == "world" 时为 true
  tf::TransformListener tf_listener_;  // 查 odom→base_link 的变换
  double last_vx_base_{0.0};  // 上一拍成功转换后的车体系 vx（tf 失败时保底）
  double last_vy_base_{0.0};  // 上一拍成功转换后的车体系 vy
};

}  // namespace hero_chassis_controller

#endif  // HERO_CHASSIS_CONTROLLER_DRIVE_CONTROLLER_H
