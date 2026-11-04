// Copyright 2026 heifeiyang. All rights reserved.

#include "hero_chassis_controller/drive_controller.h"

#include <pluginlib/class_list_macros.h>  // PLUGINLIB_EXPORT_CLASS 住在这里

#include <string>
#include <vector>

namespace hero_chassis_controller
{
// ======================= init：一次性准备 =======================
bool DriveController::init(hardware_interface::EffortJointInterface* hw,
                           ros::NodeHandle& nh)
{
  // ① 从参数服务器读"四个轮子叫什么"
  std::vector<std::string> names;
  if (!nh.getParam("joints", names) || names.empty())
  {
    ROS_ERROR("[drive] 参数 joints 为空，放弃加载");
    return false;
  }

  // ② 读控制参数：劲多大、开多久、左右方向
  nh.param<double>("effort", effort_, 0.5);
  nh.param<double>("duration", duration_, 3.0);
  nh.param<double>("left_sign", left_sign_, 1.0);
  nh.param<double>("right_sign", right_sign_, 1.0);

  // ③ 按名字要"把手"（拿不到会抛异常）
  for (const auto& n : names)
  {
    try
    {
      joints_.push_back(hw->getHandle(n));
      ROS_INFO("[drive] 拿到关节: %s", n.c_str());
    }
    catch (const std::exception& e)
    {
      ROS_ERROR("[drive] 关节 %s 不存在: %s", n.c_str(), e.what());
      return false;
    }
  }
  // ④ 读运动学与比例控制参数
  nh.param<double>("wheel_radius", wheel_radius_, 0.07625);
  nh.param<double>("k_kin", k_kin_, 0.4);

  // ⭐ 速度模式（选做 7）：
  //    "base"  = /cmd_vel 直接当作【车体系】速度（默认）
  //    "world" = /cmd_vel 当作【世界系】速度，先用 tf 转到车体系
  // ⭐ 特色功能：加速度限制（0 = 不限制）
  nh.param<double>("max_accel_linear", max_accel_linear_, 0.0);
  nh.param<double>("max_accel_angular", max_accel_angular_, 0.0);
  if (max_accel_linear_ > 0.0 || max_accel_angular_ > 0.0)
  {
    ROS_INFO("[drive] 加速度限制: 线 %.2f m/s^2, 角 %.2f rad/s^2",
             max_accel_linear_, max_accel_angular_);
  }
  else
  {
    ROS_INFO("[drive] 加速度限制: 关闭（瞬间响应）");
  }

  nh.param<std::string>("frame_mode", frame_mode_, std::string("base"));
  nh.param<std::string>("world_frame", world_frame_, std::string("odom"));
  world_mode_ = (frame_mode_ == "world");
  if (!world_mode_ && frame_mode_ != "base")
  {
    ROS_WARN("[drive] frame_mode='%s' 不认识，按 base 处理（可选 base/world）",
             frame_mode_.c_str());
    frame_mode_ = "base";
  }
  ROS_INFO("[drive] 速度模式: %s（世界坐标系名: %s）",
           world_mode_ ? "world 全局坐标系" : "base 底盘坐标系",
           world_frame_.c_str());
  // ⭐⭐ 初始化四个 PID（一个轮子一个，作业要求）
  //     参数从 <nh>/<关节名>_pid/{p, i, d, i_clamp} 读
  for (size_t i = 0; i < joints_.size() && i < 4; ++i)
  {
    const std::string pid_ns = joints_[i].getName() + "_pid";
    ros::NodeHandle pid_nh(nh, pid_ns);  // 开子命名空间
    if (!pid_[i].init(pid_nh))
    {
      ROS_ERROR("[drive] 读不到 %s 的 PID 参数", pid_ns.c_str());
      return false;
    }
    // ⭐ 注意：不用再调 initDynamicReconfig()
    //    因为 control_toolbox 的 init() 内部【已经】初始化了 dynamic
    //    reconfigure （见 pid.h 的注释："Initializes dynamic reconfigure for
    //    PID gains"） 再调一次会报 "Tried to advertise a service that is
    //    already advertised"
    ROS_INFO("[drive] %s 的 PID 就绪", pid_ns.c_str());
  }

  // ④ 创建可视化话题（rqt_plot 用）
  speed_pub_ =
      nh.advertise<std_msgs::Float64MultiArray>("/drive/wheel_speed", 10);
  target_pub_ =
      nh.advertise<std_msgs::Float64MultiArray>("/drive/wheel_target", 10);

  // ④ 创建 /odom 的发布者（队列 50，防止卡顿时丢消息）
  odom_pub_ = nh.advertise<nav_msgs::Odometry>("/odom", 50);

  // ④ 订阅 /cmd_vel（键盘控制节点会往这个话题发消息）
  cmd_sub_ =
      nh.subscribe("/cmd_vel", 1, &DriveController::cmdVelCallback,
                   this);  // ⭐ 绝对名，否则会被拼到 /controller/<名字>/ 下

  ROS_INFO("[drive] init 成功: %zu 个轮子, effort=%.2f, duration=%.1f 秒",
           joints_.size(), effort_, duration_);
  return true;
}

// ======================= starting：记起点 =======================
// ⭐ 速率限幅：把 cur 朝 target 推，一步最多 max_step
double DriveController::MoveToward(double cur, double target, double max_step)
{
  const double d = target - cur;
  if (d > max_step)
  {
    return cur + max_step;
  }
  if (d < -max_step)
  {
    return cur - max_step;
  }
  return target;
}

void DriveController::starting(const ros::Time& time)
{
  start_time_ = time;  // ⭐ 用框架给的时间（仿真时间），不用 ros::Time::now()
  ROS_INFO("[drive] 开始，计时 0 / %.1f 秒", duration_);
}

// ======================= 收到 /cmd_vel 时的回调 =======================
void DriveController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  cmd_vx_ = msg->linear.x;   // 前进速度（m/s），正数 = 向前
  cmd_vy_ = msg->linear.y;   // ⭐ 横移速度（m/s），正数 = 向左（麦轮专属）
  cmd_wz_ = msg->angular.z;  // 转向角速度（rad/s），正数 = 左转

  // ⭐ 只在【指令真的变了】时打印一次。
  //    teleop 按住不放会以 ~3.5 Hz 重复发同样的值，
  //    如果每帧都 ROS_INFO，日志刷屏 + stdout 块缓冲 → 看起来"半天没反应"
  static double last_vx = 1e9, last_wz = 1e9;
  const bool changed = (std::abs(cmd_vx_ - last_vx) > 1e-6) ||
                       (std::abs(cmd_wz_ - last_wz) > 1e-6);
  if (changed)
  {
    ROS_INFO("[drive] cmd_vel changed: vx=%.2f m/s, wz=%.2f rad/s", cmd_vx_,
             cmd_wz_);
    last_vx = cmd_vx_;
    last_wz = cmd_wz_;
  }
}

// ======================= update：每 1ms 一次 =======================
void DriveController::update(const ros::Time& time, const ros::Duration& period)
{
  // ⭐ dt = 距上次 update 过了多久（ros_control 每 1ms 调一次 → 约 0.001 秒）
  //    积分、微分都要用它，所以放在最前面
  const double dt = (period.toSec() > 1e-9) ? period.toSec() : 0.001;

  // ============ 第一步：逆运动学（cmd_vel → 四个轮子的目标转速）============
  //
  // 输入：
  //     cmd_vx_  前进速度 m/s（+ 向前）
  //     cmd_wz_  转向角速度 rad/s（+ 左转）
  //     vy       横移速度 m/s（+ 向左）—— 先固定 0，键盘控制用不上
  //
  // 公式来源：麦轮四个轮子的速度是「平移」和「转动」的叠加。
  //   每个轮子的贡献 = 平移量 ± 转动量
  //   转动量 = wz × k_kin（k = (轮距+轮基)/2，也就是轮子到车心的平均距离）
  //   最后除以轮半径，把「线速度 m/s」换成「角速度 rad/s」
  //
  // ⚠️ 这个公式的【符号约定】是从教科书抄的，
  //    对不对必须靠实验验证：按 i 车往前走 = 对；往后退 = 整体乘 -1。
  // ⭐⭐⭐ 选做 7：世界坐标系速度 → 底盘坐标系速度 ⭐⭐⭐
  //
  //    base 模式：/cmd_vel 就是车体系速度，直接用
  //    world 模式：/cmd_vel 是世界系（odom/map）速度
  //                车头转了 90° 后，"往世界 +x 走" 在车上等于"往左平移"
  //                → 必须用 tf 把【向量】转到 base_link 系再送进逆运动学
  //
  //    用的就是 requirement 给的 transformVector：
  //        世界系向量 --tf--> 车体系向量
  double vx = cmd_vx_;
  double vy = cmd_vy_;
  double wz = cmd_wz_;  // 两套坐标系都是 z 朝上 → 绕 z 的角速度不变

  if (world_mode_)
  {
    geometry_msgs::Vector3Stamped v_in, v_out;
    v_in.header.frame_id = world_frame_;  // "odom" 或 "map"
    v_in.header.stamp = ros::Time(0);     // 0 = 用最新的变换
    v_in.vector.x = cmd_vx_;
    v_in.vector.y = cmd_vy_;
    v_in.vector.z = 0.0;

    try
    {
      // ⭐ 把"世界系里的速度向量"变换到 base_link 系
      //    transformVector 只管方向+大小，不管位置（正合我们意）
      tf_listener_.transformVector("base_link", v_in, v_out);
      vx = v_out.vector.x;
      vy = v_out.vector.y;
      last_vx_base_ = vx;
      last_vy_base_ = vy;
    }
    catch (const tf::TransformException& e)
    {
      // 变换还没准备好（刚启动那几百毫秒）→ 用上一拍的值，别突然乱动
      ROS_WARN_THROTTLE(2.0, "[drive] world 模式：tf 变换失败（%s），沿用上一拍",
                        e.what());
      vx = last_vx_base_;
      vy = last_vy_base_;
    }
  }

  // ⭐⭐⭐ 特色功能：加速度限制（速率限幅 / slew rate limit）⭐⭐⭐
  //
  //  问题：teleop 一按下 i，目标从 0 直接跳到 6.56 rad/s
  //        → PID 立刻输出 3.28 N*m（超过打滑阈值约 2 N*m）
  //        → 轮子打滑，里程计立刻开始飘，机械也受冲击载荷
  //
  //  修法：不许"目标"一步跳过去，每拍最多变化 max_accel × dt
  //        → 目标变成一条斜坡 → PID 跟着斜坡走 → 力矩平缓
  //
  //  ⚠️ 限制的是【目标 value】，不是【PID 输出】：
  //     限制目标 → 误差自然变小 → PID 自己就会温和下来
  //     限制输出 → 会把 PID 正常的调节也削掉 → 反而更不稳
  //
  //  ⚠️ 这也是"减速限制"：松手后目标从 6.56 慢慢降到 0，
  //     所以车会按设定减速度滑行 —— 物理上合理（真车也这样）
  if (max_accel_linear_ > 0.0)
  {
    const double max_dv = max_accel_linear_ * dt;  // 这一拍最多变多少 m/s
    smooth_vx_ = MoveToward(smooth_vx_, vx, max_dv);
    smooth_vy_ = MoveToward(smooth_vy_, vy, max_dv);
    vx = smooth_vx_;
    vy = smooth_vy_;
  }
  if (max_accel_angular_ > 0.0)
  {
    const double max_dw = max_accel_angular_ * dt;  // 这一拍最多变多少 rad/s
    smooth_wz_ = MoveToward(smooth_wz_, wz, max_dw);
    wz = smooth_wz_;
  }

  const double w_fl = (vx - vy - k_kin_ * wz) / wheel_radius_;  // 左前
  const double w_fr = (vx + vy + k_kin_ * wz) / wheel_radius_;  // 右前
  const double w_rl = (vx + vy - k_kin_ * wz) / wheel_radius_;  // 左后
  const double w_rr = (vx - vy + k_kin_ * wz) / wheel_radius_;  // 右后

  // ⚠️ 顺序必须和参数里 joints 的写法一致：[左前, 右前,
  // 左后, 右后]
  const double target[4] = {w_fl, w_fr, w_rl, w_rr};

  // ============ 第二步：比例控制（P）============
  //
  //   误差 = 目标转速 - 实际转速
  //   力矩 = kp × 误差
  //
  // 这就是 PID 里的 P。没有 I（消不掉稳态误差），没有 D（会稍微超调），
  // 但足够让轮子"朝着目标转速走"。
  for (size_t i = 0; i < joints_.size() && i < 4; ++i)
  {
    const double actual = joints_[i].getVelocity();  // rad/s
    const double err = target[i] - actual;
    // ⭐ 用官方 control_toolbox::Pid（P/I/D 三项由它内部算，I 项自带限幅）
    double effort = pid_[i].computeCommand(err, period);

    // 安全限幅：不能超过 URDF 里 transmission 的 max_effort(5)
    if (effort > 5.0) effort = 5.0;
    if (effort < -5.0) effort = -5.0;

    // 方向系数：如果某个轮子装反了，用 left_sign_ / right_sign_ 纠
    const double sign = (i % 2 == 0) ? left_sign_ : right_sign_;
    joints_[i].setCommand(effort * sign);
  }

  // ============ 可视化：发实际/目标轮速给 rqt_plot ============
  // ⚠️ update 每 1ms 一次，1kHz 直接发会把 rqt_plot 压垮
  //    → 每 20 拍发一次 = 50 Hz，画曲线足够
  if (++vis_cnt_ % 20 == 0)
  {
    std_msgs::Float64MultiArray speed_msg, target_msg;
    speed_msg.data.resize(4);
    target_msg.data.resize(4);
    for (size_t i = 0; i < 4 && i < joints_.size(); ++i)
    {
      speed_msg.data[i] = joints_[i].getVelocity();  // 实际
      target_msg.data[i] = target[i];                // 目标
    }
    speed_pub_.publish(speed_msg);
    target_pub_.publish(target_msg);
  }

  // ============ 第三步：正运动学（四个轮速 → 车体速度）============
  //
  // 逆运动学是「车体速度 → 四轮转速」，这里是反过程。
  // 用【实际转速】而不是目标转速 —— 因为里程计要反映"车真的走了多少"。
  //
  // 推导（把逆运动学四式相加/相减）：
  //     vx = ( w_fl + w_fr + w_rl + w_rr) * r / 4
  //     vy = (-w_fl + w_fr + w_rl - w_rr) * r / 4
  //     wz = (-w_fl + w_fr - w_rl + w_rr) * r / (4 * k)
  double act[4];
  for (size_t i = 0; i < 4 && i < joints_.size(); ++i)
  {
    act[i] = joints_[i].getVelocity();
  }
  const double body_vx =
      (act[0] + act[1] + act[2] + act[3]) * wheel_radius_ / 4.0;
  const double body_vy =
      (-act[0] + act[1] + act[2] - act[3]) * wheel_radius_ / 4.0;
  const double body_wz =
      (-act[0] + act[1] - act[2] + act[3]) * wheel_radius_ / (4.0 * k_kin_);

  // ============ 第四步：积分成位姿 ============
  //
  // ⚠️ 关键：body_vx
  // 是"车头方向"的速度，不是世界坐标系的速度
  //    车头转了 90° 之后，往前开就不是 +x 了
  //    → 要用 yaw 把"车身速度"旋转到"世界速度"
  const double c = std::cos(odom_yaw_);
  const double s = std::sin(odom_yaw_);
  odom_x_ += (body_vx * c - body_vy * s) * dt;
  odom_y_ += (body_vx * s + body_vy * c) * dt;
  odom_yaw_ += body_wz * dt;

  // ============ 第五步：发布 /odom ============
  nav_msgs::Odometry odom;
  odom.header.stamp = ros::Time::now();
  odom.header.frame_id = "odom";      // 这个世界叫 odom
  odom.child_frame_id = "base_link";  // 车身坐标系叫 base_link

  // ① 位置（姿态用四元数，这里只绕 z 转，所以手工算）
  odom.pose.pose.position.x = odom_x_;
  odom.pose.pose.position.y = odom_y_;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = std::sin(odom_yaw_ / 2.0);
  odom.pose.pose.orientation.w = std::cos(odom_yaw_ / 2.0);

  // ② 速度（车身坐标系下的速度）
  odom.twist.twist.linear.x = body_vx;
  odom.twist.twist.linear.y = body_vy;
  odom.twist.twist.linear.z = 0.0;
  odom.twist.twist.angular.x = 0.0;
  odom.twist.twist.angular.y = 0.0;
  odom.twist.twist.angular.z = body_wz;

  odom_pub_.publish(odom);

  // ============ 第六步：广播 TF（odom → base_link）============
  //
  // /odom 是"发一条消息"，/tf 是"持续广播坐标系树"。
  // 两者都要发（ROS 的规矩），上层组件靠 TF 做坐标变换。
  //
  // ⚠️ static：这个变量只创建一次，之后每拍复用。
  //    如果写成局部变量，每 1ms 构造一个新的 broadcaster → 疯狂申请连接。
  static tf::TransformBroadcaster tf_br;

  // ⭐⭐ TF 降频到 50Hz + 跳过重复时间戳
  //
  //  为什么必须降频：
  //    ① update() 每 1ms 调一次 = 1000Hz，但 TF 正常只要 30-50Hz
  //    ② 1ms 内可能连续调 2-3 次，ros::Time::now() 拿到的却是同一个时刻
  //       → 同一帧、同一时间戳广播多次
  //       → tf2 收到后打印 TF_REPEATED_DATA 并丢掉多余的（功能不受影响，但刷屏）
  //
  //  ⭐ 用【框架给的 time】（仿真时间），不是 ros::Time::now()（墙上时间）
  //     仿真暂停时 now() 还在走，会插出错误的时间戳
  if (++tf_cnt_ % 20 == 0 && time != last_tf_stamp_)
  {
    last_tf_stamp_ = time;

    tf::Transform tf_pose;
    tf_pose.setOrigin(tf::Vector3(odom_x_, odom_y_, 0.0));        // 位置
    tf_pose.setRotation(tf::createQuaternionFromYaw(odom_yaw_));  // 朝向

    // 参数说明：变换 / 时间戳 / 父坐标系 / 子坐标系
    tf_br.sendTransform(tf::StampedTransform(tf_pose, time, "odom", "base_link"));
  }

  // 每 1 秒打一次当前状态（THROTTLE 避免刷屏）
  ROS_INFO_THROTTLE(
      1.0,
      "[drive] cmd(vx=%.2f wz=%.2f) target_left=%.1f target_right=%.1f "
      "actual_fl=%.1f actual_fr=%.1f",
      vx, wz, w_fl, w_fr, joints_[0].getVelocity(), joints_[1].getVelocity());
  ROS_INFO_THROTTLE(
      1.0, "[drive] odom: x=%.2f y=%.2f yaw=%.1f°  v=(%.2f, %.2f)", odom_x_,
      odom_y_, odom_yaw_ * 180.0 / M_PI, body_vx, body_vy);
}

// ======================= stopping：收工 =======================
void DriveController::stopping(const ros::Time&)
{
  for (auto& j : joints_) j.setCommand(0.0);  // 保险：停下时一定松油门
  ROS_INFO("[drive] 停止");
}

}  // namespace hero_chassis_controller

// ===== 出厂登记：告诉 pluginlib "这个类可以被实例化" =====
PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::DriveController,
                       controller_interface::ControllerBase)
