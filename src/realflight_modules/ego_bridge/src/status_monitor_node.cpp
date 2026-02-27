/**
 * @file status_monitor_node.cpp
 * @brief EGO 飞行状态监视器 — 终端实时仪表盘。
 *
 * 独立 ROS 节点，纯订阅 Topic，不依赖 ego_api 库。
 * 在终端以 4Hz 刷新一个中文面板，显示：
 *   - FSM 状态链（当前状态高亮）
 *   - 控制权归属（EGO / OVERRIDE）
 *   - 实时位姿（位置 + 偏航 + 速度）
 *   - 目标点与到达状态
 *   - EGO 规划指令内容
 *   - Override 控制指令 + 任务 ID
 *   - Topic 健康状态（超时告警）
 *
 * 自动检测终端是否支持 ANSI 颜色（isatty），不支持则纯文本输出。
 *
 * 启动方式（在另一个终端窗口）：
 *   roslaunch ego_bridge ego_bridge_monitor.launch
 */

#include <ros/ros.h>
#include <cstdio>
#include <cmath>
#include <string>
#include <unistd.h>  // isatty, fileno

// 消息类型
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int32.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>

// ═══════════════════════════════════════════════════════════════
//  ANSI 颜色码定义
//  如果终端不支持（如输出重定向到文件），所有颜色码设为空字符串
// ═══════════════════════════════════════════════════════════════
struct Colors {
    const char* reset;
    const char* bold;
    const char* dim;           // 暗灰色
    const char* red;
    const char* green;
    const char* yellow;
    const char* cyan;
    const char* white;
    const char* bg_green;      // 绿色背景（用于高亮当前状态）
    const char* bg_red;        // 红色背景
    const char* bg_cyan;       // 青色背景

    // 根据终端能力初始化颜色
    void init(bool use_color) {
        if (use_color) {
            reset    = "\033[0m";
            bold     = "\033[1m";
            dim      = "\033[2m";
            red      = "\033[0;31m";
            green    = "\033[0;32m";
            yellow   = "\033[0;33m";
            cyan     = "\033[0;36m";
            white    = "\033[0;37m";
            bg_green = "\033[42;97m";   // 绿底白字
            bg_red   = "\033[41;97m";   // 红底白字
            bg_cyan  = "\033[46;30m";   // 青底黑字
        } else {
            // 无色模式 — 所有颜色码为空
            reset = bold = dim = red = green = yellow = cyan = white = "";
            bg_green = bg_red = bg_cyan = "";
        }
    }
};

// ═══════════════════════════════════════════════════════════════
//  面板常量
// ═══════════════════════════════════════════════════════════════
static const int PANEL_W = 70;  // 面板内容宽度（不含边框符号）

// ═══════════════════════════════════════════════════════════════
//  监视器节点类
// ═══════════════════════════════════════════════════════════════
class StatusMonitor {
public:
    StatusMonitor(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
        // 自动检测终端颜色支持
        bool use_color = isatty(fileno(stdout));
        clr_.init(use_color);
        use_color_ = use_color;

        // ── 订阅 Topic ──
        sub_flight_state_ = nh.subscribe("/ego_bridge/flight_state", 10,
                                         &StatusMonitor::flightStateCb, this);
        sub_control_mode_ = nh.subscribe("/ego_bridge/control_mode", 10,
                                         &StatusMonitor::controlModeCb, this);
        sub_reach_status_ = nh.subscribe("/ego_bridge/reach_status", 10,
                                         &StatusMonitor::reachStatusCb, this);
        sub_odom_         = pnh.subscribe("odom", 10,
                                         &StatusMonitor::odomCb, this);
        sub_cmd_          = pnh.subscribe("cmd", 10,
                                         &StatusMonitor::cmdCb, this);
        sub_override_cmd_ = nh.subscribe("/ego_bridge/override_cmd", 10,
                                         &StatusMonitor::overrideCmdCb, this);
        sub_trigger_      = nh.subscribe("/ego_api/override_trigger", 10,
                                         &StatusMonitor::triggerCb, this);

        // 记录启动时间
        start_time_ = ros::Time::now();

        // 首次清屏
        if (use_color_) {
            printf("\033[2J");  // 清屏
        }

        // 4Hz 定时器驱动渲染
        timer_ = nh.createTimer(ros::Duration(0.25), &StatusMonitor::render, this);
    }

private:
    Colors clr_;
    bool   use_color_;
    ros::Time start_time_;
    ros::Timer timer_;

    // ── 订阅者 ──
    ros::Subscriber sub_flight_state_, sub_control_mode_, sub_reach_status_;
    ros::Subscriber sub_odom_, sub_cmd_, sub_override_cmd_, sub_trigger_;

    // ── 缓存数据 ──
    std::string flight_state_ = "---";       // 当前 FSM 状态字符串
    uint8_t     control_mode_ = 0;           // 0=EGO, 1=OVERRIDE
    uint8_t     reach_status_ = 0;           // 0=飞行中, 1=已到达
    int         task_id_      = 0;           // 最近的 override 任务 ID

    // 里程计
    double odom_x_ = 0, odom_y_ = 0, odom_z_ = 0;
    double odom_yaw_ = 0;                   // 弧度
    double speed_ = 0;                       // 合速度 m/s

    // EGO 指令
    double cmd_px_ = 0, cmd_py_ = 0, cmd_pz_ = 0;
    double cmd_vx_ = 0, cmd_vy_ = 0, cmd_vz_ = 0;
    double cmd_yaw_ = 0;
    uint32_t cmd_traj_id_ = 0;
    uint8_t  cmd_traj_flag_ = 0;

    // Override 指令
    double ovr_px_ = 0, ovr_py_ = 0, ovr_pz_ = 0;
    double ovr_yaw_ = 0;

    // Topic 时间戳（用于健康检测）
    ros::Time t_state_ = ros::Time(0);
    ros::Time t_odom_  = ros::Time(0);
    ros::Time t_cmd_   = ros::Time(0);
    ros::Time t_ovr_   = ros::Time(0);

    // ── 回调函数 ──
    void flightStateCb(const std_msgs::String::ConstPtr& msg) {
        flight_state_ = msg->data;
        t_state_ = ros::Time::now();
    }

    void controlModeCb(const std_msgs::UInt8::ConstPtr& msg) {
        control_mode_ = msg->data;
    }

    void reachStatusCb(const std_msgs::UInt8::ConstPtr& msg) {
        reach_status_ = msg->data;
    }

    void odomCb(const nav_msgs::Odometry::ConstPtr& msg) {
        odom_x_ = msg->pose.pose.position.x;
        odom_y_ = msg->pose.pose.position.y;
        odom_z_ = msg->pose.pose.position.z;

        // 四元数 → yaw
        double qx = msg->pose.pose.orientation.x;
        double qy = msg->pose.pose.orientation.y;
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;
        odom_yaw_ = std::atan2(2.0 * (qw * qz + qx * qy),
                               1.0 - 2.0 * (qy * qy + qz * qz));

        double vx = msg->twist.twist.linear.x;
        double vy = msg->twist.twist.linear.y;
        double vz = msg->twist.twist.linear.z;
        speed_ = std::sqrt(vx * vx + vy * vy + vz * vz);

        t_odom_ = ros::Time::now();
    }

    void cmdCb(const quadrotor_msgs::PositionCommand::ConstPtr& msg) {
        cmd_px_ = msg->position.x;  cmd_py_ = msg->position.y;  cmd_pz_ = msg->position.z;
        cmd_vx_ = msg->velocity.x;  cmd_vy_ = msg->velocity.y;  cmd_vz_ = msg->velocity.z;
        cmd_yaw_ = msg->yaw;
        cmd_traj_id_   = msg->trajectory_id;
        cmd_traj_flag_ = msg->trajectory_flag;
        t_cmd_ = ros::Time::now();
    }

    void overrideCmdCb(const quadrotor_msgs::PositionCommand::ConstPtr& msg) {
        ovr_px_ = msg->position.x;  ovr_py_ = msg->position.y;  ovr_pz_ = msg->position.z;
        ovr_yaw_ = msg->yaw;
        t_ovr_ = ros::Time::now();
    }

    void triggerCb(const std_msgs::Int32::ConstPtr& msg) {
        task_id_ = msg->data;
    }

    // ═══════════════════════════════════════════════════════════
    //  画线工具
    // ═══════════════════════════════════════════════════════════

    // 顶部边框: ╔══════...══════╗
    void printTop() {
        printf("%s\u2554", clr_.cyan);
        for (int i = 0; i < PANEL_W; i++) printf("\u2550");
        printf("\u2557%s\n", clr_.reset);
    }

    // 底部边框: ╚══════...══════╝
    void printBot() {
        printf("%s\u255a", clr_.cyan);
        for (int i = 0; i < PANEL_W; i++) printf("\u2550");
        printf("\u255d%s\n", clr_.reset);
    }

    // 分隔线: ╠══════...══════╣
    void printSep() {
        printf("%s\u2560", clr_.cyan);
        for (int i = 0; i < PANEL_W; i++) printf("\u2550");
        printf("\u2563%s\n", clr_.reset);
    }

    // 打印一行内容，自动左对齐并补齐到 PANEL_W 宽度
    // fmt 是 printf 格式串，内容不含边框符号
    void printRow(const char* content) {
        // content 可能含 ANSI 码，需要计算可见宽度来对齐
        // 简化处理：直接打印，右侧边框换行
        printf("%s\u2551%s %s", clr_.cyan, clr_.reset, content);
        // 用换行 + 光标右移模拟右边框（简化方案：不严格对齐）
        printf("\n");
    }

    // 打印一行，带边框和内容居中
    void printCentered(const char* content) {
        printf("%s\u2551%s %s\n", clr_.cyan, clr_.reset, content);
    }

    // ═══════════════════════════════════════════════════════════
    //  渲染主函数 — 每 0.25s 调用一次
    // ═══════════════════════════════════════════════════════════
    void render(const ros::TimerEvent& /*e*/) {
        ros::Time now = ros::Time::now();
        double uptime = (now - start_time_).toSec();
        int up_min = (int)(uptime / 60.0);
        int up_sec = (int)uptime % 60;

        char buf[512];

        // 光标归位（不清屏，避免闪烁）
        if (use_color_) {
            printf("\033[H");
        }

        // ──────── 标题 ────────
        printTop();
        snprintf(buf, sizeof(buf),
            " %s%s  EGO \u98de\u884c\u72b6\u6001\u76d1\u89c6\u5668  %s"
            "                          "
            "%s\u8fd0\u884c %02d:%02d%s",
            clr_.bold, clr_.cyan, clr_.reset,
            clr_.dim, up_min, up_sec, clr_.reset);
        printRow(buf);
        printSep();

        // ──────── FSM 状态链 ────────
        renderStateChain(buf, sizeof(buf));
        printRow(buf);
        printSep();

        // ──────── 控制权 ────────
        if (control_mode_ == 1) {
            snprintf(buf, sizeof(buf),
                " %s%s \u2588 OVERRIDE \u63a5\u7ba1\u4e2d %s"
                "    \u4efb\u52a1 ID: %s%s%d%s",
                clr_.bold, clr_.bg_red, clr_.reset,
                clr_.bold, clr_.yellow,
                task_id_, clr_.reset);
        } else {
            snprintf(buf, sizeof(buf),
                " %s%s \u2588 EGO \u81ea\u4e3b\u5bfc\u822a %s"
                "            ",
                clr_.bold, clr_.bg_green, clr_.reset);
        }
        printRow(buf);
        printSep();

        // ──────── 位姿信息 ────────
        double yaw_deg = odom_yaw_ * 180.0 / M_PI;
        snprintf(buf, sizeof(buf),
            " %s\u4f4d\u7f6e%s  X:%s%+7.2f%s  Y:%s%+7.2f%s  Z:%s%+7.2f%s",
            clr_.bold, clr_.reset,
            clr_.green, odom_x_, clr_.reset,
            clr_.green, odom_y_, clr_.reset,
            clr_.green, odom_z_, clr_.reset);
        printRow(buf);

        snprintf(buf, sizeof(buf),
            " %s\u59ff\u6001%s  \u504f\u822a:%s%+6.1f\u00b0%s"
            "    \u901f\u5ea6:%s%5.2f%s m/s",
            clr_.bold, clr_.reset,
            clr_.yellow, yaw_deg, clr_.reset,
            clr_.cyan, speed_, clr_.reset);
        printRow(buf);
        printSep();

        // ──────── 目标点 + 到达状态 ────────
        renderTarget(buf, sizeof(buf));
        printRow(buf);
        printSep();

        // ──────── EGO 指令 ────────
        double cmd_yaw_deg = cmd_yaw_ * 180.0 / M_PI;
        snprintf(buf, sizeof(buf),
            " %sEGO\u6307\u4ee4%s pos(%s%.1f%s,%s%.1f%s,%s%.1f%s)"
            " vel(%s%.1f%s,%s%.1f%s,%s%.1f%s)"
            " yaw:%s%.0f\u00b0%s traj#%s%u%s",
            clr_.bold, clr_.reset,
            clr_.white, cmd_px_, clr_.reset,
            clr_.white, cmd_py_, clr_.reset,
            clr_.white, cmd_pz_, clr_.reset,
            clr_.dim, cmd_vx_, clr_.reset,
            clr_.dim, cmd_vy_, clr_.reset,
            clr_.dim, cmd_vz_, clr_.reset,
            clr_.yellow, cmd_yaw_deg, clr_.reset,
            clr_.cyan, cmd_traj_id_, clr_.reset);
        printRow(buf);

        // ──────── Override 指令（仅 OVERRIDE 模式显示内容） ────────
        if (control_mode_ == 1) {
            double ovr_yaw_deg = ovr_yaw_ * 180.0 / M_PI;
            snprintf(buf, sizeof(buf),
                " %sOVR\u6307\u4ee4%s pos(%s%.1f%s,%s%.1f%s,%s%.1f%s)"
                " yaw:%s%.0f\u00b0%s"
                "  \u4efb\u52a1ID:%s%s%d%s",
                clr_.bold, clr_.reset,
                clr_.white, ovr_px_, clr_.reset,
                clr_.white, ovr_py_, clr_.reset,
                clr_.white, ovr_pz_, clr_.reset,
                clr_.yellow, ovr_yaw_deg, clr_.reset,
                clr_.bold, clr_.yellow, task_id_, clr_.reset);
        } else {
            snprintf(buf, sizeof(buf),
                " %sOVR\u6307\u4ee4%s %s---  (\u672a\u63a5\u7ba1)%s",
                clr_.bold, clr_.reset,
                clr_.dim, clr_.reset);
        }
        printRow(buf);
        printSep();

        // ──────── Topic 健康 ────────
        renderHealth(buf, sizeof(buf), now);
        printRow(buf);
        printBot();

        // 底部提示
        printf(" %s%s[Ctrl+C \u9000\u51fa]  roslaunch ego_bridge ego_bridge_monitor.launch%s\n",
               clr_.dim, clr_.white, clr_.reset);

        fflush(stdout);
    }

    // ═══════════════════════════════════════════════════════════
    //  渲染子函数
    // ═══════════════════════════════════════════════════════════

    // FSM 状态链：当前状态高亮，其余暗灰
    void renderStateChain(char* buf, size_t sz) {
        // 6 个状态名和对应缩写
        const char* names[]  = {"IDLE", "PRE_OFFBOARD", "TAKEOFF", "HOVER", "TRACKING", "LANDING"};
        const char* labels[] = {"\u5f85\u673a", "\u9884\u8fde\u63a5",
                                "\u8d77\u98de", "\u60ac\u505c",
                                "\u8ddf\u8e2a", "\u964d\u843d"};
        const char* arrows[] = {" \u2192 ", " \u2192 ", " \u2192 ", " \u21c4 ", " \u2192 ", ""};

        // 构建字符串
        int pos = 0;
        pos += snprintf(buf + pos, sz - pos, " ");

        for (int i = 0; i < 6; i++) {
            bool active = (flight_state_ == names[i]);
            if (active) {
                pos += snprintf(buf + pos, sz - pos,
                    "%s \u2605 %s %s", clr_.bg_green, labels[i], clr_.reset);
            } else {
                pos += snprintf(buf + pos, sz - pos,
                    "%s \u00b7 %s %s", clr_.dim, labels[i], clr_.reset);
            }
            if (i < 5) {
                pos += snprintf(buf + pos, sz - pos, "%s%s%s",
                    clr_.dim, arrows[i], clr_.reset);
            }
        }
    }

    // 目标点 + 距离 + 到达状态
    void renderTarget(char* buf, size_t sz) {
        if (t_state_ == ros::Time(0)) {
            snprintf(buf, sz,
                " %s\u76ee\u6807%s  %s\u7b49\u5f85\u8fde\u63a5 ego_bridge ...%s",
                clr_.bold, clr_.reset, clr_.dim, clr_.reset);
            return;
        }

        // 计算到目标点的距离（如果有 EGO 指令的话，用 cmd 的 position 作为当前目标）
        double dx = cmd_px_ - odom_x_;
        double dy = cmd_py_ - odom_y_;
        double dz = cmd_pz_ - odom_z_;
        double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

        const char* status_icon;
        const char* status_color;
        if (reach_status_ == 1) {
            status_icon  = "\u2705 \u5df2\u5230\u8fbe";
            status_color = clr_.green;
        } else {
            status_icon  = "\u23f3 \u98de\u884c\u4e2d";
            status_color = clr_.yellow;
        }

        snprintf(buf, sz,
            " %s\u76ee\u6807%s  (%s%.2f%s, %s%.2f%s, %s%.2f%s)"
            "  \u8ddd\u79bb:%s%.2f%sm  %s%s%s",
            clr_.bold, clr_.reset,
            clr_.white, cmd_px_, clr_.reset,
            clr_.white, cmd_py_, clr_.reset,
            clr_.white, cmd_pz_, clr_.reset,
            clr_.cyan, dist, clr_.reset,
            status_color, status_icon, clr_.reset);
    }

    // Topic 健康状态
    void renderHealth(char* buf, size_t sz, const ros::Time& now) {
        int pos = 0;
        pos += snprintf(buf + pos, sz - pos, " %s\u5065\u5eb7%s ", clr_.bold, clr_.reset);

        // odom
        pos += renderTopicAge(buf + pos, sz - pos, "odom", t_odom_, now);
        pos += snprintf(buf + pos, sz - pos, "  ");

        // cmd (EGO)
        pos += renderTopicAge(buf + pos, sz - pos, "cmd", t_cmd_, now);
        pos += snprintf(buf + pos, sz - pos, "  ");

        // state (bridge)
        pos += renderTopicAge(buf + pos, sz - pos, "state", t_state_, now);
        pos += snprintf(buf + pos, sz - pos, "  ");

        // override (仅在 OVERRIDE 时检测)
        if (control_mode_ == 1) {
            pos += renderTopicAge(buf + pos, sz - pos, "ovr", t_ovr_, now);
        } else {
            pos += snprintf(buf + pos, sz - pos, "%sovr:--%s", clr_.dim, clr_.reset);
        }
    }

    // 单个 Topic 的延迟显示
    int renderTopicAge(char* buf, size_t sz, const char* name,
                       const ros::Time& stamp, const ros::Time& now)
    {
        if (stamp == ros::Time(0)) {
            return snprintf(buf, sz, "%s%s:%s\u274c%s",
                            clr_.dim, name, clr_.red, clr_.reset);
        }
        double age = (now - stamp).toSec();
        if (age > 1.0) {
            // 超时 — 红色警告
            return snprintf(buf, sz, "%s:%.1fs%s\u26a0%s",
                            name, age, clr_.red, clr_.reset);
        } else {
            // 正常 — 绿色对勾
            return snprintf(buf, sz, "%s:%.2fs%s\u2714%s",
                            name, age, clr_.green, clr_.reset);
        }
    }
};

// ═══════════════════════════════════════════════════════════════
//  Main
// ═══════════════════════════════════════════════════════════════
int main(int argc, char** argv) {
    ros::init(argc, argv, "status_monitor_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    StatusMonitor monitor(nh, pnh);

    ros::spin();
    return 0;
}
