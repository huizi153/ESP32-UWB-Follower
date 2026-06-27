/*
 * go2_uwb_follower.cpp
 * =====================
 * UWB 引导的 Go2 仿狗跟随控制器 (C++ / ROS2)
 *
 * 架构:
 *   ESP32 串口 (~15Hz) -> 本节点 -> /api/sport/request -> Go2
 *
 * 三个关键设计:
 *   1. 指令限频 -- 每 200ms 发布一条 Move, 期间多帧平均 (防抽搐)
 *   2. 数据平滑 -- 尖峰过滤 -> 帧间平均 -> 低通滤波, 三层去抖
 *   3. 死区防抖 -- 距离 +/-15cm / 角度 +/-8deg 死区, 抑制 UWB 浮动
 *
 * 编译 & 运行: 见 README.md
 */

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

/* POSIX serial (Linux) */
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <errno.h>

/* ROS2 */
#include "rclcpp/rclcpp.hpp"
#include "unitree_api/msg/request.hpp"

using namespace std::chrono_literals;

/* ================================================================
 *  Go2 Sport API ID mapping (matches go2_driver)
 * ================================================================ */
constexpr int32_t API_ID_BALANCESTAND = 1002;
constexpr int32_t API_ID_STOPMOVE     = 1003;
constexpr int32_t API_ID_STANDUP      = 1004;
constexpr int32_t API_ID_STANDDOWN    = 1005;
constexpr int32_t API_ID_SIT          = 1006;
constexpr int32_t API_ID_RISESIT      = 1007;
constexpr int32_t API_ID_MOVE         = 1008;

/* ================================================================
 *  Configuration parameters
 * ================================================================ */
namespace cfg {
    /* -- Following distance -- */
    constexpr float TARGET_DISTANCE_CM   = 100.0f;   /* ideal standoff 1m          */
    constexpr float DIST_DEADZONE_CM     =  15.0f;   /* distance dead zone          */
    constexpr float ANGLE_DEADZONE_DEG   =   8.0f;   /* angle dead zone            */

    /* -- PID gains (distance PID, angle pure P) -- */
    constexpr float KP_DIST  = 0.006f;    /* P: m/s per cm err         */
    constexpr float KI_DIST  = 0.0003f;   /* I: steady-state correct   */
    constexpr float KD_DIST  = 0.002f;    /* D: dampen overshoot       */
    constexpr float KP_ANGLE = 0.025f;    /* P: rad/s per deg err      */

    /* -- Speed limits -- */
    constexpr float MAX_FWD_SPEED  =  0.8f;
    constexpr float MAX_BACK_SPEED = -0.4f;
    constexpr float MAX_TURN_SPEED =  0.8f;
    constexpr float MAX_ACCEL      =  0.5f;   /* m/s^2, anti-jerk      */

    /* -- Signal loss -- */
    constexpr double SIGNAL_TIMEOUT_SEC = 3.0;
    constexpr double SEARCH_DURATION_SEC = 8.0;
    constexpr float  SEARCH_TURN_SPEED  = 0.4f;

    /* -- Filter / de-jitter -- */
    constexpr float FILTER_ALPHA   = 0.15f;   /* distance LP coefficient    */
    constexpr float AZI_ALPHA      = 0.22f;   /* angle LP coefficient       */
    constexpr float SPIKE_LIMIT_CM = 50.0f;   /* spike rejection threshold  */

    /* -- Command pacing -- */
    constexpr auto  CMD_INTERVAL = 200ms;     /* Move publish interval       */
    constexpr auto  MAIN_LOOP    = 10ms;      /* main loop period            */

    /* -- Serial -- */
    constexpr int   SERIAL_BAUD    = 115200;
    constexpr int   BUF_SIZE       = 256;
    constexpr int   ACCUM_MAX      = 8;       /* max frames per command      */
    constexpr int   NO_DATA_THRESH = 5;       /* consecutive empty reads     */

    /* -- Recovery: frames needed to exit search -- */
    constexpr int   RECOVERY_FRAMES = 3;
}

/* ================================================================
 *  Frame accumulator
 * ================================================================ */
class FrameAccum {
    int   dist_sum_   = 0;
    int   azi_sum_    = 0;
    int   count_      = 0;
    float dist_prev_  = 0.0f;
public:
    void reset() { dist_sum_=0; azi_sum_=0; count_=0; }

    /* Add frame (with spike detection). Returns false if dropped. */
    bool add(int dist, int azi, int* dropped) {
        if (count_ > 0) {
            float jump = std::fabs((float)dist - dist_prev_);
            if (jump > cfg::SPIKE_LIMIT_CM) {
                if (dropped) *dropped += 1;
                return false;
            }
        }
        dist_prev_ = (float)dist;
        dist_sum_ += dist;
        azi_sum_  += azi;
        count_++;
        return true;
    }

    bool ready() const { return count_ > 0; }
    int  count() const { return count_; }

    void get_avg(float* dist, float* azi) const {
        if (count_ > 0) {
            *dist = (float)dist_sum_ / (float)count_;
            *azi  = (float)azi_sum_  / (float)count_;
        } else { *dist=0; *azi=0; }
    }
};

/* ================================================================
 *  Low-pass filter
 * ================================================================ */
class LowPassFilter {
    float alpha_;
    float value_ = 0.0f;
    bool  ready_ = false;
public:
    explicit LowPassFilter(float a) : alpha_(a) {}

    float update(float raw) {
        if (!ready_) { value_=raw; ready_=true; }
        else         { value_ += alpha_ * (raw - value_); }
        return value_;
    }
    float get() const { return value_; }
    void  reset() { value_=0; ready_=false; }
};

/* ================================================================
 *  Serial port (Linux POSIX)
 * ================================================================ */
class SerialPort {
    int  fd_ = -1;
public:
    ~SerialPort() { close(); }

    bool open(const std::string& port, int baud) {
        fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0) {
            RCLCPP_ERROR(rclcpp::get_logger("serial"),
                         "Cannot open %s: %s", port.c_str(), strerror(errno));
            return false;
        }

        struct termios tty;
        tcgetattr(fd_, &tty);
        cfsetospeed(&tty, B115200);
        cfsetispeed(&tty, B115200);
        tty.c_cflag = CS8 | CLOCAL | CREAD;
        tty.c_iflag = IGNPAR;
        tty.c_oflag = 0;
        tty.c_lflag = 0;
        tty.c_cc[VMIN]  = 0;
        tty.c_cc[VTIME] = 0;
        tcflush(fd_, TCIFLUSH);
        tcsetattr(fd_, TCSANOW, &tty);

        RCLCPP_INFO(rclcpp::get_logger("serial"),
                    "Opened %s @ %d baud", port.c_str(), baud);
        return true;
    }

    void close() {
        if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
    }

    bool is_open() const { return fd_ >= 0; }

    /*
     * Non-blocking read of one "dist,azi" frame.
     * Returns true when a frame was successfully parsed.
     */
    bool read_frame(int* dist_cm, int* azi_deg) {
        static char buf[cfg::BUF_SIZE];
        static int  pos = 0;

        /* Check for available data */
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(fd_, &fds);
        struct timeval tv = {0, 0};
        if (select(fd_+1, &fds, NULL, NULL, &tv) <= 0)
            return false;

        char tmp[128];
        ssize_t n = ::read(fd_, tmp, sizeof(tmp)-1);
        if (n <= 0) return false;
        tmp[n] = '\0';

        for (ssize_t i = 0; i < n; i++) {
            char c = tmp[i];
            if (c == '\n' || c == '\r') {
                if (pos > 0) {
                    buf[pos] = '\0';
                    pos = 0;
                    int d=0, a=0;
                    if (sscanf(buf, "%d , %d", &d, &a) == 2) {
                        *dist_cm = d;
                        *azi_deg = a;
                        return true;
                    }
                }
            } else if (pos < (int)(sizeof(buf)-1)) {
                buf[pos++] = c;
            }
        }
        return false;
    }
};

/* ================================================================
 *  Main control node
 * ================================================================ */
class UwbFollowerNode : public rclcpp::Node {
public:
    UwbFollowerNode() : Node("go2_uwb_follower") {
        /* Declare ROS2 parameters */
        this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
        this->declare_parameter<int>("serial_baud", cfg::SERIAL_BAUD);

        /* Publisher -> /api/sport/request */
        req_pub_ = this->create_publisher<unitree_api::msg::Request>(
            "/api/sport/request", 10);

        /* Open serial port */
        std::string port = this->get_parameter("serial_port").as_string();
        if (!ser_.open(port, cfg::SERIAL_BAUD)) {
            RCLCPP_ERROR(get_logger(), "Serial open failed. Exiting.");
            rclcpp::shutdown();
            return;
        }

        /* Stand up the dog */
        send_action(API_ID_STANDUP);
        rclcpp::sleep_for(2s);

        /* Main loop timer (~100 Hz) */
        timer_ = this->create_wall_timer(cfg::MAIN_LOOP, [this]() { main_tick(); });

        /* Initialise timestamps */
        last_signal_time_ = this->now().seconds();
        last_cmd_time_    = this->now();
        last_pid_time_    = this->now();

        RCLCPP_INFO(get_logger(),
            "UWB Follower started. Target=%.0f cm, deadzone_dist=%.0f cm, "
            "deadzone_angle=%.0f deg, cmd_interval=%ld ms",
            cfg::TARGET_DISTANCE_CM, cfg::DIST_DEADZONE_CM,
            cfg::ANGLE_DEADZONE_DEG, cfg::CMD_INTERVAL.count());
    }

    ~UwbFollowerNode() override {
        RCLCPP_INFO(get_logger(), "Shutting down: stop move + stand down");
        send_action(API_ID_STOPMOVE);
        rclcpp::sleep_for(500ms);
        send_action(API_ID_STANDDOWN);
        ser_.close();
    }

private:
    /* ============ ROS2 communication ============ */
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    /* ============ Hardware ============ */
    SerialPort ser_;

    /* ============ Control state ============ */
    enum State { FOLLOWING=0, APPROACHING, BACKING_OFF, TURNING, SEARCHING, LOST };
    static const char* state_name(State s) {
        switch(s) {
            case FOLLOWING:  return "FOLLOWING";
            case APPROACHING:return "APPROACHING";
            case BACKING_OFF:return "BACKING_OFF";
            case TURNING:    return "TURNING";
            case SEARCHING:  return "SEARCHING";
            case LOST:       return "LOST";
            default:         return "?";
        }
    }

    State  state_ = FOLLOWING;

    /* Filters */
    LowPassFilter dist_filt_{cfg::FILTER_ALPHA};
    LowPassFilter azi_filt_{cfg::AZI_ALPHA};

    /* PID state */
    float dist_error_sum_  = 0.0f;
    float dist_error_prev_ = 0.0f;
    float prev_vx_   = 0.0f;
    float prev_vyaw_ = 0.0f;
    rclcpp::Time last_pid_time_;

    /* Signal tracking */
    double      last_signal_time_ = 0.0;
    double      search_start_time_ = 0.0;
    rclcpp::Time last_cmd_time_;

    /* Frame accumulator */
    FrameAccum  accum_;

    /* Stats */
    int no_data_count_   = 0;
    int recovery_count_  = 0;   /* signal-recovery confirmation counter */
    int total_frames_    = 0;
    int spike_drops_     = 0;

    /* ============ Publish actions ============ */

    void send_move(float vx, float vyaw) {
        auto req = unitree_api::msg::Request();
        req.header.identity.api_id = API_ID_MOVE;
        char buf[128];
        snprintf(buf, sizeof(buf),
                 "{\"x\": %.3f, \"y\": 0.000, \"z\": %.3f}", vx, vyaw);
        req.parameter = std::string(buf);
        req_pub_->publish(req);
    }

    void send_action(int32_t api_id) {
        auto req = unitree_api::msg::Request();
        req.header.identity.api_id = api_id;
        req.parameter = "";
        req_pub_->publish(req);
    }

    /* ============ Main loop tick ============ */

    void main_tick() {
        /* 1. Read serial */
        int dist_cm = 0, azi_deg = 0;
        bool got_frame = ser_.read_frame(&dist_cm, &azi_deg);

        if (got_frame) {
            process_frame(dist_cm, azi_deg);
            no_data_count_  = 0;
            recovery_count_ = 0;
        } else {
            no_data_count_++;
            if (no_data_count_ >= cfg::NO_DATA_THRESH)
                handle_no_signal();
        }

        /* 2. Check if it's time to dispatch a command */
        auto now = this->now();
        if ((now - last_cmd_time_) >= cfg::CMD_INTERVAL) {
            if (accum_.ready() && state_ != LOST && state_ != SEARCHING) {
                dispatch_command();
            }
            last_cmd_time_ = now;
        }
    }

    /* ============ Frame processing ============ */

    void process_frame(int dist_cm, int azi_deg) {
        total_frames_++;
        last_signal_time_ = this->now().seconds();

        /* Signal recovery confirmation (SEARCHING/LOST -> FOLLOWING) */
        if (state_ == SEARCHING || state_ == LOST) {
            recovery_count_++;
            if (recovery_count_ >= cfg::RECOVERY_FRAMES) {
                RCLCPP_INFO(get_logger(), "Signal recovered! Leaving %s state.",
                            state_name(state_));
                state_ = FOLLOWING;
                dist_filt_.reset();
                azi_filt_.reset();
                dist_error_sum_  = 0.0f;
                dist_error_prev_ = 0.0f;
                accum_.reset();
                recovery_count_ = 0;
            }
            /* Still confirming -- accumulate but don't dispatch */
            accum_.add(dist_cm, azi_deg, &spike_drops_);
            return;
        }

        /* Accumulate frame (spike filtering inside add()) */
        accum_.add(dist_cm, azi_deg, &spike_drops_);
    }

    /* ============ Signal loss ============ */

    void handle_no_signal() {
        double now_sec = this->now().seconds();
        double elapsed = now_sec - last_signal_time_;

        if (elapsed < cfg::SIGNAL_TIMEOUT_SEC)
            return;  /* Brief dropout, hold last command */

        if (state_ != SEARCHING && state_ != LOST) {
            state_ = SEARCHING;
            search_start_time_ = now_sec;
            dist_filt_.reset();
            azi_filt_.reset();
            dist_error_sum_  = 0.0f;
            dist_error_prev_ = 0.0f;
            accum_.reset();
            RCLCPP_WARN(get_logger(),
                        "Signal lost for %.1fs, starting search...", elapsed);
        }

        if (state_ == SEARCHING) {
            if (now_sec - search_start_time_ > cfg::SEARCH_DURATION_SEC) {
                state_ = LOST;
                RCLCPP_WARN(get_logger(), "Search timeout. Stopping and waiting.");
                send_action(API_ID_STOPMOVE);
                rclcpp::sleep_for(300ms);
                send_move(0.0f, 0.0f);
                return;
            }
            /* Spin to search */
            send_move(0.0f, cfg::SEARCH_TURN_SPEED);
        }
        /* LOST: do nothing, wait for signal */
    }

    /* ============ Command computation & dispatch ============ */

    void dispatch_command() {
        if (!accum_.ready()) return;

        /* Extract per-cycle averaged distance and angle */
        float dist_avg = 0.0f, azi_avg = 0.0f;
        accum_.get_avg(&dist_avg, &azi_avg);
        accum_.reset();

        /* Low-pass filter */
        float dist_f = dist_filt_.update(dist_avg);
        float azi_f  = azi_filt_.update(azi_avg);

        /* Update state machine */
        update_state(dist_f, azi_f);

        /* PID computation */
        float vx = 0.0f, vyaw = 0.0f;
        compute_pid(dist_f, azi_f, &vx, &vyaw);

        /* Publish */
        send_move(vx, vyaw);

        RCLCPP_INFO(get_logger(),
            "[%s] raw=%d,%d | avg=%.1f,%.1f | filt=%.1f,%.1f | "
            "vx=%.3f vyaw=%.3f | frames=%d spikes=%d",
            state_name(state_),
            (int)dist_avg, (int)azi_avg,
            dist_avg, azi_avg, dist_f, azi_f,
            vx, vyaw, total_frames_, spike_drops_);
    }

    void update_state(float dist_f, float azi_f) {
        float abs_azi = std::fabs(azi_f);

        if (dist_f < cfg::TARGET_DISTANCE_CM - cfg::DIST_DEADZONE_CM) {
            state_ = BACKING_OFF;
        } else if (dist_f > cfg::TARGET_DISTANCE_CM + cfg::DIST_DEADZONE_CM + 30.0f) {
            state_ = APPROACHING;
        } else if (abs_azi > cfg::ANGLE_DEADZONE_DEG * 2.0f &&
                   std::fabs(dist_f - cfg::TARGET_DISTANCE_CM) < cfg::DIST_DEADZONE_CM + 20.0f) {
            state_ = TURNING;
        } else {
            state_ = FOLLOWING;
        }
    }

    void compute_pid(float dist_f, float azi_f, float* vx, float* vyaw) {
        /* -- Distance PID -- */
        float err = dist_f - cfg::TARGET_DISTANCE_CM;
        if (std::fabs(err) < cfg::DIST_DEADZONE_CM) err = 0.0f;

        /* dt */
        auto now = this->now();
        double dt = (now - last_pid_time_).seconds();
        if (dt <= 0.0 || !last_pid_time_.nanoseconds()) dt = 0.02;
        last_pid_time_ = now;

        /* P */
        float p = cfg::KP_DIST * err;

        /* I (anti-windup) */
        if (std::fabs(err) < 80.0f) {
            dist_error_sum_ += err * (float)dt;
            if (dist_error_sum_ >  30.0f) dist_error_sum_ =  30.0f;
            if (dist_error_sum_ < -30.0f) dist_error_sum_ = -30.0f;
        } else {
            dist_error_sum_ *= 0.9f;
        }
        float i = cfg::KI_DIST * dist_error_sum_;

        /* D */
        float d = cfg::KD_DIST * (err - dist_error_prev_) / (float)dt;
        dist_error_prev_ = err;

        float vx_cmd = p + i + d;
        if (vx_cmd > cfg::MAX_FWD_SPEED)  vx_cmd = cfg::MAX_FWD_SPEED;
        if (vx_cmd < cfg::MAX_BACK_SPEED) vx_cmd = cfg::MAX_BACK_SPEED;

        /* Acceleration limit */
        float accel_lim = cfg::MAX_ACCEL * (float)dt;
        float delta = vx_cmd - prev_vx_;
        if      (delta >  accel_lim) vx_cmd = prev_vx_ + accel_lim;
        else if (delta < -accel_lim) vx_cmd = prev_vx_ - accel_lim;
        prev_vx_ = vx_cmd;

        /* -- Angle P -- */
        float azi = azi_f;
        if (std::fabs(azi) < cfg::ANGLE_DEADZONE_DEG) azi = 0.0f;

        float vyaw_cmd = cfg::KP_ANGLE * azi;
        if (vyaw_cmd >  cfg::MAX_TURN_SPEED) vyaw_cmd =  cfg::MAX_TURN_SPEED;
        if (vyaw_cmd < -cfg::MAX_TURN_SPEED) vyaw_cmd = -cfg::MAX_TURN_SPEED;

        /* Reverse turn direction when backing up */
        if (vx_cmd < 0.0f) vyaw_cmd = -vyaw_cmd;
        prev_vyaw_ = vyaw_cmd;

        *vx   = vx_cmd;
        *vyaw = vyaw_cmd;
    }
};

/* ================================================================
 *  main
 * ================================================================ */
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UwbFollowerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
