#include <algorithm>
#include <array>
#include <vector>
#include <queue>
#include <deque>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <cstring>
#include <iostream>
#include <cmath>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <csignal>
#include <geometry_msgs/msg/wrench_stamped.hpp>

#include <rclcpp/rclcpp.hpp>

static void fillWrench(geometry_msgs::msg::WrenchStamped &msg,
                       const rclcpp::Time &stamp,
                       const std::string &frame_id,
                       const std::array<float, 6> &data)
{
    auto round_int = [](float v) -> double {
        return static_cast<double>(std::llround(static_cast<double>(v)));
    };
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;
    // 发布值取整数（四舍五入），便于下游显示/调试
    msg.wrench.force.x = round_int(data[0]);
    msg.wrench.force.y = round_int(data[1]);
    msg.wrench.force.z = round_int(data[2]);
    msg.wrench.torque.x = round_int(data[3]);
    msg.wrench.torque.y = round_int(data[4]);
    msg.wrench.torque.z = round_int(data[5]);
}

static geometry_msgs::msg::Wrench toWrench(const std::array<float, 6> &data)
{
    geometry_msgs::msg::Wrench w;
    w.force.x = data[0];
    w.force.y = data[1];
    w.force.z = data[2];
    w.torque.x = data[3];
    w.torque.y = data[4];
    w.torque.z = data[5];
    return w;
}

static geometry_msgs::msg::Wrench roundWrench(const geometry_msgs::msg::Wrench &in)
{
    auto round_int = [](double v) -> double {
        return static_cast<double>(std::llround(v));
    };
    geometry_msgs::msg::Wrench out;
    out.force.x = round_int(in.force.x);
    out.force.y = round_int(in.force.y);
    out.force.z = round_int(in.force.z);
    out.torque.x = round_int(in.torque.x);
    out.torque.y = round_int(in.torque.y);
    out.torque.z = round_int(in.torque.z);
    return out;
}

// 与 src/Force/force_filter.cpp 相同的一阶低通 / 滑动平均，左右臂各用一份状态。
class WrenchFilter
{
public:
    enum class Type { Lowpass, MovingAverage, None };

    void configureLowpass(double cutoff_hz, double sample_hz)
    {
        type_ = Type::Lowpass;
        const double dt = 1.0 / sample_hz;
        const double rc = 1.0 / (2.0 * M_PI * cutoff_hz);
        alpha_ = dt / (rc + dt);
        reset();
    }

    void configureMovingAverage(int window)
    {
        type_ = Type::MovingAverage;
        window_ = std::max(1, window);
        reset();
    }

    void configureNone()
    {
        type_ = Type::None;
        reset();
    }

    void reset()
    {
        lp_initialized_ = false;
        for (int i = 0; i < 3; ++i) {
            lp_force_[i] = 0.0;
            lp_torque_[i] = 0.0;
        }
        force_buf_.clear();
        torque_buf_.clear();
    }

    double alpha() const { return alpha_; }

    geometry_msgs::msg::Wrench apply(const geometry_msgs::msg::Wrench &raw)
    {
        if (type_ == Type::Lowpass) {
            return lowpass(raw);
        }
        if (type_ == Type::MovingAverage) {
            return movingAverage(raw);
        }
        return raw;
    }

private:
    geometry_msgs::msg::Wrench lowpass(const geometry_msgs::msg::Wrench &raw)
    {
        const double rf[3] = {raw.force.x, raw.force.y, raw.force.z};
        const double rt[3] = {raw.torque.x, raw.torque.y, raw.torque.z};
        if (!lp_initialized_) {
            for (int i = 0; i < 3; ++i) {
                lp_force_[i] = rf[i];
                lp_torque_[i] = rt[i];
            }
            lp_initialized_ = true;
        } else {
            for (int i = 0; i < 3; ++i) {
                lp_force_[i] = alpha_ * rf[i] + (1.0 - alpha_) * lp_force_[i];
                lp_torque_[i] = alpha_ * rt[i] + (1.0 - alpha_) * lp_torque_[i];
            }
        }
        geometry_msgs::msg::Wrench out;
        out.force.x = lp_force_[0];
        out.force.y = lp_force_[1];
        out.force.z = lp_force_[2];
        out.torque.x = lp_torque_[0];
        out.torque.y = lp_torque_[1];
        out.torque.z = lp_torque_[2];
        return out;
    }

    geometry_msgs::msg::Wrench movingAverage(const geometry_msgs::msg::Wrench &raw)
    {
        force_buf_.push_back({raw.force.x, raw.force.y, raw.force.z});
        torque_buf_.push_back({raw.torque.x, raw.torque.y, raw.torque.z});
        while (static_cast<int>(force_buf_.size()) > window_) {
            force_buf_.pop_front();
            torque_buf_.pop_front();
        }

        double af[3] = {0.0, 0.0, 0.0};
        double at[3] = {0.0, 0.0, 0.0};
        const auto n = static_cast<double>(force_buf_.size());
        for (const auto &f : force_buf_) {
            for (int i = 0; i < 3; ++i) {
                af[i] += f[i];
            }
        }
        for (const auto &t : torque_buf_) {
            for (int i = 0; i < 3; ++i) {
                at[i] += t[i];
            }
        }

        geometry_msgs::msg::Wrench out;
        out.force.x = af[0] / n;
        out.force.y = af[1] / n;
        out.force.z = af[2] / n;
        out.torque.x = at[0] / n;
        out.torque.y = at[1] / n;
        out.torque.z = at[2] / n;
        return out;
    }

    Type type_{Type::Lowpass};
    double alpha_{0.0};
    bool lp_initialized_{false};
    double lp_force_[3]{};
    double lp_torque_[3]{};
    int window_{10};
    std::deque<std::array<double, 3>> force_buf_;
    std::deque<std::array<double, 3>> torque_buf_;
};

class ForceSensor
{
public:
    using Data = std::array<float, 6>;

    ForceSensor(const std::string &port = "/dev/ttyUSB1", unsigned int baud = 115200, double timeout = 1.0)
        : port_(port), baud_(baud), timeout_(timeout), fd_(-1), running_(false)
    {}

    ~ForceSensor()
    {
        stopContinuous();
        disconnect();
    }

    bool connect()
    {
        fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0) {
            std::cerr << "连接串口失败: " << strerror(errno) << "\n";
            return false;
        }
        termios tio{};
        cfsetispeed(&tio, B115200);
        cfsetospeed(&tio, B115200);
        tio.c_cflag |= CLOCAL | CREAD;
        tio.c_cflag &= ~CSIZE;
        tio.c_cflag |= CS8;
        tio.c_cflag &= ~PARENB;
        tio.c_cflag &= ~CSTOPB;
        tio.c_cc[VMIN] = 0;
        tio.c_cc[VTIME] = static_cast<unsigned char>(timeout_ * 10);
        tcsetattr(fd_, TCSANOW, &tio);
        std::cout << " 已连接到 " << port_ << '\n';
        return true;
    }

    void disconnect()
    {
        running_ = false;
        if (thread_.joinable())
            thread_.join();
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
            std::cout << " 已断开连接\n";
        }
    }

    bool configureSensor(unsigned int sample_rate = 100,
                         const std::string &unit = "MVPV")
    {
        if (fd_ < 0) return false;
        auto send = [&](const std::string &cmd) {
            std::string s = cmd + "\r\n";
            ssize_t n = ::write(fd_, s.data(), s.size());
            if (n < 0) {
                std::cerr << "串口写入失败: " << strerror(errno) << "\n";
            }
            usleep(100000);
            // 读掉应答或者忽略
        };
        std::cout << " 配置传感器...\n";
        send("AT+SMPF=" + std::to_string(sample_rate));
        send("AT+DCPCU=" + unit);
        send("AT+DCPM=?");
        std::cout << " 配置完成\n";
        return true;
    }

    bool startContinuous()
    {
        if (running_ || fd_ < 0) return false;
        running_ = true;
        thread_ = std::thread(&ForceSensor::readerThread, this);
        return true;
    }

    void stopContinuous()
    {
        running_ = false;
        if (thread_.joinable())
            thread_.join();
    }

    bool getData(Data &out, double timeout = 1.0)
    {
        std::unique_lock<std::mutex> lk(mutex_);
        if (cv_.wait_for(lk, std::chrono::duration<double>(timeout),
                         [&]{ return !queue_.empty() || !running_; }))
        {
            if (!queue_.empty()) {
                out = queue_.front();
                queue_.pop();
                return true;
            }
        }
        return false;
    }

    /** 非阻塞取一包；队列空则返回 false（用于多路传感器同线程轮询） */
    bool tryGetData(Data &out)
    {
        std::lock_guard<std::mutex> lk(mutex_);
        if (queue_.empty()) {
            return false;
        }
        out = queue_.front();
        queue_.pop();
        return true;
    }

private:
    void readerThread()
    {
        ssize_t n = ::write(fd_, "AT+GSD\r\n", 8);
        if (n < 0) {
            std::cerr << "无法发送AT+GSD: " << strerror(errno) << "\n";
        }
        std::vector<uint8_t> buf;
        buf.reserve(1024);
        const unsigned PACKET_LEN = 30;

        while (running_) {
            uint8_t tmp[256];
            ssize_t n = ::read(fd_, tmp, sizeof(tmp));
            if (n > 0) {
                buf.insert(buf.end(), tmp, tmp + n);
                while (buf.size() >= PACKET_LEN) {
                    size_t idx = 0;
                    while (idx + 1 < buf.size() &&
                           !(buf[idx]==0xAA && buf[idx+1]==0x55)) {
                        ++idx;
                    }
                    if (idx > 0) {
                        buf.erase(buf.begin(), buf.begin()+idx);
                    }
                    if (buf.size() < PACKET_LEN) break;
                    std::vector<uint8_t> pkt(buf.begin(), buf.begin()+PACKET_LEN);
                    buf.erase(buf.begin(), buf.begin()+PACKET_LEN);
                    unsigned pktnum;
                    Data data;
                    if (parsePacket(pkt, pktnum, data) && isValid(data)) {
                        std::lock_guard<std::mutex> lg(mutex_);
                        queue_.push(data);
                        cv_.notify_one();
                    }
                }
            }
            usleep(5000);
        }
    }

    static bool parsePacket(const std::vector<uint8_t> &pkt,
                            unsigned int &pktnum,
                            Data &out)
    {
        if (pkt.size() < 30 || pkt[0]!=0xAA || pkt[1]!=0x55)
            return false;
        pktnum = (pkt[4]<<8) | pkt[5];
        for (int i=0;i<6;i++) {
            const uint8_t *p = &pkt[6 + i*4];
            uint32_t val = (uint32_t(p[3])<<24)|(uint32_t(p[2])<<16)|
                           (uint32_t(p[1])<<8)|uint32_t(p[0]);
            float f;
            std::memcpy(&f, &val, sizeof(f));
            out[i] = f;
        }
        return true;
    }

    static bool isValid(const Data &d)
    {
        static const float fxmin=-600, fxmax=600;
        static const float tmin=-20, tmax=20;
        for (int i=0;i<3;i++)
            if (d[i]<fxmin || d[i]>fxmax) return false;
        for (int i=3;i<6;i++)
            if (d[i]<tmin || d[i]>tmax) return false;
        return true;
    }

    std::string port_;
    unsigned int baud_;
    double timeout_;
    int fd_;
    std::atomic<bool> running_;
    std::thread thread_;
    std::mutex mutex_;
    std::condition_variable cv_;
    std::queue<Data> queue_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto request_stop = [](int) { rclcpp::shutdown(); };
    std::signal(SIGINT, request_stop);
    std::signal(SIGTERM, request_stop);
    std::signal(SIGHUP, request_stop);
    auto node = std::make_shared<rclcpp::Node>("force_node");

    // 约定：port0=/dev/ttyUSB1 -> force/left_arm（左臂）；port1=/dev/ttyUSB2 -> force/right_arm（右臂）
    const std::string port0 = node->declare_parameter<std::string>("port0", "/dev/ttyUSB1");
    const std::string port1 = node->declare_parameter<std::string>("port1", "/dev/ttyUSB2");
    const std::string topic0 = node->declare_parameter<std::string>("topic0", "force/left_arm");
    const std::string topic1 = node->declare_parameter<std::string>("topic1", "force/right_arm");
    const std::string frame0 = node->declare_parameter<std::string>("frame_id0", "l_arm_force_link");
    const std::string frame1 = node->declare_parameter<std::string>("frame_id1", "r_arm_force_link");
    // 右臂(/dev/ttyUSB2) 数据同时镜像到 force，兼容仍订阅「force」的旧节点
    const std::string mirror_topic = node->declare_parameter<std::string>("mirror_topic", "force");
    const bool publish_mirror = node->declare_parameter<bool>("publish_mirror", true);
    const std::string mirror_frame_id = node->declare_parameter<std::string>("mirror_frame_id", "force_link");

    const std::string filtered_topic0 = node->declare_parameter<std::string>("output_topic1", "/wrench_filtered_left");
    const std::string filtered_topic1 = node->declare_parameter<std::string>("output_topic2", "/wrench_filtered_right");
    std::string filter_type = node->declare_parameter<std::string>("filter_type", "lowpass");
    const double cutoff_freq = node->declare_parameter<double>("cutoff_frequency", 10.0);
    const double sample_rate = node->declare_parameter<double>("sample_rate", 100.0);
    const int ma_window_size = node->declare_parameter<int>("window_size", 10);

    WrenchFilter filter0;
    WrenchFilter filter1;
    if (filter_type == "moving_average") {
        filter0.configureMovingAverage(ma_window_size);
        filter1.configureMovingAverage(ma_window_size);
        RCLCPP_INFO(node->get_logger(), "力滤波: 滑动平均 window=%d", ma_window_size);
    } else if (filter_type == "none") {
        filter0.configureNone();
        filter1.configureNone();
        RCLCPP_INFO(node->get_logger(), "力滤波: 关闭");
    } else {
        if (filter_type != "lowpass") {
            RCLCPP_WARN(node->get_logger(), "未知 filter_type=%s，改用 lowpass", filter_type.c_str());
            filter_type = "lowpass";
        }
        filter0.configureLowpass(cutoff_freq, sample_rate);
        filter1.configureLowpass(cutoff_freq, sample_rate);
        RCLCPP_INFO(node->get_logger(),
                    "力滤波: 低通 cutoff=%.1f Hz sample=%.1f Hz alpha=%.4f",
                    cutoff_freq, sample_rate, filter0.alpha());
    }

    auto pub0 = node->create_publisher<geometry_msgs::msg::WrenchStamped>(topic0, 10);
    auto pub1 = node->create_publisher<geometry_msgs::msg::WrenchStamped>(topic1, 10);
    auto pub_filtered0 = node->create_publisher<geometry_msgs::msg::WrenchStamped>(filtered_topic0, 10);
    auto pub_filtered1 = node->create_publisher<geometry_msgs::msg::WrenchStamped>(filtered_topic1, 10);
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_mirror;
    if (publish_mirror && !mirror_topic.empty()) {
        pub_mirror = node->create_publisher<geometry_msgs::msg::WrenchStamped>(mirror_topic, 10);
    }

    ForceSensor sensor0(port0);
    ForceSensor sensor1(port1);

    if (!sensor0.connect()) {
        RCLCPP_ERROR(node->get_logger(), "串口0连接失败: %s", port0.c_str());
        return 1;
    }
    if (!sensor1.connect()) {
        RCLCPP_ERROR(node->get_logger(), "串口1连接失败: %s", port1.c_str());
        sensor0.disconnect();
        return 1;
    }

    sensor0.configureSensor(100, "MVPV");
    sensor1.configureSensor(100, "MVPV");
    {
        std::string mir = publish_mirror ? (" | 右臂镜像 -> " + mirror_topic) : std::string();
        RCLCPP_INFO(node->get_logger(),
                    "左臂(USB1) %s -> %s | 右臂(USB2) %s -> %s | 滤波 -> %s / %s%s",
                    port0.c_str(), topic0.c_str(), port1.c_str(), topic1.c_str(),
                    filtered_topic0.c_str(), filtered_topic1.c_str(), mir.c_str());
    }
    sensor0.startContinuous();
    sensor1.startContinuous();

    ForceSensor::Data data0{};
    ForceSensor::Data data1{};
    geometry_msgs::msg::Wrench filtered0{};
    geometry_msgs::msg::Wrench filtered1{};
    while (rclcpp::ok()) {
        while (sensor0.tryGetData(data0)) {
            const auto stamp = node->now();
            geometry_msgs::msg::WrenchStamped msg;
            fillWrench(msg, stamp, frame0, data0);
            pub0->publish(msg);

            const auto raw0 = toWrench(data0);
            filtered0 = roundWrench(filter0.apply(raw0));
            geometry_msgs::msg::WrenchStamped fmsg;
            fmsg.header.stamp = stamp;
            fmsg.header.frame_id = frame0;
            fmsg.wrench = filtered0;
            pub_filtered0->publish(fmsg);
        }
        while (sensor1.tryGetData(data1)) {
            const auto stamp = node->now();
            geometry_msgs::msg::WrenchStamped msg;
            fillWrench(msg, stamp, frame1, data1);
            pub1->publish(msg);
            if (pub_mirror) {
                geometry_msgs::msg::WrenchStamped mcopy = msg;
                mcopy.header.frame_id = mirror_frame_id;
                pub_mirror->publish(mcopy);
            }

            const auto raw1 = toWrench(data1);
            filtered1 = roundWrench(filter1.apply(raw1));
            geometry_msgs::msg::WrenchStamped fmsg;
            fmsg.header.stamp = stamp;
            fmsg.header.frame_id = frame1;
            fmsg.wrench = filtered1;
            pub_filtered1->publish(fmsg);
        }

        auto rint = [](double v) -> long long {
            return std::llround(v);
        };
        RCLCPP_INFO_THROTTLE(
            node->get_logger(), *node->get_clock(), 1000,
            "左臂滤波: %lld %lld %lld | 右臂滤波: %lld %lld %lld",
            rint(filtered0.force.x), rint(filtered0.force.y), rint(filtered0.force.z),
            rint(filtered1.force.x), rint(filtered1.force.y), rint(filtered1.force.z));

        rclcpp::sleep_for(std::chrono::milliseconds(2));
        rclcpp::spin_some(node);
    }

    sensor0.stopContinuous();
    sensor1.stopContinuous();
    sensor0.disconnect();
    sensor1.disconnect();
    rclcpp::shutdown();
    return 0;
}