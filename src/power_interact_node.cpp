#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <serial/serial.h>
#include <sstream>
#include <iomanip>
#include <vector>
#include <std_srvs/Trigger.h>

class AirComputerPowerInteract
{
public:
    AirComputerPowerInteract()
    {
        // 1. 串口初始化，根据实际硬件修改端口、波特率
        try
        {
            ser.setPort("/dev/ttyS2");
            ser.setBaudrate(115200);
            serial::Timeout to = serial::Timeout::simpleTimeout(100);
            ser.setTimeout(to);
            ser.open();
            ROS_INFO("Serial port open success: /dev/ttyS2");
        }
        catch (serial::IOException &e)
        {
            ROS_ERROR("Serial port open failed: %s", e.what());
        }

        // 2. 发布器：所有收发报文通过std_msgs/String输出
        pub_raw_msg = nh.advertise<std_msgs::String>("power_board_raw_msg", 10);
        pub_gas_gen_normal = nh.advertise<std_msgs::Bool>("gas_generator_normal", 10);

        // 3. 创建4个服务，分别对应4大功能
        srv_trigger_gen = nh.advertiseService("trigger_gas_generator",
                                              &AirComputerPowerInteract::srvTriggerGenCb, this);
        srv_data_link_down = nh.advertiseService("datalink_power_down",
                                                 &AirComputerPowerInteract::srvDataLinkDownCb, this);
        srv_data_link_up = nh.advertiseService("datalink_power_up",
                                               &AirComputerPowerInteract::srvDataLinkUpCb, this);
        srv_check_gen = nh.advertiseService("check_gas_generator",
                                            &AirComputerPowerInteract::srvCheckGenCb, this);

        ROS_INFO("Air computer power interact node ready, 4 services created");
    }

    // 循环读取串口，解析电源板上报报文
    void run()
    {
        ros::Rate rate(100);
        std::vector<uint8_t> recv_buf;
        while (ros::ok())
        {
            // ===== 功能1: 检查触发反馈超时(无 ASCII "finish open" 视为失败) =====
            if (trigger_pending_ && ros::Time::now() >= trigger_feedback_deadline_) {
                ROS_WARN("Gas generator trigger: no 'finish open' within %.1fs, assumed failed",
                         trigger_feedback_timeout_s_);
                std_msgs::Bool gen_bool;
                gen_bool.data = false;
                pub_gas_gen_normal.publish(gen_bool);
                trigger_pending_ = false;
            }

            size_t len = ser.available();
            if (len > 0)
            {
                std::vector<uint8_t> temp(len);
                ser.read(temp.data(), len);
                recv_buf.insert(recv_buf.end(), temp.begin(), temp.end());

                // ===== 功能1 ASCII 反馈检测(必须在 0x71 帧解析之前,否则会被字节丢弃) =====
                // 气体发生器触发成功后,电源板回 "finish open" ASCII 流(无帧头 0x71)
                if (trigger_pending_ && !recv_buf.empty()) {
                    std::string ascii_data(recv_buf.begin(), recv_buf.end());
                    if (ascii_data.find("finish open") != std::string::npos) {
                        ROS_INFO("Gas generator trigger confirmed: 'finish open' ASCII feedback");
                        std_msgs::Bool gen_bool;
                        gen_bool.data = true;
                        pub_gas_gen_normal.publish(gen_bool);
                        trigger_pending_ = false;
                        recv_buf.clear();
                    } else if (recv_buf.size() > 128) {
                        // ASCII 字节累积但既不含 "finish open" 也不是 0x71 帧头,
                        // 防止在长任务期间无限增长
                        ROS_WARN("Non-frame bytes buffered > 128B without 'finish open', clearing");
                        recv_buf.clear();
                    }
                }

                // 逐字节查找帧头0x71，7字节完整帧解析
                while (recv_buf.size() >= 7)
                {
                    if (recv_buf[0] != 0x71)
                    {
                        recv_buf.erase(recv_buf.begin());
                        continue;
                    }
                    // 取一完整7字节帧
                    std::vector<uint8_t> frame(recv_buf.begin(), recv_buf.begin() + 7);
                    recv_buf.erase(recv_buf.begin(), recv_buf.begin() + 7);

                    // 校验帧尾
                    if (frame[6] != 0x5C)
                    {
                        ROS_WARN("Frame tail error, drop frame");
                        continue;
                    }
                    // 校验累加和 Byte5
                    uint8_t calc_sum = frame[0] + frame[1] + frame[2] + frame[3] + frame[4];
                    if (calc_sum != frame[5])
                    {
                        ROS_WARN("Checksum error, drop frame");
                        continue;
                    }

                    // 格式化报文字符串并发布
                    std::string frame_str = byteVec2HexStr(frame);
                    ROS_INFO("Recv power board frame: %s", frame_str.c_str());
                    publishRawString(frame_str);

                    // 解析上报状态（功能4）
                    uint8_t gen_state = frame[4];
                    std_msgs::Bool gen_bool;
                    if (gen_state == 0x01) {
                        ROS_INFO("Gas generator report: NORMAL");
                        gen_bool.data = true;
                        pub_gas_gen_normal.publish(gen_bool);
                    } else if (gen_state == 0x02) {
                        ROS_WARN("Gas generator report: ABNORMAL");
                        gen_bool.data = false;
                        pub_gas_gen_normal.publish(gen_bool);
                    }
                }
            }
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh;
    ros::Publisher pub_raw_msg;
    ros::Publisher pub_gas_gen_normal;
    serial::Serial ser;

    // 4个服务服务端
    ros::ServiceServer srv_trigger_gen;
    ros::ServiceServer srv_data_link_down;
    ros::ServiceServer srv_data_link_up;
    ros::ServiceServer srv_check_gen;

    // ===== 功能1 触发反馈状态 =====
    // 触发后等待电源板回 ASCII "finish open";超时未回视为失败
    bool trigger_pending_ = false;
    ros::Time trigger_feedback_deadline_;
    static constexpr double trigger_feedback_timeout_s_ = 5.0;  // 等待 "finish open" 上限

    // ==================== 服务回调函数 ====================
    // 功能1：气体发生器触发指令 0x71 02 00 00 00 sum 5C
    // 成功反馈: 电源板返回 ASCII "finish open" (无帧头,直接在串口流里)
    // 失败反馈: 无(电源板不回任何东西)
    bool srvTriggerGenCb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
    {
        std::vector<uint8_t> frame = {0x71, 0x02, 0x00, 0x00, 0x00};
        fillCheckSum(frame);
        frame.push_back(0x5C);
        sendFrame(frame);
        // 标记等待 ASCII 反馈,设置超时截止时间
        trigger_pending_ = true;
        trigger_feedback_deadline_ = ros::Time::now() + ros::Duration(trigger_feedback_timeout_s_);
        res.success = true;
        res.message = "Send gas generator trigger cmd ok, awaiting 'finish open' feedback";
        return true;
    }

    // 功能2：数据链下电 0x71 00 02 00 00 sum 5C
    bool srvDataLinkDownCb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
    {
        std::vector<uint8_t> frame = {0x71, 0x00, 0x02, 0x00, 0x00};
        fillCheckSum(frame);
        frame.push_back(0x5C);
        sendFrame(frame);
        res.success = true;
        res.message = "Send datalink power down cmd ok";
        return true;
    }

    // 功能2：数据链上电 0x71 00 01 00 00 sum 5C
    bool srvDataLinkUpCb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
    {
        std::vector<uint8_t> frame = {0x71, 0x00, 0x01, 0x00, 0x00};
        fillCheckSum(frame);
        frame.push_back(0x5C);
        sendFrame(frame);
        res.success = true;
        res.message = "Send datalink power up cmd ok";
        return true;
    }

    // 功能3：检查气体发生器 0x71 00 00 01 00 sum 5C
    bool srvCheckGenCb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
    {
        std::vector<uint8_t> frame = {0x71, 0x00, 0x00, 0x01, 0x00};
        fillCheckSum(frame);
        frame.push_back(0x5C);
        sendFrame(frame);
        res.success = true;
        res.message = "Send check gas generator cmd ok";
        return true;
    }

    // ==================== 工具函数 ====================
    // 填充Byte5累加校验和
    void fillCheckSum(std::vector<uint8_t> &frame)
    {
        uint8_t sum = 0;
        for (auto b : frame) sum += b;
        frame.push_back(sum);
    }

    // 发送串口帧 + 发布字符串报文
    void sendFrame(const std::vector<uint8_t> &frame)
    {
        if (ser.isOpen())
            ser.write(frame.data(), frame.size());
        std::string hex_str = byteVec2HexStr(frame);
        ROS_INFO("Send cmd frame: %s", hex_str.c_str());
        publishRawString(hex_str);
    }

    // 字节数组转空格分隔十六进制字符串
    std::string byteVec2HexStr(const std::vector<uint8_t> &data)
    {
        std::stringstream ss;
        ss << std::hex << std::setfill('0');
        for (size_t i = 0; i < data.size(); ++i)
        {
            ss << "0x" << std::setw(2) << (int)data[i];
            if (i != data.size() - 1) ss << " ";
        }
        return ss.str();
    }

    // 发布std_msgs/String报文话题
    void publishRawString(const std::string &msg_str)
    {
        std_msgs::String msg;
        msg.data = msg_str;
        pub_raw_msg.publish(msg);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "air_power_interact_node");
    AirComputerPowerInteract node;
    node.run();
    return 0;
}