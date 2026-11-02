#pragma once
#include "motor_controller_export.h"
#include "MotorProtocol.h"
#include "SerialPort.h"
#include <memory>
#include <string>
#include <functional>
#include <atomic>
#include <mutex>
#include <thread>
#include <chrono>

struct MOTOR_CONTROLLER_API ControlCommand
{
    uint8_t motor_id; // 电机ID (0-14)
    uint8_t mode;     // 控制模式 (0-5)
    bool timeout;     // 是否设置超时
    float torque;     // 扭矩值
    float speed;      // 速度值
    float position;   // 位置值
    float kp;         // KP系数
    float kd;         // KD系数
    bool valid;       // 是否有效指令

    ControlCommand() : motor_id(0), mode(0), timeout(false),
                       torque(0), speed(0), position(0),
                       kp(0), kd(0), valid(false)
    {
    }
};

class MOTOR_CONTROLLER_API MotorController
{
public:
    MotorController();
    ~MotorController();

    // ===== 初始化和连接 =====
    bool initialize(const std::string &port, int baudrate = 6000000, double timeout = 0.2);
    void disconnect();
    bool isConnected() const;

    void setControlCommand(
        uint8_t motor_id,
        uint8_t mode,
        bool timeout,
        float torque = 0.0f,
        float speed = 0.0f,
        float position = 0.0f,
        float kp = 0.0f,
        float kd = 0.0f);

    void startAsyncControl(int frequency = 100);
    void stopAsyncControl();
    bool isAsyncRunning() const;
    void setAsyncFrequency(int frequency);

    void stopMotor(uint8_t motor_id);
    void startMotor(uint8_t motor_id);
    void zeroTorqueMode(uint8_t motor_id);
    void setTorque(uint8_t motor_id, float torque);
    void setDampingMode(uint8_t motor_id, float kd);
    void setSpeed(uint8_t motor_id, float speed, float kd);
    void setPosition(uint8_t motor_id, float position, float kp, float kd);
    void setMixedControl(uint8_t motor_id, float position, float speed,
                         float kp, float kd, float torque_ff = 0.0f);
    bool clearFault(uint8_t motor_id);
    bool resetMotor(uint8_t motor_id);
    bool isMotorStopped(uint8_t motor_id) const;

    std::shared_ptr<MotorFeedback> getLatestFeedback() const;
    bool hasNewFeedback() const;

    static void printFeedback(const MotorFeedback &feedback);

    using FeedbackCallback = std::function<void(const MotorFeedback &)>;
    void setFeedbackCallback(FeedbackCallback callback);

private:
    std::unique_ptr<SerialPort> m_serial;
    std::unique_ptr<MotorProtocol> m_protocol;
    FeedbackCallback m_callback;

    std::atomic<bool> m_connected;
    std::string m_port;
    int m_baudrate;
    double m_timeout;

    std::atomic<bool> m_asyncRunning;
    std::thread m_controlThread;
    std::atomic<int> m_controlFrequency;
    std::mutex m_commandMutex;
    ControlCommand m_currentCommand;

    mutable std::mutex m_feedbackMutex;
    std::shared_ptr<MotorFeedback> m_latestFeedback;
    mutable std::atomic<bool> m_hasNewFeedback;

    void onDataReceived(const std::vector<uint8_t> &data);
    void processFeedback(const std::vector<uint8_t> &data);
    void controlThreadFunc();
    bool sendCurrentCommand();
    bool sendSingleCommand(const std::vector<uint8_t> &packet);
};