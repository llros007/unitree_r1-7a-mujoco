#include "MotorController.h"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <algorithm>

MotorController::MotorController()
    : m_connected(false), m_baudrate(6000000), m_timeout(0.2),
    m_asyncRunning(false), m_controlFrequency(100), m_hasNewFeedback(false) {
    m_serial = std::make_unique<SerialPort>();
    m_protocol = std::make_unique<MotorProtocol>();
}

MotorController::~MotorController() {
    disconnect();
}

bool MotorController::initialize(const std::string& port, int baudrate, double timeout) {
    if (m_connected) {
        disconnect();
    }

    m_port = port;
    m_baudrate = baudrate;
    m_timeout = timeout;

    m_connected = m_serial->open(port, baudrate, timeout);

    if (m_connected) {
        // 启动异步接收
        m_serial->startAsyncRead([this](const std::vector<uint8_t>& data) {
            this->onDataReceived(data);
            });

        std::cout << "Motor controller initialized" << std::endl;
        std::cout << "  Port: " << port << std::endl;
        std::cout << "  Baudrate: " << baudrate << std::endl;
        std::cout << "  Timeout: " << timeout << "s" << std::endl;
    }
    else {
        std::cerr << "Motor controller initialization failed" << std::endl;
    }

    return m_connected;
}

void MotorController::disconnect() {
    stopAsyncControl();
    if (m_connected) {
        m_serial->stopAsyncRead();
        m_serial->close();
        m_connected = false;
        std::cout << "Motor controller disconnected" << std::endl;
    }
}

bool MotorController::isConnected() const {
    return m_connected;
}

void MotorController::setControlCommand(
    uint8_t motor_id, uint8_t mode, bool timeout,
    float torque, float speed, float position,
    float kp, float kd) {

    std::lock_guard<std::mutex> lock(m_commandMutex);
    m_currentCommand.motor_id = motor_id;
    m_currentCommand.mode = mode;
    m_currentCommand.timeout = timeout;
    m_currentCommand.torque = torque;
    m_currentCommand.speed = speed;
    m_currentCommand.position = position;
    m_currentCommand.kp = kp;
    m_currentCommand.kd = kd;
    m_currentCommand.valid = true;
}

void MotorController::startAsyncControl(int frequency) {
    if (m_asyncRunning) {
        stopAsyncControl();
    }

    if (!m_connected) {
        std::cerr << "[Error] Cannot start async control: not connected" << std::endl;
        return;
    }

    m_controlFrequency = frequency;
    m_asyncRunning = true;
    m_controlThread = std::thread(&MotorController::controlThreadFunc, this);

    std::cout << "Async control started at " << frequency << " Hz" << std::endl;
}

void MotorController::stopAsyncControl() {
    m_asyncRunning = false;
    if (m_controlThread.joinable()) {
        m_controlThread.join();
    }
    std::cout << "Async control stopped" << std::endl;
}

bool MotorController::isAsyncRunning() const {
    return m_asyncRunning;
}

void MotorController::setAsyncFrequency(int frequency) {
    if (frequency > 0) {
        m_controlFrequency = frequency;
        std::cout << "Async frequency set to " << frequency << " Hz" << std::endl;
    }
}

void MotorController::controlThreadFunc() {
    auto interval = std::chrono::milliseconds(1000 / m_controlFrequency.load());

    while (m_asyncRunning) {
        auto startTime = std::chrono::steady_clock::now();

        // 发送当前指令
        if (m_connected) {
            sendCurrentCommand();
        }

        // 计算需要等待的时间
        auto elapsed = std::chrono::steady_clock::now() - startTime;
        auto waitTime = interval - std::chrono::duration_cast<std::chrono::milliseconds>(elapsed);

        if (waitTime > std::chrono::milliseconds(0)) {
            std::this_thread::sleep_for(waitTime);
        }
    }
}

bool MotorController::sendCurrentCommand() {
    std::lock_guard<std::mutex> lock(m_commandMutex);

    if (!m_currentCommand.valid) {
        return false;
    }

    try {
        auto packet = m_protocol->buildControlPacket(
            m_currentCommand.motor_id,
            m_currentCommand.mode,
            m_currentCommand.timeout,
            m_currentCommand.torque,
            m_currentCommand.speed,
            m_currentCommand.position,
            m_currentCommand.kp,
            m_currentCommand.kd);

        m_serial->write(packet);
        return true;
    }
    catch (const std::exception& e) {
        std::cerr << "[Async Control Error] " << e.what() << std::endl;
        return false;
    }
}

// ===== 高级控制接口实现 =====

// 停止电机
void MotorController::stopMotor(uint8_t motor_id) {
    setControlCommand(motor_id, 0, false, 0, 0, 0, 0, 0);
    if (!m_asyncRunning) {
        startAsyncControl(m_controlFrequency);
    }
}

// 启动电机，切换到FOC模式
void MotorController::startMotor(uint8_t motor_id) {
    setControlCommand(motor_id, 1, true, 0, 0, 0, 0, 0);
    if (!m_asyncRunning) {
        startAsyncControl(m_controlFrequency);
    }
}

// 零力矩模式
void MotorController::zeroTorqueMode(uint8_t motor_id) {
    setControlCommand(motor_id, 1, true, 0, 0, 0, 0, 0);
    if (!m_asyncRunning) {
        startAsyncControl(m_controlFrequency);
    }
}

// 纯力矩模式
void MotorController::setTorque(uint8_t motor_id, float torque) {
    setControlCommand(motor_id, 1, true, torque, 0, 0, 0, 0);
    if (!m_asyncRunning) {
        startAsyncControl(m_controlFrequency);
    }
}

//阻尼模式
void MotorController::setDampingMode(uint8_t motor_id,float kd) {
    setControlCommand(motor_id, 1, true, 0, 0, 0, 0, kd);
    if (!m_asyncRunning) {
        startAsyncControl(m_controlFrequency);
    }
}

// 设置速度模式
void MotorController::setSpeed(uint8_t motor_id, float speed, float kd) {
    setControlCommand(motor_id, 1, true, 0, speed, 0, 0, kd);
    if (!m_asyncRunning) {
        startAsyncControl(m_controlFrequency);
    }
}

// 设置位置模式
void MotorController::setPosition(uint8_t motor_id, float position, float kp, float kd) {
    setControlCommand(motor_id, 1, true, 0, 0, position, kp, kd);
    if (!m_asyncRunning) {
        startAsyncControl(m_controlFrequency);
    }
}

// 设置混合控制模式
void MotorController::setMixedControl(uint8_t motor_id, float position, float speed,
    float kp, float kd, float torque_ff) {
    setControlCommand(motor_id, 1, true, torque_ff, speed, position, kp, kd);
    if (!m_asyncRunning) {
        startAsyncControl(m_controlFrequency);
    }
}

std::shared_ptr<MotorFeedback> MotorController::getLatestFeedback() const {
    std::lock_guard<std::mutex> lock(m_feedbackMutex);
    return m_latestFeedback;
}

bool MotorController::hasNewFeedback() const {
    return m_hasNewFeedback.load();
}

void MotorController::setFeedbackCallback(FeedbackCallback callback) {
    m_callback = callback;
}

void MotorController::onDataReceived(const std::vector<uint8_t>& data) {
    processFeedback(data);
}

void MotorController::processFeedback(const std::vector<uint8_t>& data) {
    // 数据已经是完整的数据包（26字节）
    if (data.size() != 26) {
        // 如果大小不对，可能是数据包不完整，忽略
        return;
    }

    if (data[0] != 0xFC || data[1] != 0xEE) {
        return;
    }

    auto feedback = m_protocol->parseFeedbackPacket(data);
    if (!feedback) {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(m_feedbackMutex);
        m_latestFeedback = feedback;
        m_hasNewFeedback = true;
    }

    // 如果设置了回调，调用回调
    if (m_callback) {
        m_callback(*feedback);
    }
}

void MotorController::printFeedback(const MotorFeedback& feedback) {
    std::cout << "\n========== Motor Feedback ==========" << std::endl;
    std::cout << "Motor ID: " << static_cast<int>(feedback.motor_id) << std::endl;
    std::cout << "Mode: " << (feedback.mode == 1 ? "FOC" : "Stop") << std::endl;
    std::cout << "Timeout: " << (feedback.timeout ? "Yes" : "No") << std::endl;
    std::cout << "Driver Temp: " << static_cast<int>(feedback.temp_driver) << " C" << std::endl;
    std::cout << "Winding Temp: " << static_cast<int>(feedback.temp_winding) << " C" << std::endl;

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "Voltage: " << feedback.voltage << " V" << std::endl;
    std::cout << "Torque: " << feedback.torque << " Nm" << std::endl;
    std::cout << "Speed: " << feedback.speed << " rad/s" << std::endl;
    std::cout << "Position: " << feedback.position << " rad" << std::endl;

    if (feedback.error_code != 0) {
        std::cout << "Error: 0x" << std::hex << feedback.error_code << std::dec;
        std::cout << " [" << MotorProtocol::getErrorDescription(feedback.error_code) << "]" << std::endl;
    }

    if (feedback.warning_code != 0) {
        std::cout << "Warning: 0x" << std::hex << static_cast<int>(feedback.warning_code) << std::dec;
        std::cout << " [" << MotorProtocol::getWarningDescription(feedback.warning_code) << "]" << std::endl;
    }

    std::cout << "===================================" << std::endl;
}

// ===== 清除故障和复位指令实现 =====

bool MotorController::clearFault(uint8_t motor_id) {
    if (!m_connected) {
        std::cerr << "[Error] Cannot clear fault: not connected" << std::endl;
        return false;
    }

    try {
        auto packet = m_protocol->buildClearPacket(motor_id);
        return sendSingleCommand(packet);
    }
    catch (const std::exception& e) {
        std::cerr << "[Clear Fault Error] " << e.what() << std::endl;
        return false;
    }
}

bool MotorController::resetMotor(uint8_t motor_id) {
    if (!m_connected) {
        std::cerr << "[Error] Cannot reset motor: not connected" << std::endl;
        return false;
    }

    // 检查电机是否处于停止状态
    if (!isMotorStopped(motor_id)) {
        std::cerr << "[Error] Motor " << static_cast<int>(motor_id) 
                  << " is not stopped. Please stop the motor before reset." << std::endl;
        return false;
    }

    try {
        auto packet = m_protocol->buildResetPacket(motor_id);
        return sendSingleCommand(packet);
    }
    catch (const std::exception& e) {
        std::cerr << "[Reset Motor Error] " << e.what() << std::endl;
        return false;
    }
}

bool MotorController::isMotorStopped(uint8_t motor_id) const {
    std::lock_guard<std::mutex> lock(m_feedbackMutex);
    
    if (!m_latestFeedback) {
        // 没有反馈数据时，认为电机未停止（保守处理）
        return false;
    }

    // 检查电机ID匹配且模式为停止模式（mode == 0）
    return (m_latestFeedback->motor_id == motor_id && m_latestFeedback->mode == 0);
}

bool MotorController::sendSingleCommand(const std::vector<uint8_t>& packet) {
    if (!m_connected) {
        return false;
    }

    try {
        // 如果异步控制正在运行，先暂停
        bool wasRunning = m_asyncRunning;
        if (wasRunning) {
            // 注意：这里不能调用stopAsyncControl()，因为会阻塞等待线程
            // 改为仅暂停发送，但不停止线程
            // 实际上更好的做法是直接发送，不影响异步控制
        }

        // 发送单次命令
        m_serial->write(packet);
        
        // 短暂延时等待命令执行
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        // 如果之前异步控制正在运行，继续运行
        // 由于我们没有停止异步控制，这里不需要额外操作

        return true;
    }
    catch (const std::exception& e) {
        std::cerr << "[Send Single Command Error] " << e.what() << std::endl;
        return false;
    }
}