// Stdin -> FOC five-parameter stream. One line in, one state line out.
// Line: q0..q6 dq0..dq6 tau0..tau6 kp0..kp6 kd0..kd6
// URDF frame. Quantization and calibration happen in ArmMotorBus.
#include "arm_motor_bus.h"

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <unistd.h>

namespace {

std::atomic<bool> g_running{true};

void onSignal(int) {
    g_running = false;
}

void sendZeroTorque(ArmMotorBus& bus) {
    std::array<ArmJointCmd, kArmJointCount> cmds{};
    for (int i = 0; i < kArmJointCount; ++i) {
        cmds[i].mode = 1;
        cmds[i].timed_out = false;
    }
    bus.step(cmds);
}

bool waitForFeedback(ArmMotorBus& bus) {
    const auto period = std::chrono::milliseconds(20);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    auto next = std::chrono::steady_clock::now();
    while (g_running && std::chrono::steady_clock::now() < deadline) {
        sendZeroTorque(bus);
        if (bus.allFeedbackReady()) {
            return true;
        }
        next += period;
        std::this_thread::sleep_until(next);
    }
    return false;
}

int g_state_fd = STDOUT_FILENO;

void printState(const std::array<ArmJointState, kArmJointCount>& st) {
    std::ostringstream out;
    out << "STATE";
    for (int i = 0; i < kArmJointCount; ++i) {
        out << ' ' << st[i].q;
    }
    for (int i = 0; i < kArmJointCount; ++i) {
        out << ' ' << st[i].dq;
    }
    for (int i = 0; i < kArmJointCount; ++i) {
        out << ' ' << st[i].tau_est;
    }
    for (int i = 0; i < kArmJointCount; ++i) {
        out << ' ' << static_cast<int>(st[i].mode);
    }
    for (int i = 0; i < kArmJointCount; ++i) {
        out << ' ' << st[i].error_code;
    }
    out << '\n';
    const std::string line = out.str();
    if (write(g_state_fd, line.data(), line.size()) < 0) {
        g_running = false;
    }
}

}  // namespace

int main(int argc, char** argv) {
    std::string port = "/dev/ttyUSB0";
    std::string calib = "../config/joint_calib.txt";
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--port" && i + 1 < argc) {
            port = argv[++i];
        } else if (std::string(argv[i]) == "--calib" && i + 1 < argc) {
            calib = argv[++i];
        }
    }

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    // Library logs use stdout. Keep that stream for the parent parser only.
    g_state_fd = dup(STDOUT_FILENO);
    if (g_state_fd < 0 || dup2(STDERR_FILENO, STDOUT_FILENO) < 0) {
        std::cerr << "Failed to split state stream\n";
        return 1;
    }
    std::cout.rdbuf(std::cerr.rdbuf());

    ArmMotorBus bus;
    if (!bus.open(port, kArmDefaultBaudrate, calib)) {
        std::cerr << "Failed to open " << port << '\n';
        return 1;
    }
    if (!waitForFeedback(bus)) {
        std::cerr << "Feedback timeout\n";
        bus.stop();
        bus.close();
        return 1;
    }
    bus.clearFaults();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    sendZeroTorque(bus);

    std::cout << std::unitbuf;
    std::cerr << "cmd stream ready on " << port << '\n';
    printState(bus.states());

    std::string line;
    while (g_running && std::getline(std::cin, line)) {
        if (line.empty()) {
            continue;
        }
        if (line == "STOP") {
            break;
        }
        std::istringstream in(line);
        std::array<float, 35> v{};
        bool ok = true;
        for (float& x : v) {
            if (!(in >> x)) {
                ok = false;
                break;
            }
        }
        if (!ok) {
            std::cerr << "Bad command line\n";
            break;
        }
        std::array<ArmJointCmd, kArmJointCount> cmds{};
        for (int i = 0; i < kArmJointCount; ++i) {
            cmds[i].mode = 1;
            cmds[i].timed_out = false;
            cmds[i].q = v[i];
            cmds[i].dq = v[7 + i];
            cmds[i].tau = v[14 + i];
            cmds[i].kp = v[21 + i];
            cmds[i].kd = v[28 + i];
        }
        bus.step(cmds);
        printState(bus.states());
    }

    bus.stop();
    bus.close();
    return 0;
}
