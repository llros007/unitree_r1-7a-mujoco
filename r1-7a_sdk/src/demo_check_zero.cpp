#include "arm_motor_bus.h"

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <string>
#include <termios.h>
#include <thread>
#include <unistd.h>

namespace {

using Clock = std::chrono::steady_clock;

std::atomic<bool> g_running{true};

void onSignal(int) {
    g_running = false;
}

constexpr float kPi = 3.14159265f;

struct Options {
    std::string port = "/dev/ttyUSB3";
    int baud = kArmDefaultBaudrate;
    int rate_hz = 20;
    float tol_deg = 3.f;
};

class StdinRaw {
public:
    StdinRaw() {
        if (!isatty(STDIN_FILENO) || tcgetattr(STDIN_FILENO, &old_) != 0) {
            return;
        }
        termios raw = old_;
        raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
        raw.c_cc[VMIN] = 0;
        raw.c_cc[VTIME] = 0;
        if (tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw) != 0) {
            return;
        }
        const int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        if (flags >= 0) {
            fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
        }
        active_ = true;
    }
    ~StdinRaw() {
        if (active_) {
            tcsetattr(STDIN_FILENO, TCSAFLUSH, &old_);
        }
    }
    bool readKey(int& out) {
        unsigned char c = 0;
        if (read(STDIN_FILENO, &c, 1) != 1) {
            return false;
        }
        out = static_cast<int>(c);
        return true;
    }

private:
    termios old_{};
    bool active_ = false;
};

void printUsage(const char* argv0) {
    std::cout
        << "Usage: " << argv0 << " [--port PATH] [--tol-deg F]\n"
        << "Zero-torque live check. Move each joint by hand.\n"
        << "A joint is at URDF zero when |q| <= tolerance (default 3 deg).\n"
        << "q or Ctrl-C quits and stops the motors.\n";
}

bool parseArgs(int argc, char** argv, Options& opt) {
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "-h") == 0 || std::strcmp(argv[i], "--help") == 0) {
            printUsage(argv[0]);
            return false;
        }
        if (std::strcmp(argv[i], "--port") == 0 && i + 1 < argc) {
            opt.port = argv[++i];
        } else if (std::strcmp(argv[i], "--baud") == 0 && i + 1 < argc) {
            opt.baud = std::stoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--tol-deg") == 0 && i + 1 < argc) {
            opt.tol_deg = std::stof(argv[++i]);
        } else if (std::strcmp(argv[i], "--rate") == 0 && i + 1 < argc) {
            opt.rate_hz = std::stoi(argv[++i]);
        } else {
            std::cerr << "Unknown: " << argv[i] << "\n";
            printUsage(argv[0]);
            return false;
        }
    }
    if (opt.tol_deg < 0.1f) {
        opt.tol_deg = 0.1f;
    }
    if (opt.rate_hz < 5) {
        opt.rate_hz = 5;
    }
    return true;
}

void sendZeroTorque(ArmMotorBus& bus) {
    std::array<ArmJointCmd, kArmJointCount> cmds{};
    for (int i = 0; i < kArmJointCount; ++i) {
        cmds[i].mode = 1;
        cmds[i].timed_out = false;
        cmds[i].kp = 0.f;
        cmds[i].kd = 0.f;
    }
    bus.step(cmds);
}

void sendStop(ArmMotorBus& bus) {
    std::array<ArmJointCmd, kArmJointCount> cmds{};
    for (int i = 0; i < kArmJointCount; ++i) {
        cmds[i].mode = 0;
        cmds[i].timed_out = false;
    }
    bus.step(cmds);
}

bool waitForFeedback(ArmMotorBus& bus, int rate_hz) {
    const auto period = std::chrono::microseconds(1000000 / rate_hz);
    const auto deadline = Clock::now() + std::chrono::milliseconds(3000);
    auto next = Clock::now();
    while (g_running && Clock::now() < deadline) {
        sendZeroTorque(bus);
        if (bus.allFeedbackReady()) {
            return true;
        }
        next += period;
        std::this_thread::sleep_until(next);
    }
    return false;
}

void printTable(const ArmMotorBus& bus, float tol_deg) {
    const auto st = bus.states();
    const auto mot = bus.motorStates();
    const float tol = tol_deg * kPi / 180.f;
    int ready = 0;
    std::cout << "\033[2J\033[H";
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "用手逐个摆关节。零位 = 标定后的 URDF 角接近 0。容差 "
              << tol_deg << " deg。按 q 退出。\n\n";
    std::cout << "关节   URDF角(deg)   离零位(deg)   电机角(deg)   结果\n";
    for (int i = 0; i < kArmJointCount; ++i) {
        const float deg = st[i].q * 180.f / kPi;
        const float err = std::fabs(st[i].q);
        const bool ok = err <= tol && st[i].error_code == 0;
        if (ok) {
            ++ready;
        }
        std::cout << " j" << i << "   " << std::setw(8) << deg << "      " << std::setw(8)
                  << (err * 180.f / kPi) << "      " << std::setw(8)
                  << (mot[i].q * 180.f / kPi) << "     " << (ok ? "在零位" : "未到")
                  << "  err=0x" << std::hex << st[i].error_code << std::dec << "\n";
    }
    std::cout << "\n";
    if (ready == kArmJointCount) {
        std::cout << "全部 " << kArmJointCount << " 个关节都在零位附近。\n";
    } else {
        std::cout << ready << "/" << kArmJointCount << " 个在零位，还有 "
                  << (kArmJointCount - ready) << " 个没到。\n";
    }
    std::cout << std::flush;
}

}  // namespace

int main(int argc, char** argv) {
    Options opt;
    if (!parseArgs(argc, argv, opt)) {
        return argc > 1 ? 1 : 0;
    }

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    ArmMotorBus bus;
    if (!bus.open(opt.port, opt.baud)) {
        return 1;
    }
    if (!waitForFeedback(bus, opt.rate_hz)) {
        std::cerr << "没有收到 7 个关节的反馈。\n";
        bus.stop();
        bus.close();
        return 1;
    }

    StdinRaw term;
    const auto period = std::chrono::microseconds(1000000 / opt.rate_hz);
    auto next = Clock::now();
    while (g_running) {
        int key = 0;
        while (term.readKey(key)) {
            if (key == 'q' || key == 3) {
                g_running = false;
            }
        }
        sendZeroTorque(bus);
        printTable(bus, opt.tol_deg);
        next += period;
        std::this_thread::sleep_until(next);
    }

    for (int n = 0; n < 10; ++n) {
        sendStop(bus);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    bus.stop();
    bus.close();
    std::cout << "\n已停机。\n";
    return 0;
}
