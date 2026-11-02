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

using JointArray = std::array<float, kArmJointCount>;
using Clock = std::chrono::steady_clock;

std::atomic<bool> g_running{true};

void onSignal(int) {
    g_running = false;
}

constexpr float kPi = 3.14159265f;

constexpr JointArray kLowerLimit = {-2.723f, 0.f, -0.349f, 0.f, -2.269f, -0.524f, -1.571f};
constexpr JointArray kUpperLimit = {2.723f, 3.1416f, 3.491f, 3.1415f, 2.269f, 0.524f, 1.571f};

struct Options {
    std::string port = "/dev/ttyUSB3";
    int baud = kArmDefaultBaudrate;
    float kp = 25.f;
    float kd = 1.0f;
    float step_deg = 0.5f;
    float limit_margin = 0.1f;   // URDF soft limit (rad), stay inside mechanical stop
    float max_lead = 0.12f;      // max |q_cmd - q_meas|; bounds FOC torque ~ kp*lead
    float stall_err = 0.25f;     // if stuck this long, snap cmd to meas
    int stall_cycles = 15;
    int rate_hz = 50;
    int print_every_ms = 500;
};

class StdinRaw {
public:
    StdinRaw() {
        if (!isatty(STDIN_FILENO)) {
            return;
        }
        if (tcgetattr(STDIN_FILENO, &old_) != 0) {
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
        restore();
    }

    void restore() {
        if (active_) {
            tcsetattr(STDIN_FILENO, TCSAFLUSH, &old_);
            active_ = false;
        }
    }

    /** Returns true if a byte was read. Handles ESC [ sequences for arrows. */
    bool readKey(int& out) {
        unsigned char c = 0;
        if (read(STDIN_FILENO, &c, 1) != 1) {
            return false;
        }
        if (c == 27) {
            unsigned char seq[2] = {0, 0};
            if (read(STDIN_FILENO, &seq[0], 1) != 1) {
                out = 27;
                return true;
            }
            if (seq[0] != '[') {
                out = 27;
                return true;
            }
            if (read(STDIN_FILENO, &seq[1], 1) != 1) {
                out = 27;
                return true;
            }
            switch (seq[1]) {
                case 'A':
                    out = 1000;
                    return true;  // up
                case 'B':
                    out = 1001;
                    return true;  // down
                case 'C':
                    out = 1002;
                    return true;  // right
                case 'D':
                    out = 1003;
                    return true;  // left
                default:
                    out = 27;
                    return true;
            }
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
        << "Usage: " << argv0 << " [options]\n"
        << "Keyboard joint teleop (URDF frame, rad). Run in a real terminal (not piped).\n"
        << "Keys:\n"
        << "  0-6        select joint j0-j6\n"
        << "  w / s      +step / -step on selected joint\n"
        << "  Up / Down  same as w / s\n"
        << "  [ / ]      halve / double step (degrees)\n"
        << "  r          snap command to measured pose\n"
        << "  z          all joints command -> 0 (URDF)\n"
        << "  space      brief stop (mode 0) 200ms then resume FOC\n"
        << "  q or Ctrl+C  quit and stop motors\n"
        << "Safety: cmd is lead-limited to meas (anti crush at hard stops).\n"
        << "Options:\n"
        << "  --port PATH   default /dev/ttyUSB3\n"
        << "  --kp --kd     default 25 / 1.0 (lower kp if still fights at limit)\n"
        << "  --step-deg F  per key press (default 0.5 deg)\n"
        << "  --margin F    URDF limit margin rad (default 0.1)\n"
        << "  --lead F      max |cmd-meas| rad (default 0.12)\n"
        << "  --rate N      control Hz (default 50)\n";
}

bool parseArgs(int argc, char** argv, Options& opt) {
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "-h") == 0 || std::strcmp(argv[i], "--help") == 0) {
            printUsage(argv[0]);
            return false;
        }
        auto need = [&](float& dst) {
            if (i + 1 >= argc) {
                return false;
            }
            dst = std::stof(argv[++i]);
            return true;
        };
        auto need_i = [&](int& dst) {
            if (i + 1 >= argc) {
                return false;
            }
            dst = std::stoi(argv[++i]);
            return true;
        };
        if (std::strcmp(argv[i], "--port") == 0) {
            if (i + 1 >= argc) {
                return false;
            }
            opt.port = argv[++i];
        } else if (std::strcmp(argv[i], "--baud") == 0) {
            if (!need_i(opt.baud)) {
                return false;
            }
        } else if (std::strcmp(argv[i], "--kp") == 0) {
            if (!need(opt.kp)) {
                return false;
            }
        } else if (std::strcmp(argv[i], "--kd") == 0) {
            if (!need(opt.kd)) {
                return false;
            }
        } else if (std::strcmp(argv[i], "--step-deg") == 0) {
            if (!need(opt.step_deg)) {
                return false;
            }
        } else if (std::strcmp(argv[i], "--rate") == 0) {
            if (!need_i(opt.rate_hz)) {
                return false;
            }
        } else if (std::strcmp(argv[i], "--margin") == 0) {
            if (!need(opt.limit_margin)) {
                return false;
            }
        } else if (std::strcmp(argv[i], "--lead") == 0) {
            if (!need(opt.max_lead)) {
                return false;
            }
        } else {
            std::cerr << "Unknown: " << argv[i] << "\n";
            printUsage(argv[0]);
            return false;
        }
    }
    return true;
}

float softLower(int j, float margin) {
    return kLowerLimit[j] + margin;
}

float softUpper(int j, float margin) {
    return kUpperLimit[j] - margin;
}

float clampJointSoft(int j, float q, float margin) {
    return std::min(softUpper(j, margin), std::max(softLower(j, margin), q));
}

bool measNearUpper(int j, float q_meas, float margin) {
    return q_meas >= softUpper(j, margin);
}

bool measNearLower(int j, float q_meas, float margin) {
    return q_meas <= softLower(j, margin);
}

/** Keep command close to measurement so FOC does not wind up torque at a hard stop. */
void clampCmdLead(JointArray& q_cmd, const JointArray& q_meas, float lead) {
    for (int i = 0; i < kArmJointCount; ++i) {
        const float lo = q_meas[i] - lead;
        const float hi = q_meas[i] + lead;
        q_cmd[i] = std::min(hi, std::max(lo, q_cmd[i]));
    }
}

bool tryNudgeJoint(int j, float delta, float margin, const JointArray& q_meas,
                   JointArray& q_cmd, std::array<int, kArmJointCount>& block_dir,
                   std::array<int, kArmJointCount>& stall_inhibit) {
    const int dir = (delta > 0.f) ? 1 : -1;
    // Same direction as the stop: ignore. Do not touch q_cmd, so a retreat stays.
    if (block_dir[j] == dir) {
        return false;
    }
    // Opposite direction clears the latch and is allowed immediately.
    if (block_dir[j] == -dir) {
        block_dir[j] = 0;
        stall_inhibit[j] = 25;
    }
    const bool meas_inside =
        q_meas[j] >= softLower(j, margin) && q_meas[j] <= softUpper(j, margin);
    if (meas_inside) {
        if (dir > 0 && measNearUpper(j, q_meas[j], margin)) {
            return false;
        }
        if (dir < 0 && measNearLower(j, q_meas[j], margin)) {
            return false;
        }
    }
    float next = q_cmd[j] + delta;
    if (meas_inside) {
        next = clampJointSoft(j, next, margin);
    }
    q_cmd[j] = next;
    return true;
}

/** While a direction is latched, command must not point back into the stop. */
void applyOneWayStop(JointArray& q_cmd, const JointArray& q_meas,
                     const std::array<int, kArmJointCount>& block_dir) {
    for (int i = 0; i < kArmJointCount; ++i) {
        if (block_dir[i] > 0) {
            q_cmd[i] = std::min(q_cmd[i], q_meas[i]);
        } else if (block_dir[i] < 0) {
            q_cmd[i] = std::max(q_cmd[i], q_meas[i]);
        }
    }
}

void sendFoc(ArmMotorBus& bus, const JointArray& q_cmd, const JointArray& q_meas,
             float kp, float kd, float max_lead) {
    std::array<ArmJointCmd, kArmJointCount> cmds{};
    for (int i = 0; i < kArmJointCount; ++i) {
        cmds[i].mode = 1;
        cmds[i].timed_out = false;
        cmds[i].q = q_cmd[i];
        cmds[i].dq = 0.f;
        cmds[i].tau = 0.f;
        const float err = std::fabs(q_cmd[i] - q_meas[i]);
        // Softer when already close; cap effective stiffness at hard stop.
        float kp_i = kp;
        if (err > 1e-4f) {
            kp_i = std::min(kp, kp * max_lead / err);
        }
        cmds[i].kp = kp_i;
        cmds[i].kd = kd;
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

void sendZeroTorque(ArmMotorBus& bus) {
    std::array<ArmJointCmd, kArmJointCount> cmds{};
    for (int i = 0; i < kArmJointCount; ++i) {
        cmds[i].mode = 1;
        cmds[i].timed_out = false;
    }
    bus.step(cmds);
}

bool waitForFeedback(ArmMotorBus& bus, int rate_hz, int timeout_ms) {
    const auto period = std::chrono::microseconds(1000000 / rate_hz);
    const auto deadline = Clock::now() + std::chrono::milliseconds(timeout_ms);
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

constexpr float kAtTolDeg = 3.f;

void printStatus(int sel, float step_deg, const JointArray& q_cmd,
                 const std::array<ArmJointState, kArmJointCount>& st,
                 const std::array<int, kArmJointCount>& block_dir) {
    std::cout << "\033[2J\033[H";
    std::cout << "=== R1 keyboard teleop ===\n"
              << "Selected j" << sel << "  step=" << std::fixed << std::setprecision(2)
              << step_deg << " deg  |  0-6 选择  w/s 点动  q 退出  r 解除 BLOCK\n"
              << "零位/限位：相差 " << kAtTolDeg << " deg 以内算到达。距离为实测角。\n\n";
    for (int i = 0; i < kArmJointCount; ++i) {
        const float q_deg = st[i].q * 180.f / kPi;
        const float zero_deg = std::fabs(q_deg);
        const float lo_deg = (st[i].q - kLowerLimit[i]) * 180.f / kPi;
        const float hi_deg = (kUpperLimit[i] - st[i].q) * 180.f / kPi;
        const bool near_lo = lo_deg <= hi_deg;
        const float lim_deg = near_lo ? lo_deg : hi_deg;
        const char* lim_state = lim_deg < 0.f ? "超出" : (lim_deg <= kAtTolDeg ? "已到" : "未到");
        const char mark = (i == sel) ? '*' : ' ';
        std::cout << std::setprecision(1);
        std::cout << mark << " j" << i << "  当前 " << std::setw(7) << q_deg << "°"
                  << "  零位 " << (zero_deg <= kAtTolDeg ? "已到" : "未到")
                  << "  距离 " << std::setw(6) << zero_deg << "°"
                  << "  限位 " << lim_state << (near_lo ? "下限" : "上限")
                  << "  距离 " << std::setw(6) << lim_deg << "°";
        if (block_dir[i] > 0) {
            std::cout << "  BLOCK +";
        } else if (block_dir[i] < 0) {
            std::cout << "  BLOCK -";
        }
        if (st[i].error_code != 0) {
            std::cout << "  err=0x" << std::hex << st[i].error_code << std::dec;
        }
        std::cout << "\n";
        (void)q_cmd;
    }
    std::cout << std::flush;
}

void shutdown(ArmMotorBus& bus, int rate_hz) {
    const auto period = std::chrono::microseconds(1000000 / rate_hz);
    const auto deadline = Clock::now() + std::chrono::milliseconds(400);
    auto next = Clock::now();
    while (Clock::now() < deadline) {
        sendStop(bus);
        next += period;
        std::this_thread::sleep_until(next);
    }
    bus.stop();
    bus.close();
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

    if (!waitForFeedback(bus, opt.rate_hz, 3000)) {
        std::cerr << "No motor feedback.\n";
        shutdown(bus, opt.rate_hz);
        return 1;
    }

    JointArray q_cmd{};
    const auto st0 = bus.states();
    for (int i = 0; i < kArmJointCount; ++i) {
        q_cmd[i] = st0[i].q;
    }

    int selected = 0;
    float step_rad = opt.step_deg * kPi / 180.f;
    bool pause_foc = false;
    auto pause_until = Clock::now();
    std::array<int, kArmJointCount> stall_count{};
    std::array<int, kArmJointCount> block_dir{};
    std::array<int, kArmJointCount> stall_inhibit{};

    StdinRaw term;
    std::cout << "FOC from current pose. Focus this terminal for keys.\n"
              << "If you hear clicking at a stop: release keys, press 'r', lower --kp.\n";

    const auto period = std::chrono::microseconds(1000000 / opt.rate_hz);
    auto next_tick = Clock::now();
    auto last_print = Clock::now();

    while (g_running) {
        const auto st = bus.states();
        JointArray q_meas{};
        for (int i = 0; i < kArmJointCount; ++i) {
            q_meas[i] = st[i].q;
        }

        int key = 0;
        while (term.readKey(key)) {
            if (key == 3 || key == 'q') {
                g_running = false;
                break;
            }
            if (key >= '0' && key <= '6') {
                selected = key - '0';
            } else if (key == 'w' || key == 1000) {
                tryNudgeJoint(selected, step_rad, opt.limit_margin, q_meas, q_cmd, block_dir,
                              stall_inhibit);
            } else if (key == 's' || key == 1001) {
                tryNudgeJoint(selected, -step_rad, opt.limit_margin, q_meas, q_cmd, block_dir,
                              stall_inhibit);
            } else if (key == 'W') {
                tryNudgeJoint(selected, 5.f * step_rad, opt.limit_margin, q_meas, q_cmd, block_dir,
                              stall_inhibit);
            } else if (key == 'S') {
                tryNudgeJoint(selected, -5.f * step_rad, opt.limit_margin, q_meas, q_cmd, block_dir,
                              stall_inhibit);
            } else if (key == '[') {
                step_rad *= 0.5f;
                if (step_rad < 0.0005f) {
                    step_rad = 0.0005f;
                }
            } else if (key == ']') {
                step_rad *= 2.f;
            } else if (key == 'r') {
                const auto st_now = bus.states();
                for (int i = 0; i < kArmJointCount; ++i) {
                    q_cmd[i] = st_now[i].q;
                    block_dir[i] = 0;
                    stall_count[i] = 0;
                    stall_inhibit[i] = 0;
                }
            } else if (key == 'z') {
                for (int i = 0; i < kArmJointCount; ++i) {
                    q_cmd[i] = clampJointSoft(i, 0.f, opt.limit_margin);
                }
            } else if (key == ' ') {
                pause_foc = true;
                pause_until = Clock::now() + std::chrono::milliseconds(200);
            }
        }

        clampCmdLead(q_cmd, q_meas, opt.max_lead);
        applyOneWayStop(q_cmd, q_meas, block_dir);

        constexpr float kUnloadRad = 0.03f;
        for (int i = 0; i < kArmJointCount; ++i) {
            if (stall_inhibit[i] > 0) {
                --stall_inhibit[i];
                stall_count[i] = 0;
                continue;
            }
            if (block_dir[i] != 0) {
                stall_count[i] = 0;
                continue;
            }
            const float err = q_cmd[i] - q_meas[i];
            const bool pushing = std::fabs(err) > 0.65f * opt.max_lead;
            const bool not_moving = std::fabs(st[i].dq) < 0.08f;
            if (pushing && not_moving) {
                ++stall_count[i];
                if (stall_count[i] >= opt.stall_cycles) {
                    const int sign = (err > 0.f) ? 1 : -1;
                    block_dir[i] = sign;
                    q_cmd[i] = q_meas[i] - static_cast<float>(sign) * kUnloadRad;
                    stall_count[i] = 0;
                }
            } else {
                stall_count[i] = 0;
            }
        }
        applyOneWayStop(q_cmd, q_meas, block_dir);

        if (pause_foc) {
            sendStop(bus);
            if (Clock::now() >= pause_until) {
                pause_foc = false;
            }
        } else {
            sendFoc(bus, q_cmd, q_meas, opt.kp, opt.kd, opt.max_lead);
        }

        if (Clock::now() - last_print >= std::chrono::milliseconds(opt.print_every_ms)) {
            printStatus(selected, step_rad * 180.f / kPi, q_cmd, st, block_dir);
            last_print = Clock::now();
        }

        next_tick += period;
        std::this_thread::sleep_until(next_tick);
    }

    term.restore();
    std::cout << "\nExiting...\n";
    shutdown(bus, opt.rate_hz);
    return 0;
}
