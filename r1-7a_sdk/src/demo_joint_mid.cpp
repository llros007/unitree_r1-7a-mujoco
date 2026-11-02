#include "arm_motor_bus.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

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

enum class MidMode { Motor, Urdf };

struct Options {
    std::string port = "/dev/ttyUSB3";
    int baud = kArmDefaultBaudrate;
    float kp = 50.f;
    float kd = 1.0f;
    float speed = 1.5f;
    float max_track_err = 1.2f;
    int rate_hz = 50;
    int hold_ms = 2000;
    MidMode mid_mode = MidMode::Motor;
    bool all_at_end = true;
};

void printUsage(const char* argv0) {
    std::cout
        << "Usage: " << argv0 << " [options]\n"
        << "Move one joint at a time to its mid position, hold, print motor angles.\n"
        << "  --mode motor   motor angle 0 => URDF q = calib offset (default)\n"
        << "  --mode urdf    URDF q = (lower+upper)/2 from model limits\n"
        << "Options:\n"
        << "  --port PATH    serial (default /dev/ttyUSB3)\n"
        << "  --baud N       default " << kArmDefaultBaudrate << "\n"
        << "  --kp / --kd    FOC gains (default 50 / 1.0)\n"
        << "  --speed F      peak joint speed rad/s (default 1.5)\n"
        << "  --max-err F    abort if |q_cmd-q_meas| > F (default 1.2)\n"
        << "  --rate N       Hz (default 50)\n"
        << "  --hold-ms N    pause at each point (default 2000)\n"
        << "  --no-final-all skip final move: all joints to mid together\n"
        << "Ctrl-C stops motors and exits.\n";
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
        } else if (std::strcmp(argv[i], "--speed") == 0) {
            if (!need(opt.speed)) {
                return false;
            }
        } else if (std::strcmp(argv[i], "--max-err") == 0) {
            if (!need(opt.max_track_err)) {
                return false;
            }
        } else if (std::strcmp(argv[i], "--rate") == 0) {
            if (!need_i(opt.rate_hz)) {
                return false;
            }
        } else if (std::strcmp(argv[i], "--hold-ms") == 0) {
            if (!need_i(opt.hold_ms)) {
                return false;
            }
        } else if (std::strcmp(argv[i], "--mode") == 0) {
            if (i + 1 >= argc) {
                return false;
            }
            const char* m = argv[++i];
            if (std::strcmp(m, "motor") == 0) {
                opt.mid_mode = MidMode::Motor;
            } else if (std::strcmp(m, "urdf") == 0) {
                opt.mid_mode = MidMode::Urdf;
            } else {
                std::cerr << "Unknown --mode " << m << " (motor|urdf)\n";
                return false;
            }
        } else if (std::strcmp(argv[i], "--no-final-all") == 0) {
            opt.all_at_end = false;
        } else {
            std::cerr << "Unknown option: " << argv[i] << "\n";
            printUsage(argv[0]);
            return false;
        }
    }
    return true;
}

void printUrdfStates(const std::array<ArmJointState, kArmJointCount>& s) {
    std::cout << std::fixed << std::setprecision(4);
    for (int i = 0; i < kArmJointCount; ++i) {
        std::cout << "  j" << i << " urdf_q=" << s[i].q << " rad (" << (s[i].q * 180.f / kPi)
                  << " deg) mode=" << static_cast<int>(s[i].mode)
                  << " err=0x" << std::hex << s[i].error_code << std::dec << "\n";
    }
}

void printMotorStates(const std::array<ArmJointState, kArmJointCount>& m) {
    std::cout << std::fixed << std::setprecision(4);
    for (int i = 0; i < kArmJointCount; ++i) {
        std::cout << "  j" << i << " motor_q=" << m[i].q << " rad (" << (m[i].q * 180.f / kPi)
                  << " deg)\n";
    }
}

void sendFoc(ArmMotorBus& bus, const JointArray& q, const JointArray& dq, float kp, float kd) {
    std::array<ArmJointCmd, kArmJointCount> cmds{};
    for (int i = 0; i < kArmJointCount; ++i) {
        cmds[i].mode = 1;
        cmds[i].timed_out = false;
        cmds[i].q = q[i];
        cmds[i].dq = dq[i];
        cmds[i].tau = 0.f;
        cmds[i].kp = kp;
        cmds[i].kd = kd;
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

void sendStop(ArmMotorBus& bus) {
    std::array<ArmJointCmd, kArmJointCount> cmds{};
    for (int i = 0; i < kArmJointCount; ++i) {
        cmds[i].mode = 0;
        cmds[i].timed_out = false;
    }
    bus.step(cmds);
}

bool waitForFeedback(ArmMotorBus& bus, int rate_hz, int timeout_ms) {
    const auto period = std::chrono::microseconds(1000000 / rate_hz);
    const auto deadline = Clock::now() + std::chrono::milliseconds(timeout_ms);
    auto next = Clock::now();
    std::cout << "Probing feedback (zero-torque FOC)...\n";
    while (g_running && Clock::now() < deadline) {
        sendZeroTorque(bus);
        if (bus.allFeedbackReady()) {
            std::cout << "Feedback ready for all " << kArmJointCount << " joints.\n";
            return true;
        }
        next += period;
        std::this_thread::sleep_until(next);
    }
    return false;
}

void shutdown(ArmMotorBus& bus, int rate_hz) {
    const auto period = std::chrono::microseconds(1000000 / rate_hz);
    const auto deadline = Clock::now() + std::chrono::milliseconds(500);
    auto next = Clock::now();
    while (Clock::now() < deadline) {
        sendStop(bus);
        next += period;
        std::this_thread::sleep_until(next);
    }
    bus.stop();
    bus.close();
}

float midTargetUrdf(const ArmMotorBus& bus, int joint, MidMode mode) {
    if (mode == MidMode::Motor) {
        return bus.jointCalibration(joint).offset;
    }
    return 0.5f * (kLowerLimit[joint] + kUpperLimit[joint]);
}

float clampUrdf(int joint, float q) {
    return std::min(kUpperLimit[joint], std::max(kLowerLimit[joint], q));
}

bool checkHealthy(ArmMotorBus& bus, const JointArray& q_cmd, float max_err) {
    const auto st = bus.states();
    for (int i = 0; i < kArmJointCount; ++i) {
        if (st[i].mode != 1 || st[i].error_code != 0) {
            printUrdfStates(st);
            std::cerr << "Fault on joint " << i << "\n";
            return false;
        }
        if (std::fabs(q_cmd[i] - st[i].q) > max_err) {
            printUrdfStates(st);
            std::cerr << "Tracking error on j" << i << ": " << std::fabs(q_cmd[i] - st[i].q)
                      << " rad\n";
            return false;
        }
    }
    return true;
}

bool moveOneJoint(ArmMotorBus& bus, const Options& opt, JointArray& q_cmd, int joint,
                  float q_goal) {
    q_goal = clampUrdf(joint, q_goal);
    const float start = q_cmd[joint];
    const float delta = q_goal - start;
    const float duration_s =
        std::max(0.3f, kPi * 0.5f * std::fabs(delta) / std::max(0.1f, opt.speed));

    std::cout << std::fixed << std::setprecision(4) << "  Move j" << joint << " " << start
              << " -> " << q_goal << " rad (" << duration_s << " s)\n";

    const auto period = std::chrono::microseconds(1000000 / opt.rate_hz);
    const auto t0 = Clock::now();
    auto next = t0;
    while (g_running) {
        const float t = std::chrono::duration<float>(Clock::now() - t0).count();
        const float s = std::min(1.f, t / duration_s);
        const float blend = 0.5f * (1.f - std::cos(kPi * s));
        const float blend_rate =
            (s < 1.f) ? 0.5f * kPi * std::sin(kPi * s) / duration_s : 0.f;

        JointArray dq{};
        q_cmd[joint] = start + delta * blend;
        dq[joint] = delta * blend_rate;
        sendFoc(bus, q_cmd, dq, opt.kp, opt.kd);
        if (!checkHealthy(bus, q_cmd, opt.max_track_err)) {
            return false;
        }
        if (s >= 1.f) {
            q_cmd[joint] = q_goal;
            return true;
        }
        next += period;
        std::this_thread::sleep_until(next);
    }
    return false;
}

bool holdPose(ArmMotorBus& bus, const Options& opt, const JointArray& q_cmd, int ms) {
    const JointArray zero{};
    const auto period = std::chrono::microseconds(1000000 / opt.rate_hz);
    const auto deadline = Clock::now() + std::chrono::milliseconds(ms);
    auto next = Clock::now();
    while (g_running && Clock::now() < deadline) {
        sendFoc(bus, q_cmd, zero, opt.kp, opt.kd);
        if (!checkHealthy(bus, q_cmd, opt.max_track_err)) {
            return false;
        }
        next += period;
        std::this_thread::sleep_until(next);
    }
    return g_running;
}

bool enableFocAt(ArmMotorBus& bus, const Options& opt, JointArray& q_cmd) {
    const JointArray zero{};
    const auto period = std::chrono::microseconds(1000000 / opt.rate_hz);
    const auto deadline = Clock::now() + std::chrono::milliseconds(2000);
    auto next = Clock::now();
    while (g_running && Clock::now() < deadline) {
        sendFoc(bus, q_cmd, zero, opt.kp, opt.kd);
        const auto st = bus.states();
        bool ok = true;
        for (int i = 0; i < kArmJointCount; ++i) {
            if (st[i].mode != 1) {
                ok = false;
                break;
            }
        }
        if (ok) {
            return true;
        }
        next += period;
        std::this_thread::sleep_until(next);
    }
    std::cerr << "FOC enable timeout.\n";
    return false;
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

    const char* mode_name = (opt.mid_mode == MidMode::Motor) ? "motor (q_motor=0)" : "urdf mid";
    std::cout << "demo_joint_mid  port=" << opt.port << " mode=" << mode_name
              << " speed=" << opt.speed << " rad/s\n";

    if (!waitForFeedback(bus, opt.rate_hz, 3000)) {
        std::cerr << "No feedback.\n";
        shutdown(bus, opt.rate_hz);
        return 1;
    }

    JointArray q_cmd{};
    const auto st0 = bus.states();
    for (int i = 0; i < kArmJointCount; ++i) {
        q_cmd[i] = st0[i].q;
    }
    std::cout << "Start URDF pose:\n";
    printUrdfStates(st0);
    std::cout << "Start motor pose:\n";
    printMotorStates(bus.motorStates());

    std::cout << "Mid targets (" << mode_name << "):\n";
    for (int i = 0; i < kArmJointCount; ++i) {
        const float t = midTargetUrdf(bus, i, opt.mid_mode);
        std::cout << "  j" << i << " target urdf_q=" << t << " rad\n";
    }

    if (!enableFocAt(bus, opt, q_cmd)) {
        shutdown(bus, opt.rate_hz);
        return 1;
    }
    std::cout << "FOC enabled.\n";

    for (int j = 0; j < kArmJointCount && g_running; ++j) {
        const float goal = midTargetUrdf(bus, j, opt.mid_mode);
        std::cout << "\n=== Point " << (j + 1) << "/" << kArmJointCount
                  << ": joint j" << j << " to mid ===\n";
        if (!moveOneJoint(bus, opt, q_cmd, j, goal)) {
            shutdown(bus, opt.rate_hz);
            return 1;
        }
        if (!holdPose(bus, opt, q_cmd, opt.hold_ms)) {
            shutdown(bus, opt.rate_hz);
            return 1;
        }
        std::cout << "At point " << (j + 1) << " (j" << j << " at mid, others follow cmd):\n";
        printUrdfStates(bus.states());
        std::cout << "Motor angles:\n";
        printMotorStates(bus.motorStates());
        if (opt.mid_mode == MidMode::Motor) {
            const float mq = bus.motorStates()[j].q;
            std::cout << "  => j" << j << " motor_q should be near 0, actual " << mq
                      << " rad (" << (mq * 180.f / kPi) << " deg)\n";
        }
    }

    if (opt.all_at_end && g_running) {
        std::cout << "\n=== Final: all joints to mid ===\n";
        for (int j = 0; j < kArmJointCount && g_running; ++j) {
            const float goal = midTargetUrdf(bus, j, opt.mid_mode);
            if (!moveOneJoint(bus, opt, q_cmd, j, goal)) {
                shutdown(bus, opt.rate_hz);
                return 1;
            }
        }
        if (!holdPose(bus, opt, q_cmd, opt.hold_ms)) {
            shutdown(bus, opt.rate_hz);
            return 1;
        }
        std::cout << "All joints at mid:\n";
        printUrdfStates(bus.states());
        printMotorStates(bus.motorStates());
    }

    std::cout << "Sequence done.\n";
    shutdown(bus, opt.rate_hz);
    return 0;
}
