// Copyright 2025 Enactic, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <atomic>
#include <can_interface_resolver.hpp>
#include <chrono>
#include <cstdio>
#include <controller/control.hpp>
#include <controller/dynamics.hpp>
#include <csignal>
#include <filesystem>
#include <iostream>
#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>
#include <openarm_port/openarm_init.hpp>
#include <periodic_timer_thread.hpp>
#include <robot_state.hpp>
#include <thread>
#include <yamlloader.hpp>

std::atomic<bool> keep_running(true);

void signal_handler(int signal) {
    if (signal == SIGINT) {
        std::cout << "\nCtrl+C detected. Exiting loop..." << std::endl;
        keep_running = false;
    }
}

class LeaderArmThread : public PeriodicTimerThread {
public:
    LeaderArmThread(std::shared_ptr<RobotSystemState> robot_state, Control *control_l,
                    double hz = 500.0)
        : PeriodicTimerThread(hz), robot_state_(robot_state), control_l_(control_l) {}

protected:
    void before_start() override { std::cout << "leader start thread " << std::endl; }

    void after_stop() override { std::cout << "leader stop thread " << std::endl; }

    void on_timer() override {
        static auto prev_time = std::chrono::steady_clock::now();
        static int tick = 0;

        control_l_->bilateral_step();

        auto now = std::chrono::steady_clock::now();

        auto elapsed_us =
            std::chrono::duration_cast<std::chrono::microseconds>(now - prev_time).count();
        prev_time = now;

        if (tick < 5 || tick % 500 == 0) {
            auto resp = robot_state_->arm_state().get_all_responses();
            auto ref = robot_state_->arm_state().get_all_references();
            auto grip = robot_state_->hand_state().get_all_responses();
            std::cout << "[Leader t=" << tick << " " << elapsed_us << "us] resp:";
            for (auto& s : resp) std::cout << " " << s.position;
            std::cout << "  grip:";
            for (auto& s : grip) std::cout << " " << s.position;
            std::cout << "  ref:";
            for (auto& s : ref) std::cout << " " << s.position;
            std::cout << std::endl;
        }
        tick++;
    }

private:
    std::shared_ptr<RobotSystemState> robot_state_;
    Control *control_l_;
};

class FollowerArmThread : public PeriodicTimerThread {
public:
    FollowerArmThread(std::shared_ptr<RobotSystemState> robot_state, Control *control_f,
                      double hz = 500.0)
        : PeriodicTimerThread(hz), robot_state_(robot_state), control_f_(control_f) {}

protected:
    void before_start() override { std::cout << "follower start thread " << std::endl; }

    void after_stop() override { std::cout << "follower stop thread " << std::endl; }

    void on_timer() override {
        static auto prev_time = std::chrono::steady_clock::now();
        static int tick = 0;

        control_f_->bilateral_step();

        auto now = std::chrono::steady_clock::now();

        auto elapsed_us =
            std::chrono::duration_cast<std::chrono::microseconds>(now - prev_time).count();
        prev_time = now;

        if (tick < 5 || tick % 500 == 0) {
            auto resp = robot_state_->arm_state().get_all_responses();
            auto ref = robot_state_->arm_state().get_all_references();
            auto grip = robot_state_->hand_state().get_all_responses();
            std::cout << "[Follower t=" << tick << " " << elapsed_us << "us] resp:";
            for (auto& s : resp) std::cout << " " << s.position;
            std::cout << "  grip:";
            for (auto& s : grip) std::cout << " " << s.position;
            std::cout << "  ref:";
            for (auto& s : ref) std::cout << " " << s.position;
            std::cout << std::endl;
        }
        tick++;
    }

private:
    std::shared_ptr<RobotSystemState> robot_state_;
    Control *control_f_;
};

class AdminThread : public PeriodicTimerThread {
public:
    AdminThread(std::shared_ptr<RobotSystemState> leader_state,
                std::shared_ptr<RobotSystemState> follower_state, Control *control_l,
                Control *control_f, double hz = 500.0,
                const std::string &record_path = "")
        : PeriodicTimerThread(hz),
          leader_state_(leader_state),
          follower_state_(follower_state),
          control_l_(control_l),
          control_f_(control_f),
          record_file_(nullptr),
          record_interval_(static_cast<int>(hz / 50.0)) {
        if (!record_path.empty()) {
            record_file_ = std::fopen(record_path.c_str(), "wb");
            if (!record_file_)
                std::cerr << "[AdminThread] Failed to open record file: "
                          << record_path << std::endl;
            else
                std::cout << "[AdminThread] Recording joints to "
                          << record_path << " at 50Hz" << std::endl;
        }
        if (record_interval_ < 1) record_interval_ = 1;
    }

    ~AdminThread() {
        if (record_file_) {
            std::fclose(record_file_);
            record_file_ = nullptr;
        }
    }

protected:
    void before_start() override { std::cout << "admin start thread " << std::endl; }

    void after_stop() override {
        if (record_file_) {
            std::fclose(record_file_);
            record_file_ = nullptr;
        }
        std::cout << "admin stop thread " << std::endl;
    }

    void on_timer() override {
        static auto prev_time = std::chrono::steady_clock::now();
        static int tick = 0;
        auto now = std::chrono::steady_clock::now();

        // get response
        auto leader_arm_resp = leader_state_->arm_state().get_all_responses();
        auto follower_arm_resp = follower_state_->arm_state().get_all_responses();

        auto leader_hand_resp = leader_state_->hand_state().get_all_responses();
        auto follower_hand_resp = follower_state_->hand_state().get_all_responses();
        auto follower_hand_resp_raw = follower_hand_resp;

        if (tick < 5) {
            std::cout << "[Admin t=" << tick << "] leader_resp:";
            for (auto& s : leader_arm_resp) std::cout << " " << s.position;
            std::cout << "  follower_resp:";
            for (auto& s : follower_arm_resp) std::cout << " " << s.position;
            std::cout << std::endl;
        }

        // Write binary record at 50Hz
        if (record_file_ && tick % record_interval_ == 0) {
            auto epoch = std::chrono::system_clock::now().time_since_epoch();
            double ts = std::chrono::duration<double>(epoch).count();
            std::fwrite(&ts, sizeof(double), 1, record_file_);
            for (auto& s : leader_arm_resp)
                std::fwrite(&s.position, sizeof(double), 1, record_file_);
            for (auto& s : leader_hand_resp)
                std::fwrite(&s.position, sizeof(double), 1, record_file_);
            for (auto& s : follower_arm_resp)
                std::fwrite(&s.position, sizeof(double), 1, record_file_);
            for (auto& s : follower_hand_resp)
                std::fwrite(&s.position, sizeof(double), 1, record_file_);
            std::fflush(record_file_);
        }

        // Gentle push away from full elbow extension (within 20 deg of straight)
        constexpr double elbow_zone = 0.349;
        constexpr double elbow_blend = 1.05;
        for (auto* resp : {&leader_arm_resp, &follower_arm_resp}) {
            if ((*resp)[3].position < elbow_zone)
                (*resp)[3].position += (elbow_zone - (*resp)[3].position) * elbow_blend;
        }

        constexpr double gripper_scale = 2.58;
        constexpr double gripper_offset = 0.06;

        // Leader ← follower: undo offset+scale from follower back to leader range
        leader_state_->arm_state().set_all_references(follower_arm_resp);
        for (auto& s : follower_hand_resp) s.position = s.position / gripper_scale - gripper_offset;
        leader_state_->hand_state().set_all_references(follower_hand_resp);

        // Follower ← leader: offset so closed=0, then scale up to follower range
        follower_state_->arm_state().set_all_references(leader_arm_resp);
        for (auto& s : leader_hand_resp) s.position = (s.position + gripper_offset) * gripper_scale;

        // Ensure minimum closing force: if follower ref is more closed (closer to 0)
        // than actual, keep at least min_grip_error of position error
        constexpr double min_grip_error = 0.3;
        for (size_t i = 0; i < leader_hand_resp.size(); ++i) {
            double actual = follower_hand_resp_raw[i].position;
            if (leader_hand_resp[i].position > actual &&
                leader_hand_resp[i].position - actual < min_grip_error)
                leader_hand_resp[i].position = actual + min_grip_error;
        }
        follower_state_->hand_state().set_all_references(leader_hand_resp);

        auto elapsed_us =
            std::chrono::duration_cast<std::chrono::microseconds>(now - prev_time).count();
        prev_time = now;

        tick++;
    }

private:
    std::shared_ptr<RobotSystemState> leader_state_;
    std::shared_ptr<RobotSystemState> follower_state_;
    Control *control_l_;
    Control *control_f_;
    FILE *record_file_;
    int record_interval_;
};

int main(int argc, char **argv) {
    try {
        std::signal(SIGINT, signal_handler);

        std::string arm_side = "right_arm";
        std::string leader_urdf_path;
        std::string follower_urdf_path;
        std::string leader_can_interface;
        std::string follower_can_interface;
        std::string record_path;

        // Parse --record flag from anywhere in argv
        for (int i = 1; i < argc; ++i) {
            if (std::string(argv[i]) == "--record" && i + 1 < argc) {
                record_path = argv[++i];
            }
        }

        // Build positional args (everything that isn't --record / its value)
        std::vector<std::string> pos_args;
        for (int i = 1; i < argc; ++i) {
            if (std::string(argv[i]) == "--record") { ++i; continue; }
            pos_args.push_back(argv[i]);
        }

        if (pos_args.size() < 2) {
            std::cerr
                << "Usage: " << argv[0]
                << " <leader_urdf_path> <follower_urdf_path> [arm_side] [leader_can] [follower_can]"
                << " [--record <path>]"
                << std::endl;
            std::cerr << "\nCAN interfaces are auto-resolved from USB serial numbers "
                         "when not specified." << std::endl;
            return 1;
        }

        leader_urdf_path = pos_args[0];
        follower_urdf_path = pos_args[1];

        if (pos_args.size() >= 3) {
            arm_side = pos_args[2];
            if (arm_side != "left_arm" && arm_side != "right_arm") {
                std::cerr << "[ERROR] Invalid arm_side: " << arm_side
                          << ". Must be 'left_arm' or 'right_arm'." << std::endl;
                return 1;
            }
        }

        if (pos_args.size() >= 5) {
            leader_can_interface = pos_args[3];
            follower_can_interface = pos_args[4];
        } else {
            openarm::print_interface_map();

            std::string side = (arm_side == "left_arm") ? "left" : "right";
            leader_can_interface = openarm::resolve_arm_interface("leader_" + side);
            follower_can_interface = openarm::resolve_arm_interface("follower_" + side);

            if (leader_can_interface.empty()) {
                std::cerr << "[ERROR] Could not find CAN adapter for leader_"
                          << side << " by USB serial!" << std::endl;
                return 1;
            }
            if (follower_can_interface.empty()) {
                std::cerr << "[ERROR] Could not find CAN adapter for follower_"
                          << side << " by USB serial!" << std::endl;
                return 1;
            }
            std::cout << "Auto-resolved leader  -> " << leader_can_interface << std::endl;
            std::cout << "Auto-resolved follower -> " << follower_can_interface << std::endl;
        }

        // URDF file existence check
        if (!std::filesystem::exists(leader_urdf_path)) {
            std::cerr << "[ERROR] Leader URDF not found: " << leader_urdf_path << std::endl;
            return 1;
        }
        if (!std::filesystem::exists(follower_urdf_path)) {
            std::cerr << "[ERROR] Follower URDF not found: " << follower_urdf_path << std::endl;
            return 1;
        }

        // Setup dynamics
        std::string root_link = "openarm_body_link0";
        std::string leaf_link =
            (arm_side == "left_arm") ? "openarm_left_hand" : "openarm_right_hand";

        // Output confirmation
        std::cout << "=== OpenArm Bilateral Control ===" << std::endl;
        std::cout << "Arm side         : " << arm_side << std::endl;
        std::cout << "Leader CAN       : " << leader_can_interface << std::endl;
        std::cout << "Follower CAN     : " << follower_can_interface << std::endl;
        std::cout << "Leader URDF path : " << leader_urdf_path << std::endl;
        std::cout << "Follower URDF path: " << follower_urdf_path << std::endl;
        std::cout << "Root link         : " << root_link << std::endl;
        std::cout << "Leaf link         : " << leaf_link << std::endl;

        YamlLoader leader_loader("config/leader.yaml");
        YamlLoader follower_loader("config/follower.yaml");

        // Leader parameters
        std::vector<double> leader_kp = leader_loader.get_vector("LeaderArmParam", "Kp");
        std::vector<double> leader_kd = leader_loader.get_vector("LeaderArmParam", "Kd");
        std::vector<double> leader_Fc = leader_loader.get_vector("LeaderArmParam", "Fc");
        std::vector<double> leader_k = leader_loader.get_vector("LeaderArmParam", "k");
        std::vector<double> leader_Fv = leader_loader.get_vector("LeaderArmParam", "Fv");
        std::vector<double> leader_Fo = leader_loader.get_vector("LeaderArmParam", "Fo");

        // Follower parameters
        std::vector<double> follower_kp = follower_loader.get_vector("FollowerArmParam", "Kp");
        std::vector<double> follower_kd = follower_loader.get_vector("FollowerArmParam", "Kd");
        std::vector<double> follower_Fc = follower_loader.get_vector("FollowerArmParam", "Fc");
        std::vector<double> follower_k = follower_loader.get_vector("FollowerArmParam", "k");
        std::vector<double> follower_Fv = follower_loader.get_vector("FollowerArmParam", "Fv");
        std::vector<double> follower_Fo = follower_loader.get_vector("FollowerArmParam", "Fo");

        Dynamics *leader_arm_dynamics = new Dynamics(leader_urdf_path, root_link, leaf_link);
        leader_arm_dynamics->Init();

        Dynamics *follower_arm_dynamics = new Dynamics(follower_urdf_path, root_link, leaf_link);
        follower_arm_dynamics->Init();

        std::cout << "=== Initializing Leader OpenArm ===" << std::endl;
        openarm::can::socket::OpenArm *leader_openarm =
            openarm_init::OpenArmInitializer::initialize_openarm(leader_can_interface, true);

        std::cout << "=== Initializing Follower OpenArm ===" << std::endl;
        openarm::can::socket::OpenArm *follower_openarm =
            openarm_init::OpenArmInitializer::initialize_openarm(follower_can_interface, true);

        size_t leader_arm_motor_num = leader_openarm->get_arm().get_motors().size();
        size_t follower_arm_motor_num = follower_openarm->get_arm().get_motors().size();
        size_t leader_hand_motor_num = leader_openarm->get_gripper().get_motors().size();
        size_t follower_hand_motor_num = follower_openarm->get_gripper().get_motors().size();

        std::cout << "leader arm motor num : " << leader_arm_motor_num << std::endl;
        std::cout << "follower arm motor num : " << follower_arm_motor_num << std::endl;
        std::cout << "leader hand motor num : " << leader_hand_motor_num << std::endl;
        std::cout << "follower hand motor num : " << follower_hand_motor_num << std::endl;

        // Declare robot_state (Joint and motor counts are assumed to be equal)
        std::shared_ptr<RobotSystemState> leader_state =
            std::make_shared<RobotSystemState>(leader_arm_motor_num, leader_hand_motor_num);

        std::shared_ptr<RobotSystemState> follower_state =
            std::make_shared<RobotSystemState>(follower_arm_motor_num, follower_hand_motor_num);

        Control *control_leader = new Control(
            leader_openarm, leader_arm_dynamics, follower_arm_dynamics, leader_state,
            1.0 / FREQUENCY, ROLE_LEADER, arm_side, leader_arm_motor_num, leader_hand_motor_num);
        Control *control_follower =
            new Control(follower_openarm, leader_arm_dynamics, follower_arm_dynamics,
                        follower_state, 1.0 / FREQUENCY, ROLE_FOLLOWER, arm_side,
                        follower_arm_motor_num, follower_hand_motor_num);

        // set parameter
        control_leader->SetParameter(leader_kp, leader_kd, leader_Fc, leader_k, leader_Fv,
                                     leader_Fo);

        control_follower->SetParameter(follower_kp, follower_kd, follower_Fc, follower_k,
                                       follower_Fv, follower_Fo);

        // Read leader's current position so follower can smoothly move to it
        leader_openarm->refresh_all();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        leader_openarm->recv_all(10000);

        OpenArmJointConverter arm_conv(leader_arm_motor_num);
        OpenArmJGripperJointConverter grip_conv(leader_hand_motor_num);

        std::vector<MotorState> leader_arm_ms;
        for (const auto& m : leader_openarm->get_arm().get_motors())
            leader_arm_ms.push_back({m.get_position(), m.get_velocity(), 0.0});
        std::vector<MotorState> leader_grip_ms;
        for (const auto& m : leader_openarm->get_gripper().get_motors())
            leader_grip_ms.push_back({m.get_position(), m.get_velocity(), 0.0});

        auto leader_arm_joints = arm_conv.motor_to_joint(leader_arm_ms);
        auto leader_grip_joints = grip_conv.motor_to_joint(leader_grip_ms);

        // Offset + scale leader gripper to follower range before setting startup target
        constexpr double gripper_scale_startup = 2.58;
        constexpr double gripper_offset_startup = 0.06;
        for (auto& s : leader_grip_joints) s.position = (s.position + gripper_offset_startup) * gripper_scale_startup;

        follower_state->arm_state().set_all_references(leader_arm_joints);
        follower_state->hand_state().set_all_references(leader_grip_joints);
        std::thread thread_f(&Control::AdjustPosition, control_follower);
        thread_f.join();

        // Start control process
        LeaderArmThread leader_thread(leader_state, control_leader, FREQUENCY);
        FollowerArmThread follower_thread(follower_state, control_follower, FREQUENCY);
        AdminThread admin_thread(leader_state, follower_state, control_leader, control_follower,
                                 FREQUENCY, record_path);

        // thread start in control
        leader_thread.start_thread();
        follower_thread.start_thread();
        admin_thread.start_thread();

        while (keep_running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        leader_thread.stop_thread();
        follower_thread.stop_thread();
        admin_thread.stop_thread();

        leader_openarm->disable_all();
        follower_openarm->disable_all();

    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
    }

    return 0;
}