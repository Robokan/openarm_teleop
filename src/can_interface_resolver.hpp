// Resolve CAN interface names from USB serial numbers.
//
// The kernel assigns can0, can1, ... in arbitrary order depending on USB
// enumeration timing, so we look up the serial number from sysfs to find
// which canX corresponds to which physical arm.

#pragma once

#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <string>

namespace openarm {

struct ArmSerials {
    static const std::map<std::string, std::string>& serial_to_arm() {
        static const std::map<std::string, std::string> m = {
            {"001B00523630501120353355", "leader_left"},
            {"004500533630501120353355", "leader_right"},
            {"003100553630501120353355", "follower_right"},
            {"004900303945501620303651", "follower_left"},
        };
        return m;
    }

    static const std::map<std::string, std::string>& arm_to_serial() {
        static const std::map<std::string, std::string> m = {
            {"leader_left",    "001B00523630501120353355"},
            {"leader_right",   "004500533630501120353355"},
            {"follower_right", "003100553630501120353355"},
            {"follower_left",  "004900303945501620303651"},
        };
        return m;
    }
};

inline std::string get_can_serial(const std::string& iface) {
    try {
        auto devpath = std::filesystem::canonical(
            "/sys/class/net/" + iface + "/device/..");
        std::ifstream f(devpath / "serial");
        std::string serial;
        std::getline(f, serial);
        while (!serial.empty() && (serial.back() == '\n' || serial.back() == '\r'))
            serial.pop_back();
        return serial;
    } catch (...) {
        return "";
    }
}

inline std::map<std::string, std::string> build_interface_map() {
    std::map<std::string, std::string> arm_to_iface;
    const auto& s2a = ArmSerials::serial_to_arm();

    try {
        for (auto& entry : std::filesystem::directory_iterator("/sys/class/net")) {
            std::string name = entry.path().filename().string();
            if (name.substr(0, 3) != "can") continue;

            std::string serial = get_can_serial(name);
            auto it = s2a.find(serial);
            if (it != s2a.end()) {
                arm_to_iface[it->second] = name;
            }
        }
    } catch (...) {}

    return arm_to_iface;
}

inline void print_interface_map() {
    auto mapping = build_interface_map();
    const auto& a2s = ArmSerials::arm_to_serial();

    std::cout << "CAN interface mapping:" << std::endl;
    for (auto& [arm, serial] : a2s) {
        auto it = mapping.find(arm);
        std::string iface = (it != mapping.end()) ? it->second : "NOT FOUND";
        std::cout << "  " << arm << " -> " << iface
                  << "  (serial: " << serial << ")" << std::endl;
    }
}

inline std::string resolve_arm_interface(const std::string& arm_name) {
    auto mapping = build_interface_map();
    auto it = mapping.find(arm_name);
    if (it != mapping.end()) {
        return it->second;
    }
    return "";
}

}  // namespace openarm
