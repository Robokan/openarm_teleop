#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <chrono>
#include <thread>
#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>
#include <openarm_port/openarm_init.hpp>

int main() {
    std::string can_iface = "can1";

    std::cout << "=== Test 1: Raw socket read ===" << std::endl;

    int raw_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (raw_fd < 0) { perror("socket"); return 1; }

    struct ifreq ifr;
    strncpy(ifr.ifr_name, can_iface.c_str(), IFNAMSIZ - 1);
    if (ioctl(raw_fd, SIOCGIFINDEX, &ifr) < 0) { perror("ioctl"); return 1; }

    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(raw_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) { perror("bind"); return 1; }

    // Send a refresh command to motor 1
    can_frame send_frame;
    memset(&send_frame, 0, sizeof(send_frame));
    send_frame.can_id = 0x01;
    send_frame.can_dlc = 8;
    uint8_t refresh_data[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    memcpy(send_frame.data, refresh_data, 8);
    write(raw_fd, &send_frame, sizeof(send_frame));
    std::cout << "Sent enable to motor 1" << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    fd_set read_fds;
    struct timeval tv;
    FD_ZERO(&read_fds);
    FD_SET(raw_fd, &read_fds);
    tv.tv_sec = 0;
    tv.tv_usec = 50000;
    int sel = select(raw_fd + 1, &read_fds, nullptr, nullptr, &tv);
    std::cout << "Raw socket select() = " << sel << std::endl;

    if (sel > 0) {
        can_frame recv_frame;
        int nbytes = read(raw_fd, &recv_frame, sizeof(recv_frame));
        std::cout << "Raw read() = " << nbytes << " bytes, can_id=0x"
                  << std::hex << recv_frame.can_id << std::dec << std::endl;
    }
    close(raw_fd);

    std::cout << "\n=== Test 2: Library OpenArm recv_all ===" << std::endl;

    auto* openarm = openarm_init::OpenArmInitializer::initialize_openarm(can_iface, true, false);

    auto motors_init = openarm->get_arm().get_motors();
    std::cout << "After init, motor0 pos = " << motors_init[0].get_position() << std::endl;

    // Send MIT command
    std::vector<openarm::damiao_motor::MITParam> cmds;
    for (int i = 0; i < 7; ++i)
        cmds.push_back({0, 0, 0, 0, 0});
    openarm->get_arm().mit_control_all(cmds);
    std::cout << "Sent mit_control_all (7 motors)" << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    std::cout << "Slept 50ms" << std::endl;

    auto pos_before = openarm->get_arm().get_motors()[0].get_position();
    openarm->recv_all(50000);
    auto pos_after = openarm->get_arm().get_motors()[0].get_position();
    std::cout << "recv_all(50000): motor0 before=" << pos_before
              << " after=" << pos_after
              << " changed=" << (pos_before != pos_after ? "YES" : "NO") << std::endl;

    // Now try raw socket on the SAME interface
    std::cout << "\n=== Test 3: Raw socket after library init ===" << std::endl;
    int raw_fd2 = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    strncpy(ifr.ifr_name, can_iface.c_str(), IFNAMSIZ - 1);
    ioctl(raw_fd2, SIOCGIFINDEX, &ifr);
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    bind(raw_fd2, (struct sockaddr*)&addr, sizeof(addr));

    // Send refresh via library
    openarm->refresh_all();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // Try reading response on the raw socket
    FD_ZERO(&read_fds);
    FD_SET(raw_fd2, &read_fds);
    tv.tv_sec = 0;
    tv.tv_usec = 50000;
    sel = select(raw_fd2 + 1, &read_fds, nullptr, nullptr, &tv);
    std::cout << "Raw socket2 select() = " << sel << std::endl;
    if (sel > 0) {
        can_frame recv_frame;
        int nbytes = read(raw_fd2, &recv_frame, sizeof(recv_frame));
        std::cout << "Raw read() = " << nbytes << " bytes, can_id=0x"
                  << std::hex << recv_frame.can_id << std::dec << std::endl;
    }

    // Also try the library's recv_all again
    openarm->refresh_all();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    auto pos_before2 = openarm->get_arm().get_motors()[0].get_position();
    openarm->recv_all(50000);
    auto pos_after2 = openarm->get_arm().get_motors()[0].get_position();
    std::cout << "Library recv_all again: before=" << pos_before2
              << " after=" << pos_after2
              << " changed=" << (pos_before2 != pos_after2 ? "YES" : "NO") << std::endl;

    close(raw_fd2);
    openarm->disable_all();
    return 0;
}
