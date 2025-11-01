
#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <thread>
#include <chrono>

// Configure the serial port
int setupSerial(const char* port) {
    int fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        std::cerr << "Error opening serial port\n";
        return -1;
    }

    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(fd, &tty) != 0) {
        std::cerr << "Error from tcgetattr\n";
        close(fd);
        return -1;
    }

    cfsetospeed(&tty, B9600);
    cfsetispeed(&tty, B9600);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN]  = 0;   // non-blocking read
    tty.c_cc[VTIME] = 10;  // 1-second timeout (10 deciseconds)

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cerr << "Error from tcsetattr\n";
        close(fd);
        return -1;
    }

    return fd;
}

// Function to query inverter and print response
void readInverter(int fd) {
    char buf[256];

    // Read-only status command
    const char* query = "*STATUS?\r";  // Example: Phocos ASCII read-only command
    write(fd, query, strlen(query));

    // Read inverter response
    int n = read(fd, buf, sizeof(buf)-1);
    if (n > 0) {
        buf[n] = '\0';
        std::cout << "Inverter data: " << buf << "\n";
    } else {
        std::cerr << "No response or timeout\n";
    }
}

int main() {
    const char* port = "/dev/ttyUSB0";  // Change to your RS232 port
    int fd = setupSerial(port);
    if (fd < 0) return 1;

    std::cout << "Starting Phocos inverter monitoring (read-only)...\n";

    while (true) {
        readInverter(fd);
        std::this_thread::sleep_for(std::chrono::seconds(1));  // poll every 1 second
    }

    close(fd);
    return 0;
}
