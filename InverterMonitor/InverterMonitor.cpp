#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <thread>
#include <chrono>
#include <signal.h>
#include <csignal>

/**
 * @brief Sets up the serial communication on the specified port.
 *
 * This function opens the serial port, configures the baud rate, data format, and
 * other communication parameters using the termios structure.
 *
 * @param port The name of the serial port to open (e.g., "/dev/ttyUSB0").
 * @return The file descriptor for the serial port on success, or -1 on failure.
 */
int setupSerial(const char* port) {
    // Open the serial port with read and write access
    int fd = open(port, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        std::cerr << "Error opening serial port: " << port << std::endl;
        return -1;
    }

    // Initialize the termios structure to zero
    struct termios tty;
    memset(&tty, 0, sizeof(tty));

    // Get the current serial port settings
    if (tcgetattr(fd, &tty) != 0) {
        std::cerr << "Error from tcgetattr" << std::endl;
        close(fd);
        return -1;
    }

    // Set the baud rate to 9600 for both input and output
    cfsetospeed(&tty, B9600);
    cfsetispeed(&tty, B9600);

    // Configure data format: 8 data bits, no parity, 1 stop bit
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8 data bits
    tty.c_iflag &= ~IGNBRK;                        // Ignore break condition

    // Disable local and output processing
    tty.c_lflag = 0;
    tty.c_oflag = 0;

    // Set non-blocking read with a timeout of 1 second (10 deciseconds)
    tty.c_cc[VMIN]  = 0;                           // Non-blocking read
    tty.c_cc[VTIME] = 10;                          // 1-second timeout

    // Disable software flow control
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);

    // Enable local connection and receive
    tty.c_cflag |= (CLOCAL | CREAD);

    // Disable hardware flow control, parity, and extra stop bits
    tty.c_cflag &= ~(PARENB | PARODD);             // No parity
    tty.c_cflag &= ~CSTOPB;                        // 1 stop bit
    tty.c_cflag &= ~CRTSCTS;                       // No hardware flow control

    // Apply the new configuration to the serial port
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cerr << "Error setting serial port attributes" << std::endl;
        close(fd);
        return -1;
    }

    return fd;
}

/**
 * @brief Sends a command to the inverter and reads the response.
 *
 * This function writes a predefined command to the serial port and then reads
 * the response from the inverter.
 *
 * @param fd The file descriptor for the serial port.
 */
void readInverter(int fd) {
    const char* command = "*STATUS?";  // The command to send to the inverter
    char buffer[256];                 // Buffer to store the response
    int bytes_read;

    // Write the command to the serial port
    if (write(fd, command, strlen(command)) < 0) {
        std::cerr << "Failed to write command to serial port" << std::endl;
        return;
    }

    // Wait a moment to allow the inverter to process the command
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Read the response from the inverter
    bytes_read = read(fd, buffer, sizeof(buffer) - 1);
    if (bytes_read < 0) {
        std::cerr << "Error reading from serial port" << std::endl;
        return;
    }

    // Null-terminate the buffer to make it a valid string
    buffer[bytes_read] = '\0';

    // Print the response from the inverter
    std::cout << "Inverter Response: " << buffer << std::endl;
}

/**
 * @brief Signal handler for graceful program termination.
 *
 * This function is called when the user sends a termination signal (e.g., Ctrl+C).
 * It prints a message and exits the program.
 *
 * @param signal The signal number received.
 */
void signalHandler(int signal) {
    if (signal == SIGINT) {
        std::cout << "\nCaught Ctrl+C. Exiting gracefully..." << std::endl;
        exit(0);
    }
}

// Function to read key-value pairs from the .env file
std::map<std::string, std::string> readEnvFile() {
    string filename = ".env"
    std::map<std::string, std::string> config;
    std::ifstream file(filename);

    if (!file.is_open()) {
        std::cerr << "Error: Could not open .env file: " << filename << std::endl;
        return config;
    }

    std::string line;
    while (std::getline(file, line)) {
        // Skip empty lines and comments
        if (line.empty() || line[0] == '#') continue;

        size_t equalsPos = line.find('=');
        if (equalsPos == std::string::npos) continue;  // Skip invalid lines

        std::string key = line.substr(0, equalsPos);
        std::string value = line.substr(equalsPos + 1);

        config[key] = value;
    }

    file.close();
    return config;
}

/**
 * @brief Main function that runs the inverter monitor.
 *
 * This function sets up the serial port, installs the signal handler, and runs
 * a loop that continuously sends commands and reads responses from the inverter.
 */
int main() {
    // Load configuration from .env file
    std::map<std::string, std::string> config = readEnvFile(".env");

    // Check if required variables are set
    if (config.find("PORT") == config.end()) {
        std::cerr << "Error: PORT not defined in .env" << std::endl;
        return 1;
    }

    if (config.find("BAUD_RATE") == config.end()) {
        std::cerr << "Error: BAUD_RATE not defined in .env" << std::endl;
        return 1;
    }

    // Parse port and baud rate
    std::string port = config["PORT"];
    int baudRate = std::stoi(config["BAUD_RATE"]);

    // Set up the serial port
    int serial_fd = setupSerial(port, baudRate);
    if (serial_fd < 0) {
        return 1;
    }

    // Set up signal handler
    signal(SIGINT, signalHandler);

    // Main loop
    while (true) {
        readInverter(serial_fd);
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }

    close(serial_fd);
    return 0;
}