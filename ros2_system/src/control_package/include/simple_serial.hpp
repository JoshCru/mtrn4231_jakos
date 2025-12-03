/**
 * @file simple_serial.hpp
 * @brief Simple cross-platform serial communication wrapper
 *
 * Lightweight serial library for communicating with Teensy/Arduino
 * Works on Linux and macOS without external dependencies
 */

#pragma once

#include <string>
#include <stdexcept>

#ifdef _WIN32
#include <windows.h>
#else
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#endif

class SimpleSerial {
public:
    SimpleSerial() : fd_(-1), is_open_(false) {}

    ~SimpleSerial() {
        close();
    }

    /**
     * @brief Open serial port
     * @param port Port name (e.g., "/dev/ttyACM0" on Linux, "/dev/cu.usbmodem*" on macOS)
     * @param baudrate Baud rate (default: 115200)
     */
    void open(const std::string& port, int baudrate = 115200) {
        if (is_open_) {
            throw std::runtime_error("Port already open");
        }

#ifdef _WIN32
        // Windows implementation (if needed)
        throw std::runtime_error("Windows not implemented");
#else
        // POSIX (Linux/macOS)
        fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0) {
            throw std::runtime_error("Failed to open port: " + port);
        }

        struct termios tty;
        if (tcgetattr(fd_, &tty) != 0) {
            ::close(fd_);
            throw std::runtime_error("Failed to get terminal attributes");
        }

        // Set baud rate
        speed_t speed;
        switch (baudrate) {
            case 9600:   speed = B9600; break;
            case 115200: speed = B115200; break;
            default:
                ::close(fd_);
                throw std::runtime_error("Unsupported baud rate");
        }

        cfsetospeed(&tty, speed);
        cfsetispeed(&tty, speed);

        // 8N1 mode
        tty.c_cflag &= ~PARENB;  // No parity
        tty.c_cflag &= ~CSTOPB;  // 1 stop bit
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;      // 8 bits
        tty.c_cflag &= ~CRTSCTS; // No hardware flow control
        tty.c_cflag |= CREAD | CLOCAL; // Enable read, ignore modem controls

        // Raw mode
        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // No software flow control
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
        tty.c_oflag &= ~OPOST; // Raw output

        // Timeout settings
        tty.c_cc[VMIN] = 0;   // Non-blocking
        tty.c_cc[VTIME] = 1;  // 0.1 second timeout

        if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
            ::close(fd_);
            throw std::runtime_error("Failed to set terminal attributes");
        }

        // Flush buffers
        tcflush(fd_, TCIOFLUSH);
#endif

        is_open_ = true;
        port_name_ = port;
    }

    /**
     * @brief Close serial port
     */
    void close() {
        if (is_open_) {
#ifndef _WIN32
            ::close(fd_);
#endif
            is_open_ = false;
            fd_ = -1;
        }
    }

    /**
     * @brief Check if port is open
     */
    bool isOpen() const {
        return is_open_;
    }

    /**
     * @brief Write string to serial port
     * @param data String to write
     * @return Number of bytes written
     */
    size_t write(const std::string& data) {
        if (!is_open_) {
            throw std::runtime_error("Port not open");
        }

#ifndef _WIN32
        ssize_t bytes_written = ::write(fd_, data.c_str(), data.length());
        if (bytes_written < 0) {
            throw std::runtime_error("Write failed");
        }
        return static_cast<size_t>(bytes_written);
#else
        return 0;
#endif
    }

    /**
     * @brief Read available data from serial port
     * @param max_bytes Maximum bytes to read
     * @return String with read data
     */
    std::string read(size_t max_bytes = 256) {
        if (!is_open_) {
            throw std::runtime_error("Port not open");
        }

#ifndef _WIN32
        char buffer[256];
        ssize_t bytes_read = ::read(fd_, buffer, std::min(max_bytes, sizeof(buffer) - 1));
        if (bytes_read > 0) {
            buffer[bytes_read] = '\0';
            return std::string(buffer, bytes_read);
        }
#endif
        return "";
    }

    /**
     * @brief Read line from serial port (until '\n')
     * @param timeout_ms Timeout in milliseconds
     * @return Line read (without '\n')
     */
    std::string readline(int timeout_ms = 100) {
        if (!is_open_) {
            throw std::runtime_error("Port not open");
        }

        std::string line;
        auto start = std::chrono::steady_clock::now();

        while (true) {
            std::string data = read(1);
            if (!data.empty()) {
                char c = data[0];
                if (c == '\n') {
                    // Remove trailing '\r' if present
                    if (!line.empty() && line.back() == '\r') {
                        line.pop_back();
                    }
                    return line;
                }
                line += c;
            }

            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - start
            ).count();

            if (elapsed > timeout_ms) {
                break;
            }

            // Small delay to avoid busy-waiting
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }

        return line; // Return partial line on timeout
    }

    /**
     * @brief Check how many bytes are available to read
     */
    size_t available() {
        if (!is_open_) {
            return 0;
        }

#ifndef _WIN32
        int bytes_available = 0;
        ioctl(fd_, FIONREAD, &bytes_available);
        return static_cast<size_t>(bytes_available);
#else
        return 0;
#endif
    }

    /**
     * @brief Get port name
     */
    std::string getPort() const {
        return port_name_;
    }

private:
    int fd_;
    bool is_open_;
    std::string port_name_;
};
