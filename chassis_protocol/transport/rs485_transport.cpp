#include "rs485_transport.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

namespace chassis {

static speed_t baudRateToSpeed(uint32_t baud) {
    switch (baud) {
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 921600: return B921600;
        default:     return B115200;
    }
}

Rs485Transport::Rs485Transport(const TransportConfig& config)
    : config_(config) {}

Rs485Transport::~Rs485Transport() {
    disconnect();
}

bool Rs485Transport::connect() {
    if (connected_.load()) {
        return true;
    }

    fd_ = ::open(config_.device_path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        setError(std::string("open: ") + ::strerror(errno));
        return false;
    }

    if (!configurePort()) {
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    connected_.store(true);
    running_.store(true);
    receive_thread_ = std::thread(&Rs485Transport::receiveLoop, this);
    return true;
}

bool Rs485Transport::configurePort() {
    struct termios tty{};
    if (::tcgetattr(fd_, &tty) != 0) {
        setError(std::string("tcgetattr: ") + ::strerror(errno));
        return false;
    }

    const speed_t speed = baudRateToSpeed(config_.baud_rate);
    ::cfsetispeed(&tty, speed);
    ::cfsetospeed(&tty, speed);

    // Raw mode
    ::cfmakeraw(&tty);

    // 8 data bits, no parity, 1 stop bit (8N1)
    tty.c_cflag &= ~static_cast<tcflag_t>(PARENB);  // No parity
    tty.c_cflag &= ~static_cast<tcflag_t>(CSTOPB);  // 1 stop bit
    tty.c_cflag &= ~static_cast<tcflag_t>(CSIZE);
    tty.c_cflag |= CS8;                              // 8 data bits
    tty.c_cflag &= ~static_cast<tcflag_t>(CRTSCTS); // No hardware flow control
    tty.c_cflag |= CREAD | CLOCAL;                   // Enable receiver, ignore modem lines

    // Non-blocking read
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 1; // 100ms inter-character timeout

    if (::tcsetattr(fd_, TCSANOW, &tty) != 0) {
        setError(std::string("tcsetattr: ") + ::strerror(errno));
        return false;
    }

    ::tcflush(fd_, TCIOFLUSH);
    return true;
}

void Rs485Transport::disconnect() {
    running_.store(false);
    connected_.store(false);

    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }

    if (receive_thread_.joinable()) {
        receive_thread_.join();
    }
}

bool Rs485Transport::send(const uint8_t* data, std::size_t len) {
    if (!connected_.load() || fd_ < 0) {
        setError("Not connected");
        return false;
    }

    std::lock_guard<std::mutex> lock(send_mutex_);

    // Drain any pending receive data before transmitting (half-duplex)
    ::tcflush(fd_, TCIFLUSH);

    std::size_t total_written = 0;
    while (total_written < len) {
        ssize_t n = ::write(fd_, data + total_written, len - total_written);
        if (n < 0) {
            if (errno == EINTR) continue;
            setError(std::string("write: ") + ::strerror(errno));
            return false;
        }
        total_written += static_cast<std::size_t>(n);
    }

    // Ensure all bytes are physically sent before releasing bus
    ::tcdrain(fd_);
    return true;
}

void Rs485Transport::setReceiveCallback(std::function<void(const uint8_t*, std::size_t)> cb) {
    receive_cb_ = std::move(cb);
}

bool Rs485Transport::isConnected() const {
    return connected_.load();
}

std::string Rs485Transport::getLastError() const {
    std::lock_guard<std::mutex> lock(error_mutex_);
    return last_error_;
}

void Rs485Transport::setError(const std::string& err) {
    std::lock_guard<std::mutex> lock(error_mutex_);
    last_error_ = err;
}

void Rs485Transport::receiveLoop() {
    uint8_t buf[512];

    while (running_.load()) {
        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(fd_, &read_fds);

        struct timeval tv{};
        tv.tv_sec  = 0;
        tv.tv_usec = 100 * 1000; // 100ms polling interval

        int sel = ::select(fd_ + 1, &read_fds, nullptr, nullptr, &tv);
        if (sel < 0) {
            if (errno == EINTR) continue;
            setError(std::string("select: ") + ::strerror(errno));
            break;
        }
        if (sel == 0) continue;

        ssize_t n = ::read(fd_, buf, sizeof(buf));
        if (n < 0) {
            if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) continue;
            setError(std::string("read: ") + ::strerror(errno));
            break;
        }

        if (receive_cb_ && n > 0) {
            receive_cb_(buf, static_cast<std::size_t>(n));
        }
    }
}

} // namespace chassis
