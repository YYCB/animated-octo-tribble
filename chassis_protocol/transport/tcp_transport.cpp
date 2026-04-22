#include "tcp_transport.hpp"

#include <arpa/inet.h>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

namespace chassis {

static constexpr int    RECV_BUFFER_SIZE = 4096;
static constexpr int    MAX_RECONNECT_ATTEMPTS = 5;
static constexpr long   INITIAL_BACKOFF_MS = 500;
static constexpr long   MAX_BACKOFF_MS = 16000;

TcpTransport::TcpTransport(const TransportConfig& config)
    : config_(config) {}

TcpTransport::~TcpTransport() {
    disconnect();
}

bool TcpTransport::connectSocket() {
    // Resolve host
    struct addrinfo hints{};
    hints.ai_family   = AF_INET;
    hints.ai_socktype = SOCK_STREAM;

    struct addrinfo* res = nullptr;
    const std::string port_str = std::to_string(config_.port);
    int rc = ::getaddrinfo(config_.host.c_str(), port_str.c_str(), &hints, &res);
    if (rc != 0) {
        setError(std::string("getaddrinfo: ") + ::gai_strerror(rc));
        return false;
    }

    int fd = ::socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if (fd < 0) {
        setError(std::string("socket: ") + ::strerror(errno));
        ::freeaddrinfo(res);
        return false;
    }

    int opt = 1;
    ::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &opt, sizeof(opt));

    // Set connect timeout via SO_SNDTIMEO
    struct timeval tv{};
    tv.tv_sec  = static_cast<long>(config_.timeout_ms / 1000);
    tv.tv_usec = static_cast<long>((config_.timeout_ms % 1000) * 1000);
    ::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    rc = ::connect(fd, res->ai_addr, res->ai_addrlen);
    ::freeaddrinfo(res);

    if (rc < 0) {
        setError(std::string("connect: ") + ::strerror(errno));
        ::close(fd);
        return false;
    }

    socket_fd_ = fd;
    return true;
}

bool TcpTransport::connect() {
    if (connected_.load()) {
        return true;
    }

    if (!connectSocket()) {
        return false;
    }

    connected_.store(true);
    running_.store(true);
    receive_thread_ = std::thread(&TcpTransport::receiveLoop, this);
    return true;
}

void TcpTransport::disconnect() {
    running_.store(false);
    connected_.store(false);

    if (socket_fd_ >= 0) {
        ::shutdown(socket_fd_, SHUT_RDWR);
        ::close(socket_fd_);
        socket_fd_ = -1;
    }

    if (receive_thread_.joinable()) {
        receive_thread_.join();
    }
}

bool TcpTransport::send(const uint8_t* data, std::size_t len) {
    if (!connected_.load() || socket_fd_ < 0) {
        setError("Not connected");
        return false;
    }

    std::lock_guard<std::mutex> lock(send_mutex_);
    std::size_t total_written = 0;
    while (total_written < len) {
        ssize_t n = ::write(socket_fd_,
                            data + total_written,
                            len - total_written);
        if (n < 0) {
            if (errno == EINTR) continue;
            setError(std::string("write: ") + ::strerror(errno));
            connected_.store(false);
            return false;
        }
        if (n == 0) {
            setError("Connection closed during write");
            connected_.store(false);
            return false;
        }
        total_written += static_cast<std::size_t>(n);
    }
    return true;
}

void TcpTransport::setReceiveCallback(std::function<void(const uint8_t*, std::size_t)> cb) {
    receive_cb_ = std::move(cb);
}

bool TcpTransport::isConnected() const {
    return connected_.load();
}

std::string TcpTransport::getLastError() const {
    std::lock_guard<std::mutex> lock(error_mutex_);
    return last_error_;
}

void TcpTransport::setError(const std::string& err) {
    std::lock_guard<std::mutex> lock(error_mutex_);
    last_error_ = err;
}

void TcpTransport::receiveLoop() {
    uint8_t buf[RECV_BUFFER_SIZE];
    long backoff_ms = INITIAL_BACKOFF_MS;

    while (running_.load()) {
        if (!connected_.load() || socket_fd_ < 0) {
            // Attempt reconnect with exponential backoff
            std::this_thread::sleep_for(std::chrono::milliseconds(backoff_ms));
            backoff_ms = std::min(backoff_ms * 2, MAX_BACKOFF_MS);

            if (!running_.load()) break;

            if (connectSocket()) {
                connected_.store(true);
                backoff_ms = INITIAL_BACKOFF_MS;
            }
            continue;
        }

        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(socket_fd_, &read_fds);

        struct timeval tv{};
        tv.tv_sec  = 0;
        tv.tv_usec = 100 * 1000; // 100ms polling interval

        int sel = ::select(socket_fd_ + 1, &read_fds, nullptr, nullptr, &tv);
        if (sel < 0) {
            if (errno == EINTR) continue;
            setError(std::string("select: ") + ::strerror(errno));
            connected_.store(false);
            ::close(socket_fd_);
            socket_fd_ = -1;
            continue;
        }

        if (sel == 0) continue; // timeout, loop

        ssize_t n = ::recv(socket_fd_, buf, sizeof(buf), 0);
        if (n < 0) {
            if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) continue;
            setError(std::string("recv: ") + ::strerror(errno));
            connected_.store(false);
            ::close(socket_fd_);
            socket_fd_ = -1;
            continue;
        }
        if (n == 0) {
            // Peer disconnected
            setError("Remote host closed connection");
            connected_.store(false);
            ::close(socket_fd_);
            socket_fd_ = -1;
            continue;
        }

        if (receive_cb_) {
            receive_cb_(buf, static_cast<std::size_t>(n));
        }
    }
}

} // namespace chassis
