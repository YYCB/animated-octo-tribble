#include "udp_transport.hpp"

#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

namespace chassis {

static constexpr int UDP_RECV_BUFFER_SIZE = 65535;

UdpTransport::UdpTransport(const TransportConfig& config)
    : config_(config) {}

UdpTransport::~UdpTransport() {
    disconnect();
}

bool UdpTransport::connect() {
    if (connected_.load()) {
        return true;
    }

    socket_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0) {
        setError(std::string("socket: ") + ::strerror(errno));
        return false;
    }

    // Resolve remote address
    struct addrinfo hints{};
    hints.ai_family   = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;

    struct addrinfo* res = nullptr;
    const std::string port_str = std::to_string(config_.port);
    int rc = ::getaddrinfo(config_.host.c_str(), port_str.c_str(), &hints, &res);
    if (rc != 0) {
        setError(std::string("getaddrinfo: ") + ::gai_strerror(rc));
        ::close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }
    std::memcpy(&remote_addr_, res->ai_addr, sizeof(remote_addr_));
    ::freeaddrinfo(res);

    // Bind a local port for receiving (bind to any available port)
    std::memset(&local_addr_, 0, sizeof(local_addr_));
    local_addr_.sin_family      = AF_INET;
    local_addr_.sin_addr.s_addr = INADDR_ANY;
    local_addr_.sin_port        = 0; // OS assigns port

    int opt = 1;
    ::setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    if (::bind(socket_fd_,
               reinterpret_cast<const struct sockaddr*>(&local_addr_),
               sizeof(local_addr_)) < 0) {
        setError(std::string("bind: ") + ::strerror(errno));
        ::close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }

    connected_.store(true);
    running_.store(true);
    receive_thread_ = std::thread(&UdpTransport::receiveLoop, this);
    return true;
}

void UdpTransport::disconnect() {
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

bool UdpTransport::send(const uint8_t* data, std::size_t len) {
    if (!connected_.load() || socket_fd_ < 0) {
        setError("Not connected");
        return false;
    }

    std::lock_guard<std::mutex> lock(send_mutex_);
    ssize_t n = ::sendto(socket_fd_,
                         data,
                         len,
                         0,
                         reinterpret_cast<const struct sockaddr*>(&remote_addr_),
                         sizeof(remote_addr_));
    if (n < 0) {
        setError(std::string("sendto: ") + ::strerror(errno));
        return false;
    }
    return static_cast<std::size_t>(n) == len;
}

void UdpTransport::setReceiveCallback(std::function<void(const uint8_t*, std::size_t)> cb) {
    receive_cb_ = std::move(cb);
}

bool UdpTransport::isConnected() const {
    return connected_.load();
}

std::string UdpTransport::getLastError() const {
    std::lock_guard<std::mutex> lock(error_mutex_);
    return last_error_;
}

void UdpTransport::setError(const std::string& err) {
    std::lock_guard<std::mutex> lock(error_mutex_);
    last_error_ = err;
}

void UdpTransport::receiveLoop() {
    std::vector<uint8_t> buf(UDP_RECV_BUFFER_SIZE);

    while (running_.load()) {
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
            break;
        }
        if (sel == 0) continue;

        struct sockaddr_in sender_addr{};
        socklen_t sender_len = sizeof(sender_addr);

        ssize_t n = ::recvfrom(socket_fd_,
                               buf.data(),
                               buf.size(),
                               0,
                               reinterpret_cast<struct sockaddr*>(&sender_addr),
                               &sender_len);
        if (n < 0) {
            if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) continue;
            setError(std::string("recvfrom: ") + ::strerror(errno));
            break;
        }

        if (receive_cb_ && n > 0) {
            receive_cb_(buf.data(), static_cast<std::size_t>(n));
        }
    }
}

} // namespace chassis
