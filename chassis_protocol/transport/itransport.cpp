#include "itransport.hpp"
#include "tcp_transport.hpp"
#include "udp_transport.hpp"
#include "rs485_transport.hpp"

#include <stdexcept>

namespace chassis {

std::unique_ptr<ITransport> ITransport::create(TransportType type, const TransportConfig& config) {
    switch (type) {
        case TransportType::TCP:
            return std::make_unique<TcpTransport>(config);
        case TransportType::UDP:
            return std::make_unique<UdpTransport>(config);
        case TransportType::RS485:
            return std::make_unique<Rs485Transport>(config);
        default:
            throw std::invalid_argument("Unknown TransportType");
    }
}

} // namespace chassis
