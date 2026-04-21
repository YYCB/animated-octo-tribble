#pragma once

#include "ichassis_controller.hpp"
#include "../transport/itransport.hpp"
#include "../frame/frame_codec.hpp"
#include "version_negotiator.hpp"

#include <atomic>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace chassis {

class ChassisHal : public IChassisController {
public:
    explicit ChassisHal(std::unique_ptr<ITransport> transport,
                        uint32_t protocol_version = 1);
    ~ChassisHal() override;

    bool connect();
    void disconnect();
    bool isConnected() const;

    // IChassisController
    bool sendVelocity(double vx, double vy, double omega) override;
    bool sendLightCmd(LightMode mode, uint8_t r, uint8_t g, uint8_t b, double blink_hz) override;
    bool emergencyStop(const std::string& reason) override;
    ChassisStatus  getStatus()      const override;
    ChassisInfo    getInfo()        const override;
    OdometryData   getOdometry()    const override;
    BatteryData    getBatteryInfo() const override;
    void setStatusCallback(std::function<void(const ChassisStatus&)> cb) override;
    void setOdometryCallback(std::function<void(const OdometryData&)> cb) override;
    void setBatteryCallback(std::function<void(const BatteryData&)> cb) override;

protected:
    std::unique_ptr<ITransport> transport_;
    uint32_t                    protocol_version_;
    uint32_t                    seq_{0};

    mutable std::mutex status_mutex_;
    mutable std::mutex odom_mutex_;
    mutable std::mutex battery_mutex_;

    ChassisStatus current_status_;
    OdometryData  current_odom_;
    BatteryData   current_battery_;
    ChassisInfo   info_;

    std::function<void(const ChassisStatus&)> status_cb_;
    std::function<void(const OdometryData&)>  odom_cb_;
    std::function<void(const BatteryData&)>   battery_cb_;

    std::atomic<bool> connected_{false};
    std::atomic<bool> running_{false};
    std::thread       reconnect_thread_;
    std::mutex        cb_mutex_;

    StreamDecoder    stream_decoder_;
    VersionNegotiator negotiator_;

    void onDataReceived(const uint8_t* data, std::size_t len);
    void onFrameDecoded(const DecodedFrame& frame);
    void dispatchStatusFrame(const DecodedFrame& frame);
    void dispatchOdometryFrame(const DecodedFrame& frame);
    void dispatchBatteryFrame(const DecodedFrame& frame);
    void reconnectLoop();

    std::vector<uint8_t> buildVelocityPayload(double vx, double vy, double omega);
    std::vector<uint8_t> buildLightPayload(LightMode mode, uint8_t r, uint8_t g, uint8_t b, double hz);
    std::vector<uint8_t> buildStopPayload(const std::string& reason);
    uint32_t             nextSeq();
};

} // namespace chassis
