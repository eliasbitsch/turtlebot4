#ifndef ROSBRIDGE_CLIENT_HPP
#define ROSBRIDGE_CLIENT_HPP

#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>
#include <nlohmann/json.hpp>

#include <string>
#include <functional>
#include <mutex>
#include <atomic>
#include <thread>
#include <map>

namespace turtlebot4 {

using json = nlohmann::json;
using websocketpp::connection_hdl;
typedef websocketpp::client<websocketpp::config::asio_client> WsClient;

class RosbridgeClient {
public:
    using MessageCallback = std::function<void(const std::string& topic, const json& msg)>;

    RosbridgeClient(const std::string& uri = "ws://localhost:9090");
    ~RosbridgeClient();

    // Non-copyable
    RosbridgeClient(const RosbridgeClient&) = delete;
    RosbridgeClient& operator=(const RosbridgeClient&) = delete;

    // Connect to rosbridge server
    bool connect();

    // Disconnect from server
    void disconnect();

    // Check connection status
    bool is_connected() const { return connected_; }

    // Subscribe to a topic with callback
    void subscribe(const std::string& topic, MessageCallback callback);

    // Unsubscribe from a topic
    void unsubscribe(const std::string& topic);

    // Publish a message to a topic
    void publish(const std::string& topic, const std::string& msg_type, const json& msg);

    // Advertise a topic (required before publishing)
    void advertise(const std::string& topic, const std::string& msg_type);

    // Run the client (blocking) - call from dedicated thread
    void run();

    // Stop the client
    void stop();

private:
    void on_open(connection_hdl hdl);
    void on_close(connection_hdl hdl);
    void on_fail(connection_hdl hdl);
    void on_message(connection_hdl hdl, WsClient::message_ptr msg);

    void send_json(const json& j);

    std::string uri_;
    WsClient client_;
    connection_hdl connection_;
    std::atomic<bool> connected_;
    std::atomic<bool> running_;

    std::mutex callback_mutex_;
    std::map<std::string, MessageCallback> callbacks_;

    std::mutex send_mutex_;
};

} // namespace turtlebot4

#endif // ROSBRIDGE_CLIENT_HPP
