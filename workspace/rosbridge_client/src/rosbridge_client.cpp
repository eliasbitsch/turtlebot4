#include "rosbridge_client.hpp"
#include <iostream>

namespace turtlebot4 {

RosbridgeClient::RosbridgeClient(const std::string& uri)
    : uri_(uri), connected_(false), running_(false) {

    // Disable logging
    client_.clear_access_channels(websocketpp::log::alevel::all);
    client_.clear_error_channels(websocketpp::log::elevel::all);

    client_.init_asio();

    // Set handlers
    client_.set_open_handler(
        std::bind(&RosbridgeClient::on_open, this, std::placeholders::_1));
    client_.set_close_handler(
        std::bind(&RosbridgeClient::on_close, this, std::placeholders::_1));
    client_.set_fail_handler(
        std::bind(&RosbridgeClient::on_fail, this, std::placeholders::_1));
    client_.set_message_handler(
        std::bind(&RosbridgeClient::on_message, this, std::placeholders::_1, std::placeholders::_2));
}

RosbridgeClient::~RosbridgeClient() {
    stop();
}

bool RosbridgeClient::connect() {
    websocketpp::lib::error_code ec;
    WsClient::connection_ptr con = client_.get_connection(uri_, ec);

    if (ec) {
        std::cerr << "[RosbridgeClient] Connection error: " << ec.message() << std::endl;
        return false;
    }

    client_.connect(con);
    return true;
}

void RosbridgeClient::disconnect() {
    if (connected_) {
        websocketpp::lib::error_code ec;
        client_.close(connection_, websocketpp::close::status::normal, "Client disconnect", ec);
        connected_ = false;
    }
}

void RosbridgeClient::run() {
    running_ = true;
    client_.run();
    running_ = false;
}

void RosbridgeClient::stop() {
    if (running_) {
        disconnect();
        client_.stop();
    }
}

void RosbridgeClient::subscribe(const std::string& topic, MessageCallback callback) {
    {
        std::lock_guard<std::mutex> lock(callback_mutex_);
        callbacks_[topic] = callback;
    }

    if (connected_) {
        json sub_msg = {
            {"op", "subscribe"},
            {"topic", topic}
        };
        send_json(sub_msg);
        std::cout << "[RosbridgeClient] Subscribed to " << topic << std::endl;
    }
}

void RosbridgeClient::unsubscribe(const std::string& topic) {
    {
        std::lock_guard<std::mutex> lock(callback_mutex_);
        callbacks_.erase(topic);
    }

    if (connected_) {
        json unsub_msg = {
            {"op", "unsubscribe"},
            {"topic", topic}
        };
        send_json(unsub_msg);
    }
}

void RosbridgeClient::advertise(const std::string& topic, const std::string& msg_type) {
    if (connected_) {
        json adv_msg = {
            {"op", "advertise"},
            {"topic", topic},
            {"type", msg_type}
        };
        send_json(adv_msg);
        std::cout << "[RosbridgeClient] Advertised " << topic << " as " << msg_type << std::endl;
    }
}

void RosbridgeClient::publish(const std::string& topic, const std::string& msg_type, const json& msg) {
    if (connected_) {
        json pub_msg = {
            {"op", "publish"},
            {"topic", topic},
            {"msg", msg}
        };
        send_json(pub_msg);
    }
}

void RosbridgeClient::send_json(const json& j) {
    std::lock_guard<std::mutex> lock(send_mutex_);
    websocketpp::lib::error_code ec;
    client_.send(connection_, j.dump(), websocketpp::frame::opcode::text, ec);
    if (ec) {
        std::cerr << "[RosbridgeClient] Send error: " << ec.message() << std::endl;
    }
}

void RosbridgeClient::on_open(connection_hdl hdl) {
    connection_ = hdl;
    connected_ = true;
    std::cout << "[RosbridgeClient] Connected to " << uri_ << std::endl;

    // Re-subscribe to all registered topics
    std::lock_guard<std::mutex> lock(callback_mutex_);
    for (const auto& [topic, _] : callbacks_) {
        json sub_msg = {
            {"op", "subscribe"},
            {"topic", topic}
        };
        send_json(sub_msg);
        std::cout << "[RosbridgeClient] Subscribed to " << topic << std::endl;
    }
}

void RosbridgeClient::on_close(connection_hdl hdl) {
    (void)hdl;
    connected_ = false;
    std::cout << "[RosbridgeClient] Connection closed" << std::endl;
}

void RosbridgeClient::on_fail(connection_hdl hdl) {
    (void)hdl;
    connected_ = false;
    std::cerr << "[RosbridgeClient] Connection failed" << std::endl;
}

void RosbridgeClient::on_message(connection_hdl hdl, WsClient::message_ptr msg) {
    (void)hdl;

    try {
        json j = json::parse(msg->get_payload());

        if (j.contains("op") && j["op"] == "publish" && j.contains("topic")) {
            std::string topic = j["topic"];

            std::lock_guard<std::mutex> lock(callback_mutex_);
            auto it = callbacks_.find(topic);
            if (it != callbacks_.end() && it->second) {
                it->second(topic, j.contains("msg") ? j["msg"] : json{});
            }
        }
    } catch (const json::parse_error& e) {
        std::cerr << "[RosbridgeClient] JSON parse error: " << e.what() << std::endl;
    }
}

} // namespace turtlebot4
