#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>
#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>

// For JSON, you can use nlohmann/json or send raw strings
// #include <nlohmann/json.hpp>

using websocketpp::connection_hdl;
typedef websocketpp::client<websocketpp::config::asio_client> client;

// Function to create a dynamic cmd_vel message
std::string create_cmd_vel_msg(double linear_x, double linear_y, double linear_z,
                                 double angular_x, double angular_y, double angular_z) {
    std::ostringstream oss;
    oss << R"({"op":"publish","topic":"/cmd_vel","msg":{"header":{"stamp":{"sec":0,"nanosec":0},"frame_id":""},"twist":{"linear":{"x":)"
        << linear_x << R"(,"y":)" << linear_y << R"(,"z":)" << linear_z
        << R"(},"angular":{"x":)" << angular_x << R"(,"y":)" << angular_y << R"(,"z":)" << angular_z
        << R"(}}}})";
    return oss.str();
}

// Function to subscribe to a topic
std::string create_subscribe_msg(const std::string& topic) {
    return R"({"op":"subscribe","topic":")" + topic + R"("})";
}

// Handler for incoming messages (subscriptions)
void on_message(client* c, connection_hdl hdl, client::message_ptr msg) {
    std::cout << "Received: " << msg->get_payload() << std::endl;
}

void on_open(client* c, connection_hdl hdl) {
    // Subscribe to /scan topic
    std::string scan_sub = create_subscribe_msg("/scan");
    c->send(hdl, scan_sub, websocketpp::frame::opcode::text);
    std::cout << "Subscribed to /scan" << std::endl;

    // Subscribe to /odom topic
    std::string odom_sub = create_subscribe_msg("/odom");
    c->send(hdl, odom_sub, websocketpp::frame::opcode::text);
    std::cout << "Subscribed to /odom" << std::endl;

    // Send dynamic geometry_msgs/TwistStamped message to /cmd_vel
    // Example: move backwards with linear.x = -0.2
    std::string cmd_msg = create_cmd_vel_msg(-0.2, 0.0, 0.0, 0.0, 0.0, 0.0);
    c->send(hdl, cmd_msg, websocketpp::frame::opcode::text);
    std::cout << "Sent cmd_vel: " << cmd_msg << std::endl;
}

int main() {
    client c;
    // Disable all access and error logging
    c.clear_access_channels(websocketpp::log::alevel::all);
    c.clear_error_channels(websocketpp::log::elevel::all);
    c.init_asio();

    // Set handlers
    c.set_open_handler(std::bind(&on_open, &c, std::placeholders::_1));
    c.set_message_handler(std::bind(&on_message, &c, std::placeholders::_1, std::placeholders::_2));

    std::string uri = "ws://localhost:9090";
    websocketpp::lib::error_code ec;
    client::connection_ptr con = c.get_connection(uri, ec);
    if (ec) {
        std::cout << "Could not create connection because: " << ec.message() << std::endl;
        return 1;
    }
    c.connect(con);
    c.run();
    return 0;
}
