#ifndef ODOM_PARSER_HPP
#define ODOM_PARSER_HPP

#include "parser_base.hpp"
#include "shared_memory.hpp"
#include "datatypes.hpp"
#include <nlohmann/json.hpp>
#include <memory>

namespace turtlebot4 {

using json = nlohmann::json;

class OdomParser : public ParserBase<ParsedOdom> {
public:
    OdomParser(ThreadSafeQueue<std::string>& input_queue);
    ~OdomParser() override;

    // Attach shared memory for odom output
    void attach_shared_memory(SharedMemory<SharedOdometry>* shm);

protected:
    ParsedOdom parse(const std::string& json_str) override;
    void on_parsed(const ParsedOdom& data) override;

private:
    SharedMemory<SharedOdometry>* odom_shm_;
    uint64_t sequence_;
};

} // namespace turtlebot4

#endif // ODOM_PARSER_HPP
