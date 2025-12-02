#ifndef BUMPER_PARSER_HPP
#define BUMPER_PARSER_HPP

#include "parser_base.hpp"
#include "datatypes.hpp"
#include <nlohmann/json.hpp>

namespace turtlebot4 {

using json = nlohmann::json;

class BumperParser : public ParserBase<ParsedBumper> {
public:
    BumperParser(ThreadSafeQueue<std::string>& input_queue);
    ~BumperParser() override = default;

protected:
    ParsedBumper parse(const std::string& json_str) override;
};

} // namespace turtlebot4

#endif // BUMPER_PARSER_HPP
