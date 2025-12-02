#ifndef SCAN_PARSER_HPP
#define SCAN_PARSER_HPP

#include "parser_base.hpp"
#include "datatypes.hpp"
#include <nlohmann/json.hpp>

namespace turtlebot4 {

using json = nlohmann::json;

class ScanParser : public ParserBase<ParsedScan> {
public:
    ScanParser(ThreadSafeQueue<std::string>& input_queue);
    ~ScanParser() override = default;

protected:
    ParsedScan parse(const std::string& json_str) override;
};

} // namespace turtlebot4

#endif // SCAN_PARSER_HPP
