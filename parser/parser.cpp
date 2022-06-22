#include <modern/parser.hpp>
namespace CloudParserLibrary {

void ParserFactory::register_format(std::string format, InterfaceParser* ptr) {
  // ParserFactory::factories.insert(std::make_pair(format, creator));
  ParserFactory::factories[format] = ptr;
}

size_t ParserFactory::get_size() { return factories.size(); }

InterfaceParser* ParserFactory::get_parser(const std::string format) {
  try {
    InterfaceParser* factory_pos = factories.at(format);
    return factory_pos;
  } catch (const std::out_of_range& e) {
    pcl::console::print_error("An exception occurred: Format %s is not supported\n", format.c_str());
    std::exit(-1);
  }
}

}  // namespace CloudParserLibrary
