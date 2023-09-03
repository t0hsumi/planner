#include "nestedlist.hpp"

static NestedList parse_nested_tokens(const std::vector<std::string> &tokens,
                                      const size_t &size, size_t &index) {
  // parse lisp-like tokens recursively
  NestedList ret;

  for (; index < size; ++index) {
    if (tokens[index] == ")") {
      return ret;
    } else if (tokens[index] == "(") {
      ++index;
      NestedList lst = parse_nested_tokens(tokens, size, index);
      ret.addNestedList(lst);
    } else {
      ret.addWord(tokens[index]);
    }
  }

  std::cerr << "missing closing parenthesis" << std::endl;
  exit(1);
}

NestedList generate_nestedlist(const std::vector<std::string> &tokens) {
  assert(tokens[0] == "(");

  size_t index = 1;
  return parse_nested_tokens(tokens, tokens.size(), index);
}
