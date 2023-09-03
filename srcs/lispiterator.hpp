#ifndef LISPITERATOR_HPP
#define LISPITERATOR_HPP

#include "nestedlist.hpp"

class LispIterator {
public:
  LispIterator() {}
  LispIterator(NestedList lst) : position(0), contents(lst.getList()) {}

  bool is_word() const { return contents[position].isword(); }
  bool is_structure() const { return !contents[position].isword(); }
  bool empty() const { return position == contents.size(); }
  std::string get_word() const {
    _raise_if(is_structure(), "not a word in LispIterator.get_word()");
    return contents[position].getword();
  }
  LispIterator peek() const {
    _raise_if(is_word(), "not a structure in LispIterator.peek().");
    _raise_if(position == contents.size(),
              "end of contents in LispIterator.peek()");
    return LispIterator(contents[position]);
  }
  void next() {
    _raise_if(position == contents.size(),
              "end of contents in LispIterator.next()");
    position += 1;
  }
  bool try_match_word(std::string word) {
    if (is_word() && get_word() == word) {
      next();
      return true;
    } else
      return false;
  }
  void match_end() const {
    _raise_if(position != contents.size(), "expected to be at end");
  }

private:
  size_t position;
  std::vector<NestedList> contents;

  void _raise_if(bool condition, std::string msg) const {
    if (condition) {
      std::cerr << msg << std::endl;
      exit(1);
    }
  }
};

#endif
