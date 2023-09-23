#ifndef NESTEDLIST_HPP
#define NESTEDLIST_HPP

#include <cassert>
#include <iostream>
#include <string>
#include <vector>

class NestedList {
public:
  NestedList() {}

  void addWord(std::string in) { data.push_back(NestedList(in)); }
  void addNestedList(const NestedList &in) { data.push_back(in); }

  const std::vector<NestedList> &getList() const {
    assert(!isWord);
    return data;
  }
  std::string getword() const {
    assert(isWord);
    return str;
  }

  bool isword() const { return isWord; }

private:
  NestedList(std::string str) : isWord(true), str(str) {}

  std::vector<NestedList> data;
  bool isWord = false;
  std::string str;
};

NestedList generate_nestedlist(const std::vector<std::string> &tokens);

#endif
