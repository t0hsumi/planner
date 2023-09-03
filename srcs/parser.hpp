#ifndef PARSER_HPP
#define PARSER_HPP

#include <queue>
#include <set>
#include <unordered_map>

#include "lispiterator.hpp"

class Type {
public:
  Type(std::string name, std::string parent) : name(name), parent(parent) {}

  std::string name;
  std::string parent;
};

class Variable {
public:
  Variable(std::string name, std::string type) : name(name), type(type) {}

  std::string name;
  std::string type;
};

class Predicate {
public:
  std::string name;
  std::vector<Variable> params;
};

class Cond {
public:
  std::string key;
  std::vector<Variable> variables;
};

class Formula {
public:
  std::vector<Cond> pos;
  std::vector<Cond> neg;
};

class ActionStmt {
public:
  std::string name;
  std::vector<Variable> parameters;
  Formula precond;
  Formula effect;
};

class DomainDef {
public:
  std::string name;
  std::vector<std::string> requirements;
  std::unordered_map<std::string, Type> types;
  std::vector<Predicate> preconditions;
  std::vector<ActionStmt> actions;
};

DomainDef parse_domain(LispIterator &iter);

class Object {
public:
  Object(std::string name, std::string type) : name(name), type(type) {}

  std::string name;
  std::string type;
};

class Fact {
public:
  Fact() {}
  Fact(std::string name) : name(name) {}
  std::string name;
};

class ProblemDef {
public:
  std::string probname;
  std::string domname;
  std::vector<Variable> objs;
  std::vector<Fact> init;
  std::vector<Fact> goal;
};

ProblemDef parse_problem(LispIterator &iter);

#endif
