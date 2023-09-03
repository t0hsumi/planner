#ifndef PLANNER_HPP
#define PLANNER_HPP

#include <algorithm>
#include <cassert>
#include <regex>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "parser.hpp"

class Operator {
public:
  Operator() {}
  Operator(std::string name, std::multiset<std::string> precond,
           std::multiset<std::string> add, std::multiset<std::string> del)
      : name(name), preconditions(precond), add_effects(add), del_effects(del) {
  }

  std::string name;
  std::multiset<std::string> preconditions;
  std::multiset<std::string> add_effects;
  std::multiset<std::string> del_effects;

  bool applicable(const std::multiset<std::string> &state) {
    for (auto pred : (*this).preconditions) {
      if (state.find(pred) == state.end())
        return false;
    }
    return true;
    /* return (*this).preconditions <= state; */
  }

  std::multiset<std::string> apply(const std::multiset<std::string> &state) {
    assert(applicable(state));

    std::multiset<std::string> tmp;
    std::multiset<std::string> ret;
    std::set_difference(
        std::begin(state), std::end(state), std::begin((*this).del_effects),
        std::end((*this).del_effects), std::inserter(tmp, std::end(tmp)));
    std::set_union(
        std::begin(tmp), std::end(tmp), std::begin((*this).add_effects),
        std::end((*this).add_effects), std::inserter(ret, std::end(ret)));
    return ret;
  }
};

class Task {
public:
  Task(std::string name, std::multiset<std::string> facts,
       std::multiset<std::string> init, std::multiset<std::string> goal,
       std::vector<Operator> operators)
      : name(name), facts(facts), init(init), goal(goal), operators(operators) {
  }

  std::string name;
  std::multiset<std::string> facts;
  std::multiset<std::string> init;
  std::multiset<std::string> goal;
  std::vector<Operator> operators;

  bool goal_reached(const std::multiset<std::string> &state) const {
    /* return goal <= state; */
    for (auto f : goal) {
      if (state.find(f) == state.end())
        return false;
    }
    return true;
  }

  std::vector<std::pair<Operator, std::multiset<std::string>>>
  get_successor_states(std::multiset<std::string> state) const {
    std::vector<std::pair<Operator, std::multiset<std::string>>> ret;

    for (auto op : operators) {
      if (op.applicable(state)) {
        ret.push_back(std::make_pair(op, op.apply(state)));
      }
    }
    return ret;
  }
};

Task generate_task(const DomainDef &dom, const ProblemDef &prob);

class SearchNode {
public:
  SearchNode() {}
  SearchNode(std::multiset<std::string> state, SearchNode *parent,
             std::string action, size_t cost)
      : state(state), parent(parent), action(action), g(cost) {}

  std::multiset<std::string> state;
  SearchNode *parent;
  std::string action;
  size_t g;

  std::vector<std::string> extract_solution() {
    std::vector<std::string> solution;
    SearchNode *node = this;

    while (node->parent != nullptr) {
      solution.push_back(node->action);
      node = node->parent;
    }
    std::reverse(solution.begin(), solution.end());

    return solution;
  }
};

SearchNode *make_root_node(std::multiset<std::string> initial_state);
SearchNode *make_child_node(SearchNode *parent_node, const std::string &action,
                            const std::multiset<std::string> &state);

#endif
