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

// clause version of operator
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

// vector version of operator
class VecOperator {
public:
  VecOperator() {}
  VecOperator(std::string name, std::vector<size_t> precond,
              std::vector<size_t> add, std::vector<size_t> del)
      : name(name), preconditions(precond), add_eff(add), del_eff(del) {}

  std::string name;
  std::vector<size_t> preconditions;
  std::vector<size_t> add_eff;
  std::vector<size_t> del_eff;

  bool applicable(const std::vector<bool> &state) {
    for (auto elem : preconditions) {
      if (!state[elem])
        return false;
    }
    return true;
  }

  std::vector<bool> apply(const std::vector<bool> &state) {
    auto ret = state;
    for (auto idx : del_eff)
      ret[idx] = false;
    for (auto idx : add_eff)
      ret[idx] = true;
    return ret;
  }
};

class Task {
public:
  Task(std::string name, std::vector<bool> init, std::multiset<size_t> goal,
       std::vector<VecOperator> operators, int batch_size)
      : name(name), init(init), goal(goal), operators(operators), batch_size(batch_size) {}

  std::string name;
  std::vector<bool> init;
  std::multiset<size_t> goal;
  std::vector<VecOperator> operators;
  int batch_size;

  bool goal_reached(const std::vector<bool> &state) const {
    for (auto f : goal) {
      if (!state[f])
        return false;
    }
    return true;
  }

  std::vector<std::pair<VecOperator, std::vector<bool>>>
  get_successor_states(std::vector<bool> state, size_t id) const {
    std::vector<std::vector<std::pair<VecOperator, std::vector<bool>>>> ret(
        batch_size);

    for (auto op : operators) {
      if (op.applicable(state))
        ret[id].push_back(std::make_pair(op, op.apply(state)));
    }

    return ret[id];
  }
};

Task generate_task(const DomainDef &dom, const ProblemDef &prob, int batch_size);

class SearchNode {
public:
  SearchNode() {}
  SearchNode(std::vector<bool> state, SearchNode *parent, std::string action,
             size_t cost)
      : state(state), parent(parent), action(action), g(cost) {}

  std::vector<bool> state;
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

SearchNode *make_root_node(std::vector<bool> initial_state);
SearchNode *make_child_node(SearchNode *parent_node, const std::string &action,
                            const std::vector<bool> &state);

#endif
