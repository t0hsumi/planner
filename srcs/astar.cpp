#include <unordered_map>

#include "astar.hpp"
#include "planner.hpp"
#include "trie.hpp"

static std::tuple<size_t, size_t, size_t, SearchNode *>
make_open_entry(SearchNode *node, size_t h, size_t node_tiebreaker) {
  return std::make_tuple(node->g + h, h, node_tiebreaker, node);
}

struct TupleCmp {
  bool
  operator()(const std::tuple<size_t, size_t, size_t, SearchNode *> &l,
             const std::tuple<size_t, size_t, size_t, SearchNode *> &r) const {
    if (std::get<0>(l) != std::get<0>(r))
      return std::get<0>(l) > std::get<0>(r);
    else if (std::get<1>(l) != std::get<1>(r))
      return std::get<1>(l) > std::get<1>(r);
    else
      return std::get<2>(l) > std::get<2>(r);
  }
};

std::vector<std::string> astar(const Task &task, bool use_trie) {
  std::priority_queue<
      std::tuple<size_t, size_t, size_t, SearchNode *>,
      std::vector<std::tuple<size_t, size_t, size_t, SearchNode *>>, TupleCmp>
      open;
  std::unordered_map<std::vector<bool>, size_t> state_cost;
  std::vector<SearchNode *> addrs;
  hFFHeuristic heuristic(task);
  Trie trie;

  if (use_trie) {
    std::cout << "trie init begin" << std::endl;
    trie.initialize(task);
    std::cout << "trie init end" << std::endl;
  }

  state_cost[task.init] = 0;
  size_t node_tiebreaker = 0;

  auto root = make_root_node(task.init);
  auto init_h = heuristic(*root);
  std::cout << "Initial h value: " << init_h << std::endl;

  addrs.push_back(root);
  open.push(make_open_entry(root, init_h, node_tiebreaker));

  std::cout << "Search start: " << task.name << std::endl;

  int niteration = 0;

  while (!open.empty()) {
    auto next = open.top();
    open.pop();
    SearchNode *pop_node = std::get<3>(next);
    auto pop_state = pop_node->state;

    bool is_lowest_cost = false;
    if (state_cost.find(pop_state) != state_cost.end() &&
        state_cost[pop_state] == pop_node->g)
      is_lowest_cost = true;

    if (is_lowest_cost) {
      ++niteration;
      if (task.goal_reached(pop_state)) {
        std::cout << "Goal reached. Start extraction of solution." << std::endl;
        std::cout << "Search end: " << task.name << std::endl;
        auto ret = pop_node->extract_solution();
        for (auto addr : addrs) {
          delete addr;
        }
        std::cout << niteration << " nodes expanded" << std::endl;
        return ret;
      }

      std::vector<std::pair<VecOperator, std::vector<bool>>> successors;
      if (use_trie)
        successors = trie.get_successor_states(pop_state);
      else
        successors = task.get_successor_states(pop_state);
      for (auto p : successors) {
        auto op = p.first;
        auto succ_state = p.second;

        auto succ_node = make_child_node(pop_node, op.name, succ_state);
        addrs.push_back(succ_node);
        auto h = heuristic(*succ_node);
        if (h == 1e9)
          continue;
        size_t old_succ_g = 1e9;

        if (state_cost.find(succ_state) != state_cost.end()) {
          old_succ_g = state_cost[succ_state];
        }

        if (succ_node->g < old_succ_g) {
          ++node_tiebreaker;
          open.push(make_open_entry(succ_node, h, node_tiebreaker));
          state_cost[succ_state] = succ_node->g;
        }
      }
    }
  }

  for (auto addr : addrs) {
    delete addr;
  }
  std::cout << "No solution found" << std::endl;
  std::cout << "Search end: " << task.name << std::endl;
  std::cout << niteration << " nodes expanded" << std::endl;
  return std::vector<std::string>();
}
