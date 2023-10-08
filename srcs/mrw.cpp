#include <cstdint>
#include <random>
#include <unordered_set>
#include <vector>

#include "astar.hpp"
#include "mrw.hpp"
#include "planner.hpp"
#include "trie.hpp"

static std::tuple<size_t, size_t, SearchNode *>
make_open_entry(SearchNode *node, size_t h, size_t node_tiebreaker) {
  return std::make_tuple(h, node_tiebreaker, node);
}

struct TupleCmp {
  bool operator()(const std::tuple<size_t, size_t, SearchNode *> &l,
                  const std::tuple<size_t, size_t, SearchNode *> &r) const {
    if (std::get<0>(l) != std::get<0>(r))
      return std::get<0>(l) > std::get<0>(r);
    return std::get<1>(l) > std::get<1>(r);
  }
};

static SearchNode *MonteCarloRandomWalk(SearchNode *current_node,
                                        const Task &task, const Trie &trie,
                                        const bool &use_trie,
                                        hFFHeuristic &heuristic,
                                        std::vector<SearchNode *> &addrs) {
  std::random_device rd;
  std::mt19937_64 eng(rd());
  size_t min_value = 0;
  size_t max_value = SIZE_MAX;
  std::uniform_int_distribution<size_t> dist(min_value, max_value);

  size_t h_min = 1e18;
  SearchNode *node_min = nullptr;

  for (int i = 0; i < NUM_WALK; ++i) {
    auto node = current_node;
    for (size_t j = 0; j < LENGTH_WALK; j++) {
      std::vector<std::pair<VecOperator, std::vector<bool>>> successors;
      if (use_trie)
        successors = trie.get_successor_states(node->state);
      else
        successors = task.get_successor_states(node->state);
      if (successors.empty())
        break;
      auto selected_succ = successors[dist(eng) % successors.size()];
      auto op = selected_succ.first;
      auto succ_state = selected_succ.second;
      node = make_child_node(node, op.name, succ_state);
      addrs.push_back(node);

      if (task.goal_reached(node->state))
        return node;
    }
    if (heuristic(node->state) < h_min) {
      node_min = node;
      h_min = heuristic(node->state);
    }
  }
  if (node_min)
    return node_min;
  else
    return current_node;
}

std::vector<std::string> mrw(const Task &task, bool use_trie) {
  std::priority_queue<std::tuple<size_t, size_t, SearchNode *>,
                      std::vector<std::tuple<size_t, size_t, SearchNode *>>,
                      TupleCmp>
      openlist;

  hFFHeuristic heuristic(task);
  std::vector<SearchNode *> addrs;
  Trie trie;
  size_t node_tiebreaker = 0;

  if (use_trie) {
    std::cout << "trie init begin" << std::endl;
    trie.initialize(task);
    std::cout << "trie init end" << std::endl;
  }

  auto root = make_root_node(task.init);
  SearchNode *node = root;
  auto h_min = heuristic(root->state);
  size_t counter = 0;

  addrs.push_back(root);
  openlist.push(make_open_entry(root, h_min, node_tiebreaker));

  std::cout << "Search start: " << task.name << std::endl;

  while (!task.goal_reached(node->state)) {
    std::vector<std::pair<VecOperator, std::vector<bool>>> successors;
    if (use_trie)
      successors = trie.get_successor_states(node->state);
    else
      successors = task.get_successor_states(node->state);
    if (counter > MAX_STEPS || successors.size() == 0) {
      node = root;
      counter = 0;
    }
    node = MonteCarloRandomWalk(node, task, trie, use_trie, heuristic, addrs);
    if (heuristic(node->state) < h_min) {
      h_min = heuristic(node->state);
      counter = 0;
    } else
      ++counter;
  }

  std::cout << "Goal reached. Start extraction of solution." << std::endl;
  std::cout << "Search end: " << task.name << std::endl;
  auto ret = node->extract_solution();
  for (auto addr : addrs) {
    delete addr;
  }
  return ret;
}
