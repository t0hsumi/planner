#include <unordered_set>

#include "bfs.hpp"
#include "trie.hpp"

std::vector<std::string> bfs(const Task &task, bool use_trie) {
  std::queue<SearchNode *> queue;
  std::unordered_set<std::vector<bool>> closed;
  std::vector<SearchNode *> addrs;
  Trie trie;

  if (use_trie) {
    /* std::cout << "trie init begin" << std::endl; */
    trie.initialize(task);
    /* std::cout << "trie init end" << std::endl; */
  }

  /* std::cout << "Search start: " << task.name << std::endl; */
  queue.push(make_root_node(task.init));
  addrs.push_back(queue.front());

  closed.insert(task.init);
  int niteration = 0;
  while (!queue.empty()) {
    ++niteration;
    auto node = queue.front();
    queue.pop();
    if (task.goal_reached(node->state)) {
      /* std::cout << "Goal reached. Start extraction of solution." <<
       * std::endl; */
      /* std::cout << "Search end: " << task.name << std::endl; */
      auto ret = node->extract_solution();
      for (auto addr : addrs) {
        delete addr;
      }
      /* std::cout << niteration << " nodes expanded" << std::endl; */
      return ret;
    }

    std::vector<std::pair<VecOperator, std::vector<bool>>> successors;
    if (use_trie)
      successors = trie.get_successor_states(node->state);
    else
      successors = task.get_successor_states(node->state);
    for (auto p : successors) {
      auto op = p.first;
      auto successor_state = p.second;
      if (std::find(closed.begin(), closed.end(), successor_state) ==
          closed.end()) {
        auto child = make_child_node(node, op.name, successor_state);
        queue.push(child);
        addrs.push_back(child);
        closed.insert(successor_state);
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
