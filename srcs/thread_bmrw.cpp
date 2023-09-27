#include <cstdint>
#include <random>

#include "astar.hpp"
#include "bmrw.hpp"
#include "planner.hpp"
#include "thread_bmrw.hpp"
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

static void MonteCarloRandomWalk(std::vector<SearchNode *> batch,
                                 std::vector<SearchNode *> &walkers,
                                 const Task &task, const Trie &trie,
                                 const bool use_trie,
                                 std::vector<hFFHeuristic> &heuristics,
                                 std::vector<SearchNode *> &addrs,
                                 int thread_id) {
  std::random_device rd;
  std::mt19937_64 eng(rd());
  size_t min_value = 0;
  size_t max_value = SIZE_MAX;
  std::uniform_int_distribution<size_t> dist(min_value, max_value);
  auto current_node = batch[thread_id];

  size_t h_min = 1e18;
  SearchNode *node_min = nullptr;

  for (int i = 0; i < NUM_WALK; ++i) {
    auto node = current_node;
    for (size_t j = 0; j < LENGTH_WALK; j++) {
      std::vector<std::pair<Operator, std::multiset<std::string>>> successors;
      if (use_trie) {
        successors = trie.get_successor_states(node->state, thread_id);
      } else
        successors = task.get_successor_states(node->state, thread_id);
      if (successors.empty())
        break;
      auto selected_succ = successors[dist(eng) % successors.size()];
      auto op = selected_succ.first;
      auto succ_state = selected_succ.second;
      node = make_child_node(node, op.name, succ_state);

      // TODO
      /* addrs.push_back(node); */

      if (task.goal_reached(node->state)) {
        walkers[thread_id] = node;
        return;
      }
    }
    if (heuristics[thread_id](node->state) < h_min) {
      node_min = node;
      h_min = heuristics[thread_id](node->state);
    }
  }
  if (node_min) {
    walkers[thread_id] = node_min;
  } else {
    walkers[thread_id] = current_node;
  }
}

void fillBatch(
    std::priority_queue<std::tuple<size_t, size_t, SearchNode *>,
                        std::vector<std::tuple<size_t, size_t, SearchNode *>>,
                        TupleCmp> &openlist,
    std::vector<SearchNode *> &batch) {
  size_t offset = 0;
  for (int i = 0; i < BATCH_SIZE; ++i) {
    if (openlist.empty()) {
      if (offset == 0)
        offset = i;
      batch[i] = batch[i - offset];
    } else {
      auto top = openlist.top();
      openlist.pop();
      batch[i] = std::get<2>(top);
    }
  }
}

std::vector<std::string> thread_bmrw(const Task &task, bool use_trie) {
  std::priority_queue<std::tuple<size_t, size_t, SearchNode *>,
                      std::vector<std::tuple<size_t, size_t, SearchNode *>>,
                      TupleCmp>
      openlist;
  std::vector<std::multiset<std::string>> closedlist;
  std::vector<SearchNode *> batch(BATCH_SIZE);
  std::vector<SearchNode *> walkers(BATCH_SIZE);

  std::vector<hFFHeuristic> heuristics(BATCH_SIZE);
  for (int i = 0; i < BATCH_SIZE; ++i) {
    heuristics[i].initialize(task);
  }
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
  auto h_min = heuristics[0](root->state);

  /* addrs.push_back(root); */
  openlist.push(make_open_entry(root, h_min, node_tiebreaker));

  std::cout << "Search start: " << task.name << std::endl;

  while (true) {
    if (openlist.empty()) {
      std::vector<std::pair<Operator, std::multiset<std::string>>> successors;
      if (use_trie)
        successors = trie.get_successor_states(root->state, 0);
      else
        successors = task.get_successor_states(root->state, 0);
      for (auto p : successors) {
        auto op = p.first;
        auto succ_state = p.second;

        auto succ_node = make_child_node(root, op.name, succ_state);
        /* addrs.push_back(succ_node); */
        auto h = heuristics[0](succ_node->state);
        ++node_tiebreaker;
        openlist.push(make_open_entry(succ_node, h, node_tiebreaker));
      }
    }
    fillBatch(openlist, batch);

    std::vector<std::thread> threads;
    for (int i = 0; i < BATCH_SIZE; ++i) {
      threads.push_back(std::thread(MonteCarloRandomWalk, std::ref(batch),
                                    std::ref(walkers), std::ref(task),
                                    std::ref(trie), std::ref(use_trie),
                                    std::ref(heuristics), std::ref(addrs), i));
    }
    for (std::thread &t : threads) {
      t.join();
    }

    /* for (int i = 0; i < BATCH_SIZE; ++i) { */
    /*   MonteCarloRandomWalk(batch, walkers, task, trie, use_trie, heuristics,
     */
    /*                        addrs, i); */
    /* } */

    for (int i = 0; i < BATCH_SIZE; ++i) {
      if (task.goal_reached(walkers[i]->state)) {
        std::cout << "Goal reached. Start extraction of solution." << std::endl;
        std::cout << "Search end: " << task.name << std::endl;
        auto ret = walkers[i]->extract_solution();
        return ret;
      }
      for (auto iter = closedlist.begin(); iter != closedlist.end(); ++iter) {
        auto state = *iter;
        if (walkers[i]->state == state)
          continue;
      }
      closedlist.push_back(walkers[i]->state);
      ++node_tiebreaker;
      openlist.push(make_open_entry(
          walkers[i], heuristics[i](walkers[i]->state), node_tiebreaker));
    }
  }
}
