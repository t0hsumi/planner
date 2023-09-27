#ifndef TRIE_HPP
#define TRIE_HPP

#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include "planner.hpp"
#include "thread_bmrw.hpp"

struct TrieNode {
  std::unordered_map<std::string, TrieNode *> children;
  std::vector<Operator> applicable_actions;

  TrieNode() : children(std::unordered_map<std::string, TrieNode *>()) {}
  ~TrieNode() {
    for (auto &kv : children)
      delete kv.second;
  }
};

class Trie {
public:
  Trie() { root = new TrieNode(); }
  ~Trie() { delete root; }

  void initialize(const Task &task) {
    for (auto op : task.operators) {
      TrieNode *curr = root;

      for (auto precond : op.preconditions) {
        if (curr->children.find(precond) == curr->children.end()) {
          curr->children[precond] = new TrieNode();
        }
        curr = curr->children[precond];
      }
      curr->applicable_actions.push_back(op);
    }
  }

  std::vector<std::pair<Operator, std::multiset<std::string>>>
  get_successor_states(const std::multiset<std::string> &state, int id) const {
    std::vector<std::vector<std::pair<Operator, std::multiset<std::string>>>>
        ret(BATCH_SIZE);
    std::vector<std::vector<TrieNode *>> reachable_nodes(BATCH_SIZE);

    reachable_nodes[id].push_back(root);

    for (auto fact : state) {
      std::vector<TrieNode *> add_nodes;
      for (auto reachable_node : reachable_nodes[id]) {
        if (reachable_node->children.find(fact) !=
            reachable_node->children.end())
          add_nodes.push_back(reachable_node->children[fact]);
      }
      reachable_nodes[id].insert(reachable_nodes[id].end(), add_nodes.begin(),
                                 add_nodes.end());
    }

    for (auto reachable_node : reachable_nodes[id]) {
      for (auto op : reachable_node->applicable_actions) {
        ret[id].push_back(std::make_pair(op, op.apply(state)));
      }
    }

    return ret[id];
  }

private:
  TrieNode *root;
};

#endif
