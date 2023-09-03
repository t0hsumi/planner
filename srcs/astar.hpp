#ifndef ASTAR_HPP
#define ASTAR_HPP

#include <set>
#include <string>
#include <tuple>
#include <vector>

#include "planner.hpp"

class RelaxedOperator {
public:
  RelaxedOperator(std::string name, std::multiset<std::string> precond,
                  std::multiset<std::string> add_effects)
      : name(name), precond(precond), add_effects(add_effects),
        counter(precond.size()) {}

  RelaxedOperator &operator=(const RelaxedOperator &r) {
    name = r.name;
    precond = r.precond;
    add_effects = r.add_effects;
    counter = r.counter;
    return *this;
  }
  bool operator==(const RelaxedOperator &r) const { return name == r.name; }

  std::string name;
  std::multiset<std::string> precond;
  std::multiset<std::string> add_effects;
  const size_t cost = 1;
  int counter;
};

class RelaxedFact {
public:
  RelaxedFact() {}
  RelaxedFact(std::string name)
      : name(name), expanded(false), cheapest_achiver(nullptr) {}

  std::string name;
  std::vector<RelaxedOperator *> precondition_of;
  bool expanded = false;
  RelaxedOperator *cheapest_achiver;
  size_t distance = 1e9;
};

struct CustomCompare {
  bool operator()(const std::tuple<size_t, size_t, RelaxedFact> &l,
                  const std::tuple<size_t, size_t, RelaxedFact> &r) const {
    if (std::get<0>(l) != std::get<0>(r))
      return std::get<0>(l) > std::get<0>(r);
    return std::get<1>(l) > std::get<1>(r);
  }
};

class hFFHeuristic {
public:
  hFFHeuristic(const Task &task) {
    start_state = RelaxedFact("start");
    init = task.init;
    goal = task.goal;
    tie_breaker = 0;

    for (auto fact : task.facts) {
      this->facts.insert({fact, RelaxedFact(fact)});
    }

    for (auto op : task.operators) {
      auto ro = new RelaxedOperator(op.name, op.preconditions, op.add_effects);
      operators.push_back(ro);

      for (auto var : op.preconditions) {
        facts[var].precondition_of.push_back(ro);
      }

      if (op.preconditions.empty()) {
        start_state.precondition_of.push_back(ro);
      }
    }
  }

  ~hFFHeuristic() {
    for (auto addr : operators) {
      delete addr;
    }
  }

  size_t operator()(const SearchNode &node) {
    auto state = node.state;
    std::priority_queue<std::tuple<size_t, size_t, RelaxedFact>,
                        std::vector<std::tuple<size_t, size_t, RelaxedFact>>,
                        CustomCompare>
        pq;

    init_distance(state);
    pq.push(std::make_tuple(0, tie_breaker, start_state));
    ++tie_breaker;

    for (auto fact : state) {
      pq.push(std::make_tuple(facts[fact].distance, tie_breaker, facts[fact]));
      ++tie_breaker;
    }

    dijkstra(pq);

    return calc_goal_h();
  }

private:
  std::unordered_map<std::string, RelaxedFact> facts;
  std::vector<RelaxedOperator *> operators;
  std::multiset<std::string> init;
  std::multiset<std::string> goal;
  size_t tie_breaker;
  RelaxedFact start_state;

  void reset_fact(const std::multiset<std::string> &state, RelaxedFact &f) {
    f.expanded = false;
    f.cheapest_achiver = nullptr;
    if (state.find(f.name) != state.end()) {
      f.distance = 0;
    } else {
      f.distance = 1e9;
    }
  }
  void init_distance(const std::multiset<std::string> &state) {
    reset_fact(state, start_state);
    for (auto iter = facts.begin(); iter != facts.end(); ++iter) {
      RelaxedFact &fact = (*iter).second;
      reset_fact(state, fact);
    }
    for (auto op : operators) {
      op->counter = op->precond.size();
    }
  }
  size_t get_cost(const RelaxedOperator &op) {
    size_t cost = 0;

    for (auto pre : op.precond) {
      cost += facts[pre].distance;
    }
    return cost + op.cost;
  }

  void dijkstra(
      std::priority_queue<std::tuple<size_t, size_t, RelaxedFact>,
                          std::vector<std::tuple<size_t, size_t, RelaxedFact>>,
                          CustomCompare> &pq) {
    std::multiset<std::string> achieved_goals;

    while (goal != achieved_goals && !pq.empty()) {
      auto tuple = pq.top();
      pq.pop();
      RelaxedFact &fact = std::get<2>(tuple);

      if (goal.find(fact.name) != goal.end()) {
        achieved_goals.insert(fact.name);
      }

      if (!fact.expanded) {
        for (auto op : fact.precondition_of) {
          op->counter -= 1;
          if (op->counter <= 0) {
            for (auto n : op->add_effects) {
              RelaxedFact &neighbor = facts[n];
              auto tmp_dist = get_cost(*op);
              if (tmp_dist < neighbor.distance) {
                neighbor.distance = tmp_dist;
                neighbor.cheapest_achiver = op;
                pq.push(std::make_tuple(tmp_dist, tie_breaker, neighbor));
                ++tie_breaker;
              }
            }
          }
        }
        fact.expanded = true;
      }
    }
  }

  size_t calc_goal_h(void) {
    std::vector<RelaxedOperator> relaxed_plan;
    size_t hAdd_value = 0;

    for (auto f : goal) {
      hAdd_value += facts[f].distance;
    }

    if (hAdd_value < 1e9) {
      std::queue<RelaxedFact> q;
      std::unordered_set<std::string> closed_list;

      for (auto g : goal) {
        q.push(facts[g]);
        closed_list.insert(g);
      }
      while (!q.empty()) {
        auto fact = q.front();
        q.pop();
        if (fact.cheapest_achiver &&
            std::find(relaxed_plan.begin(), relaxed_plan.end(),
                      *fact.cheapest_achiver) == relaxed_plan.end()) {
          for (auto pre : fact.cheapest_achiver->precond) {
            if (closed_list.find(pre) == closed_list.end()) {
              q.push(facts[pre]);
              closed_list.insert(pre);
            }
          }
          relaxed_plan.push_back(*fact.cheapest_achiver);
        }
      }
      return relaxed_plan.size();
    } else
      return 1e9;
  }
};

std::vector<std::string> astar(const Task &task, bool use_trie);

#endif
