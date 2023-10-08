#ifndef ASTAR_HPP
#define ASTAR_HPP

#include <set>
#include <string>
#include <tuple>
#include <vector>

#include "planner.hpp"

class RelaxedOperator {
public:
  RelaxedOperator(std::string name, std::vector<size_t> precond,
                  std::vector<size_t> add_effects)
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
  std::vector<size_t> precond;
  std::vector<size_t> add_effects;
  const size_t cost = 1;
  int counter;
};

class RelaxedFact {
public:
  RelaxedFact() {}
  RelaxedFact(size_t id) : id(id), expanded(false), cheapest_achiver(nullptr) {}

  size_t id;
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
  hFFHeuristic() {}

  hFFHeuristic(const Task &task) {
    start_state = RelaxedFact(SIZE_MAX);
    init = task.init;
    goal = task.goal;
    tie_breaker = 0;

    this->facts.resize(task.init.size());
    for (size_t i = 0; i < facts.size(); ++i) {
      this->facts[i].id = i;
    }

    for (auto op : task.operators) {
      auto ro = new RelaxedOperator(op.name, op.preconditions, op.add_eff);
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

  void initialize(const Task &task) {
    start_state = RelaxedFact(SIZE_MAX);
    init = task.init;
    goal = task.goal;
    tie_breaker = 0;

    this->facts.resize(task.init.size());
    for (size_t i = 0; i < facts.size(); ++i) {
      this->facts[i].id = i;
    }

    for (auto op : task.operators) {
      auto ro = new RelaxedOperator(op.name, op.preconditions, op.add_eff);
      operators.push_back(ro);

      for (auto var : op.preconditions) {
        facts[var].precondition_of.push_back(ro);
      }

      if (op.preconditions.empty()) {
        start_state.precondition_of.push_back(ro);
      }
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

    for (size_t i = 0; i < state.size(); ++i) {
      if (!state[i])
        continue;
      pq.push(std::make_tuple(facts[i].distance, tie_breaker, facts[i]));
      ++tie_breaker;
    }

    dijkstra(pq);

    return calc_goal_h();
  }

  size_t operator()(const std::vector<bool> &state) {
    std::priority_queue<std::tuple<size_t, size_t, RelaxedFact>,
                        std::vector<std::tuple<size_t, size_t, RelaxedFact>>,
                        CustomCompare>
        pq;

    init_distance(state);
    pq.push(std::make_tuple(0, tie_breaker, start_state));
    ++tie_breaker;

    for (size_t i = 0; i < state.size(); ++i) {
      if (!state[i])
        continue;
      pq.push(std::make_tuple(facts[i].distance, tie_breaker, facts[i]));
      ++tie_breaker;
    }

    dijkstra(pq);

    return calc_goal_h();
  }

private:
  std::vector<RelaxedFact> facts;
  std::vector<RelaxedOperator *> operators;
  std::vector<bool> init;
  std::multiset<size_t> goal;
  size_t tie_breaker;
  RelaxedFact start_state;

  void reset_fact(const std::vector<bool> &state, RelaxedFact &f) {
    f.expanded = false;
    f.cheapest_achiver = nullptr;
    if (f.id == SIZE_MAX || state[f.id])
      f.distance = 0;
    else
      f.distance = 1e9;
  }
  void init_distance(const std::vector<bool> &state) {
    reset_fact(state, start_state);
    for (size_t i = 0; i < facts.size(); ++i) {
      reset_fact(state, facts[i]);
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
    std::multiset<size_t> achieved_goals;

    while (goal != achieved_goals && !pq.empty()) {
      auto tuple = pq.top();
      pq.pop();
      RelaxedFact &fact = std::get<2>(tuple);

      if (goal.find(fact.id) != goal.end())
        achieved_goals.insert(fact.id);

      if (!fact.expanded) {
        for (auto op : fact.precondition_of) {
          op->counter -= 1;
          if (op->counter <= 0) {
            for (auto id : op->add_effects) {
              RelaxedFact &neighbor = facts[id];
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
      std::unordered_set<size_t> closed_list;

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
