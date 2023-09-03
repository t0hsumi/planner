#include "planner.hpp"

static std::unordered_map<std::string, std::vector<std::string>>
get_typemap(const std::unordered_map<std::string, Type> &types,
            const std::vector<Variable> &objs) {
  // param types : typename to Type mapping
  // param objs  : list of all the objects
  // return        typename to objects mapping
  std::unordered_map<std::string, std::vector<std::string>> ret;

  for (auto obj : objs) {
    auto objtype = types.at(obj.type);
    while (objtype.name != "object") {
      ret[objtype.name].push_back(obj.name);
      objtype = types.at(objtype.parent);
    }
    ret["object"].push_back(obj.name);
  }

  return ret;
}

static std::multiset<std::string>
get_statics(const std::vector<Predicate> &preds,
            const std::vector<ActionStmt> &actions) {
  // statics is a set of facts each of which isn't affected by any actions.
  std::multiset<std::string> statics;
  std::unordered_set<std::string> effects;

  for (auto action : actions) {
    for (auto poseff : action.effect.pos) {
      effects.insert(poseff.key);
    }
    for (auto negeff : action.effect.neg) {
      effects.insert(negeff.key);
    }
  }

  for (auto pred : preds) {
    if (effects.find(pred.name) == effects.end()) {
      statics.insert(pred.name);
    }
  }

  return statics;
}

static std::multiset<std::string>
get_partial_state(const std::vector<Fact> &init) {
  std::multiset<std::string> ret;
  for (auto fact : init) {
    ret.insert(fact.name);
  }
  return ret;
}

static bool find_pred_in_init(const std::string &pred_name,
                              const std::string &param, int pos,
                              const std::multiset<std::string> &init) {
  std::string reg_ex;
  if (pos == 0) {
    reg_ex = pred_name + " " + param + ".*";
  } else {
    reg_ex = pred_name + " ";
    for (int i = 0; i < pos; ++i) {
      reg_ex += "[\\w|\\d|-]+\\s+";
    }
    reg_ex += param + ".*";
  }
  std::regex re(reg_ex, std::regex_constants::icase);
  for (auto iter = init.begin(); iter != init.end(); ++iter) {
    if (std::regex_match(*iter, re)) {
      return true;
    }
  }
  return false;
}

static void remove_invalid_objects(
    std::unordered_map<std::string, std::vector<std::string>> &param_to_objs,
    const std::vector<Cond> &pos, const std::multiset<std::string> &statics,
    const std::multiset<std::string> &init) {
  // remove invalid objects inferred from precondition and statics
  int nremoved = 0;
  for (auto iter = param_to_objs.begin(); iter != param_to_objs.end(); ++iter) {
    auto param = (*iter).first;
    auto &objs = (*iter).second;
    for (Cond pred : pos) {
      if (statics.find(pred.key) != statics.end()) {
        int pos = -1, count = 0;
        for (Variable &val : pred.variables) {
          if (val.name == param) {
            pos = count;
          }
          ++count;
        }
        if (pos != -1) {
          for (auto it = objs.begin(); it != objs.end();) {
            if (!find_pred_in_init(pred.key, *it, pos, init)) {
              it = objs.erase(it);
              ++nremoved;
            } else
              ++it;
          }
        }
      }
    }
  }
  std::cout << "Static analysis removed " << nremoved << " possible objects."
            << std::endl;
}

static std::vector<std::unordered_map<std::string, std::string>>
generate_assignments(
    const std::unordered_map<std::string, std::vector<std::string>>
        &param_to_objs) {
  std::queue<std::unordered_map<std::string, std::string>> que;
  std::vector<std::unordered_map<std::string, std::string>> ret;

  // initialize que by first assignment
  auto fst_param = (*param_to_objs.begin()).first;
  auto fst_objs = (*param_to_objs.begin()).second;
  for (auto iter = fst_objs.begin(); iter != fst_objs.end(); ++iter) {
    std::unordered_map<std::string, std::string> tmp;
    tmp.insert({fst_param, *iter});
    que.push(tmp);
  }

  for (auto iter = std::next(param_to_objs.begin());
       iter != param_to_objs.end(); ++iter) {
    auto param = (*iter).first;
    auto objs = (*iter).second;
    std::queue<std::unordered_map<std::string, std::string>> tmpque;
    while (!que.empty()) {
      auto assign = que.front();
      que.pop();
      for (auto it = objs.begin(); it != objs.end(); ++it) {
        assign[param] = *it;
        tmpque.push(assign);
      }
    }
    que = tmpque;
  }

  while (!que.empty()) {
    auto assign = que.front();
    que.pop();
    ret.push_back(assign);
  }
  return ret;
}

static std::string
_ground_atom(const Cond &precond,
             const std::unordered_map<std::string, std::string> &assign) {
  std::string ret = precond.key;
  for (auto val : precond.variables) {
    ret += " " + assign.at(val.name);
  }
  return ret;
}

static std::multiset<std::string>
_ground_atoms(const std::vector<Cond> atoms,
              const std::unordered_map<std::string, std::string> &assign) {
  std::multiset<std::string> ret;
  for (auto cond : atoms) {
    ret.insert(_ground_atom(cond, assign));
  }
  return ret;
}

static std::string
get_action_name(const ActionStmt &action,
                const std::unordered_map<std::string, std::string> &assign) {
  std::string ret = "(" + action.name;
  for (auto param : action.parameters) {
    ret += " " + assign.at(param.name);
  }
  ret += ")";
  return ret;
}

static Operator
create_operator(const ActionStmt &action,
                const std::unordered_map<std::string, std::string> &assign,
                const std::multiset<std::string> &statics,
                const std::multiset<std::string> &init) {
  std::string action_name = get_action_name(action, assign);
  std::multiset<std::string> preconditions;
  std::multiset<std::string> add;
  std::multiset<std::string> del;

  for (auto precond : action.precond.pos) {
    auto fact = _ground_atom(precond, assign);
    auto pred_name = precond.key;
    if (statics.find(pred_name) != statics.end()) {
      if (init.find(fact) == init.end()) {
        return Operator();
      }
    } else {
      preconditions.insert(fact);
    }
  }
  auto add_effect = _ground_atoms(action.effect.pos, assign);
  auto del_effect = _ground_atoms(action.effect.neg, assign);

  std::set_difference(del_effect.begin(), del_effect.end(), add_effect.begin(),
                      add_effect.end(), std::inserter(del, del.end()));
  std::set_difference(add_effect.begin(), add_effect.end(),
                      preconditions.begin(), preconditions.end(),
                      std::inserter(add, add.end()));

  return Operator(action_name, preconditions, add, del);
}

static std::vector<Operator> get_action_instance(
    const ActionStmt &action,
    const std::unordered_map<std::string, std::vector<std::string>> &typemap,
    const std::multiset<std::string> &statics,
    const std::multiset<std::string> &init) {
  std::vector<Operator> ret;

  if (action.parameters.empty()) {
    std::cout << "Static analysis removed 0 possible objects." << std::endl;
    auto op = create_operator(
        action, std::unordered_map<std::string, std::string>(), statics, init);
    if (op.name != "")
      ret.push_back(op);
  } else {
    std::unordered_map<std::string, std::vector<std::string>> param_to_objs;
    for (auto param : action.parameters) {
      param_to_objs.insert({param.name, typemap.at(param.type)});
    }
    remove_invalid_objects(param_to_objs, action.precond.pos, statics, init);
    auto assignments = generate_assignments(param_to_objs);
    for (auto assign : assignments) {
      auto op = create_operator(action, assign, statics, init);
      if (op.name != "") {
        ret.push_back(op);
      }
    }
  }
  return ret;
}

static std::vector<Operator> get_action_instances(
    const std::vector<ActionStmt> &actions,
    const std::unordered_map<std::string, std::vector<std::string>> &typemap,
    const std::multiset<std::string> &statics,
    const std::multiset<std::string> &init) {
  std::vector<Operator> operators;

  for (auto action : actions) {
    auto tmp = get_action_instance(action, typemap, statics, init);
    operators.insert(operators.end(), tmp.begin(), tmp.end());
  }

  return operators;
}

static std::multiset<std::string>
collect_facts(std::vector<Operator> ops, std::multiset<std::string> goal) {
  std::multiset<std::string> ret;
  std::multiset<std::string> tmp;

  for (auto op : ops) {
    std::multiset<std::string> tmp1;
    std::multiset<std::string> tmp2;
    std::multiset<std::string> tmp3;
    std::set_union(std::begin(op.preconditions), std::end(op.preconditions),
                   std::begin(op.add_effects), std::end(op.add_effects),
                   std::inserter(tmp1, std::end(tmp1)));
    std::set_union(std::begin(tmp1), std::end(tmp1), std::begin(op.del_effects),
                   std::end(op.del_effects),
                   std::inserter(tmp2, std::end(tmp2)));
    std::set_union(std::begin(tmp2), std::end(tmp2), std::begin(tmp),
                   std::end(tmp), std::inserter(tmp3, std::end(tmp3)));
    tmp = tmp3;
  }
  std::set_union(std::begin(goal), std::end(goal), std::begin(tmp),
                 std::end(tmp), std::inserter(ret, std::end(ret)));

  return ret;
}

static std::multiset<std::string>
remove_statics_from_initial_state(const std::multiset<std::string> &facts,
                                  const std::multiset<std::string> &init) {
  std::multiset<std::string> ret;
  std::set_intersection(std::begin(facts), std::end(facts), std::begin(init),
                        std::end(init), std::inserter(ret, std::end(ret)));
  return ret;
}

static std::vector<Operator>
relevance_analysis(std::vector<Operator> &operators,
                   const std::multiset<std::string> &goal) {
  std::multiset<std::string> relevant_facts;
  std::multiset<std::string> old_relevant_facts;

  bool changed = true;
  for (auto str : goal) {
    relevant_facts.insert(str);
  }

  while (changed) {
    old_relevant_facts = relevant_facts;
    for (auto op : operators) {
      std::multiset<std::string> new_addlst;
      std::multiset<std::string> new_dellst;

      std::set_intersection(
          std::begin(relevant_facts), std::end(relevant_facts),
          std::begin(op.add_effects), std::end(op.add_effects),
          std::inserter(new_addlst, std::end(new_addlst)));
      std::set_intersection(
          std::begin(relevant_facts), std::end(relevant_facts),
          std::begin(op.del_effects), std::end(op.del_effects),
          std::inserter(new_dellst, std::end(new_dellst)));
      if (!new_addlst.empty() || !new_dellst.empty()) {
        std::multiset<std::string> tmp;
        std::set_union(std::begin(op.preconditions), std::end(op.preconditions),
                       std::begin(relevant_facts), std::end(relevant_facts),
                       std::inserter(tmp, std::end(tmp)));
        relevant_facts = tmp;
      }
    }
    changed = relevant_facts != old_relevant_facts;
  }

  for (auto iter = operators.begin(); iter != operators.end();) {
    auto op = *iter;
    std::multiset<std::string> new_addlst;
    std::multiset<std::string> new_dellst;

    std::set_union(std::begin(relevant_facts), std::end(relevant_facts),
                   std::begin(op.add_effects), std::end(op.add_effects),
                   std::inserter(new_addlst, std::end(new_addlst)));
    std::set_union(std::begin(relevant_facts), std::end(relevant_facts),
                   std::begin(op.del_effects), std::end(op.del_effects),
                   std::inserter(new_dellst, std::end(new_dellst)));
    op.add_effects = new_addlst;
    op.del_effects = new_dellst;
    if (new_addlst.empty() && new_dellst.empty()) {
      iter = operators.erase(iter);
    } else
      ++iter;
  }
  return operators;
}

Task generate_task(const DomainDef &dom, const ProblemDef &prob) {
  assert(dom.name == prob.domname);

  auto typemap = get_typemap(dom.types, prob.objs);
  auto statics = get_statics(dom.preconditions, dom.actions);
  auto init = get_partial_state(prob.init);
  auto operators = get_action_instances(dom.actions, typemap, statics, init);
  auto goal = get_partial_state(prob.goal);
  auto facts = collect_facts(operators, goal);
  init = remove_statics_from_initial_state(facts, init);
  operators = relevance_analysis(operators, goal);

  return Task(prob.probname, facts, init, goal, operators);
}

SearchNode *make_root_node(std::multiset<std::string> initial_state) {
  auto ret = new SearchNode();

  ret->state = initial_state;
  ret->parent = nullptr;
  ret->action = "";
  ret->g = 0;
  return ret;
}

SearchNode *make_child_node(SearchNode *parent_node, const std::string &action,
                            const std::multiset<std::string> &state) {
  auto ret = new SearchNode();

  ret->state = state;
  ret->parent = parent_node;
  ret->action = action;
  ret->g = parent_node->g + 1;
  return ret;
}
