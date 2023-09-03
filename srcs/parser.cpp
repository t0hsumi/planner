#include "parser.hpp"
#include "lispiterator.hpp"

static std::string get_keyword(const LispIterator &iter) {
  std::string name = iter.get_word();
  if (name[0] != ':') {
    std::cerr << "Error keywords have to start with a colon ':'" << std::endl;
  }
  return name.substr(1);
}

static std::vector<std::string> parse_requirements_stmt(LispIterator &iter) {
  std::vector<std::string> ret;

  if (!iter.try_match_word(":requirements")) {
    std::cerr << "Error requirements list must contain keyword ':requirements'"
              << std::endl;
    exit(1);
  }
  while (!iter.empty()) {
    ret.push_back(iter.get_word());
    iter.next();
  }
  return ret;
}

static std::unordered_map<std::string, Type>
parse_type_stmt(LispIterator &iter) {
  // return type name to Type mapping
  std::unordered_map<std::string, Type> ret;

  if (!iter.try_match_word(":types")) {
    std::cerr << "Error requirements list must contain keyword ':requirements'"
              << std::endl;
    exit(1);
  }
  std::queue<std::string> tmplst;
  while (!iter.empty()) {
    std::string var = iter.get_word();
    iter.next();
    if (var == "-") {
      std::string parent_type = iter.get_word();
      iter.next();
      while (!tmplst.empty()) {
        ret.insert({tmplst.front(), Type(tmplst.front(), parent_type)});
        tmplst.pop();
      }
    } else
      tmplst.push(var);
  }
  while (!tmplst.empty()) {
    ret.insert({tmplst.front(), Type(tmplst.front(), "object")});
    tmplst.pop();
  }
  if (ret.find("object") == ret.end()) {
    ret.insert({{"object", Type("object", "")}});
  }
  return ret;
}

static std::vector<Variable> parse_variable_list(LispIterator &iter) {
  std::vector<Variable> ret;

  std::queue<std::string> tmplst;
  while (!iter.empty()) {
    std::string var = iter.get_word();
    iter.next();
    if (var == "-") {
      std::string type = iter.get_word();
      iter.next();
      while (!tmplst.empty()) {
        ret.push_back(Variable(tmplst.front(), type));
        tmplst.pop();
      }
    } else
      tmplst.push(var);
  }
  while (!tmplst.empty()) {
    ret.push_back(Variable(tmplst.front(), "object"));
    tmplst.pop();
  }
  return ret;
}

static std::string parse_name(LispIterator &iter) {
  std::string ret;

  ret = iter.get_word();
  iter.next();
  return ret;
}

static std::vector<Predicate> parse_predicates_list(LispIterator &iter) {
  std::vector<Predicate> predicates;

  while (!iter.empty()) {
    Predicate pred;
    auto it = iter.peek();

    pred.name = parse_name(it);
    pred.params = parse_variable_list(it);
    predicates.push_back(pred);

    iter.next();
  }
  return predicates;
}

static std::vector<Predicate> parse_predicate_stmt(LispIterator &iter) {
  if (!iter.try_match_word(":predicates")) {
    std::cerr << "Error requirements list must contain keyword ':predicates'"
              << std::endl;
    exit(1);
  }
  return parse_predicates_list(iter);
}

static std::vector<Variable> parse_parameters(LispIterator &iter) {
  if (!iter.try_match_word(":parameters")) {
    std::cerr << "Error requirements list must contain keyword ':parameters"
              << std::endl;
    exit(1);
  }

  auto it = iter.peek();
  iter.next();
  return parse_variable_list(it);
}

static Cond parse_cond(LispIterator &iter) {
  Cond ret;

  ret.key = iter.get_word();
  iter.next();
  ret.variables = parse_variable_list(iter);

  return ret;
}

static void _parse_formula(LispIterator &iter, Formula &f) {
  std::string key = iter.get_word();
  iter.next();
  if (key == "and") {
    while (!iter.empty()) {
      auto childformula = iter.peek();
      iter.next();
      _parse_formula(childformula, f);
    }
  } else if (key == "not") {
    auto childformula = iter.peek();
    auto cond = parse_cond(childformula);
    f.neg.push_back(cond);
  } else {
    Cond c;
    c.key = key;
    c.variables = parse_variable_list(iter);
    f.pos.push_back(c);
  }
}

static Formula parse_formula(LispIterator &iter) {
  Formula ret;

  _parse_formula(iter, ret);
  return ret;
}

static Formula parse_precondition_stmt(LispIterator &iter) {
  if (!iter.try_match_word(":precondition")) {
    std::cerr << "Error requirements list must contain keyword ':precondition'"
              << std::endl;
    exit(1);
  }

  auto it = iter.peek();
  iter.next();
  return parse_formula(it);
}

static Formula parse_effect_stmt(LispIterator &iter) {
  if (!iter.try_match_word(":effect")) {
    std::cerr << "Error requirements list must contain keyword ':effect'"
              << std::endl;
    exit(1);
  }

  auto it = iter.peek();
  iter.next();
  return parse_formula(it);
}

static ActionStmt parse_action_stmt(LispIterator &iter) {
  if (!iter.try_match_word(":action")) {
    std::cerr << "Error requirements list must contain keyword ':action'"
              << std::endl;
    exit(1);
  }

  ActionStmt action;
  action.name = parse_name(iter);
  action.parameters = parse_parameters(iter);
  action.precond = parse_precondition_stmt(iter);
  action.effect = parse_effect_stmt(iter);
  return action;
}

static std::vector<Fact> parse_predicate_instance_list(LispIterator &iter) {
  std::vector<Fact> ret;

  while (!iter.empty()) {
    Fact pred;
    auto it = iter.peek();

    pred.name = parse_name(it);
    while (!it.empty()) {
      pred.name = pred.name + " " + it.get_word();
      it.next();
    }
    ret.push_back(pred);

    iter.next();
  }
  return ret;
}

static std::vector<Fact> parse_init_stmt(LispIterator &iter) {
  if (!iter.try_match_word(":init")) {
    std::cerr << "Error requirements list must contain keyword ':init'"
              << std::endl;
    exit(1);
  }

  return parse_predicate_instance_list(iter);
}

static std::vector<Fact> parse_goal_stmt(LispIterator &iter) {
  if (!iter.try_match_word(":goal")) {
    std::cerr << "Error requirements list must contain keyword ':goal'"
              << std::endl;
    exit(1);
  }
  std::vector<Fact> ret;

  auto it = iter.peek();
  std::string key = it.get_word();
  it.next();
  if (key == "and") {
    ret = parse_predicate_instance_list(it);
  } else {
    auto f = key;
    while (!it.empty()) {
      f += " " + it.get_word();
      it.next();
    }
    ret.push_back(f);
  }

  return ret;
}

static std::string parse_keyword(LispIterator &iter, std::string keyword) {
  // first content of the iter expected to be the string keyword
  std::string ret;

  if (!iter.try_match_word(keyword)) {
    std::cerr << "Error requirements list must contain keyword '" << keyword
              << "'" << std::endl;
    exit(1);
  }

  ret = iter.get_word();
  iter.next();
  iter.match_end();

  return ret;
}

DomainDef parse_domain(LispIterator &iter) {
  DomainDef dom;

  iter.try_match_word("define");

  auto it = iter.peek();
  dom.name = parse_keyword(it, "domain");
  iter.next();

  while (!iter.empty()) {
    it = iter.peek();
    std::string key = get_keyword(it);
    if (key == "requirements") {
      dom.requirements = parse_requirements_stmt(it);
    } else if (key == "types") {
      dom.types = parse_type_stmt(it);
    } else if (key == "predicates") {
      dom.preconditions = parse_predicate_stmt(it);

    } else if (key == "action") {
      dom.actions.push_back(parse_action_stmt(it));
    } else {
      std::cerr << "unknown keyword detected: " << key << std::endl;
      exit(1);
    }
    iter.next();
  }
  // in case of no :type decleration
  if (dom.types.empty()) {
    dom.types.insert({"object", Type("object", "")});
  }
  iter.match_end();
  return dom;
}

static std::vector<Variable> parse_obj_stmt(LispIterator &iter) {
  if (!iter.try_match_word(":objects")) {
    std::cerr << "Error requirements list must contain keyword ':objects'"
              << std::endl;
    exit(1);
  }

  return parse_variable_list(iter);
}

ProblemDef parse_problem(LispIterator &iter) {
  ProblemDef ret;

  iter.try_match_word("define");

  auto it = iter.peek();
  ret.probname = parse_keyword(it, "problem");
  iter.next();

  it = iter.peek();
  ret.domname = parse_keyword(it, ":domain");
  iter.next();

  it = iter.peek();
  if (it.get_word() == ":objects") {
    ret.objs = parse_obj_stmt(it);
    iter.next();
  }

  it = iter.peek();
  ret.init = parse_init_stmt(it);
  iter.next();

  it = iter.peek();
  ret.goal = parse_goal_stmt(it);
  iter.next();

  iter.match_end();
  return ret;
}
