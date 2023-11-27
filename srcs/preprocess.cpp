#include <cerrno>
#include <cstddef>
#include <fstream>
#include <iostream>

#include "planner.hpp"
#include "preprocess.hpp"
#include "utils.hpp"

static std::string extract_path(const std::string &probfile) {
  std::string ret;

  // fetch preprocess result path from environmental variable
  const char *envvar = "DOWNWARD_PREPROCESSES";
  const char *val = std::getenv(envvar);
  if (val == nullptr) {
    std::cerr << "Environmental value " << envvar << " not exists" << std::endl;
    std::exit(1);
  }
  std::string path(val);

  // get problem and domain name
  auto ProbTokens = generate_token(probfile);
  std::string domname = ProbTokens[8];

  size_t sep_idx = probfile.find_last_of("/");
  std::string filename_with_ext =
      (sep_idx != std::string::npos) ? probfile.substr(sep_idx + 1) : probfile;
  size_t extension_idx = filename_with_ext.find_last_of(".");
  std::string probname = filename_with_ext.substr(0, extension_idx);

  // $DOWNWARD_PREPROCESSED/domain_name/problem_name
  return path + domname + "/" + probname;
}

static void write_task(const Task &task, const std::string &path) {
  std::ofstream fstream;

  std::filesystem::path directory = std::filesystem::path(path).parent_path();
  if (!std::filesystem::exists(directory)) {
    std::filesystem::create_directories(directory);
  }

  fstream.open(path, std::ios::out);
  if (fstream.is_open()) {
    // name
    fstream << task.name << std::endl;
    // batch size
    fstream << task.batch_size << std::endl;
    // number of propositions
    fstream << task.init.size() << std::endl;
    // initial state
    for (bool elem : task.init) {
      fstream << (elem ? 1 : 0) << " ";
    }
    fstream << std::endl;
    // goal state
    fstream << task.goal.size() << std::endl;
    for (const auto &elem : task.goal) {
      fstream << elem << " ";
    }
    fstream << std::endl;
    // number of operators
    fstream << task.operators.size() << std::endl;
    // operators
    for (const auto &op : task.operators) {
      // name
      fstream << op.name << std::endl;
      // precond size -> elem
      fstream << op.preconditions.size() << " ";
      for (const auto &elem : op.preconditions) {
        fstream << elem << " ";
      }
      fstream << std::endl;
      // add size -> elem
      fstream << op.add_eff.size() << " ";
      for (const auto &elem : op.add_eff) {
        fstream << elem << " ";
      }
      fstream << std::endl;
      // del size -> elem
      fstream << op.del_eff.size() << " ";
      for (const auto &elem : op.del_eff) {
        fstream << elem << " ";
      }
      fstream << std::endl;
    }
    fstream.close();
  } else {
    std::cerr << "write_task: can't open file: " << path << std::endl;
    std::cerr << strerror(errno) << std::endl;
    std::exit(1);
  }
}

static Task fetch_task(const std::string &path) {
  std::ifstream fstream(path);

  if (fstream.is_open()) {
    // name
    std::string name;
    fstream >> name;
    // batch size
    size_t batch_size;
    fstream >> batch_size;
    // number of propositions
    size_t nprops;
    fstream >> nprops;
    // initial state
    std::vector<bool> init(nprops);
    for (size_t i = 0; i < nprops; ++i) {
      int tmp;
      fstream >> tmp;
      init[i] = (tmp == 1);
    }
    // goal
    std::multiset<size_t> goal;
    size_t goal_size;
    fstream >> goal_size;
    for (size_t i = 0; i < goal_size; ++i) {
      size_t tmp;
      fstream >> tmp;
      goal.insert(tmp);
    }
    // number of operators
    size_t nops;
    fstream >> nops;
    // operators
    std::vector<VecOperator> operators(nops);
    for (size_t i = 0; i < nops; ++i) {
      // operation name
      std::string op_name;
      std::string tmp;
      while (true) {
        fstream >> tmp;
        if (!tmp.empty() && tmp.back() == ')') {
          op_name += tmp;
          break;
        } else {
          op_name += tmp + " ";
        }
      }
      // precond
      size_t npreconds;
      fstream >> npreconds;
      std::vector<size_t> precond(npreconds);
      for (size_t j = 0; j < npreconds; ++j) {
        fstream >> precond[j];
      }
      // add
      size_t nadds;
      fstream >> nadds;
      std::vector<size_t> add_eff(nadds);
      for (size_t j = 0; j < nadds; ++j) {
        fstream >> add_eff[j];
      }
      // del
      size_t ndels;
      fstream >> ndels;
      std::vector<size_t> del_eff(ndels);
      for (size_t j = 0; j < ndels; ++j) {
        fstream >> del_eff[j];
      }
      operators[i] = VecOperator(op_name, precond, add_eff, del_eff);
    }

    fstream.close();

    return Task(name, init, goal, operators, batch_size);
  } else {
    std::cerr << "fetch_task: can't open file: " << path << std::endl;
    std::cerr << strerror(errno) << std::endl;
    std::exit(1);
  }
}

Task preprocess(const std::string &domfile, const std::string &probfile,
                const size_t &batch_size) {
  std::string task_path = extract_path(probfile);
  if (std::filesystem::exists(task_path)) {
    Task task = fetch_task(task_path);
    return task;
  } else {
    // parse domain file
    auto DomTokens = generate_token(domfile);
    auto Domlst = generate_nestedlist(DomTokens);
    LispIterator DomIter(Domlst);
    auto dom = parse_domain(DomIter);

    // parse problem file
    auto ProbTokens = generate_token(probfile);
    auto Problst = generate_nestedlist(ProbTokens);
    LispIterator ProbIter(Problst);
    auto prob = parse_problem(ProbIter);

    // generate task
    Task task = generate_task(dom, prob, batch_size);

    write_task(task, task_path);
    return task;
  }
}
