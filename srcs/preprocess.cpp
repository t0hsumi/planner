#include <cerrno>
#include <fstream>
#include <iostream>

#include "planner.hpp"
#include "preprocess.hpp"
#include "utils.hpp"

static std::string extract_path(const std::string &probfile) {
  std::string ret;
  std::smatch m;

  std::regex_match(probfile, m,
                   std::regex(R"(.+\/benchmarks\/([^/]+/[^/]+)\.pddl)"));
  ret = "/mnt/c/Users/ohsum/programing/ut/lab/planner/preprocess/" + m[1].str();
  return ret;
}

static void write_task(const Task &task, const std::string &path) {
  std::ofstream fstream(path);

  std::filesystem::path directory = std::filesystem::path(path).parent_path();
  if (!std::filesystem::exists(directory)) {
    std::filesystem::create_directories(directory);
  }

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
    std::cerr << "can't open file: " << path << std::endl;
    std::cerr << strerror(errno) << std::endl;
    std::exit(1);
  }
}

Task preprocess(const std::string &domfile, const std::string &probfile,
                const size_t &batch_size) {
  std::string task_path = extract_path(probfile);
  /* if (std::filesystem::exists(task_path)) {  */
  /*   std::cout << "file exists" << std::endl; */
  /* }                                          */
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

  Task task = generate_task(dom, prob, batch_size);

  write_task(task, task_path);

  return task;
}
