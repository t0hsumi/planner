#include <chrono>
#include <cxxopts.hpp>

#include "astar.hpp"
#include "bfs.hpp"
#include "bmrw.hpp"
#include "lispiterator.hpp"
#include "mrw.hpp"
#include "nestedlist.hpp"
#include "parser.hpp"
#include "planner.hpp"
#include "thread_bmrw.hpp"
#include "utils.hpp"

int main(int argc, char **argv) {
  std::string domfile;
  std::string probfile;
  bool use_trie;
  auto search = thread_bmrw;

  cxxopts::Options options("cmd parser");
  try {
    options.add_options()("domfile", "Domain File",
                          cxxopts::value<std::string>(domfile))(
        "probfile", "Problem File", cxxopts::value<std::string>(probfile))(
        "t,trie", "Using Trie for successor generation",
        cxxopts::value<bool>(use_trie)->default_value("false"))(
        "a,astar", "Using astar search, not bfs",
        cxxopts::value<bool>()->default_value("false"))(
        "m,mrw", "Using MonteCarloRandomWalk",
        cxxopts::value<bool>()->default_value("false"))(
        "b,bmrw", "Using Batch MonteCarloRandomWalk",
        cxxopts::value<bool>()->default_value("false"));
    options.parse_positional({"domfile", "probfile"});

    auto result = options.parse(argc, argv);
    /* if (result["astar"].as<bool>()) {       */
    /*   search = astar;                       */
    /* } else if (result["mrw"].as<bool>()) {  */
    /*   search = mrw;                         */
    /* } else if (result["bmrw"].as<bool>()) { */
    /*   search = bmrw;                        */
    /* }                                       */
  } catch (cxxopts::exceptions::exception &e) {
    std::cerr << options.help() << std::endl;
    std::cerr << e.what() << std::endl;
    exit(1);
  }

  std::cout << "Parse Start" << std::endl;
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

  std::cout << "Parse end" << std::endl;
  std::cout << dom.preconditions.size() << " Predicates parsed" << std::endl;
  std::cout << dom.actions.size() << " Actions parsed" << std::endl;
  std::cout << prob.objs.size() << " Objects parsed" << std::endl;

  std::cout << "Task generation start" << std::endl;
  // combine domain def and prob def
  auto task = generate_task(dom, prob);

  std::cout << "Task generation end" << std::endl;
  std::cout << task.facts.size() << " Facts created" << std::endl;
  std::cout << task.operators.size() << " Operators created" << std::endl;

  // breadth first search
  auto start = std::chrono::system_clock::now();
  auto solution = search(task, true);
  auto end = std::chrono::system_clock::now();
  auto elapsed = static_cast<double>(
      std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
          .count());
  write_solution(solution, elapsed);

  // check plan
  std::string cmd = "validate " + domfile + " " + probfile + " solution";
  system(cmd.c_str());
}
