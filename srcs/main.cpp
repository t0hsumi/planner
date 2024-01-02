#include <chrono>
#include <cxxopts.hpp>

#include "planner.hpp"
#include "preprocess.hpp"
#include "thread_bmrw.hpp"
#include "utils.hpp"

int main(int argc, char **argv) {
  std::string domfile;
  std::string probfile;
  size_t batch_size;

  // argparse
  cxxopts::Options options("cmd parser");
  try {
    options.add_options()("domfile", "Domain File",
                          cxxopts::value<std::string>(domfile))(
        "probfile", "Problem File", cxxopts::value<std::string>(probfile))(
        "n,nthreads", "Number of threads",
        cxxopts::value<size_t>()->default_value("1"));
    options.parse_positional({"domfile", "probfile"});

    auto result = options.parse(argc, argv);
    batch_size = result["nthreads"].as<size_t>();
  } catch (cxxopts::exceptions::exception &e) {
    std::cerr << options.help() << std::endl;
    std::cerr << e.what() << std::endl;
    exit(1);
  }

  // generate task from pddl
  Task task = preprocess(domfile, probfile, batch_size);

  auto start = std::chrono::system_clock::now();

  /* std::cout << dom.preconditions.size() << " Predicates parsed" << std::endl;
   */
  /* std::cout << dom.actions.size() << " Actions parsed" << std::endl; */
  /* std::cout << prob.objs.size() << " Objects parsed" << std::endl; */

  /* std::cout << task.init.size() << " Facts created" << std::endl;          */
  /* std::cout << task.operators.size() << " Operators created" << std::endl; */

  // search
  auto solution = thread_bmrw(task, true, batch_size);

  auto end = std::chrono::system_clock::now();

  auto elapsed = static_cast<double>(
      std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
          .count());
  if (solution.empty()) {
    std::cout << "No solution found." << std::endl;
    return 1;
  } else
    write_solution(solution, elapsed);

  // check plan
  /* std::string cmd = "validate " + domfile + " " + probfile + " solution"; */
  /* system(cmd.c_str());                                                    */
}
