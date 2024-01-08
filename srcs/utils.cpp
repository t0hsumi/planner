#include "utils.hpp"

static std::string addSpace(const std::string &str) {
  std::regex lparen("\\(");
  std::string lparenReplace = " ( ";

  std::regex rparen("\\)");
  std::string rparenReplace = " ) ";

  std::regex question("\\?");
  std::string questionReplace = " ?";

  std::string result = std::regex_replace(str, lparen, lparenReplace);
  result = std::regex_replace(result, rparen, rparenReplace);
  result = std::regex_replace(result, question, questionReplace);

  return result;
}

static std::vector<std::string> split(const std::string &str) {
  std::vector<std::string> ret;
  std::istringstream iss(str);
  std::string word;

  while (iss >> word) {
    std::transform(word.begin(), word.end(), word.begin(), ::tolower);
    ret.push_back(word);
  }

  return ret;
}

std::vector<std::string> generate_token(const std::string &filename) {
  // param filename The name of the lisp-like syntax file
  // return A vector of strings
  std::ifstream fstream;
  fstream.open(filename, std::ios::in);

  std::string in;
  std::string tmp;
  while (std::getline(fstream, tmp)) {
    // remove comments
    size_t colon = tmp.find(';');
    if (colon == std::string::npos)
      in += tmp + " ";
    else
      in += tmp.substr(0, colon) + " ";
  }
  fstream.close();
  in = addSpace(in);
  return split(in);
}

void write_solution(const std::vector<std::string> &solution,
                    const double &elapsed, const size_t &nodes_sum) {
  size_t steps_cnt = 0;
  /* std::ofstream ofile;                   */
  /* ofile.open("solution", std::ios::out); */
  for (auto s : solution) {
    if (steps_cnt == 0) {
      std::cout << "steps    " << steps_cnt << ": " << s << std::endl;
    } else {
      std::cout << std::setw(10) << steps_cnt << ": " << s << std::endl;
    }
    steps_cnt++;
  }
  /* ofile.close(); */
  std::cout << std::endl;
  std::cout << "Plan length: " << solution.size() << std::endl;
  std::cout << "Search time: " << elapsed << " msec" << std::endl;
  std::cout << "Number of Nodes expanded: " << nodes_sum << std::endl;
}
