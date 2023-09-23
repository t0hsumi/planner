#ifndef UTILS_HPP
#define UTILS_HPP

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

std::vector<std::string> generate_token(const std::string &filename);
void write_solution(const std::vector<std::string> &solution,
                    const double &elapsed);

#endif
