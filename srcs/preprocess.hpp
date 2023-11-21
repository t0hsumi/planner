#ifndef PREPROCESS_HPP
#define PREPROCESS_HPP

#include <filesystem>
#include <regex>
#include <iostream>

#include "planner.hpp"

Task preprocess(const std::string &domfile, const std::string &probfile,
                const size_t &batch_size);

#endif
