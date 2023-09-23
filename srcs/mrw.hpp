#ifndef MRW_HPP
#define MRW_HPP

#include "planner.hpp"

std::vector<std::string> mrw(const Task &task, bool use_trie);

#define MAX_STEPS 7
#define NUM_WALK 2000
#define LENGTH_WALK 10

#endif
