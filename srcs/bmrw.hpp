#ifndef BMRW_HPP
#define BMRW_HPP

#include "planner.hpp"

std::vector<std::string> bmrw(const Task &task, bool use_trie);

#define MAX_STEPS 7
#define NUM_WALK 2000
#define LENGTH_WALK 10
#define BATCH_SIZE 8

#endif
