#ifndef THREAD_BMRW_HPP
#define THREAD_BMRW_HPP

#include <thread>

#include "planner.hpp"

std::vector<std::string> thread_bmrw(const Task &task, bool use_trie, int batch_size, size_t *nnodes);

#define MAX_STEPS 7
/* #define NUM_WALK 2000 */
#define LENGTH_WALK 1000

#endif
