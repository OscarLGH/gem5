#include "arch/power/locked_mem.hh"

#include <stack>

#include "base/types.hh"

namespace gem5
{

namespace PowerISA
{
    std::unordered_map<int, std::stack<Addr>> locked_addrs;
} // namespace PowerISA
} // namespace gem5
