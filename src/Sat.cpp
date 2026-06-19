#include "Sat.h"

extern "C" {
#include "kissat.h"
}

namespace Sat {

Solver::Solver() : _s(kissat_init())
{
  // Kissat logs statistics to C stdout (which the router's std::cout redirect
  // does not capture); keep it silent.
  kissat_set_option(_s, "quiet", 1);
}

Solver::~Solver() { if (_s) kissat_release(_s); }

int Solver::newVar() { return ++_nvars; }

void Solver::addClause(const std::vector<int>& clause)
{
  for (const int lit : clause) kissat_add(_s, lit);
  kissat_add(_s, 0);   // 0 terminates a clause in Kissat's DIMACS-style API
  ++_nclauses;
}

int Solver::solve(long /*budget*/)
{
  const int r = kissat_solve(_s);
  if (r == 10) return 1;   // SATISFIABLE
  if (r == 20) return 0;   // UNSATISFIABLE
  return -1;               // interrupted / unknown
}

bool Solver::value(int var) const { return kissat_value(_s, var) > 0; }

}
