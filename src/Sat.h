#ifndef SAT_H_
#define SAT_H_

#include <vector>

// Thin C++ wrapper around the Kissat SAT solver. Variables are 1-based; literals
// are +v / -v (DIMACS convention). The interface is unchanged from the previous
// in-house solver so callers (Escape, Congestion) need no edits.
struct kissat;

namespace Sat {

class Solver {
  public:
    Solver();
    ~Solver();

    int newVar();                                   // fresh 1-based variable id
    void addClause(const std::vector<int>& clause); // disjunction of literals
    int numVars() const { return _nvars; }
    int numClauses() const { return _nclauses; }

    // 1 = SAT, 0 = UNSAT, -1 = unknown. 'budget' is accepted for interface
    // compatibility; Kissat manages its own search effort.
    int solve(long budget = 0);
    bool value(int var) const;                      // valid only after solve()==1

  private:
    Solver(const Solver&) = delete;
    Solver& operator=(const Solver&) = delete;
    kissat* _s;
    int _nvars{0};
    int _nclauses{0};
};

}
#endif
