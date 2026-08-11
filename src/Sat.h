#ifndef SAT_H_
#define SAT_H_

#include <vector>
#include <cstdint>
#include <algorithm>

namespace Sat {

class Solver {
  public:
    int newVar() { _val.push_back(0); return static_cast<int>(_val.size()); }
    void addClause(const std::vector<int>& c) { _clauses.push_back(c); }
    //int numVars() const { return static_cast<int>(_val.size()); }
    //int numClauses() const { return static_cast<int>(_clauses.size()); }

    // 1 = SAT, 0 = UNSAT, -1 = unknown (limit exceeded).
    int solve(long limit = 20000000)
    {
      _limit = limit;
      std::fill(_val.begin(), _val.end(), 0);
      return dpll();
    }
    bool value(int var) const { return _val[var - 1] == 1; }

  private:
    std::vector<int8_t> _val;
    std::vector<std::vector<int>> _clauses;
    long _limit{0};

    inline int litVal(const int lit) const
    {
      const int v = _val[(lit > 0 ? lit : -lit) - 1];
      return (v == 0) ? 0 : (lit > 0 ? v : -v);
    }

    void undo(std::vector<int>& trail)
    {
      for (const int v : trail) _val[v - 1] = 0;
      trail.clear();
    }

    int dpll()
    {
      std::vector<int> trail;
      bool changed = true;
      while (changed) {
        changed = false;
        if (--_limit < 0) { undo(trail); return -1; }
        for (const auto& cl : _clauses) {
          int unassigned = 0, lastlit = 0;
          bool sat = false;
          for (const int lit : cl) {
            const int lv = litVal(lit);
            if (lv == 1) { sat = true; break; }
            if (lv == 0) { ++unassigned; lastlit = lit; }
          }
          if (sat) continue;
          if (unassigned == 0) { undo(trail); return 0; }
          if (unassigned == 1) {
            const int var = lastlit > 0 ? lastlit : -lastlit;
            _val[var - 1] = lastlit > 0 ? 1 : -1;
            trail.push_back(var);
            changed = true;
          }
        }
      }
      int pick = 0;
      for (size_t i = 0; i < _val.size(); ++i) if (_val[i] == 0) { pick = static_cast<int>(i) + 1; break; }
      if (pick == 0) return 1;
      for (const int sign : {1, -1}) {
        _val[pick - 1] = static_cast<int8_t>(sign);
        const int r = dpll();
        if (r == 1) return 1;
        if (r == -1) { _val[pick - 1] = 0; undo(trail); return -1; }
        _val[pick - 1] = 0;
      }
      undo(trail);
      return 0;
    }
};

}
#endif
