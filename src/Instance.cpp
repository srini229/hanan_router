#include "Util.h"
#include "Placement.h"

#include <algorithm>

namespace Placement {
Instance::~Instance()
{
  for (auto& p : _pins) delete p.second;
  _pins.clear();
}


void Instance::build(const bool rebuild)
{
  if (_m) {
    for (auto& p : _m->pins()) {
      Pin* ip{nullptr};
      if (rebuild) {
        auto it = _pins.find(p.second->name());
        if (it != _pins.end()) {
          ip = it->second;
          ip->clearPorts();
        }
      } else {
        ip = new Pin(_name + SEPARATOR + p.second->name());
        _pins[p.second->name()] = ip;
      }
      if (!ip) continue;
      for (auto& port : p.second->ports()) {
        ip->addPort(port->getTransformedPort(_tr));
      }
    }
    _bbox = _tr.transform(_m->bbox());
  }
}


void Instance::setModule(const Module* m)
{
  _m = m;
  _bbox = _tr.transform(m->bbox());
}
}
