#include "Util.h"
#include "Router.h"
#include <fstream>
#include <sstream>
#include <cmath>
#include <map>
#include <set>
#include <cmath>
#include "nlohmann/json.hpp"

namespace Router {

static int layerIndex(const std::string& n)
{
  for (size_t i = 0; i < LAYER_NAMES.size(); ++i) if (LAYER_NAMES[i] == n) return static_cast<int>(i);
  return -1;
}

static bool suffix(const std::string& s, const std::string& suf, std::string& base)
{
  if (s.size() <= suf.size() || s.compare(s.size() - suf.size(), suf.size(), suf) != 0) return false;
  base = s.substr(0, s.size() - suf.size());
  return true;
}

bool replay(Router& r, const std::string& leffile, const int uu, const bool detour,
    const std::string& ndrfile, const DRC::LayerInfo& lf)
{
  std::ifstream ifs(leffile);
  if (!ifs.is_open()) {
    CERR << "replay: unable to open " << leffile << std::endl;
    return false;
  }
  std::string macro, section, layer, line;
  Geom::LayerRects obs;
  Geom::Rect mbox;
  int nsrc = 0, ntgt = 0, nobs = 0, attemptNo = 1;
  auto dbu = [uu](const double v) { return static_cast<int>(std::lround(v * uu)); };

  while (std::getline(ifs, line)) {
    std::istringstream is(line);
    std::string tok;
    if (!(is >> tok)) continue;
    if (tok == "MACRO") { is >> macro; continue; }
    if (tok == "PIN")   { is >> section; layer.clear(); continue; }
    if (tok == "OBS")   { section = "OBS"; layer.clear(); continue; }
    if (tok == "END")   { std::string w; if (!(is >> w) || w == macro) section.clear(); continue; }
    if (tok == "LAYER") { is >> layer; if (!layer.empty() && layer.back() == ';') layer.pop_back(); continue; }
    if (tok != "RECT" || layer.empty()) continue;

    double a, b, c, d;
    if (!(is >> a >> b >> c >> d)) continue;
    const Geom::Rect rect(dbu(a), dbu(b), dbu(c), dbu(d));
    std::string base;
    if (section == "SRC" && suffix(layer, "_SRC", base)) {
      const int z = layerIndex(base);
      if (z >= 0) { r.addSourceShapes(rect, z); ++nsrc; }
    } else if (section == "TGT" && suffix(layer, "_TGT", base)) {
      const int z = layerIndex(base);
      if (z >= 0) { r.addTargetShapes(rect, z); ++ntgt; }
    } else if (section == "OBS") {
      if (layer == "BBOX") { mbox = rect; continue; }
      // <layer>_p carries the obstacle set as it went in; the bare <layer> entry
      // is the sliced and bloated form the grid derived from it.
      if (!suffix(layer, "_p", base)) continue;
      const int z = layerIndex(base);
      if (z >= 0) { obs[z].push_back(rect); ++nobs; }
    }
  }
  if (!nsrc || !ntgt) {
    CERR << "replay: " << leffile << " has no SRC/TGT shapes (is it an ATTEMPT_*.lef?)" << std::endl;
    return false;
  }

  // ATTEMPT_<pass>_<dir>_<module>_<wire> : recover the pass so the attempt-mode
  // switches (same-net tracing, boundary escapes) match the dump.
  std::string wire(macro), mod;
  if (wire.compare(0, 8, "ATTEMPT_") == 0) {
    size_t p = wire.find('_', 8);
    if (p != std::string::npos) {
      try { attemptNo = std::stoi(wire.substr(8, p - 8)); } catch (const std::exception&) {}
      size_t q = wire.find('_', p + 1);
      if (q != std::string::npos) wire = wire.substr(q + 1);
    }
  }
  size_t sep = wire.find("__");
  if (sep != std::string::npos) { mod = wire.substr(0, sep); }

  r.setuu(uu);
  r.setModName(mod.empty() ? "REPLAY" : mod);
  r.setName(wire);
  r.setAttemptNo(attemptNo);
  if (mbox.valid()) r.setMBox(mbox);
  r.addObstacles(obs, true);

  // The dump carries geometry but not the net's rules, so pull them from the
  // NDR file when one is given. The wire name is <module>_<net>__<p1>__<p2>,
  // and only the NDR file knows where the module name ends.
  std::map<int, int> ndrw, ndrs;
  std::map<int, DRC::Direction> ndrd;
  std::set<int> pref;
  std::map<int, DRC::ViaArray> ndrv;
  bool usendr = false, wantDetour = detour, pinwidth = false;
  std::string netname;
  if (!ndrfile.empty()) {
    std::ifstream nf(ndrfile);
    if (!nf.is_open()) {
      CERR << "replay: unable to open " << ndrfile << std::endl;
    } else {
      const std::string head = wire.substr(0, wire.find("__"));
      nlohmann::json oj;
      try { nf >> oj; } catch (const std::exception& e) {
        CERR << "replay: bad NDR json " << ndrfile << " : " << e.what() << std::endl;
        oj = nlohmann::json::array();
      }
      auto rules = [&](const nlohmann::json& o) {
        auto it = o.find("widths");
        if (it != o.end()) for (auto& e : it->items()) {
          const int z = lf.getLayerIndex(e.key());
          if (z >= 0 && e.value().is_number()) ndrw[z] = std::lround(static_cast<double>(e.value()) * uu);
        }
        it = o.find("spaces");
        if (it != o.end()) for (auto& e : it->items()) {
          const int z = lf.getLayerIndex(e.key());
          if (z >= 0 && e.value().is_number()) ndrs[z] = std::lround(static_cast<double>(e.value()) * uu);
        }
        it = o.find("directions");
        if (it != o.end()) for (auto& e : it->items()) {
          const int z = lf.getLayerIndex(e.key());
          if (z < 0 || !e.value().is_string()) continue;
          const std::string d = e.value();
          ndrd[z] = (d == "H" || d == "h") ? DRC::Direction::HORIZONTAL
                  : (d == "V" || d == "v") ? DRC::Direction::VERTICAL
                                           : DRC::Direction::ORTHOGONAL;
        }
        it = o.find("preferred_layers");
        if (it != o.end()) for (auto& e : *it) {
          const int z = lf.getLayerIndex(e);
          if (z >= 0) pref.insert(z);
        }
        it = o.find("vias");
        if (it != o.end()) for (auto& e : it->items()) {
          const int z = lf.getLayerIndex(e.key());
          if (z < 0) continue;
          auto& v = e.value();
          int q[6] = {0, 0, 0, 0, 0, 0};
          const char* keys[6] = {"WidthX", "WidthY", "SpaceX", "SpaceY", "NumX", "NumY"};
          for (int i = 0; i < 6; ++i) {
            auto iv = v.find(keys[i]);
            if (iv != v.end() && iv->is_number()) q[i] = *iv;
          }
          ndrv[z] = DRC::ViaArray(q[0], q[1], q[2], q[3], q[4], q[5]);
        }
        usendr = true;
      };
      for (auto& m : oj) {
        auto im = m.find("module");
        if (im == m.end() || !im->is_string()) continue;
        const std::string mod2 = *im;
        if (head.size() <= mod2.size() + 1 || head.compare(0, mod2.size(), mod2) != 0
            || head[mod2.size()] != '_') continue;
        netname = head.substr(mod2.size() + 1);
        rules(m);
        auto ip = m.find("use_pin_width");
        if (ip != m.end()) pinwidth = static_cast<int>(*ip) != 0;
        auto in = m.find("nets");
        if (in == m.end()) break;
        for (auto& n : *in) {
          auto nn = n.find("name");
          if (nn == n.end() || !nn->is_string() || std::string(*nn) != netname) continue;
          rules(n);
          auto id = n.find("large_detour");
          if (id != n.end() && id->is_string() && std::string(*id) == "allowed") wantDetour = true;
        }
        break;
      }
    }
  }
  if (pinwidth) r.setusepinwidth(true);
  r.updatendr(usendr, ndrw, ndrs, ndrd, pref, ndrv);
  if (wantDetour) r.allowDetour();

  COUT << "REPLAY file=" << leffile << " wire=" << wire << " attempt=" << attemptNo
       << " src=" << nsrc << " tgt=" << ntgt << " obstacles=" << nobs
       << " bbox=" << (mbox.valid() ? mbox.str() : std::string("<none>"))
       << (wantDetour ? " detour=on" : "")
       << (usendr ? (" ndr=" + (netname.empty() ? std::string("module") : netname)) : std::string())
       << (pinwidth ? " use_pin_width=on" : "") << '\n';

  auto sol = r.findSol();
  if (r.lastSolutionFound()) {
    COUT << "REPLAY RESULT routed " << wire << '\n';
    r.writeLEF("REPLAY_SOL", &sol);
  } else {
    COUT << "REPLAY RESULT open " << wire << '\n';
    r.writeLEF("REPLAY_FAIL");
  }
  return r.lastSolutionFound();
}

}
