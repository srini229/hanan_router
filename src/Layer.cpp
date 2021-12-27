#include "Layer.h"
#include <fstream>
#include <iostream>

#include "nlohmann/json.hpp"


namespace DRC {

using json = nlohmann::json;
using ordered_json = nlohmann::ordered_json;
LayerInfo::LayerInfo(const std::string& ljfile)
{
  if (ljfile.empty()) return;
  std::ifstream ifs(ljfile);
  if (ifs) {
    ordered_json oj = json::parse(ifs);
    auto itAbs = oj.find("Abstraction");
    if (itAbs != oj.end()) {
      for (auto& l : *itAbs) {
        auto it = l.find("Layer");
        if (it != l.end()) {
          std::string name = *it;
          it = l.find("Stack");
          if (it == l.end()) {
            it = l.find("GdsLayerNo");
            int gdsNo = (it != l.end()) ? static_cast<int>(*it) : -1;
            it = l.find("UnitR");
            float mur(0), lr(0), ur(0);
            if (it != l.end()) {
              mur = (*it)["Mean"];
              lr = (*it)["L_3Sigma"];
              ur = (*it)["U_3Sigma"];
            }
            it = l.find("Pitch");
            MetalLayer* mlayer(nullptr);
            if (it != l.end() && it->is_number_integer()) {
              mlayer = new MetalLayer(gdsNo, name, mur, lr, ur);
              mlayer->setPitch(static_cast<int>(*it));
            }
            if (mlayer != nullptr) {
              it = l.find("Width");
              if (it != l.end() && it->is_number_integer()) mlayer->setWidth(static_cast<int>(*it));
              it = l.find("MinL");
              if (it != l.end() && it->is_number_integer()) mlayer->setMinL(static_cast<int>(*it));
              it = l.find("MaxL");
              if (it != l.end() && it->is_number_integer()) mlayer->setMaxL(static_cast<int>(*it));
              it = l.find("EndToEnd");
              if (it != l.end() && it->is_number_integer()) mlayer->setE2E(static_cast<int>(*it));
              it = l.find("Offset");
              if (it != l.end() && it->is_number_integer()) mlayer->setOffset(static_cast<int>(*it));
              it = l.find("Direction");
              if (it != l.end() && it->is_string()) mlayer->setDirection(*it == "H");
              _mlayers.push_back(mlayer);
              _mlayerNameMap[name] = mlayer;
            }
          }
        }
      }
      for (auto& l : *itAbs) {
        auto it = l.find("Layer");
        if (it != l.end()) {
          std::string name = *it;
          it = l.find("Stack");
          if (it != l.end()) {
            MetalLayer* layer1(nullptr), *layer2(nullptr);
            if ((*it).is_array()) {
              for (auto i : {0,1}) {
                std::string l = (*it)[i].is_string() ? static_cast<std::string>((*it)[i]) : "";
                if (!l.empty()) {
                  auto itm = _mlayerNameMap.find(l);
                  if (itm != _mlayerNameMap.end()) {
                    if (i == 0) {
                      layer1 = itm->second;
                    } else {
                      layer2 = itm->second;
                    }
                  }
                }
              }
            }
            if (layer1 != nullptr || layer2 != nullptr) {
              it = l.find("GdsLayerNo");
              int gdsNo = (it != l.end()) ? static_cast<int>(*it) : -1;
              it = l.find("R");
              float mur(0), lr(0), ur(0);
              if (it != l.end()) {
                mur = (*it)["Mean"];
                lr = (*it)["L_3Sigma"];
                ur = (*it)["U_3Sigma"];
              }
              ViaLayer* vlayer = new ViaLayer(gdsNo, name, mur, lr, ur);
              vlayer->setLayerPair(layer1, layer2);
              int x(0), y(0);
              it = l.find("WidthX");
              if (it != l.end() && it->is_number_integer()) x = static_cast<int>(*it);
              it = l.find("WidthY");
              if (it != l.end() && it->is_number_integer()) y = static_cast<int>(*it);
              vlayer->setWidth(x, y);
              x = y = 0;
              it = l.find("SpaceX");
              if (it != l.end() && it->is_number_integer()) x = static_cast<int>(*it);
              it = l.find("SpaceY");
              if (it != l.end() && it->is_number_integer()) y = static_cast<int>(*it);
              vlayer->setSpace(x, y);
              x = y = 0;
              it = l.find("VencA_L");
              if (it != l.end() && it->is_number_integer()) x = static_cast<int>(*it);
              it = l.find("VencP_L");
              if (it != l.end() && it->is_number_integer()) y = static_cast<int>(*it);
              vlayer->setCoverL(x, y);
              x = y = 0;
              it = l.find("VencA_H");
              if (it != l.end() && it->is_number_integer()) x = static_cast<int>(*it);
              it = l.find("VencP_H");
              if (it != l.end() && it->is_number_integer()) y = static_cast<int>(*it);
              vlayer->setCoverU(x, y);
              x = y = 0;
              _vlayers.push_back(vlayer);
            }
          }
        }
      }
    }
  }

  _layers.insert(_layers.end(), _mlayers.begin(), _mlayers.end());
  _layers.insert(_layers.end(), _vlayers.begin(), _vlayers.end());
  for (unsigned i = 0; i < _layers.size(); ++i) {
    _layerIndex[_layers[i]->name()] = static_cast<int>(i);
    std::cout << "layer : " << _layers[i]->name() << " index : " << i << '\n';
  }
}

LayerInfo::~LayerInfo()
{
  _vldn.clear();
  _vlup.clear();
  for (auto& v : _vlayers) delete v;
  for (auto& m : _mlayers) delete m;
}

};
