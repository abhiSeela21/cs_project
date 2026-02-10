#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>

#include "application.h"
#include "graph.h"
#include "httplib.h"

using namespace httplib;
using namespace std;

template <class result_t = chrono::milliseconds,
          class clock_t = chrono::steady_clock,
          class duration_t = chrono::milliseconds>
auto since(chrono::time_point<clock_t, duration_t> const &start) {
  return chrono::duration_cast<result_t>(clock_t::now() - start);
}

// https://stackoverflow.com/a/29752943
string replaceAll(const string &source, const string &from, const string &to) {
  string newString;
  newString.reserve(source.length()); // avoids a few memory allocations

  string::size_type lastPos = 0;
  string::size_type findPos;

  while (string::npos != (findPos = source.find(from, lastPos))) {
    newString.append(source, lastPos, findPos - lastPos);
    newString += to;
    lastPos = findPos + from.length();
  }

  // Care for the rest after last occurrence
  newString += source.substr(lastPos);

  return newString;
}

int main() {

  // Build graph
  cout << "Building graph..." << endl;
  auto start = chrono::steady_clock::now();

  graph<long long, double> G;
  vector<BuildingInfo> buildings;
  unordered_map<long long, Coordinates> coords;

  ifstream input("data/uic-fa25.osm.json");
  buildGraph(input, G, buildings, coords);
  set<long long> building_ids;

  map<long long, BuildingInfo> building_info;
  for (const auto &building : buildings) {
    building_ids.emplace(building.id);
    building_info.emplace(building.id, building);
  }

  cout << "Built in " << since(start).count() << " ms" << endl;
  cout << "Launching server!" << endl;

  Server svr;

  // Root
  svr.Get("/", [&](const Request &req, Response &res) {
    (void)req;
    res.set_file_content("./www/map.html");
  });

  // Just map to files.
  string files[] = {"css/map.css", "js/map.js", "js/autocomplete.js",
                    "js/typeahead.js"};
  for (const string &f : files) {
    svr.Get("/" + f, [&](const Request &req, Response &res) {
      (void)req;
      res.set_file_content("./www/" + f);
    });
  }

  // Find paths!
  svr.Get("/pathfinder", [&](const Request &req, Response &res) {
    if (!(req.has_param("start") || req.has_param("start-id")) ||
        !(req.has_param("end") || req.has_param("end-id"))) {
      res.set_content("[]", "application/json");
      return;
    }

    long long start_id;
    if (req.has_param("start-id")) {
      start_id = stoll(req.get_param_value("start-id"));
    } else {
      start_id = getBuildingInfo(buildings, req.get_param_value("start")).id;
    }

    long long end_id;
    if (req.has_param("end-id")) {
      end_id = stoll(req.get_param_value("end-id"));
    } else {
      end_id = getBuildingInfo(buildings, req.get_param_value("end")).id;
    }

    if (start_id == -1 or end_id == -1) {
      res.set_content("[]", "application/json");
      return;
    }

    // Based on Dijkstra's, reconstruct the path
    ostringstream oss;
    oss << "[";
    vector<long long> path = dijkstra(G, start_id, end_id, building_ids);
    for (int i = 0; i < path.size(); i++) {
      long long loc_id = path[i];
      if (coords.count(loc_id)) {
        Coordinates &c = coords.at(loc_id);
        oss << "\"" << c.lat << "::" << c.lon << "::" << i << ":"
            << "::" << loc_id << "\"";
      }

      if (building_info.count(loc_id)) {
        BuildingInfo &bi = building_info.at(loc_id);
        oss << "\"" << bi.location.lat << "::" << bi.location.lon << "::" << i
            << ":" << bi.name << "::" << loc_id << "\"";
      }
      if (i != path.size() - 1) {
        oss << ",";
      }
    }
    oss << "]";

    res.set_content(oss.str(), "application/json");
  });

  // Search buildings by name
  svr.Get("/byname", [&](const Request &req, Response &res) {
    if (!req.has_param("query")) {
      res.set_content("[]", "application/json");
      return;
    }

    string query = req.get_param_value("query");

    vector<BuildingInfo> matches;
    for (BuildingInfo &bi : buildings) {
      if (bi.name.find(query) != string::npos) {
        matches.push_back(bi);
      }
    }

    ostringstream oss;
    oss << "[";
    for (size_t i = 0; i < matches.size(); i++) {
      BuildingInfo &bi = matches[i];
      oss << "\"" << bi.location.lat << "::" << bi.location.lon
          << "::" << bi.name << "::" << bi.id << "\"";
      if (i != matches.size() - 1) {
        oss << ",";
      }
    }
    oss << "]";

    res.set_content(oss.str(), "application/json");
  });

  // Nearest building to a location
  svr.Get("/nearest", [&](const Request &req, Response &res) {
    if (!(req.has_param("lat") && req.has_param("lon"))) {
      res.set_content("[]", "application/json");
      return;
    }

    double lat = stod(req.get_param_value("lat"));
    double lon = stod(req.get_param_value("lon"));
    BuildingInfo bi = getClosestBuilding(buildings, Coordinates{lat, lon});

    ostringstream oss;
    oss << "{\"id\":" << bi.id << ",\"name\":\"" << bi.name
        << "\",\"lat\":" << bi.location.lat << ",\"lon\":" << bi.location.lon
        << "}";
    res.set_content(oss.str(), "application/json");
  });

  // A silly way of autocompleting
  size_t NUM_OPTIONS = 15;
  svr.Get("/autocomplete", [&](const Request &req, Response &res) {
    if (!req.has_param("query")) {
      res.set_content("[]", "application/json");
      return;
    }

    string query = req.get_param_value("query");

    vector<BuildingInfo> matches;
    for (BuildingInfo &bi : buildings) {
      if (bi.name.find(query) != string::npos) {
        matches.push_back(bi);
        if (matches.size() == NUM_OPTIONS) {
          break;
        }
      }
    }

    ostringstream oss;
    oss << "[";
    for (size_t i = 0; i < matches.size(); i++) {
      oss << "{\"value\":\"" << replaceAll(matches[i].name, "\"", "\\\"")
          << "\",\"id\":" << matches[i].id << "}";
      if (i != matches.size() - 1) {
        oss << ",";
      }
    }
    oss << "]";
    res.set_content(oss.str(), "application/json");
  });

  svr.Get("/nearby", [&](const Request &req, Response &res) {
    if (!req.has_param("name")) {
      res.set_content("[]", "application/json");
      return;
    }
    ostringstream oss;
    oss << "[";

    // I don't totally understand why it was originally written this way
    // Not thinking about it
    Coordinates c;
    if (req.has_param("id") && coords.count(stoll(req.get_param_value("id")))) {
      long long id = stoll(req.get_param_value("id"));
      c = coords.at(id);
      oss << "\"" << c.lat << "::" << c.lon << "::" << "::" << id << "\"";
    } else {
      BuildingInfo bi = getBuildingInfo(buildings, req.get_param_value("name"));
      c = bi.location;
      oss << "\"" << bi.location.lat << "::" << bi.location.lon
          << "::" << bi.name << "::" << bi.id << "\"";
    }

    if (c.lat == 0) {
      res.set_content("[]", "application/json");
      return;
    }

    double dist = 200;
    if (req.has_param("distance")) {
      dist = stod(req.get_param_value("distance"));
    }

    vector<BuildingInfo> close;
    for (BuildingInfo &bi : buildings) {
      if (distBetween2Points(c, bi.location) < dist) {
        close.push_back(bi);
      }
    }

    if (close.size() > 0) {
      oss << ",";
    }

    for (size_t i = 0; i < close.size(); i++) {
      BuildingInfo &bi = close[i];
      if (bi.location == c) {
        continue;
      }

      oss << "\"" << bi.location.lat << "::" << bi.location.lon
          << "::" << bi.name << "::" << bi.id << "\"";
      if (i != close.size() - 1) {
        oss << ",";
      }
    }
    oss << "]";
    res.set_content(oss.str(), "application/json");
  });

  cout << endl
       << "Open your browser to http://localhost:1251 to see the map!" << endl;
  cout << "Hit CTRL-C to stop the server." << endl;
  svr.listen("localhost", 1251);
}
