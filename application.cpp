#include "application.h"

#include <iostream>
#include <limits>
#include <map>
#include <queue> // priority_queue
#include <set>
#include <stack>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <json.hpp>
#include "dist.h"
#include "graph.h"

using namespace std;
class prioritize {
 public:
  bool operator()(const pair<long long, double>& p1,
                  const pair<long long, double>& p2) const {
    return p1.second > p2.second;
  }
};


double INF = numeric_limits<double>::max();

void buildGraph(istream &input, graph<long long, double> &g,
                vector<BuildingInfo> &buildings,
                unordered_map<long long, Coordinates> &coords) {
    // Parse JSON
    nlohmann::json j;
    input >> j;

    for (const auto &wp : j["waypoints"]) {
        long long id = wp["id"];
        double lat = wp["lat"];
        double lon = wp["lon"];

        coords[id] = Coordinates(lat, lon);
        g.addVertex(id);
    }

    for (const auto &b : j["buildings"]) {
        BuildingInfo building;
        building.id = b["id"];
        building.abbr = b["abbr"];
        building.name = b["name"];
        building.location = Coordinates(b["lat"], b["lon"]);
        buildings.push_back(building);

        g.addVertex(building.id);
    }

    for (const auto &fw : j["footways"]) {
        for (size_t i = 0; i + 1 < fw.size(); ++i) {
            long long id1 = fw[i];
            long long id2 = fw[i + 1];

            double d = distBetween2Points(coords.at(id1), coords.at(id2));

            g.addEdge(id1, id2, d);
            g.addEdge(id2, id1, d);
        }
    }

    const double MAX_BUILDING_DIST = 0.036;
    for (const auto &building : buildings) {
        for (const auto &entry : coords) {
            long long waypointID = entry.first;

            if (waypointID == building.id) continue;

            double d = distBetween2Points(building.location, entry.second);
            if (d <= MAX_BUILDING_DIST) {
                g.addEdge(building.id, waypointID, d);
                g.addEdge(waypointID, building.id, d);
            }
        }
    }
}







BuildingInfo getBuildingInfo(const vector<BuildingInfo> &buildings,
                             const string &query) {
  for (const BuildingInfo &building : buildings) {
    if (building.abbr == query) {
      return building;
    } else if (building.name.find(query) != string::npos) {
      return building;
    }
  }
  BuildingInfo fail;
  fail.id = -1;
  return fail;
}

BuildingInfo getClosestBuilding(const vector<BuildingInfo> &buildings,
                                Coordinates c) {
  double minDestDist = INF;
  BuildingInfo ret = buildings.at(0);
  for (const BuildingInfo &building : buildings) {
    double dist = distBetween2Points(building.location, c);
    if (dist < minDestDist) {
      minDestDist = dist;
      ret = building;
    }
  }
  return ret;
}

vector<long long> dijkstra(const graph<long long, double> &G, long long start,
                           long long target,
                           const set<long long> &ignoreNodes) {

  unordered_map<long long, double> dist;
  unordered_map<long long, long long> prev;
    if(start == target){
    return {start};
  }
  
  for(auto& v: G.getVertices()){
    dist[v] = INF;
  }
  priority_queue<pair<long long, double>,
               vector<pair<long long, double>>,
               prioritize> worklist;
  dist[start] = 0;

  worklist.push({start,0});
  while(!worklist.empty()){
    auto[pos, distance] = worklist.top();
    worklist.pop();
    if(dist[pos] < distance){
      continue;
    }
    if(pos == target){
      break;
    }
    for(auto &p : G.neighbors(pos)){
      if(p != target&& p != start && ignoreNodes.count(p)){
        continue;
      }
      double weight;
      if(!G.getWeight(pos,p,weight)){
        continue;
      }
      if(dist[p] > (dist[pos] + weight)){
        dist[p] = dist[pos] + weight;
        prev[p] = pos;
        worklist.push({p,dist[p]});
      }

    }

  }
    vector<long long> route;
    if(dist[target] == INF){
      return {};
    }
    long long temp = target;
    while(temp != start){
      route.push_back(temp);
      temp = prev[temp];
    }
    route.push_back(start);
    reverse(route.begin(),route.end());
    return route;
}


double pathLength(const graph<long long, double> &G,
                  const vector<long long> &path) {
  double length = 0.0;
  double weight;
  for (size_t i = 0; i + 1 < path.size(); i++) {
    bool res = G.getWeight(path.at(i), path.at(i + 1), weight);
    if (!res) {
      return -1;
    }
    length += weight;
  }
  return length;
}

void outputPath(const vector<long long> &path) {
  for (size_t i = 0; i < path.size(); i++) {
    cout << path.at(i);
    if (i != path.size() - 1) {
      cout << "->";
    }
  }
  cout << endl;
}

// Honestly this function is just a holdover from an old version of the project
void application(const vector<BuildingInfo> &buildings,
                 const graph<long long, double> &G) {
  string person1Building, person2Building;

  set<long long> buildingNodes;
  for (const auto &building : buildings) {
    buildingNodes.insert(building.id);
  }

  cout << endl;
  cout << "Enter person 1's building (partial name or abbreviation), or #> ";
  getline(cin, person1Building);

  while (person1Building != "#") {
    cout << "Enter person 2's building (partial name or abbreviation)> ";
    getline(cin, person2Building);

    // Look up buildings by query
    BuildingInfo p1 = getBuildingInfo(buildings, person1Building);
    BuildingInfo p2 = getBuildingInfo(buildings, person2Building);
    Coordinates P1Coords, P2Coords;
    string P1Name, P2Name;

    if (p1.id == -1) {
      cout << "Person 1's building not found" << endl;
    } else if (p2.id == -1) {
      cout << "Person 2's building not found" << endl;
    } else {
      cout << endl;
      cout << "Person 1's point:" << endl;
      cout << " " << p1.name << endl;
      cout << " " << p1.id << endl;
      cout << " (" << p1.location.lat << ", " << p1.location.lon << ")" << endl;
      cout << "Person 2's point:" << endl;
      cout << " " << p2.name << endl;
      cout << " " << p2.id << endl;
      cout << " (" << p2.location.lon << ", " << p2.location.lon << ")" << endl;

      Coordinates centerCoords = centerBetween2Points(p1.location, p2.location);
      BuildingInfo dest = getClosestBuilding(buildings, centerCoords);

      cout << "Destination Building:" << endl;
      cout << " " << dest.name << endl;
      cout << " " << dest.id << endl;
      cout << " (" << dest.location.lat << ", " << dest.location.lon << ")"
           << endl;

      vector<long long> P1Path = dijkstra(G, p1.id, dest.id, buildingNodes);
      vector<long long> P2Path = dijkstra(G, p2.id, dest.id, buildingNodes);

      // This should NEVER happen with how the graph is built
      if (P1Path.empty() || P2Path.empty()) {
        cout << endl;
        cout << "At least one person was unable to reach the destination "
                "building. Is an edge missing?"
             << endl;
        cout << endl;
      } else {
        cout << endl;
        cout << "Person 1's distance to dest: " << pathLength(G, P1Path);
        cout << " miles" << endl;
        cout << "Path: ";
        outputPath(P1Path);
        cout << endl;
        cout << "Person 2's distance to dest: " << pathLength(G, P2Path);
        cout << " miles" << endl;
        cout << "Path: ";
        outputPath(P2Path);
      }
    }

    //
    // another navigation?
    //
    cout << endl;
    cout << "Enter person 1's building (partial name or abbreviation), or #> ";
    getline(cin, person1Building);
  }
}
