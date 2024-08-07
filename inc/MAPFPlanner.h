#pragma once
#include "ActionModel.h"
#include "SharedEnv.h"
#include "Types.h"
#include <ctime>
#include <vector>

namespace planner {

class MAPFPlanner {
public:
  const Grid& grid_;

  MAPFPlanner(const Grid& grid) : grid_(grid){};
  ~MAPFPlanner() {}

  std::vector<HeuristicTable> heuristics;
  std::vector<int> decision;
  std::vector<int> prev_decision;
  std::vector<double> p;
  std::vector<State> prev_states;
  std::vector<State> next_states;
  std::vector<int> tasks;
  std::vector<int> ids;
  std::vector<double> p_copy;
  std::vector<bool> occupied;
  std::vector<DCR> decided;
  std::vector<bool> checked;
  SharedEnvironment* env;
  // Start kit dummy implementation

  std::vector<Traj> trajs;

  std::vector<int> traffic;

  bool traffic_control = false;

  virtual void initialize(SharedEnvironment *initial_state,
                          double preprocess_time_limit);

  vector<Action> query(const std::vector<State>& start_states,
                       const std::vector<std::deque<tasks::Task>>& goal_locations, double time_limit);

  // return next states for all agents
  virtual void plan(int time_limit, const vector<State>& curr_states, std::vector<Action> &plan);

  // Start kit dummy implementation
  std::list<pair<int, int>>
  single_agent_plan(int start, int start_direct, int end,
                    unordered_set<tuple<int, int, int>> reservation, int time);
  std::list<pair<int, int>> single_agent_plan(int start, int start_direct,
                                              int end);
  int getManhattanDistance(int loc1, int loc2);
  std::list<pair<int, int>> getNeighbors(int location, int direction);
  bool validateMove(int loc, int loc2);
};
} // namespace planner
