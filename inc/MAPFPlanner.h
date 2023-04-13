#pragma once
#include <ctime>
#include "SharedEnv.h"
#include "ActionModel.h"

class MAPFPlanner
{
public:

  SharedEnvironment* env;

	//MAPFPlanner(SharedEnvironment* env): env(env){};
  MAPFPlanner(){env = new SharedEnvironment();};
	virtual ~MAPFPlanner(){}


  virtual void initialize(int preprocess_time_limit);

  // return next states for all agents
  virtual std::vector<Action> plan(int time_limit);

  std::list<pair<int,int>>single_agent_plan(int start,int start_direct, int end);

  int getManhattanDistance(int loc1, int loc2);
  std::list<pair<int,int>> getNeighbors(int location, int direction);

  bool validateMove(int loc,int loc2);


};
