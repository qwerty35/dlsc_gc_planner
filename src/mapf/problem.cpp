#include "../include/mapf/problem.hpp"

#include <fstream>
#include <regex>

#include "../include/mapf/util.hpp"

using namespace MAPF;
Problem::Problem(Problem* P, Config _config_s, Config _config_g,
                 int _max_comp_time, int _max_timestep)
    : G(P->getG()),
      MT(P->getMT()),
      config_s(_config_s),
      config_g(_config_g),
      num_agents(P->getNum()),
      max_timestep(_max_timestep),
      max_comp_time(_max_comp_time),
      instance_initialized(false)
{
}

Problem::Problem(Problem* P, int _max_comp_time)
    : G(P->getG()),
      MT(P->getMT()),
      config_s(P->getConfigStart()),
      config_g(P->getConfigGoal()),
      num_agents(P->getNum()),
      max_timestep(P->getMaxTimestep()),
      max_comp_time(_max_comp_time),
      instance_initialized(false)
{
}

Problem::Problem(Grid* grid, int _num_agents, const ProblemAgents &agents)
                 : num_agents(_num_agents), instance_initialized(true)
{
    // read map
    G = grid;

    // read initial/goal nodes
    for (const auto& agent: agents) {
        int start_x = agent.start_node->pos.x;
        int start_y = agent.start_node->pos.y;
        int start_z = agent.start_node->pos.z;
        int current_x = agent.current_node->pos.x;
        int current_y = agent.current_node->pos.y;
        int current_z = agent.current_node->pos.z;
        int goal_x = agent.goal_node->pos.x;
        int goal_y = agent.goal_node->pos.y;
        int goal_z = agent.goal_node->pos.z;
        int obs_x = agent.obs_node->pos.x;
        int obs_y = agent.obs_node->pos.y;
        int obs_z = agent.obs_node->pos.z;
        if (!G->existNode(start_x, start_y, start_z)) {
            halt("start node (" + std::to_string(start_x) + ", " +
                 std::to_string(start_y) + "," +
                 std::to_string(start_z) + ") does not exist, invalid scenario");
        }
        if (!G->existNode(current_x, current_y, current_z)) {
            halt("current node (" + std::to_string(current_x) + ", " +
                         std::to_string(current_y) + "," +
                         std::to_string(current_z) + ") does not exist, invalid scenario");
        }
        if (!G->existNode(goal_x, goal_y, goal_z)) {
            halt("goal node (" + std::to_string(goal_x) + ", " + std::to_string(goal_y) + ", " +
                         std::to_string(goal_z) + ") does not exist, invalid scenario");
        }
        if (!G->existNode(obs_x, obs_y, obs_z)) {
            halt("obs node (" + std::to_string(obs_x) + ", " + std::to_string(obs_y) + ", " +
                 std::to_string(obs_z) + ") does not exist, invalid scenario");
        }

        Node* start = G->getNode(start_x, start_y, start_z);
        Node* s = G->getNode(current_x, current_y, current_z);
        Node* g = G->getNode(goal_x, goal_y, goal_z);
        Node* o = G->getNode(obs_x, obs_y, obs_z);
        config_start.push_back(start);
        config_s.push_back(s);
        config_g.push_back(g);
        config_o.push_back(o);
        obs_dists.push_back(agent.obs_dist);
    }

    // set default values
    MT = new std::mt19937(DEFAULT_SEED);
    max_timestep = DEFAULT_MAX_TIMESTEP;
    max_comp_time = DEFAULT_MAX_COMP_TIME;

    // check starts/goals
    if (num_agents <= 0) halt("invalid number of agents");
    const int config_s_size = config_s.size();
    if (!config_s.empty() && num_agents > config_s_size) {
        warn("given starts/goals are not sufficient\nrandomly create instances");
    }
    if (num_agents > config_s_size) {
        setRandomStartsGoals();
    }

    // trimming
    config_s.resize(num_agents);
    config_g.resize(num_agents);
}

Problem::~Problem()
{
  if (instance_initialized) {
    if (G != nullptr) delete G;
    if (MT != nullptr) delete MT;
  }
}

Node* Problem::getStart(int i) const
{
    if (!(0 <= i && i < (int)config_start.size())) halt("invalid index");
    return config_start[i];
}

Node* Problem::getCurrent(int i) const
{
  if (!(0 <= i && i < (int)config_s.size())) halt("invalid index");
  return config_s[i];
}

Node* Problem::getGoal(int i) const
{
  if (!(0 <= i && i < (int)config_g.size())) halt("invalid index");
  return config_g[i];
}

Node* Problem::getObstacle(int i) const
{
    if (!(0 <= i && i < (int)config_o.size())) halt("invalid index");
    if(getObsDist(i) > 1000){
        return nullptr;
    }
    return config_o[i];
}

float Problem::getObsDist(int i) const {
    if (!(0 <= i && i < (int)config_start.size())) halt("invalid index");
    return obs_dists[i];
}

void Problem::setRandomStartsGoals()
{
  // initialize
  config_s.clear();
  config_g.clear();

  // get grid size
  Grid* grid = reinterpret_cast<Grid*>(G);
  const int N = grid->getWidth() * grid->getDepth();

  // set starts
  std::vector<int> starts(N);
  std::iota(starts.begin(), starts.end(), 0);
  std::shuffle(starts.begin(), starts.end(), *MT);
  int i = 0;
  while (true) {
    while (G->getNode(starts[i]) == nullptr) {
      ++i;
      if (i >= N) halt("number of agents is too large.");
    }
    config_s.push_back(G->getNode(starts[i]));
    if ((int)config_s.size() == num_agents) break;
    ++i;
  }

  // set goals
  std::vector<int> goals(N);
  std::iota(goals.begin(), goals.end(), 0);
  std::shuffle(goals.begin(), goals.end(), *MT);
  int j = 0;
  while (true) {
    while (G->getNode(goals[j]) == nullptr) {
      ++j;
      if (j >= N) halt("set goal, number of agents is too large.");
    }
    // retry
    if (G->getNode(goals[j]) == config_s[config_g.size()]) {
      config_g.clear();
      std::shuffle(goals.begin(), goals.end(), *MT);
      j = 0;
      continue;
    }
    config_g.push_back(G->getNode(goals[j]));
    if ((int)config_g.size() == num_agents) break;
    ++j;
  }
}

/*
 * Note: it is hard to generate well-formed instances
 * with dense situations (e.g., ≥300 agents in arena)
 */
void Problem::setWellFormedInstance()
{
  // initialize
  config_s.clear();
  config_g.clear();

  // get grid size
  const int N = G->getNodesSize();
  Nodes prohibited, starts_goals;

  while ((int)config_g.size() < getNum()) {
    while (true) {
      // determine start
      Node* s;
      do {
        s = G->getNode(getRandomInt(0, N - 1, MT));
      } while (s == nullptr || inArray(s, prohibited));

      // determine goal
      Node* g;
      do {
        g = G->getNode(getRandomInt(0, N - 1, MT));
      } while (g == nullptr || g == s || inArray(g, prohibited));

      // ensure well formed property
      auto path = G->getPath(s, g, starts_goals);
      if (!path.empty()) {
        config_s.push_back(s);
        config_g.push_back(g);
        starts_goals.push_back(s);
        starts_goals.push_back(g);
        for (auto v : path) {
          if (!inArray(v, prohibited)) prohibited.push_back(v);
        }
        break;
      }
    }
  }
}

// I know that using "const" is something wired...
void Problem::halt(const std::string& msg) const
{
  std::cout << "error@Problem: " << msg << std::endl;
  this->~Problem();
  std::exit(1);
}

void Problem::warn(const std::string& msg) const
{
  std::cout << "warn@Problem: " << msg << std::endl;
}

void Problem::makeScenFile(const std::string& output_file)
{
  Grid* grid = reinterpret_cast<Grid*>(G);
  std::ofstream log;
  log.open(output_file, std::ios::out);
  log << "map_file=" << grid->getMapFileName() << "\n";
  log << "agents=" << num_agents << "\n";
  log << "seed=0\n";
  log << "random_problem=0\n";
  log << "max_timestep=" << max_timestep << "\n";
  log << "max_comp_time=" << max_comp_time << "\n";
  for (int i = 0; i < num_agents; ++i) {
    log << config_s[i]->pos.x << "," << config_s[i]->pos.y << ","
        << config_g[i]->pos.x << "," << config_g[i]->pos.y << "\n";
  }
  log.close();
}
