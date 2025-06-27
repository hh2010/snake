#include "util.hpp"
#include "game.hpp"

#include "zig_zag_agent.hpp"
#include "cell_tree_agent.hpp"
#include "hamiltonian_cycle.hpp"
#include "cheat_agent.hpp"

#include <unistd.h>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <memory>
#include <functional>
#include <thread>
#include <mutex>
#include <chrono>

// Global variable to track current game number for debug output
thread_local int current_game_number = 0;

//------------------------------------------------------------------------------
// Logging games
//------------------------------------------------------------------------------

struct Log {
  std::vector<Coord> snake_pos;
  std::vector<int>   snake_size;
  std::vector<Coord> apple_pos;
  std::vector<int>   eat_turns;

  void log(Game const& game, Game::Event event) {
    snake_pos.push_back(game.snake_pos());
    snake_size.push_back(game.snake.size());
    if (event == Game::Event::eat) {
      eat_turns.push_back(game.turn);
    }
    if (apple_pos.empty() || event == Game::Event::eat) {
      apple_pos.push_back(game.apple_pos);
    }
  }
};

class LoggedGame : public Game {
public:
  Log log;
  
  LoggedGame(CoordRange dims, RNG const& rng, bool ignore_snake_collisions = false) 
    : Game(dims, rng, ignore_snake_collisions) {
    log.log(*this, Game::Event::none);
  }
  Event move(Dir d) {
    Event e = Game::move(d);
    log.log(*this, e);
    return e;
  }
};

//------------------------------------------------------------------------------
// Stats of multiple games
//------------------------------------------------------------------------------

struct Stats {
  std::vector<int> turns;
  std::vector<bool> wins;
  
  // Unreachable metrics for cell tree agents
  std::vector<int> first_unreachable_turns;
  std::vector<int> lengths_at_first_unreachable;
  std::vector<int> steps_with_unreachables;
  std::vector<int> cumulative_unreachable_cells;
  
  void add(Game const& game);
  void add(Game const& game, Agent* agent);
};

void Stats::add(Game const& game) {
  wins.push_back(game.win());
  if (game.win()) {
    turns.push_back(game.turn);
  }
}

void Stats::add(Game const& game, Agent* agent) {
  add(game);
  
  // Extract metrics from cell tree agents
  auto* cell_agent = dynamic_cast<CellTreeAgent*>(agent);
  if (cell_agent) {
    auto metrics = cell_agent->getMetrics();
    if (metrics.first_unreachable_step >= 0) {
      first_unreachable_turns.push_back(metrics.first_unreachable_step);
      lengths_at_first_unreachable.push_back(metrics.length_at_first_unreachable);
    }
    steps_with_unreachables.push_back(metrics.steps_with_unreachables);
    cumulative_unreachable_cells.push_back(metrics.cumulative_unreachable_cells);
  }
}

std::ostream& operator << (std::ostream& out, Stats const& stats) {
  out << "turns: mean " << mean(stats.turns);
  out << ", stddev " << stddev(stats.turns);
  out << ", quantiles " << quantiles(stats.turns);
  if (mean(stats.wins) < 1) {
    out << "  LOST: " << (1-mean(stats.wins))*100 << "%";
  }
  
  // Add unreachable metrics if they exist
  if (!stats.first_unreachable_turns.empty()) {
    out << "\nunreachable metrics:";
    out << "\n  first unreachable: mean " << mean(stats.first_unreachable_turns);
    out << ", quantiles " << quantiles(stats.first_unreachable_turns);
    out << "\n  length at first unreachable: mean " << mean(stats.lengths_at_first_unreachable);
    out << ", quantiles " << quantiles(stats.lengths_at_first_unreachable);
    out << "\n  steps with unreachables: mean " << mean(stats.steps_with_unreachables);
    out << ", quantiles " << quantiles(stats.steps_with_unreachables);
    out << "\n  cumulative unreachable cells: mean " << mean(stats.cumulative_unreachable_cells);
    out << ", quantiles " << quantiles(stats.cumulative_unreachable_cells);
  }
  
  return out;
}

//------------------------------------------------------------------------------
// Configuration
//------------------------------------------------------------------------------

enum class Trace {
  no, eat, all
};
struct Config {
  int num_rounds = 100;
  CoordRange board_size = {30,30};
  Trace trace = Trace::no;
  bool quiet = false;
  int num_threads = static_cast<int>(std::thread::hardware_concurrency());
  std::string json_file;
  bool json_compact = true;
  RNG rng = global_rng;
  int min_trace_turn = 0;       // Minimum turn to start tracing from
  int max_trace_turn = INT_MAX; // Maximum turn to trace until
  std::string flood_fill_json;  // Path to output flood fill debug JSON
  int flood_fill_turn = -1;     // Turn number to start logging flood fill (-1 = disabled)
  int round_to_run = 0;
  
  void parse_optional_args(int argc, const char** argv);
};

//------------------------------------------------------------------------------
// Agents
//------------------------------------------------------------------------------

struct AgentFactory {
  std::string name;
  std::string description;
  std::function<std::unique_ptr<Agent>(Config&)> make;
};
AgentFactory agents[] = {
  {"cheat", "Takes shortest path to apple ignoring snake collisions", [](Config&) {
    return std::make_unique<CheatAgent>();
  }},
  {"zig-zag", "Follows a fixed zig-zag cycle", [](Config&) {
    return std::make_unique<FixedZigZagAgent>();
  }},
  {"fixed", "Follows a fixed but random cycle", [](Config& config) {
    return std::make_unique<FixedCycleAgent>(random_hamiltonian_cycle(config.board_size, config.rng));
  }},
  {"zig-zag-cut", "Follows a zig-zag cycle, but can take shortcuts", [](Config& config) {
    return std::make_unique<CutAgent>();
  }},
  {"cell", "Limit movement to a tree of 2x2 cells", [](Config&) {
    return std::make_unique<CellTreeAgent>();
  }},
  {"cell1", "Cell tree agent with limited lookahead", [](Config&) {
    auto agent = std::make_unique<CellTreeAgent>();
    agent->lookahead = Lookahead::one;
    return agent;
  }},
  {"cell-keep", "Cell tree agent which doesn't move snake in lookahead", [](Config&) {
    auto agent = std::make_unique<CellTreeAgent>();
    agent->lookahead = Lookahead::many_keep_tail;
    return agent;
  }},
  {"cell-fixed", "Cell agent that doesn't recalculate paths", [](Config&) {
    auto agent = std::make_unique<CellTreeAgent>();
    agent->recalculate_path = false;
    return agent;
  }},
  {"cell-variant", "Cell tree agent with penalties on moving in the tree", [](Config&) {
    auto agent = std::make_unique<CellTreeAgent>();
    agent->same_cell_penalty = 500-1;
    agent->new_cell_penalty = 2400;
    agent->parent_cell_penalty = 0;
    return agent;
  }},
  {"phc", "Perturbed Hamiltonian cycle (zig-zag cycle)", [](Config& config) {
    auto agent = std::make_unique<PerturbedHamiltonianCycle>(make_zig_zag_path(config.board_size));
    return agent;
  }},
  {"dhcr", "Dynamic Hamiltonian Cycle Repair", [](Config& config) {
    auto agent = std::make_unique<DynamicHamiltonianCycleRepair>(make_zig_zag_path(config.board_size));
    return agent;
  }},
  {"dhcr-nascar", "Dynamic Hamiltonian Cycle Repair with Nascar mode", [](Config& config) {
    auto agent = std::make_unique<DynamicHamiltonianCycleRepair>(make_zig_zag_path(config.board_size));
    agent->wall_follow_overshoot = 1;
    return agent;
  }},
};

void list_agents(std::ostream& out = std::cout) {
  out << "Available agents:" << std::endl;
  for (auto const& a : agents) {
    out << "  " << std::left << std::setw(20) << a.name;
    out << a.description << std::endl;
  }
}

AgentFactory const& find_agent(std::string const& name) {
  for (auto const& a : agents) {
    if (a.name == name) return a;
  }
  throw std::invalid_argument("Unknown agent: " + name + "\nUse `list` command to list available agents.");
}

//------------------------------------------------------------------------------
// Argument handling
//------------------------------------------------------------------------------

void print_help(const char* name, std::ostream& out = std::cout) {
  Config def;
  using namespace std;
  out << "Usage: " << name << " <mode> <args>" << endl;
  out << endl;
  out << "These modes are available:" << endl;
  out << "  help                Show this message." << endl;
  out << "  list                List available agents." << endl;
  out << "  all                 Play all agents against each other, output csv summary." << endl;
  out << "  <agent>             Play with the given agent." << endl;
  out << endl;
  out << "Optional arguments:" << endl;
  out << "  -n, --n <rounds>    Run the given number of rounds (default: " << def.num_rounds << ")." << endl;
  out << "  -s, --size <size>   Size of the (square) board (default: " << def.board_size.w << ")." << endl;
  out << "      --seed <n>      Random seed." << endl;
  out << "  -T, --trace-all [min max]  Print the game state after each move." << endl;
  out << "                      When min and max are provided, only print turns between min and max (inclusive)." << endl;
  out << "  -t, --trace [min max]      Print the game state each time the snake eats an apple." << endl;
  out << "                      When min and max are provided, only print turns between min and max (inclusive)." << endl;
  out << "      --no-color      Don't use ANSI color codes in trace output" << endl;
  out << "  -q, --quiet         Don't print extra output." << endl;
  out << "      --json <file>   Write log of one run a json file." << endl;
  out << "      --json-full     Don't encode json file to save size." << endl;
  out << "  -j, --threads <n>   Specify the maximum number of threads (default: " << def.num_threads << ")." << endl;
  out << endl;
  list_agents(out);
}

void Config::parse_optional_args(int argc, const char** argv) {
  auto parse_min_max_turns = [&](int& i) {
    if (i+2 < argc && isdigit(argv[i+1][0]) && isdigit(argv[i+2][0])) {
      min_trace_turn = std::stoi(argv[++i]);
      max_trace_turn = std::stoi(argv[++i]);
    }
  };

  for (int i=0; i<argc; ++i) {
    std::string arg = argv[i];
    if (arg == "-n" || arg == "--n") {
      if (i+1 >= argc) throw std::invalid_argument("Missing argument to " + arg);
      num_rounds = std::stoi(argv[++i]);
    } else if (arg == "-s" || arg == "--size") {
      if (i+1 >= argc) throw std::invalid_argument("Missing argument to " + arg);
      int size = std::stoi(argv[++i]);
      board_size = {size,size};
    } else if (arg == "-w" || arg == "--width") {
      if (i+1 >= argc) throw std::invalid_argument("Missing argument to " + arg);
      board_size.w = std::stoi(argv[++i]);
    } else if (arg == "-h" || arg == "--height") {
      if (i+1 >= argc) throw std::invalid_argument("Missing argument to " + arg);
      board_size.h = std::stoi(argv[++i]);
    } else if (arg == "--seed") {
      if (i+1 >= argc) throw std::invalid_argument("Missing argument to " + arg);
      int seed = std::stoi(argv[++i]);
      uint64_t s[] = {1234567891234567890u,9876543210987654321u+seed};
      rng = RNG(s);
    } else if (arg == "--json") {
      if (i+1 >= argc) throw std::invalid_argument("Missing argument to " + arg);
      json_file = argv[++i];
    } else if (arg == "-t" || arg == "--trace") {
      trace = Trace::eat;
      num_rounds = 1;
      parse_min_max_turns(i);
    } else if (arg == "-T" || arg == "--trace-all") {
      trace = Trace::all;
      num_rounds = 1;
      parse_min_max_turns(i);
    } else if (arg == "-q" || arg == "--quiet") {
      quiet = true;
    } else if (arg == "-j" || arg == "--threads" || arg == "--num-threads") {
      if (i+1 >= argc) throw std::invalid_argument("Missing argument to " + arg);
      num_threads = std::stoi(argv[++i]);
    } else if (arg == "--no-color") {
      use_color = false;
    } else if (arg == "--json-full") {
      json_compact = false;
    } else if (arg == "--flood-fill-json") {
      if (i+2 >= argc) throw std::invalid_argument("Missing arguments to " + arg);
      flood_fill_json = argv[++i];
      flood_fill_turn = std::stoi(argv[++i]);
      // add some restrictions on this. cannot be used with json_file, can only be used for cell
    }
      else if (arg == "--round") {
      if (i+1 >= argc) throw std::invalid_argument("Missing argument to " + arg);
      round_to_run = std::stoi(argv[++i]);
      if (round_to_run < 1) throw std::invalid_argument("Round must be positive");
      num_rounds = 1;  // Only run a single round when --round is specified
    } else {
      throw std::invalid_argument("Unknown argument: " + arg);
    }
  }
}

//------------------------------------------------------------------------------
// Json output
//------------------------------------------------------------------------------

void write_json(std::ostream& out, int x) {
  out << x;
}
void write_json(std::ostream& out, Coord c) {
  out << "[" << c.x << "," << c.y << "]";
}
void write_json(std::ostream& out, CoordRange c) {
  out << "[" << c.w << "," << c.h << "]";
}

template <typename T>
void write_json(std::ostream& out, std::vector<T> const& xs) {
  out << "[";
  bool first = true;
  for (auto const& x : xs) {
    if (!first) {
      out << ",";
    }
    first = false;
    write_json(out, x);
  }
  out << "]";
}

// encode paths as strings to save bandwidth
//  * coordinates encoded as  '#' + x, '#' + y, since chars after '#' don't need escapes (except for \)
//  * subsequent coordinates encoded by movement direction (0...3 = Dir::up...Dir::right)
//  * three dirs fit into a char, '\0' + first + 4*(1 + second + 4*(1 + third))
//  * this is a hackish variant of base64 encoding
bool can_encode_path(std::vector<Coord> const& xs) {
  if (xs.empty() || xs[0].x > 85 || xs[0].y > 85) return false;
  for (size_t i=1; i<xs.size(); ++i) {
    if (!is_neighbor(xs[i],xs[i-1])) return false;
  }
  return true;
}
void encode_char(std::ostream& out, int c) {
  if (c == '\\') out << "\\\\";
  else if (c == '\"') out << "\\\"";
  else out << (char)c;
}
void write_json_path(std::ostream& out, std::vector<Coord> const& xs, bool compact) {
  if (!compact || !can_encode_path(xs)) {
    write_json(out,xs);
    return;
  }
  out << "\"";
  encode_char(out, xs[0].x + 35);
  encode_char(out, xs[0].y + 35);
  for (size_t i=1; i<xs.size(); i+=3) {
    int d = 0;
    if (i+2 < xs.size()) {
      d = 1 + (int)(xs[i+2] - xs[i+1]) + 4*d;
    }
    if (i+1 < xs.size()) {
      d = 1 + (int)(xs[i+1] - xs[i]) + 4*d;
    }
    d = (int)(xs[i] - xs[i-1]) + 4*d;
    encode_char(out, d + 35);
  }
  out << "\"";
}
// Encode grids as strings
// 6 bits fit into a char (base64)
void write_json_grid(std::ostream& out, Grid<bool> const& xs, bool compact) {
  out << "\"";
  for (auto it = xs.begin(); it != xs.end();) {
    int d = 0;
    for (size_t j=0; j<6 && it != xs.end() ; ++j, ++it) {
      if (*it) d |= (1<<j);
    }
    encode_char(out, d + 35);
  }
  out << "\"";
}

void write_json_log(std::ostream& out, AgentLog::LogEntry const* prev, AgentLog::NoEntry const& e, bool compact) {
  out << 0;
}
void write_json_log(std::ostream& out, AgentLog::LogEntry const* prev, AgentLog::CopyEntry const& e, bool compact) {
  out << 1;
}
void write_json_log(std::ostream& out, AgentLog::LogEntry const* prev, std::vector<Coord> const& path, bool compact) {
  // compare to previous path
  if (compact) {
    auto prev_path = std::get_if<std::vector<Coord>>(prev);
    if (prev_path && prev_path->size() >= path.size()) {
      if (std::equal(path.begin(), path.end(), prev_path->begin())) {
        // path is a prefix of previous path, encode more efficiently
        out << (1 + prev_path->size() - path.size());
        return;
      }
    }
  }
  write_json_path(out, path, compact);
}
void write_json_log(std::ostream& out, AgentLog::LogEntry const* prev, Grid<bool> const& grid, bool compact) {
  if (compact) {
    auto prev_grid = std::get_if<Grid<bool>>(prev);
    if (prev_grid && std::equal(grid.begin(), grid.end(), prev_grid->begin())) {
      out << 1;
      return;
    }
  }
  write_json_grid(out, grid, compact);
}

void write_json_log(std::ostream& out, AgentLog::LogEntry const* prev, std::vector<int> const& metrics, bool compact) {
  // Handle integer metrics vector specifically for unreachable metrics
  out << "[";
  bool first = true;
  for (auto const& x : metrics) {
    if (!first) {
      out << ",";
    }
    first = false;
    out << x;
  }
  out << "]";
}

void write_json(std::ostream& out, std::vector<AgentLog::LogEntry> const& xs, bool compact) {
  out << "[";
  AgentLog::LogEntry const* prev = nullptr;
  for (auto const& x : xs) {
    if (prev) {
      out << ",";
    }
    std::visit([&out,prev,compact](auto const& x){write_json_log(out,prev,x,compact);}, x);
    prev = &x;
  }
  out << "]";
}

void write_json(std::ostream& out, FloodFillDebug const& debug, AgentLog const& agent_log) {
  try {
    out << "{" << std::endl;
    out << "  \"size\": "; write_json(out, debug.size); out << "," << std::endl;
    out << "  \"turn\": " << debug.turn << "," << std::endl;
    out << "  \"snake_pos\": "; write_json(out, debug.snake_pos); out << "," << std::endl;
    out << "  \"snake_size\": " << debug.snake_size << "," << std::endl;
    out << "  \"apple_pos\": "; write_json(out, debug.apple_pos); out << "," << std::endl;
    out << "  \"flood_fill_debug\": {" << std::endl;
    out << "    \"start_coord\": "; write_json(out, debug.start_coord);

    // Plan output (if exists)
    if (!agent_log.logs[AgentLog::Key::plan].empty()) {
      out << "," << std::endl;  // Add comma only if there's more content
      out << "    \"" << AgentLog::key_name((AgentLog::Key)AgentLog::Key::plan) << "\": ";
      write_json(out, std::vector{agent_log.logs[AgentLog::Key::plan].back()}, false);
    }

    if (!agent_log.logs[AgentLog::Key::after_snake].empty()) {
      out << "," << std::endl;  // Add comma only if there's more content
      out << "    \"" << AgentLog::key_name((AgentLog::Key)AgentLog::Key::after_snake) << "\": ";
      write_json(out, std::vector{agent_log.logs[AgentLog::Key::after_snake].back()}, false);
    }

    // Fill states output (only add comma if we're going to write fill states)
    if (!debug.fill_states.empty()) {
      out << "," << std::endl;
      out << "    \"fill_states\": [" << std::endl;
      for (size_t i = 0; i < debug.fill_states.size(); ++i) {
        out << "      ";
        write_json_grid(out, debug.fill_states[i], true);
        if (i < debug.fill_states.size() - 1) out << ",";
        out << std::endl;
      }
      out << "    ]" << std::endl;
    }
    out << "  }" << std::endl;
    out << "}" << std::endl;
  } catch (const std::exception& e) {
    throw std::runtime_error("Failed to write JSON: " + std::string(e.what()));
  }
}

void write_json(std::string const& filename, FloodFillDebug const& debug, AgentLog const& agent_log) {
  std::ofstream out(filename);
  if (!out.is_open()) {
    throw std::runtime_error("Could not open file for writing: " + filename);
  }
  
  write_json(out, debug, agent_log);
  
  if (out.fail()) {
    out.close();
    throw std::runtime_error("Failed to write to file: " + filename);
  }
  
  out.flush();
  out.close();
}

void write_json(std::ostream& out, AgentFactory const& agent, LoggedGame const& game, AgentLog const& agent_log, bool compact = true) {
  out << "{" << std::endl;
  out << "  \"agent\": \"" << agent.name << "\"," << std::endl;
  out << "  \"agent_description\": \"" << agent.description << "\"," << std::endl;
  out << "  \"size\": "; write_json(out, game.dimensions()); out << "," << std::endl;
  out << "  \"snake_pos\": "; write_json_path(out, game.log.snake_pos, compact); out << "," << std::endl;
  if (!compact) {
    out << "  \"snake_size\": "; write_json(out, game.log.snake_size); out << "," << std::endl;
  }
  out << "  \"apple_pos\": "; write_json(out, game.log.apple_pos); out << "," << std::endl;
  out << "  \"eat_turns\": "; write_json(out, game.log.eat_turns);
  for (int i = 0; i < AgentLog::MAX_KEY; ++i) {
    if (!agent_log.logs[i].empty()) {
      out << "," << std::endl;
      out << "  \"" << AgentLog::key_name((AgentLog::Key)i) << "\": ";
      write_json(out, agent_log.logs[i], compact);
    }
  }
  out << std::endl << "}" << std::endl;
}

void write_json(std::string const& filename, AgentFactory const& agent, LoggedGame const& game, AgentLog const& agent_log, bool compact = true) {
  std::ofstream out(filename);
  if (!out.is_open()) {
    throw std::runtime_error("Could not open file for writing: " + filename);
  }
  write_json(out, agent, game, agent_log, compact);
}

//------------------------------------------------------------------------------
// Playing full games
//------------------------------------------------------------------------------

enum class Visualize {
  no, eat, all
};

// Helper function to set up and execute flood fill debugging
void setupFloodFillDebug(Game& game, Agent& agent, AgentLog* log, const std::string& output_file, bool compact) {
  // Create a fresh AgentLog if none was provided
  std::unique_ptr<AgentLog> temp_log;
  if (!log) {
    temp_log = std::make_unique<AgentLog>();
    log = temp_log.get();
  }

  game.flood_fill_debug = std::make_shared<FloodFillDebug>();
  auto& debug = *game.flood_fill_debug;
  debug.turn = game.turn;
  debug.size = game.dimensions();
  debug.snake_pos.clear();
  for (auto it = game.snake.begin(); it != game.snake.end(); ++it) {
    debug.snake_pos.push_back(*it);
  }
  debug.snake_size = game.snake.size();
  debug.apple_pos = game.apple_pos;
  debug.start_coord = game.snake_pos();
  FloodFillDebug::active_debug = &debug;
  
  // Get next move and execute it - this will trigger flood fill logging
  Dir next_move = agent(game, log);
  auto event = game.move(next_move);
  
  // Write debug info and clean up
  FloodFillDebug::active_debug = nullptr;
  try {
    write_json(output_file, debug, *log);
    std::cout << "Wrote flood fill debug info to " << output_file << std::endl;
  } catch (const std::exception& e) {
    std::cerr << "Error writing flood fill debug info: " << e.what() << std::endl;
  }
}

template <typename Game>
void play(Game& game, Agent& agent, Config const& config, AgentLog* log = nullptr) {
  while (!game.done()) {
    if (config.trace == Trace::all && game.turn >= config.min_trace_turn && game.turn <= config.max_trace_turn) std::cout << game;
    
    // Enable flood fill debugging if we're at the target turn
    if (!config.flood_fill_json.empty() && game.turn == config.flood_fill_turn) {
      setupFloodFillDebug(game, agent, log, config.flood_fill_json, config.json_compact);
      return;
    }
    
    auto event = game.move(agent(game,log));
    if (event == Game::Event::eat && config.trace == Trace::eat && game.turn >= config.min_trace_turn && game.turn <= config.max_trace_turn) std::cout << game;
  }
  if (config.trace == Trace::all && game.turn >= config.min_trace_turn && game.turn <= config.max_trace_turn) std::cout << game;
}

template <typename AgentGen>
Stats play_multiple_threaded(AgentGen make_agent, Config& config, bool is_cheat_agent = false) {
  std::mutex mutex;
  std::vector<std::thread> threads;
  int remaining = config.num_rounds;
  Stats stats;
  
  for (int thread = 0; thread < config.num_threads; ++thread) {
    threads.push_back(std::thread([&,thread,is_cheat_agent](){
      while (true) {
        std::unique_ptr<Agent> agent;
        RNG rng;
        int game_number;
        {
          std::lock_guard<std::mutex> guard(mutex);
          if (remaining <= 0) return;
          remaining--;
          game_number = config.num_rounds - remaining;
          agent = make_agent(config); // potentially uses rng
          rng = config.rng.next_rng();
        }
        Game game(config.board_size, rng, is_cheat_agent);
        play(game, *agent, config);
        {
          std::lock_guard<std::mutex> guard(mutex);
          stats.add(game, agent.get());
          if (!config.quiet) {
            // std::cout << stats.wins.size() << "/" << config.num_rounds << std::endl;
            std::cout << stats.wins.size() << "/" << config.num_rounds << "  " << stats << "\033[K\r" << std::flush;
          }
        }
      }
    }));
  }
  // wait
  for (auto& t : threads) {
    t.join();
  }
  // done
  if (!config.quiet) std::cout << "\033[K\r";
  // if (!config.quiet) std::cout << std::endl;
  return stats;
}

template <typename AgentGen>
Stats play_multiple(AgentGen make_agent, Config& config, bool is_cheat_agent = false) {
  if (config.num_threads > 1) return play_multiple_threaded(make_agent, config, is_cheat_agent);
  Stats stats;

  if (config.round_to_run > 0) {
    // Advance RNG state to the specific round
    for (int i = 1; i < config.round_to_run; ++i) {
      config.rng.next_rng(); // Skip RNG states but don't use them
    }
    
    // Now run just the one round with the correct RNG state
    Game game(config.board_size, config.rng.next_rng(), is_cheat_agent);
    auto agent = make_agent(config);
    play(game, *agent, config);
    stats.add(game, agent.get());
    if (!config.quiet) {
      if (!game.win()) std::cout << game;
      std::cout << "Ran round " << config.round_to_run << std::endl;
    }
    return stats;
  }

  for (int i = 0; i < config.num_rounds; ++i) {
    if (!config.quiet) {
      std::cout << "Starting game " << (i+1) << "/" << config.num_rounds << std::endl;
    }
    Game game(config.board_size, config.rng.next_rng());
    auto agent = make_agent(config);
    play(game, *agent, config);
    stats.add(game);
    if (!config.quiet) {
      if (!game.win()) std::cout << game;
      std::cout << (i+1) << "/" << config.num_rounds << "  " << stats << "\033[K\r" << std::flush;
      // std::cout << (i+1) << "/" << config.num_rounds << "  " << stats;
    }
  }
  if (!config.quiet) std::cout << "\033[K\r";
  return stats;
}

void play_all_agents(Config& config, std::ostream& out = std::cout) {
  using namespace std;
  out << "agent, mean, stddev, min, q.25, median, q.75, max, lost" << endl;
  for (auto const& agent : agents) {
    out << left << setw(15) << agent.name << ", " << flush;
    auto stats = play_multiple(agent.make, config);
    out << right << fixed << setprecision(1) << setw(10);
    out << setw(8) << mean(stats.turns) << ", ";
    out << setw(8) << stddev(stats.turns) << ", ";
    out << setprecision(0);
    for (auto q : quantiles(stats.turns)) {
      out << setw(8) << q << ", ";
    }
    out << setprecision(1);
    out << setw(8) << ((1-mean(stats.wins))*100) << "%" << endl;
  }
}

//------------------------------------------------------------------------------
// Optimization
// This is a hacky algorihtm to optimize algorithm parameters
//------------------------------------------------------------------------------

struct ParameterizedAgentFactory {
  std::vector<int> min_param_value;
  std::vector<int> max_param_value;
  ParameterizedAgentFactory(size_t num_params, int min_value, int max_value)
    : min_param_value(num_params, min_value)
    , max_param_value(num_params, max_value)
  {}
  size_t num_params() const { return min_param_value.size(); }
  virtual std::unique_ptr<Agent> make(std::vector<int> params, Config&) const = 0;
};

struct ParameterizedCellTreeAgent : ParameterizedAgentFactory {
  ParameterizedCellTreeAgent() : ParameterizedAgentFactory(9,0,5000) {}
  
  std::unique_ptr<Agent> make(std::vector<int> params, Config&) const override {
    auto agent = std::make_unique<CellTreeAgent>();
    auto param = params.begin();
    agent->same_cell_penalty = *param++;
    agent->new_cell_penalty = *param++;
    agent->parent_cell_penalty = *param++;
    agent->edge_penalty_in = *param++;
    agent->wall_penalty_in = *param++;
    agent->open_penalty_in = *param++;
    agent->edge_penalty_out = *param++;
    agent->wall_penalty_out = *param++;
    agent->open_penalty_out = *param++;
    assert(param == params.end());
    return agent;
  }
};

double score(Stats const& stats) {
  return mean(stats.turns) + 1e10 * (1 - mean(stats.wins));
}

void optimize_agent(ParameterizedAgentFactory& agent, Config& config, std::ostream& out) {
  using namespace std;
  int num_runs = 1000;
  int step_size = 100;
  double best_score = 1e100;
  std::vector<int> best_params = agent.min_param_value;
  for (int i = 0; i < num_runs; ++i) {
    std::vector<int> params = best_params;
    size_t which = i % (agent.num_params() + 1);
    if (which == 0) {
      // re-run
    } else {
      size_t j = which - 1;
      do {
        int delta = config.rng.random(step_size*2+1) - step_size;
        params[j] = std::max(agent.min_param_value[j], std::min(agent.max_param_value[j], params[j] + delta));
      } while (params[j] == best_params[j]);
    }
    auto stats = play_multiple([&agent,&params](Config& config){return agent.make(params, config);}, config);
    auto score = ::score(stats);
    std::cout << (which == 0 ? yellow : score < best_score ? green : white)(std::to_string(score));
    std::cout << ":  " << params;
    std::cout << std::endl;
    if (which == 0 || score < best_score) {
      best_score = score;
      best_params = params;
    }
  }
}

int main(int argc, const char** argv) {
  // Start timing for entire program
  auto program_start_time = std::chrono::high_resolution_clock::now();
  
  std::string mode = argc >= 2 ? argv[1] : "help";
  
  try {
    // Reset timers for shortest path, A* and unreachables
    ShortestPathTimer::reset();
    AStarTimer::reset();
    UnreachableTimer::reset();
    
    if (mode == "help" || mode == "--help" || mode == "-h") {
      print_help(argv[0]);
    } else if (mode == "list") {
      list_agents();
    } else if (mode == "all") {
      Config config;
      config.quiet = true;
      config.parse_optional_args(argc-2, argv+2);
      play_all_agents(config);
    } else if (mode == "optimize-cell") {
      Config config;
      config.parse_optional_args(argc-2, argv+2);
      ParameterizedCellTreeAgent agent;
      optimize_agent(agent, config, std::cout);
    } else {
      auto agent = find_agent(mode);
      Config config;
      config.parse_optional_args(argc-2, argv+2);
      if (!config.json_file.empty() || !config.flood_fill_json.empty()) {
        bool is_cheat_agent = (agent.name == "cheat");
        LoggedGame game(config.board_size, config.rng.next_rng(), is_cheat_agent);
      }
      if (!config.json_file.empty()) {
        // Advance RNG to the specified round if needed (same logic as in play_multiple)
        if (config.round_to_run > 0) {
          for (int i = 1; i < config.round_to_run; ++i) {
            config.rng.next_rng(); // Skip RNG states but don't use them
          }
        }
        LoggedGame game(config.board_size, config.rng.next_rng());
        AgentLog agent_log;
        auto a = agent.make(config);
        try {
        play(game, *a, config, &agent_log);
        } catch (const std::exception& e) {
          std::cerr << "Exception: " << e.what() << std::endl;
        }
        if (!config.json_file.empty()) {
          write_json(config.json_file, agent, game, agent_log, config.json_compact);
        }
      } else {
        // Check if we're using the CheatAgent
        bool is_cheat_agent = (agent.name == "cheat");
        
        auto stats = play_multiple(agent.make, config, is_cheat_agent);
        std::cout << stats << std::endl;
      }    
    // Print timing stats
    ShortestPathTimer::print_stats();
    AStarTimer::print_stats();
    UnreachableTimer::print_stats();
  }
}
  catch (std::exception const& e) {
  std::cerr << e.what() << std::endl;
  return EXIT_FAILURE;
  }

  // Calculate and print total program time
  auto program_end_time = std::chrono::high_resolution_clock::now();
  auto program_duration = std::chrono::duration<double>(program_end_time - program_start_time);
  std::cout << "\nTotal program execution time: " << program_duration.count() << " seconds" << std::endl;
  
  return EXIT_SUCCESS;
}

