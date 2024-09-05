#ifndef PLANNER_WRAPPER_H
#define PLANNER_WRAPPER_H

#include "ActionModel.h"
#include "MAPFPlanner.h"
#include "SharedEnv.h"
#include "timer.h"
#include <chrono>
#include <cstddef>
#include <future>
#include <vector>

namespace planner {

struct planner_metrics_t {
  size_t num_queries_;
  std::chrono::duration<double> planning_time_;

  planner_metrics_t()
      : num_queries_(0), planning_time_(std::chrono::seconds(0)) {}
};

struct planner_config {
  std::chrono::duration<double> preprocess_time;
  std::chrono::duration<double> planning_period;
};

template <class P> class wrapper {
public:
  wrapper(P *planner, Logger *logger,
          std::chrono::duration<double> preprocess_time,
          std::chrono::duration<double> planning_period)
      : planner_(planner), metrics_(), timer_(), logger_(logger),
        preprocess_time_(preprocess_time), planning_period_(planning_period) {
    auto planner_query_fn = [=](const std::vector<State> &a,
                                const std::vector<std::deque<tasks::Task>> &b,
                                double c) {
      // std::cout << "FUTURE" << planner << std::flush;
      // return planner->query(a, b, c);
      return std::vector<Action>(a.size(), Action::W);
    };
    query_ = std::packaged_task<std::vector<Action>(
        const std::vector<State> &,
        const std::vector<std::deque<tasks::Task>> &, double)>(
        planner_query_fn);
    // query_(std::bind(&P::query, planner, std::placeholders::_1,
    // std::placeholders::_2, std::placeholders::_3)); query_ =
    // std::packaged_task<std::vector<Action>(P::*(const std::vector<int>&,
    // const std::vector<int>&, double))>( std::bind(&P::query, &planner_,
    // std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    // result_ = query_.get_future();
  }

  std::vector<Action> query(const std::vector<State> &starts,
                           const std::vector<std::deque<tasks::Task>> &goals) {
    return query_internal(starts, goals, planning_period_);
  }
  // Returns the metrics of the planner
  const planner_metrics_t &get_metrics() const { return metrics_; }

  bool planner_preprocess(SharedEnvironment *env) {
    auto preprocess = [=]() {
      planner_->initialize(env, preprocess_time_);
    };
    std::packaged_task<void()> preprocess_task(preprocess);
    auto preprocess_fut = preprocess_task.get_future();
    auto thread = std::thread(std::move(preprocess_task));
    if (preprocess_fut.wait_for(preprocess_time_) == std::future_status::ready) {
      thread.join();
      return true;
    }
    thread.detach();
    return false;
  }

private:
  P *const planner_;
  planner_metrics_t metrics_;
  Timer timer_;
  Logger *const logger_;

  // std::packaged_task<std::vector<Action>(P::*(const std::vector<int>&,
  std::chrono::duration<double> preprocess_time_;
  std::chrono::duration<double> planning_period_;
  // const std::vector<int>&, double))>
  std::packaged_task<std::vector<Action>(
      const std::vector<State> &, const std::vector<std::deque<tasks::Task>> &,
      double)>
      query_;
  std::future<std::vector<Action>> result_;
  std::thread thread_;

  // Calls the planner's query method and updates the metrics
  // @param start The start states of the agents
  // @param goal The goal states of the agents
  // @return The next action for each agent
  std::vector<Action> query_internal(
      const std::vector<State> &starts,
      const std::vector<std::deque<tasks::Task>> &goals,
      std::chrono::duration<double> time_limit = std::chrono::seconds(1)) {

    metrics_.num_queries_++;
    if (result_.valid() && result_.wait_for(std::chrono::seconds(0)) !=
                               std::future_status::ready) {
      // Check valid and poll if finished, then wait.
      if (logger_) {
        logger_->log_info("Planner still running from previous cycle.");
      }
      if (result_.wait_for(time_limit) == std::future_status::ready) {
        thread_.join();
        return result_.get();
      }
      // Not ready, return empty
      return {};
    }

    // Confirm thread is finished.
    if (thread_.joinable()) {
      thread_.join();
    }
    auto planner_query_fn = [=](const std::vector<State> &states,
                                const std::vector<std::deque<tasks::Task>> &goals) -> std::vector<Action>
      {
      std::cout << "FUTURE" << planner_ << std::flush;
      return planner_->query(states, goals, planning_period_);
    };
    auto query = std::packaged_task<std::vector<Action>(
        const std::vector<State> &,
        const std::vector<std::deque<tasks::Task>> &)>(
        planner_query_fn);
    result_ = query.get_future();
    thread_ = std::thread(std::move(query), std::cref(starts), std::cref(goals));
    timer_.start();
    std::vector<Action> actions{};
    if (result_.wait_for(std::chrono::duration<double>(time_limit)) ==
        std::future_status::ready) 
    {
      std::cout << "PASSED" << std::flush;
      thread_.join();
      metrics_.planning_time_ +=
          std::chrono::nanoseconds(timer_.elapsed_time<std::chrono::nanoseconds>());
      return result_.get();
    } else {
      logger_->log_info("planner timeout");
    }

    metrics_.planning_time_ +=
        std::chrono::nanoseconds(timer_.elapsed_time<std::chrono::nanoseconds>());
    return actions;
  }
};

typedef wrapper<MAPFPlanner> MAPFPlannerWrapper;

} // namespace planner
#endif // PLANNER_WRAPPER_H
