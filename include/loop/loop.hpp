// Copyright (c) 2025 Shenzhen Yuejiang Technology Co., Ltd.
// SPDX-License-Identifier: Apache-2.0

#ifndef LOOP_H
#define LOOP_H

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <thread>
#include <vector>

#include <boost/asio.hpp>
#include <boost/system/error_code.hpp>

#ifdef __linux__
#include <pthread.h>
#include <sched.h>
#endif

namespace asio = boost::asio;

// Periodic loop utility based on boost::asio.
// Runs a user-provided callback at a fixed period on a dedicated thread.
class LoopFunc final {
 public:
    using Duration = asio::steady_timer::duration;

  LoopFunc(const std::string& name, float period_sec, std::function<void()> func,
           int bindCPU = -1)
      : name_(name),
        period_(std::chrono::duration_cast<Duration>(
            std::chrono::duration<double>(period_sec))),
        func_(std::move(func)),
        bindCPU_(bindCPU),
        running_(false),
        io_(),
        work_guard_(asio::make_work_guard(io_)),
        timer_(io_) {}

    LoopFunc(const LoopFunc&) = delete;
    LoopFunc& operator=(const LoopFunc&) = delete;
    LoopFunc(LoopFunc&&) = delete;
  LoopFunc& operator=(LoopFunc&&) = delete;

  // Starts the loop thread if not already running.
  void start() noexcept {
        bool expected = false;
    if (!running_.compare_exchange_strong(expected, true)) {
            return;
        }
    std::cout << "Loop start - name: " + name_ +
                     ", period: " + formatPeriodMs() +
                     " ms, cpu: " +
                     (bindCPU_ != -1 ? std::to_string(bindCPU_) : "unspecified")
              << std::endl;
        schedule_once();

    thread_ = std::thread([this]() { io_.run(); });

#ifdef __linux__
    if (bindCPU_ != -1) {
            try {
        setThreadAffinity(thread_.native_handle(), bindCPU_);
      } catch (const std::exception& e) {
        std::cout << "setThreadAffinity failed for " + name_ + ": " << e.what()
                  << std::endl;
      }
        }
#endif
    }

  // Stops the loop thread if it is running.
  void shutdown() noexcept {
        bool expected = true;
    if (!running_.compare_exchange_strong(expected, false)) {
            return;
        }

    io_.post([this] {
      boost::system::error_code ec;
      timer_.cancel(ec);
    });

    work_guard_.reset();
    io_.stop();

    if (thread_.joinable()) {
      thread_.join();
    }

    std::cout << "Loop end - name: " + name_ << std::endl;
    }

  ~LoopFunc() noexcept { shutdown(); }

 private:
  std::string name_;
  Duration period_;
  std::function<void()> func_;
  int bindCPU_;
  std::atomic_bool running_{false};
  asio::io_context io_;
  asio::executor_work_guard<asio::io_context::executor_type> work_guard_;
  asio::steady_timer timer_;
  std::thread thread_;

  void schedule_once() {
    timer_.expires_after(period_);
    timer_.async_wait([this](const boost::system::error_code& ec) {
      if (ec || !running_.load(std::memory_order_relaxed)) {
        return;
      }

      func_();
      schedule_once();
    });
  }

  std::string formatPeriodMs() const {
    const auto ms =
        std::chrono::duration<double, std::milli>(period_).count();
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(1) << ms;
    return stream.str();
  }

  void setThreadAffinity(std::thread::native_handle_type threadHandle,
                         int cpuId) {
#ifdef __linux__
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(cpuId, &cpuset);
    if (pthread_setaffinity_np(threadHandle, sizeof(cpu_set_t), &cpuset) != 0) {
      std::ostringstream oss;
      oss << "Error setting thread affinity: CPU " << cpuId
          << " may not be valid or accessible.";
      throw std::runtime_error(oss.str());
    }
#else
    (void)threadHandle;
    (void)cpuId;
    std::cout << "Thread affinity not supported on this platform" << std::endl;
#endif
  }
};

#endif  // LOOP_H
