#pragma once

#include <mutex>

namespace tbb {

class mutex : public std::mutex {
 public:
  using scoped_lock = std::unique_lock<mutex>;
};

}  // namespace tbb
