#pragma once

#include <cstddef>
#include <deque>
#include <vector>

#include "slam/loop_types.hpp"

namespace kiss_matcher
{
enum class LoopCandidateSource
{
    kLoopDetector,
    kNNSearch,
};

struct QueuedLoopCandidate
{
    LoopIdxPair indices_;
    LoopCandidateSource source_;
};

class LoopCandidateQueue
{
public:
    void setCapacity(const size_t capacity)
    {
        capacity_ = capacity;
    }

    bool enqueue(const LoopIdxPairs &loop_idx_pairs, const LoopCandidateSource source)
    {
        if (candidates_.size() > capacity_ ||
            loop_idx_pairs.size() > capacity_ - candidates_.size())
        {
            return false;
        }

        for (const auto &loop_idx_pair : loop_idx_pairs)
        {
            candidates_.push_back({loop_idx_pair, source});
        }
        return true;
    }

    bool tryPop(QueuedLoopCandidate &candidate)
    {
        if (candidates_.empty())
        {
            return false;
        }

        candidate = candidates_.front();
        candidates_.pop_front();
        return true;
    }

    std::vector<QueuedLoopCandidate> takeAll()
    {
        std::vector<QueuedLoopCandidate> queued_candidates(candidates_.begin(), candidates_.end());
        candidates_.clear();
        return queued_candidates;
    }

    size_t size() const
    {
        return candidates_.size();
    }

    size_t capacity() const
    {
        return capacity_;
    }

private:
    size_t capacity_ = 0;
    std::deque<QueuedLoopCandidate> candidates_;
};
}  // namespace kiss_matcher
