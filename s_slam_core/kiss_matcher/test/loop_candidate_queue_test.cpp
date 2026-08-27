#include <gtest/gtest.h>

#include "slam/loop_candidate_queue.hpp"

namespace kiss_matcher
{
TEST(LoopCandidateQueue, RejectsAnEntireBatchWhenCapacityIsInsufficient)
{
    LoopCandidateQueue queue;
    queue.setCapacity(3);

    const LoopIdxPairs first_batch{{1, 0}, {2, 0}};
    EXPECT_TRUE(queue.enqueue(first_batch, LoopCandidateSource::kNNSearch));
    EXPECT_EQ(queue.size(), 2U);

    const LoopIdxPairs oversized_batch{{3, 0}, {4, 0}};
    EXPECT_FALSE(queue.enqueue(oversized_batch, LoopCandidateSource::kLoopDetector));
    EXPECT_EQ(queue.size(), 2U);

    QueuedLoopCandidate candidate;
    ASSERT_TRUE(queue.tryPop(candidate));
    EXPECT_EQ(candidate.indices_, first_batch.front());
    EXPECT_EQ(candidate.source_, LoopCandidateSource::kNNSearch);
    ASSERT_TRUE(queue.tryPop(candidate));
    EXPECT_EQ(candidate.indices_, first_batch.back());
    EXPECT_EQ(candidate.source_, LoopCandidateSource::kNNSearch);
    EXPECT_FALSE(queue.tryPop(candidate));
}

TEST(LoopCandidateQueue, DrainsQueuedCandidatesInOrder)
{
    LoopCandidateQueue queue;
    queue.setCapacity(3);

    ASSERT_TRUE(queue.enqueue({{1, 0}}, LoopCandidateSource::kLoopDetector));
    ASSERT_TRUE(queue.enqueue({{2, 0}}, LoopCandidateSource::kNNSearch));

    const auto candidates = queue.takeAll();
    ASSERT_EQ(candidates.size(), 2U);
    EXPECT_EQ(candidates[0].indices_, LoopIdxPair(1, 0));
    EXPECT_EQ(candidates[0].source_, LoopCandidateSource::kLoopDetector);
    EXPECT_EQ(candidates[1].indices_, LoopIdxPair(2, 0));
    EXPECT_EQ(candidates[1].source_, LoopCandidateSource::kNNSearch);
    EXPECT_EQ(queue.size(), 0U);
}
}  // namespace kiss_matcher
