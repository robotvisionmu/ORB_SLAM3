#ifndef KEYFRAME_EVENT_QUEUE_H
#define KEYFRAME_EVENT_QUEUE_H

#include <array>
#include <cstdint>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <sophus/se3.hpp>

namespace ORB_SLAM3
{

class KeyFrame;

struct KeyFrameEvent
{
    uint64_t keyFrameEventId = 0;
    std::string keyFrameEventType;
    unsigned long orbKeyFrameId = 0;
    double sourceTimestamp = 0.0;
    int orbDatasetId = -1;
    std::string appSessionId;
    std::string sourceFrameRef;
    int oldOrbMapId = -1;
    int newOrbMapId = -1;
    std::array<float, 16> newPose;
    bool keyFramePoseChanged = false;
    bool keyFrameMapChanged = false;
    std::string reason;
};

class KeyFrameEventQueue
{
public:
    static KeyFrameEventQueue& Instance();

    void EnqueueCreated(KeyFrame* pKF, const std::string& reason = "tracking");
    void EnqueueRemoved(KeyFrame* pKF, const std::string& reason = "culling");
    void EnqueueStateChanged(KeyFrame* pKF,
                             int oldOrbMapId,
                             int newOrbMapId,
                             const Sophus::SE3f& newTwc,
                             bool keyFramePoseChanged,
                             bool keyFrameMapChanged,
                             const std::string& reason);

    std::vector<KeyFrameEvent> Drain();
    uint64_t LatestKeyFrameEventId() const;

private:
    KeyFrameEventQueue();

    KeyFrameEvent BuildEvent(KeyFrame* pKF,
                             const std::string& keyFrameEventType,
                             int oldOrbMapId,
                             int newOrbMapId,
                             const Sophus::SE3f& newTwc,
                             bool keyFramePoseChanged,
                             bool keyFrameMapChanged,
                             const std::string& reason);
    void QueueOrCoalesceEvent(const KeyFrameEvent& event);
    static std::array<float, 16> PoseToArray(const Sophus::SE3f& pose);

    mutable std::mutex mMutex;
    uint64_t mnNextKeyFrameEventId;
    uint64_t mnLatestKeyFrameEventId;
    std::vector<KeyFrameEvent> mvEvents;
    std::unordered_map<unsigned long, size_t> mmPendingByKeyFrame;
};

} // namespace ORB_SLAM3

#endif // KEYFRAME_EVENT_QUEUE_H
