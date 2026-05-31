#include "KeyFrameEventQueue.h"

#include "KeyFrame.h"
#include "Map.h"

#include <Eigen/Dense>

#include <algorithm>

namespace ORB_SLAM3
{

KeyFrameEventQueue& KeyFrameEventQueue::Instance()
{
    static KeyFrameEventQueue queue;
    return queue;
}

KeyFrameEventQueue::KeyFrameEventQueue() : mnNextKeyFrameEventId(1), mnLatestKeyFrameEventId(0)
{
}

std::array<float, 16> KeyFrameEventQueue::PoseToArray(const Sophus::SE3f& pose)
{
    std::array<float, 16> values;
    Eigen::Matrix4f matrix = pose.matrix();
    Eigen::Map<Eigen::Matrix<float, 4, 4, Eigen::RowMajor>>(values.data()) = matrix;
    return values;
}

KeyFrameEvent KeyFrameEventQueue::BuildEvent(KeyFrame* pKF,
                                             const std::string& keyFrameEventType,
                                             int oldOrbMapId,
                                             int newOrbMapId,
                                             const Sophus::SE3f& newTwc,
                                             bool keyFramePoseChanged,
                                             bool keyFrameMapChanged,
                                             const std::string& reason)
{
    KeyFrameEvent event;
    event.keyFrameEventType = keyFrameEventType;
    event.orbKeyFrameId = pKF->mnId;
    event.sourceTimestamp = pKF->mTimeStamp;
    event.orbDatasetId = pKF->mnDataset;
    event.appSessionId.clear();
    event.sourceFrameRef = pKF->mNameFile;
    event.oldOrbMapId = oldOrbMapId;
    event.newOrbMapId = newOrbMapId;
    event.newPose = PoseToArray(newTwc);
    event.keyFramePoseChanged = keyFramePoseChanged;
    event.keyFrameMapChanged = keyFrameMapChanged;
    event.reason = reason;
    return event;
}

void KeyFrameEventQueue::QueueOrCoalesceEvent(const KeyFrameEvent& event)
{
    auto it = mmPendingByKeyFrame.find(event.orbKeyFrameId);
    if (it == mmPendingByKeyFrame.end())
    {
        KeyFrameEvent queued = event;
        queued.keyFrameEventId = mnNextKeyFrameEventId++;
        mnLatestKeyFrameEventId = queued.keyFrameEventId;
        mvEvents.push_back(queued);
        mmPendingByKeyFrame[event.orbKeyFrameId] = mvEvents.size() - 1;
        return;
    }

    KeyFrameEvent& queued = mvEvents[it->second];

    if (event.keyFrameEventType == "removed")
    {
        queued.keyFrameEventType = "removed";
        queued.keyFrameEventId = mnNextKeyFrameEventId++;
        mnLatestKeyFrameEventId = queued.keyFrameEventId;
        queued.oldOrbMapId = event.oldOrbMapId;
        queued.newOrbMapId = -1;
        queued.newPose = event.newPose;
        queued.keyFramePoseChanged = false;
        queued.keyFrameMapChanged = event.oldOrbMapId != -1;
        queued.reason = event.reason;
        return;
    }

    if (queued.keyFrameEventType == "created")
    {
        queued.keyFrameEventId = mnNextKeyFrameEventId++;
        mnLatestKeyFrameEventId = queued.keyFrameEventId;
        queued.newOrbMapId = event.newOrbMapId;
        queued.newPose = event.newPose;
        queued.sourceFrameRef = event.sourceFrameRef;
        queued.orbDatasetId = event.orbDatasetId;
        queued.reason = queued.reason.empty() ? event.reason : queued.reason;
        return;
    }

    if (event.keyFrameEventType == "created")
    {
        queued = event;
        queued.keyFrameEventId = mnNextKeyFrameEventId++;
        mnLatestKeyFrameEventId = queued.keyFrameEventId;
        return;
    }

    queued.keyFrameEventId = mnNextKeyFrameEventId++;
    mnLatestKeyFrameEventId = queued.keyFrameEventId;
    queued.newOrbMapId = event.newOrbMapId;
    queued.newPose = event.newPose;
    queued.keyFramePoseChanged = queued.keyFramePoseChanged || event.keyFramePoseChanged;
    queued.keyFrameMapChanged = queued.keyFrameMapChanged || event.keyFrameMapChanged;
    if (!event.reason.empty())
    {
        queued.reason = event.reason;
    }
}

void KeyFrameEventQueue::EnqueueCreated(KeyFrame* pKF, const std::string& reason)
{
    if (!pKF)
    {
        return;
    }

    Map* pMap = pKF->GetMap();
    const int mapId = pMap ? static_cast<int>(pMap->GetId()) : -1;
    const Sophus::SE3f pose = pKF->GetPoseInverse();
    KeyFrameEvent event = BuildEvent(pKF, "created", -1, mapId, pose, false, false, reason);

    std::lock_guard<std::mutex> lock(mMutex);
    QueueOrCoalesceEvent(event);
}

void KeyFrameEventQueue::EnqueueRemoved(KeyFrame* pKF, const std::string& reason)
{
    if (!pKF)
    {
        return;
    }

    Map* pMap = pKF->GetMap();
    const int mapId = pMap ? static_cast<int>(pMap->GetId()) : -1;
    const Sophus::SE3f pose = pKF->GetPoseInverse();
    KeyFrameEvent event = BuildEvent(pKF, "removed", mapId, -1, pose, false, true, reason);

    std::lock_guard<std::mutex> lock(mMutex);
    QueueOrCoalesceEvent(event);
}

void KeyFrameEventQueue::EnqueueStateChanged(KeyFrame* pKF,
                                             int oldOrbMapId,
                                             int newOrbMapId,
                                             const Sophus::SE3f& newTwc,
                                             bool keyFramePoseChanged,
                                             bool keyFrameMapChanged,
                                             const std::string& reason)
{
    if (!pKF || (!keyFramePoseChanged && !keyFrameMapChanged))
    {
        return;
    }

    KeyFrameEvent event = BuildEvent(pKF, "state_changed", oldOrbMapId, newOrbMapId, newTwc, keyFramePoseChanged, keyFrameMapChanged, reason);

    std::lock_guard<std::mutex> lock(mMutex);
    QueueOrCoalesceEvent(event);
}

std::vector<KeyFrameEvent> KeyFrameEventQueue::Drain()
{
    std::lock_guard<std::mutex> lock(mMutex);
    std::vector<KeyFrameEvent> events = std::move(mvEvents);
    std::sort(events.begin(), events.end(), [](const KeyFrameEvent& lhs, const KeyFrameEvent& rhs) {
        return lhs.keyFrameEventId < rhs.keyFrameEventId;
    });
    mvEvents.clear();
    mmPendingByKeyFrame.clear();
    return events;
}

uint64_t KeyFrameEventQueue::LatestKeyFrameEventId() const
{
    std::lock_guard<std::mutex> lock(mMutex);
    return mnLatestKeyFrameEventId;
}

} // namespace ORB_SLAM3
