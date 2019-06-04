/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer/sensor/internal/collator.h"

namespace cartographer {
namespace sensor {

/**
 * 这个函数关键是传入Callback ---> 即当新的传感器数据到来时的回调函数
 * @param trajectory_id          对应的trajectory id
 * @param expected_sensor_ids    期望的传感器数据
 * @param callback               ----> 这里是 CollatedTrajectoryBuilder::HandleCollatedSensorData(...)函数
 */
void Collator::AddTrajectory(
    const int trajectory_id,
    const absl::flat_hash_set<std::string>& expected_sensor_ids,
    const Callback& callback) {

    /// 这里把回调函数传入实际存储的消息队列中
  for (const auto& sensor_id : expected_sensor_ids) {
    const auto queue_key = QueueKey{trajectory_id, sensor_id};
    queue_.AddQueue(queue_key,
                    [callback, sensor_id](std::unique_ptr<Data> data) {
                      callback(sensor_id, std::move(data));
                    });
    queue_keys_[trajectory_id].push_back(queue_key);
  }
}

void Collator::FinishTrajectory(const int trajectory_id) {
  for (const auto& queue_key : queue_keys_[trajectory_id]) {
    queue_.MarkQueueAsFinished(queue_key);
  }
}
/**
 *  添加传感器数据
 * @param trajectory_id
 * @param data
 */
void Collator::AddSensorData(const int trajectory_id,
                             std::unique_ptr<Data> data) {
  QueueKey queue_key{trajectory_id, data->GetSensorId()};
  queue_.Add(std::move(queue_key), std::move(data));
}
/** 清除缓存 **/
void Collator::Flush() { queue_.Flush(); }

absl::optional<int> Collator::GetBlockingTrajectoryId() const {
  return absl::optional<int>(queue_.GetBlocker().trajectory_id);
}

}  // namespace sensor
}  // namespace cartographer
