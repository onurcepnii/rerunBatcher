# rerunBatcher

Header-only, buffered telemetry logger for ROS 2 nodes. Solves the throughput bottleneck of the standard [Rerun](https://rerun.io) `log()` API at high message frequencies (500 Hz+) by batching messages and dispatching them in a single `send_columns` call.

---

## Requirements

- ROS 2 Humble / Iron / Jazzy
- Rerun C++ SDK 0.29.1+
- C++17

---

## Setup

Drop `rerunBatcher.hpp` into your package's `include/` directory, then add the Rerun SDK to your `CMakeLists.txt`:

```cmake
include(FetchContent)
FetchContent_Declare(
  rerun_sdk
  URL https://github.com/rerun-io/rerun/releases/download/0.29.1/rerun_cpp_sdk.zip
  DOWNLOAD_EXTRACT_TIMESTAMP ON
)
FetchContent_MakeAvailable(rerun_sdk)

target_link_libraries(your_node rerun_sdk)
```

---

## Quick Start

### `AutoLogger` — one logger per node

```cpp
#include "rerunBatcher.hpp"
#include "sensor_msgs/msg/imu.hpp"

using ImuMsg = sensor_msgs::msg::Imu;

rerunBatcher::AutoLogger<ImuMsg> logger("my_imu_logger", 250);

logger.registerScalar("imu/accel_x",
    [](const ImuMsg& msg) {
        return rerun::components::Scalar(msg.linear_acceleration.x);
    });

logger.registerPosition3D("imu/orientation",
    [](const ImuMsg& msg) {
        return rerun::components::Position3D(
            msg.orientation.x, msg.orientation.y, msg.orientation.z);
    });

// In your subscription callback:
void imuCallback(const std::shared_ptr<ImuMsg> msg) {
    logger.log(msg);                                                        // system clock
    // logger.log(msg, rclcpp::Time(msg->header.stamp).nanoseconds());      // ROS stamp
}
```

### `BatcherEngine` — shared stream across multiple topics

```cpp
rerun::RecordingStream rec("multi_topic_node");
rec.spawn().exit_on_failure();

rerunBatcher::BatcherEngine<InsMsg> ins_engine(rec, 500);
ins_engine.registerScalar("ins/heading",
    [](const InsMsg& msg) { return rerun::components::Scalar(msg.heading_deg); });

rerunBatcher::BatcherEngine<GpsMsg> gps_engine(rec, 100);
gps_engine.registerPosition3D("gps/fix",
    [](const GpsMsg& msg) {
        return rerun::components::Position3D(msg.latitude, msg.longitude, msg.altitude);
    });
```

### Hybrid logging

Infrequent events can bypass the buffer via `AutoLogger::stream()`:

```cpp
// Infrequent — sent directly
logger.stream().log("events/arm_state",
    rerun::TextLog("ARMED").with_level(rerun::TextLogLevel::Info));

// High-frequency — buffered
void imuCallback(const std::shared_ptr<ImuMsg> msg) { logger.log(msg); }
```

---

## API

### `AutoLogger<MsgT>`

| Method | Description |
|---|---|
| `AutoLogger(app_id, capacity=250)` | Spawns the Rerun viewer. |
| `registerScalar(name, fn)` | Registers a scalar extractor. |
| `registerPosition3D(name, fn)` | Registers a 3D position extractor. |
| `registerVector3D(name, fn)` | Registers a 3D vector extractor. |
| `log(msg)` | Buffers with system clock; auto-flushes at capacity. |
| `log(msg, ns)` | Buffers with explicit nanosecond timestamp. |
| `flush()` | Force-flush immediately. |
| `stream()` | Returns a reference to the underlying `RecordingStream`. |

### `BatcherEngine<MsgT>`

Same field registration methods as `AutoLogger`, plus:

| Method | Description |
|---|---|
| `BatcherEngine(rec, capacity=250)` | Constructs the engine against an existing stream. |
| `autoLog(msg)` / `autoLog(msg, ns)` | Buffer + auto-flush. |
| `registerCustom(handler)` | Registers a `FieldHandlerBase<MsgT>` subclass. |
| `setCapacity(n)` | Resize at runtime; flushes if current size ≥ n. |
| `getCapacity()` | Returns current capacity. |

### Custom component types

Subclass `FieldHandlerBase<MsgT>` to support any Rerun component not covered by the three built-in handlers:

```cpp
template <typename MsgT>
struct QuaternionHandler : public rerunBatcher::FieldHandlerBase<MsgT> {
    using Fn = std::function<rerun::components::RotationQuat(const MsgT&)>;

    QuaternionHandler(std::string name, Fn fn, int cap)
        : rerunBatcher::FieldHandlerBase<MsgT>(std::move(name)), extractor(std::move(fn))
    { data.reserve(cap); }

    void addData(const MsgT& msg) override { data.emplace_back(extractor(msg)); }
    void clear()                  override { data.clear(); }
    size_t size()           const override { return data.size(); }
    void sendColumn(rerun::RecordingStream& rec, const rerun::TimeColumn& tc) override {
        rec.send_columns(this->name, {tc}, rerun::Transform3D(data).columns());
    }

private:
    Fn extractor;
    std::vector<rerun::components::RotationQuat> data;
};

engine.registerCustom(std::make_unique<QuaternionHandler<ImuMsg>>(...));
```

---

## Notes

- **Buffer tuning:** 250 samples at 500 Hz → ~2 flushes/second. Increase for lower flush overhead; decrease for lower viewer latency.
- **Timestamp:** Prefer the ROS header stamp over the system clock for proper alignment with other topics.
- **Thread safety:** `BatcherEngine` is not thread-safe. All calls to `autoLog()` and `flush()` must come from the same thread.
- **Shutdown:** The `BatcherEngine` destructor flushes, so partial buffers at node shutdown are never lost.

---

## License

MIT — see [LICENSE](LICENSE).  
Built on the [Rerun C++ SDK](https://github.com/rerun-io/rerun).
