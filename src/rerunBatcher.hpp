#pragma once
#include <chrono>
#include <memory>
#include <vector>
#include <functional>
#include "rerun.hpp"

/**
 * @file rerunBatcher.hpp
 * @brief Buffered, type-heterogeneous Rerun logger for high-frequency ROS 2 messages.
 *
 * ## Motivation
 * The standard Rerun `log()` API performs one network round-trip per call.
 * At 500 Hz+, this saturates the Rerun server within minutes, causing rate limiting
 * and dropped samples. This library solves the problem by batching messages into
 * typed `std::vector` buffers and dispatching them in a single `send_columns` call
 * once the buffer reaches its capacity threshold.
 *
 * ## Architecture
 * ```
 * AutoLogger<MsgT>
 *   └─ BatcherEngine<MsgT>          (owns the buffer loop)
 *        └─ FieldHandlerBase<MsgT>[]    (type-erased per-field handlers)
 *             ├─ ScalarFieldHandler
 *             ├─ Position3DFieldHandler
 *             ├─ Vector3DFieldHandler
 *             └─ (user-defined subclasses)
 * ```
 *
 * ## Typical usage
 * @code
 * rerunBatcher::AutoLogger<sensor_msgs::msg::Imu> logger("my_node", 250);
 * logger.registerScalar("imu/accel_x",
 *     [](const auto& m) { return rerun::components::Scalar(m.linear_acceleration.x); });
 * // in callback:
 * logger.log(msg);
 * @endcode
 *
 * @author  Onur Çepni
 * @date    2026
 * @license MIT
 */

namespace rerunBatcher {

/** @brief Default number of messages to buffer before flushing to Rerun. */
static constexpr size_t DEFAULT_BUFFER = 250;

// ============================================================
//  Abstract base
// ============================================================

/**
 * @brief Abstract base class for type-erased per-field data handlers.
 *
 * Each concrete subclass knows how to:
 *   1. Extract a specific Rerun component (Scalar, Position3D, …) from a ROS message.
 *   2. Accumulate extracted values into an internal `std::vector`.
 *   3. Flush the vector to Rerun as a time-column batch via `send_columns`.
 *
 * Users who need component types not covered by the built-in handlers
 * (ScalarFieldHandler, Position3DFieldHandler, Vector3DFieldHandler) should subclass
 * this class and register the result with `BatcherEngine::registerCustom()`.
 *
 * @tparam MsgT The ROS 2 message type whose fields are being extracted.
 */
template <typename MsgT>
struct FieldHandlerBase {
    std::string name; ///< Rerun entity path for this field (e.g., `"imu/accel_x"`).

    explicit FieldHandlerBase(std::string n) : name(std::move(n)) {}
    virtual ~FieldHandlerBase() = default;

    /**
     * @brief Extracts the relevant component from @p msg and appends it to the buffer.
     * @param msg The incoming ROS 2 message.
     */
    virtual void addData(const MsgT& msg) = 0;

    /**
     * @brief Sends the entire buffer to Rerun as a single `send_columns` batch.
     * @param rec       Active Rerun recording stream.
     * @param timeCol  Pre-built time column aligned with the current buffer contents.
     */
    virtual void sendColumn(rerun::RecordingStream& rec,
                            const rerun::TimeColumn& timeCol) = 0;

    /** @brief Clears the internal data buffer (called after every flush). */
    virtual void clear() = 0;

    /** @brief Returns the number of elements currently in the buffer. */
    virtual size_t size() const = 0;
};

// ============================================================
//  Concrete handlers
// ============================================================

/**
 * @brief Handler for scalar time-series data.
 *
 * Suitable for any single `double` measurement: RPM, battery voltage,
 * PID outputs, heading error, etc.
 *
 * @tparam MsgT The ROS 2 message type.
 */
template <typename MsgT>
struct ScalarFieldHandler : public FieldHandlerBase<MsgT> {
    /// Extractor signature: takes a const message ref, returns a Scalar component.
    using ExtractorFn = std::function<rerun::components::Scalar(const MsgT&)>;

    /**
     * @param name      Rerun entity path (e.g., `"ins/heading_deg"`).
     * @param extractor Lambda that maps the message to a `rerun::components::Scalar`.
     * @param capacity  Pre-allocation hint matching the engine's buffer capacity.
     */
    ScalarFieldHandler(std::string name, ExtractorFn extractor, size_t capacity)
        : FieldHandlerBase<MsgT>(std::move(name)), extractor(std::move(extractor))
    { data.reserve(capacity); }

    void addData(const MsgT& msg) override { data.emplace_back(extractor(msg)); }

    void sendColumn(rerun::RecordingStream& rec,
                    const rerun::TimeColumn& timeCol) override {
        rec.send_columns(this->name, {timeCol}, rerun::Scalars(data).columns());
    }

    void clear()        override { data.clear(); }
    size_t size() const override { return data.size(); }

private:
    ExtractorFn extractor;
    std::vector<rerun::components::Scalar> data;
};

/**
 * @brief Handler for 3D positional data.
 *
 * Each buffered sample is logged as a `Points3D` entry on the Rerun timeline,
 * producing a trajectory or scatter plot. Useful for GPS fixes, odometry
 * translation components, or any (x, y, z) position.
 *
 * @tparam MsgT The ROS 2 message type.
 */
template <typename MsgT>
struct Position3DFieldHandler : public FieldHandlerBase<MsgT> {
    using ExtractorFn = std::function<rerun::components::Position3D(const MsgT&)>;

    /**
     * @param name      Rerun entity path.
     * @param extractor Lambda that maps the message to a `rerun::components::Position3D`.
     * @param capacity  Pre-allocation hint.
     */
    Position3DFieldHandler(std::string name, ExtractorFn extractor, size_t capacity)
        : FieldHandlerBase<MsgT>(std::move(name)), extractor(std::move(extractor))
    { data.reserve(capacity); }

    void addData(const MsgT& msg) override { data.emplace_back(extractor(msg)); }

    void sendColumn(rerun::RecordingStream& rec,
                    const rerun::TimeColumn& timeCol) override {
        rec.send_columns(this->name, {timeCol}, rerun::Points3D(data).columns());
    }

    void clear()        override { data.clear(); }
    size_t size() const override { return data.size(); }

private:
    ExtractorFn extractor;
    std::vector<rerun::components::Position3D> data;
};

/**
 * @brief Handler for 3D vector data.
 *
 * Logged as `Arrows3D`, making velocities, accelerations, and magnetic field
 * vectors visually inspectable in the Rerun 3D viewport.
 *
 * @tparam MsgT The ROS 2 message type.
 */
template <typename MsgT>
struct Vector3DFieldHandler : public FieldHandlerBase<MsgT> {
    using ExtractorFn = std::function<rerun::components::Vector3D(const MsgT&)>;

    /**
     * @param name      Rerun entity path.
     * @param extractor Lambda that maps the message to a `rerun::components::Vector3D`.
     * @param capacity  Pre-allocation hint.
     */
    Vector3DFieldHandler(std::string name, ExtractorFn extractor, size_t capacity)
        : FieldHandlerBase<MsgT>(std::move(name)), extractor(std::move(extractor))
    { data.reserve(capacity); }

    void addData(const MsgT& msg) override { data.emplace_back(extractor(msg)); }

    void sendColumn(rerun::RecordingStream& rec,
                    const rerun::TimeColumn& timeCol) override {
        rec.send_columns(this->name, {timeCol},
            rerun::Arrows3D::from_vectors(data).columns());
    }

    void clear()        override { data.clear(); }
    size_t size() const override { return data.size(); }

private:
    ExtractorFn extractor;
    std::vector<rerun::components::Vector3D> data;
};

// ============================================================
//  Batcher engine
// ============================================================

/**
 * @brief Mid-level buffering engine managing a heterogeneous collection of FieldHandlers.
 *
 * Uses type-erasure (`FieldHandlerBase`) to store handlers of different component
 * types in a single list. On each `autoLog()` call, every registered handler extracts
 * its component from the message and appends it to its internal buffer. When the
 * buffer reaches `capacity`, all handlers are flushed to Rerun in one `send_columns`
 * batch per handler, minimising round-trips to the server.
 *
 * Use this class directly when you need to share one `rerun::RecordingStream` across
 * multiple engines in the same ROS 2 node. For the common single-topic case, prefer
 * the higher-level `AutoLogger`.
 *
 * @tparam MsgT The ROS 2 message type consumed by this engine.
 *
 * @note The destructor calls `flush()`, so partially-filled buffers at node
 *       shutdown are never silently dropped.
 */
template <typename MsgT>
class BatcherEngine {
public:
    /**
     * @brief Constructs the buffer engine.
     * @param rec      Reference to an active `rerun::RecordingStream`. The engine
     *                 does **not** take ownership; the stream must outlive the engine.
     * @param capacity Number of messages to accumulate before a flush is triggered.
     *                 Defaults to `DEFAULT_BUFFER` (250).
     */
    explicit BatcherEngine(rerun::RecordingStream& rec, size_t capacity = DEFAULT_BUFFER)
        : rec(rec), bufferCapacity(capacity)
    { timeBuffer.reserve(capacity); }

    /** @brief Flushes any remaining buffered data before destruction. */
    ~BatcherEngine() { flush(); }

    /**
     * @brief Registers a scalar field extractor.
     * @param name Rerun entity path (e.g., `"ins/roll_deg"`).
     * @param fn   Lambda: `(const MsgT&) → rerun::components::Scalar`.
     */
    void registerScalar(const std::string& name,
                        std::function<rerun::components::Scalar(const MsgT&)> fn)
    {
        fields.emplace_back(
            std::make_unique<ScalarFieldHandler<MsgT>>(name, std::move(fn), bufferCapacity));
    }

    /**
     * @brief Registers a 3D position field extractor.
     * @param name Rerun entity path.
     * @param fn   Lambda: `(const MsgT&) → rerun::components::Position3D`.
     */
    void registerPosition3D(const std::string& name,
                            std::function<rerun::components::Position3D(const MsgT&)> fn)
    {
        fields.emplace_back(
            std::make_unique<Position3DFieldHandler<MsgT>>(name, std::move(fn), bufferCapacity));
    }

    /**
     * @brief Registers a 3D vector field extractor.
     * @param name Rerun entity path.
     * @param fn   Lambda: `(const MsgT&) → rerun::components::Vector3D`.
     */
    void registerVector3D(const std::string& name,
                          std::function<rerun::components::Vector3D(const MsgT&)> fn)
    {
        fields.emplace_back(
            std::make_unique<Vector3DFieldHandler<MsgT>>(name, std::move(fn), bufferCapacity));
    }

    /**
     * @brief Registers a fully custom `FieldHandlerBase` subclass.
     *
     * Use this to support Rerun component types not covered by the three built-in
     * handlers (e.g., `RotationQuat`, `Color`, `LineStrip3D`).
     *
     * @param handler Heap-allocated handler; ownership is transferred to the engine.
     */
    void registerCustom(std::unique_ptr<FieldHandlerBase<MsgT>> handler)
    {
        fields.emplace_back(std::move(handler));
    }

    /**
     * @brief Ingests a message using an explicit nanosecond timestamp.
     *
     * Preferred when the message carries a valid ROS header stamp, as it produces
     * a timeline that aligns correctly with other logged topics.
     *
     * @param msg           Shared pointer to the incoming ROS message.
     * @param custom_time_ns Timestamp in nanoseconds since the Unix epoch.
     */
    void autoLog(const std::shared_ptr<MsgT>& msg, int64_t custom_time_ns)
    {
        addData(msg);
        timeBuffer.emplace_back(custom_time_ns);
        if (timeBuffer.size() >= bufferCapacity)
            flush();
    }

    /**
     * @brief Ingests a message using the system high-resolution clock.
     *
     * Convenient when the message does not carry a header stamp, or when
     * wall-clock time is sufficient for the use case.
     *
     * @param msg Shared pointer to the incoming ROS message.
     */
    void autoLog(const std::shared_ptr<MsgT>& msg)
    {
        auto now   = std::chrono::high_resolution_clock::now();
        int64_t ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                         now.time_since_epoch()).count();
        autoLog(msg, ns);
    }

    /**
     * @brief Runs all registered extractors on @p msg without touching the time buffer.
     *
     * Rarely needed directly; prefer `autoLog()`. Exposed for advanced use cases where
     * the caller manages timestamp bookkeeping manually.
     *
     * @param msg Shared pointer to the ROS message.
     */
    void addData(const std::shared_ptr<MsgT>& msg)
    {
        for (auto& field : fields)
            field->addData(*msg);
    }

    /**
     * @brief Force-flushes all accumulated data to the Rerun stream immediately.
     *
     * Called automatically when the buffer reaches capacity and in the destructor.
     * Safe to call on an empty buffer (no-op).
     */
    void flush()
    {
        if (timeBuffer.empty()) return;

        auto timeColumn = rerun::TimeColumn::from_nanos_since_epoch("log_time", timeBuffer);

        for (auto& field : fields) {
            field->sendColumn(rec, timeColumn);
            field->clear();
        }
        timeBuffer.clear();
    }

    /**
     * @brief Changes the buffer capacity at runtime.
     *
     * If the number of currently buffered samples is greater than or equal to
     * @p capacity, a flush is triggered before the capacity is updated.
     *
     * @param capacity New buffer capacity (must be > 0).
     */
    void setCapacity(size_t capacity)
    {
        if (timeBuffer.size() >= capacity)
            flush();
        bufferCapacity = capacity;
    }

    /** @brief Returns the current buffer capacity. */
    size_t getCapacity() const { return bufferCapacity; }

private:
    rerun::RecordingStream& rec;
    std::vector<std::unique_ptr<FieldHandlerBase<MsgT>>> fields;
    size_t bufferCapacity;
    std::vector<int64_t> timeBuffer;
};

// ============================================================
//  High-level convenience wrapper
// ============================================================

/**
 * @brief High-level convenience wrapper around `BatcherEngine`.
 *
 * `AutoLogger` is the recommended entry point for the common case of one logger
 * per ROS 2 node. It owns the `rerun::RecordingStream`, spawns the Rerun viewer
 * on construction, and exposes a minimal API: register your fields once, then call
 * `log()` inside every subscription callback.
 *
 * For multi-topic nodes that share a single stream, use `BatcherEngine` directly.
 *
 * @tparam MsgT The ROS 2 message type to be logged.
 *
 * @code
 * rerunBatcher::AutoLogger<sensor_msgs::msg::Imu> logger("imu_logger", 250);
 * logger.registerScalar("imu/accel_z",
 *     [](const auto& m) { return rerun::components::Scalar(m.linear_acceleration.z); });
 *
 * // subscription callback:
 * logger.log(msg);
 * @endcode
 */
template <typename MsgT>
class AutoLogger {
public:
    /**
     * @brief Constructs the logger and spawns the Rerun viewer.
     * @param app_id   Application identifier shown in the Rerun UI title bar.
     * @param capacity Buffer capacity passed to the underlying `BatcherEngine`.
     */
    explicit AutoLogger(const std::string& app_id, size_t capacity = DEFAULT_BUFFER)
        : rec(app_id), engine(rec, capacity)
    { rec.spawn().exit_on_failure(); }

    AutoLogger(const AutoLogger&) = delete;
    AutoLogger& operator=(const AutoLogger&) = delete;
    AutoLogger(AutoLogger&&) = delete;
    AutoLogger& operator=(AutoLogger&&) = delete;

    /** @copydoc BatcherEngine::registerScalar */
    void registerScalar(const std::string& name,
                        std::function<rerun::components::Scalar(const MsgT&)> fn)
    { engine.registerScalar(name, std::move(fn)); }

    /** @copydoc BatcherEngine::registerPosition3D */
    void registerPosition3D(const std::string& name,
                            std::function<rerun::components::Position3D(const MsgT&)> fn)
    { engine.registerPosition3D(name, std::move(fn)); }

    /** @copydoc BatcherEngine::registerVector3D */
    void registerVector3D(const std::string& name,
                          std::function<rerun::components::Vector3D(const MsgT&)> fn)
    { engine.registerVector3D(name, std::move(fn)); }

    /**
     * @brief Buffers the message using the system high-resolution clock.
     * @param msg Shared pointer to the incoming ROS 2 message.
     */
    void log(const std::shared_ptr<MsgT>& msg) { engine.autoLog(msg); }

    /**
     * @brief Buffers the message with an explicit nanosecond timestamp.
     * @param msg Shared pointer to the incoming ROS 2 message.
     * @param ns  Timestamp in nanoseconds since the Unix epoch.
     *            Typically `rclcpp::Time(msg->header.stamp).nanoseconds()`.
     */
    void log(const std::shared_ptr<MsgT>& msg, int64_t ns) { engine.autoLog(msg, ns); }

    /** @brief Force-flushes the underlying buffer engine. */
    void flush() { engine.flush(); }

    /**
     * @brief Provides direct access to the underlying `rerun::RecordingStream`.
     *
     * Use this for hybrid logging: infrequent events (mode changes, warnings)
     * can be sent directly via the stream while high-frequency telemetry goes
     * through the buffer.
     *
     * @return Reference to the internal `rerun::RecordingStream`.
     */
    rerun::RecordingStream& stream() { return rec; }

private:
    rerun::RecordingStream rec;
    BatcherEngine<MsgT> engine;
};

} // namespace rerunBatcher