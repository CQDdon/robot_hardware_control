#ifndef SHM_HPP_
#define SHM_HPP_

#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <new>
#include <string>
#include <type_traits>

// Shared memory names.
constexpr const char *SHM_HWI_CMD_IF = "/hwi_to_hsh_cmd_if";
constexpr const char *SHM_HWI_STATE = "/hwi_to_hsh_state"; // lite state
constexpr const char *SHM_HWI_STATE_DEBUG = "/hwi_to_hsh_state_debug";
constexpr const char *SHM_HWI_SENSOR_STATE = "/hwi_to_hsh_sensor_state";
constexpr const char *SHM_HSH_CTRL = "/hsh_to_hwi_ctrl";

constexpr unsigned SHM_SNAPSHOT_READ_RETRIES = 3;
constexpr unsigned SHM_SNAPSHOT_READ_RETRIES = 20;

constexpr size_t SHM_MAX_AXES = 8;
constexpr size_t SHM_MAX_SENSORS = 8;

// Controller command interface type.
enum class CommandInterface : uint8_t
{
    None = 0,
    Position = 1,
    Velocity = 2,
    Torque = 3
};

// Control action that HSH requests on each axis.
enum class ControlAction : uint8_t
{
    None = 0,
    Idle = 1,
    ClosedLoop = 2,
    FullCalib = 3,
    Homing = 4,
    ClearErrors = 5
};

// Desired ODrive axis state. This is distinct from heartbeat feedback and
// from the deprecated ControlAction compatibility field.
enum class AxisDesiredState : uint8_t
{
    Idle = 1,
    FullCalibration = 3,
    ClosedLoop = 8,
    Homing = 11
};

enum class SafetyAction : uint8_t
{
    None = 0,
    SoftStop = 1,
    Emergency = 2
};

// Compact axis state summary derived from ODrive AxisState.
enum class AxisStateSummary : uint8_t
{
    Unknown = 0,
    Idle = 1,
    Startup = 2,
    Calibrating = 3,
    Homing = 4,
    Running = 5
};

enum class SensorType : uint8_t
{
    Unknown = 0,
    Imu = 1,
    Encoder = 2,
    Sensor_3 = 3,
    Sensor_4 = 4,
    Sensor_5 = 5,
    Sensor_6 = 6,
    Sensor_7 = 7
};

enum class SensorStatus : uint8_t
{
    Init = 0,
    Active = 1,
    Error = 2
};

struct AxisFeedbackLite
{
    uint32_t can_id;
    uint32_t error;    
    uint16_t reserved_u16_0;
    uint8_t online;
    uint8_t reserved_u8_0;
    AxisStateSummary odrive_state_summary;

    float position; // rad
    float velocity; // rad/s

    AxisFeedbackLite()
        : can_id(0), odrive_state_summary(AxisStateSummary::Unknown), error(0),
          online(0), reserved_u8_0(0), reserved_u16_0(0),
          position(0.0f), velocity(0.0f)
    {
    }
};

struct AxisFeedbackDebug
{
    uint64_t last_hb_timestamp_ns;
    uint32_t axis_error;
    uint32_t motor_error;
    uint32_t encoder_error;
    uint32_t controller_error;
    uint16_t reserved_u16_0;
    uint8_t odrive_state;          // Raw AxisState from heartbeat
    uint8_t trajectory_done_flag;  // From heartbeat (bit)


    AxisFeedbackDebug()
        : axis_error(0), odrive_state(0), trajectory_done_flag(0), reserved_u16_0(0),
          motor_error(0), encoder_error(0), controller_error(0),
          last_hb_timestamp_ns(0)
    {
    }
};

inline bool is_axis_online(const AxisFeedbackLite &ax)
{
    return ax.online != 0;
}

// Map ODrive AxisState to a compact summary used by HSH.
inline AxisStateSummary map_odrive_axis_state_summary(uint8_t axis_state)
{
    switch (axis_state)
    {
    case 1:  // IDLE
        return AxisStateSummary::Idle;
    case 2:  // STARTUP_SEQUENCE
        return AxisStateSummary::Startup;
    case 3:  // FULL_CALIBRATION_SEQUENCE
    case 4:  // MOTOR_CALIBRATION
    case 6:  // ENCODER_INDEX_SEARCH
    case 7:  // ENCODER_OFFSET_CALIBRATION
    case 10: // ENCODER_DIR_FIND
    case 12: // ENCODER_HALL_POLARITY_CALIBRATION
    case 13: // ENCODER_HALL_PHASE_CALIBRATION
        return AxisStateSummary::Calibrating;
    case 11: // HOMING
        return AxisStateSummary::Homing;
    case 8:  // CLOSED_LOOP_CONTROL
    case 9:  // LOCKIN_SPIN
        return AxisStateSummary::Running;
    case 0:  // UNDEFINED
    default:
        return AxisStateSummary::Unknown;
    }
}

struct Rs485StateBlock
{
    uint8_t is_auto_mode;          // 1 = Auto (AI 0x0005 >= 2.5V), 0 = Manual
    uint8_t reserved_u8_0;
    uint16_t reserved_u16_0;
    float e_stop_voltage;          // Đọc từ AI 0x0006 (0.0 - 5.0 V)
    float steering_pot_percent;    // Đọc từ AI 0x0007 (0.0 - 100.0 %)

    Rs485StateBlock()
        : is_auto_mode(0), reserved_u8_0(0), reserved_u16_0(0),
          e_stop_voltage(0.0f), steering_pot_percent(0.0f)
    {
    }
};

struct Rs485CommandBlock
{
    float throttle_voltage;        // Điện áp chân ga ghi xuống AO 0x0006 (0.0 - 5.0 V)
    uint32_t reserved_u32_0;

    Rs485CommandBlock()
        : throttle_voltage(0.0f), reserved_u32_0(0)
    {
    }
};

struct RelayStateBlock
{
    uint8_t relay_mask;            // 8-bit mask phản hồi trạng thái relay
    uint8_t din_mask;              // 8-bit mask đọc từ các cổng DI1..DI8 (raw pin bitmask)
    uint8_t is_auto_mode;          // 1 = Auto Mode, 0 = Manual Mode (từ DI6)
    uint8_t is_estop_active;       // 1 = E-Stop Triggered (Dừng khẩn cấp), 0 = Normal (từ DI7)
    uint32_t reserved_u32_0;

    RelayStateBlock()
        : relay_mask(0), din_mask(0), is_auto_mode(1), is_estop_active(0),
          reserved_u32_0(0)
    {
    }
};

struct RelayCommandBlock
{
    uint8_t relay_mask;            // 8-bit mask điều khiển relay
    uint8_t reserved_u8_0;
    uint16_t reserved_u16_0;
    uint32_t reserved_u32_0;

    RelayCommandBlock()
        : relay_mask(0), reserved_u8_0(0), reserved_u16_0(0), reserved_u32_0(0)
    {
    }
};

struct HwiStateBlock
{
    uint64_t timestamp_ns;
    uint32_t sequence_id;
    uint16_t reserved_u16_1;
    uint8_t reserved_u8_1;
    uint8_t axis_count;
    AxisFeedbackLite axes[SHM_MAX_AXES];
    Rs485StateBlock rs485;
    RelayStateBlock relay;

    HwiStateBlock()
        : sequence_id(0), timestamp_ns(0), axis_count(0),
          reserved_u8_1(0), reserved_u16_1(0), rs485(), relay()
    {
        for (auto &axis : axes)
        {
            axis = {};
        }
    }
};

struct HwiStateDebugBlock
{
    uint64_t timestamp_ns;
    uint32_t sequence_id;
    uint16_t reserved_u16_0;
    uint8_t axis_count;
    uint8_t reserved_u8_0;
    AxisFeedbackDebug axes[SHM_MAX_AXES];

    HwiStateDebugBlock()
        : sequence_id(0), timestamp_ns(0), axis_count(0),
          reserved_u8_0(0), reserved_u16_0(0)
    {
        for (auto &axis : axes)
        {
            axis = {};
        }
    }
};

struct SensorState
{
    uint8_t sensor_id;
    SensorType type;
    SensorStatus status;
    uint8_t online;
    uint8_t reserved_u8_0;
    float values[10];

    SensorState()
        : sensor_id(0), type(SensorType::Unknown), status(SensorStatus::Init),
          online(0), reserved_u8_0(0)
    {
        for (auto &value : values)
        {
            value = 0.0f;
        }
    }
};

struct HwiSensorStateBlock
{
    uint64_t timestamp_ns;
    uint32_t sequence_id;
    uint16_t reserved_u16_0;
    uint8_t sensor_count;
    uint8_t reserved_u8_0;
    SensorState sensors[SHM_MAX_SENSORS];

    HwiSensorStateBlock()
        : timestamp_ns(0), sequence_id(0), reserved_u16_0(0),
          sensor_count(0), reserved_u8_0(0)
    {
        for (auto &sensor : sensors)
        {
            sensor = {};
        }
    }
};

struct AxisCommandIf
{
    float value;
    uint16_t reserved_u16_0;
    // Requested run action from the ros2_control command interfaces.  This
    // occupies previously reserved storage, preserving this record's size.
    ControlAction action;
    CommandInterface interface;

    AxisCommandIf()
        : interface(CommandInterface::None),
          action(ControlAction::None), reserved_u16_0(0), value(0.0f)
    {
    }
};

struct HwiCommandIfBlock
{
    uint64_t timestamp_ns;
    uint32_t sequence_id;
    uint16_t reserved_u16_0;
    uint8_t reserved_u8_0;
    uint8_t axis_count;
    AxisCommandIf axes[SHM_MAX_AXES];
    Rs485CommandBlock rs485;
    RelayCommandBlock relay;

    HwiCommandIfBlock()
        : sequence_id(0), timestamp_ns(0), axis_count(0),
          reserved_u8_0(0), reserved_u16_0(0), rs485(), relay()
    {
        for (auto &axis : axes)
        {
            axis = {};
        }
    }
};

struct AxisControl
{
    float target;
    uint16_t reserved_u16_0;
    uint8_t reserved_u8_0;
    // Compatibility-only during H4/H5. CAN must not use this as control input.
    ControlAction action;
    AxisDesiredState desired_axis_state;
    uint8_t motion_enable;
    CommandInterface control_mode;
    uint64_t desired_state_generation;
    uint64_t target_sequence;
    uint64_t clear_error_req_id;

    AxisControl()
        : action(ControlAction::None), desired_axis_state(AxisDesiredState::Idle),
          motion_enable(0), control_mode(CommandInterface::None),
          desired_state_generation(0), target_sequence(0), clear_error_req_id(0),
          reserved_u8_0(0), reserved_u16_0(0), target(0.0f)
    {
    }
};

struct HshControlBlock
{
    uint64_t timestamp_ns;
    uint32_t sequence_id;
    uint8_t axis_count;
    uint8_t motion_enable;
    SafetyAction safety_action;
    uint8_t debug_enable;
    AxisControl axes[SHM_MAX_AXES];
    Rs485CommandBlock rs485;
    RelayCommandBlock relay;

    HshControlBlock()
        : sequence_id(0), timestamp_ns(0), axis_count(0), motion_enable(0),
          safety_action(SafetyAction::None), debug_enable(0), rs485(), relay()
    {
        for (auto &axis : axes)
        {
            axis = {};
        }
    }
};

static_assert(std::is_trivially_copyable<Rs485StateBlock>::value);
static_assert(std::is_trivially_copyable<Rs485CommandBlock>::value);
static_assert(std::is_trivially_copyable<RelayStateBlock>::value);
static_assert(std::is_trivially_copyable<RelayCommandBlock>::value);
static_assert(std::is_trivially_copyable<HwiCommandIfBlock>::value);
static_assert(std::is_trivially_copyable<HwiStateBlock>::value);
static_assert(std::is_trivially_copyable<HwiStateDebugBlock>::value);
static_assert(std::is_trivially_copyable<HwiSensorStateBlock>::value);
static_assert(std::is_trivially_copyable<HshControlBlock>::value);
static_assert(std::atomic<uint64_t>::is_always_lock_free,
              "Shared-memory sequence counter must be lock-free");

// Linux/POSIX shared-memory seqlock for latest-only, single-writer blocks.
// Readers only use a copied payload after observing the same even sequence
// number before and after the copy. Payloads must remain POD-like snapshots.
template <typename Payload>
struct alignas(64) ShmSnapshotBlock
{
    static_assert(std::is_trivially_copyable<Payload>::value,
                  "Shared-memory payload must be trivially copyable");

    std::atomic<uint64_t> sequence{0}; // odd: writer owns block, even: stable
    uint32_t payload_size{sizeof(Payload)};
    Payload payload{};
};

struct SnapshotStats
{
    std::atomic<uint64_t> read_success{0};
    std::atomic<uint64_t> read_retry{0};
    std::atomic<uint64_t> read_failure{0};
    std::atomic<uint64_t> max_attempts_used{0};
};

template <typename Payload>
inline void initialize_snapshot_block(ShmSnapshotBlock<Payload> *block)
{
    static_assert(std::is_trivially_copyable<Payload>::value,
                  "Shared-memory payload must be trivially copyable");
    new (block) ShmSnapshotBlock<Payload>();
}

template <typename Payload>
inline bool snapshot_block_valid(const ShmSnapshotBlock<Payload> *block)
{
    return block != nullptr && block->payload_size == sizeof(Payload);
}

template <typename Payload>
inline bool read_snapshot_block(const ShmSnapshotBlock<Payload> *block, Payload &out,
                                SnapshotStats *stats = nullptr)
{
    if (!snapshot_block_valid(block))
    {
        return false;
    }

    for (unsigned attempt = 0; attempt < SHM_SNAPSHOT_READ_RETRIES; ++attempt)
    {
        const uint64_t begin = block->sequence.load(std::memory_order_acquire);
        if ((begin & 1U) != 0U)
        {
            if (stats) stats->read_retry.fetch_add(1, std::memory_order_relaxed);
            continue;
        }

        std::memcpy(&out, &block->payload, sizeof(Payload));
        std::atomic_thread_fence(std::memory_order_acquire);
        const uint64_t end = block->sequence.load(std::memory_order_acquire);
        if (begin == end && (end & 1U) == 0U)
        {
            if (stats)
            {
                stats->read_success.fetch_add(1, std::memory_order_relaxed);
                auto max = stats->max_attempts_used.load(std::memory_order_relaxed);
                while (max < attempt + 1U &&
                       !stats->max_attempts_used.compare_exchange_weak(max, attempt + 1U,
                                                                         std::memory_order_relaxed)) {}
            }
            return true;
        }
        if (stats) stats->read_retry.fetch_add(1, std::memory_order_relaxed);
    }
    if (stats)
    {
        stats->read_failure.fetch_add(1, std::memory_order_relaxed);
        auto max = stats->max_attempts_used.load(std::memory_order_relaxed);
        while (max < SHM_SNAPSHOT_READ_RETRIES &&
               !stats->max_attempts_used.compare_exchange_weak(max, SHM_SNAPSHOT_READ_RETRIES,
                                                                 std::memory_order_relaxed)) {}
    }
    return false;
}

template <typename Payload>
inline bool write_snapshot_block(ShmSnapshotBlock<Payload> *block, const Payload &value)
{
    if (!snapshot_block_valid(block))
    {
        return false;
    }

    const uint64_t current = block->sequence.load(std::memory_order_relaxed);
    if ((current & 1U) != 0U)
    {
        return false; // violated single-writer ownership
    }
    block->sequence.store(current + 1U, std::memory_order_release);
    std::atomic_thread_fence(std::memory_order_release);
    std::memcpy(&block->payload, &value, sizeof(Payload));
    std::atomic_thread_fence(std::memory_order_release);
    block->sequence.store(current + 2U, std::memory_order_release);
    return true;
}

namespace hardware_ipc_common
{

    class SharedMemorySegment
    {
    public:
        SharedMemorySegment() = default;
        SharedMemorySegment(std::string name, std::size_t size);
        ~SharedMemorySegment();

        SharedMemorySegment(const SharedMemorySegment &) = delete;
        SharedMemorySegment &operator=(const SharedMemorySegment &) = delete;
        SharedMemorySegment(SharedMemorySegment &&) noexcept;
        SharedMemorySegment &operator=(SharedMemorySegment &&) noexcept;

        bool open(bool create_if_missing = true);
        void close();
        bool is_open() const noexcept { return ptr_ != nullptr; }
        void *data() noexcept { return ptr_; }
        const void *data() const noexcept { return ptr_; }
        template <typename T>
        T *as() noexcept { return static_cast<T *>(ptr_); }
        template <typename T>
        const T *as() const noexcept { return static_cast<const T *>(ptr_); }

        const std::string &name() const noexcept { return name_; }
        std::size_t size() const noexcept { return size_; }

    private:
        void reset();

        std::string name_;
        std::size_t size_{0};
        int fd_{-1};
        void *ptr_{nullptr};
    };

    class SharedMemoryInterface
    {
    public:
        SharedMemoryInterface() = default;
        ~SharedMemoryInterface();

        SharedMemoryInterface(const SharedMemoryInterface &) = delete;
        SharedMemoryInterface &operator=(const SharedMemoryInterface &) = delete;
        SharedMemoryInterface(SharedMemoryInterface &&) noexcept = default;
        SharedMemoryInterface &operator=(SharedMemoryInterface &&) noexcept = default;

        bool open(bool clear_on_open = true);
        bool write_cmd_if(const HwiCommandIfBlock &command_if);
        bool write_state(const HwiStateBlock &state);
        bool write_state_debug(const HwiStateDebugBlock &state);
        bool write_sensor_state(const HwiSensorStateBlock &state);
        bool write_control(const HshControlBlock &control);
        void close();
        bool ready() const noexcept;

        bool read_cmd_if(HwiCommandIfBlock &command_if) const;
        bool read_state(HwiStateBlock &state) const;
        bool read_state_debug(HwiStateDebugBlock &state) const;
        bool read_sensor_state(HwiSensorStateBlock &state) const;
        bool read_control(HshControlBlock &control) const;
        const SnapshotStats &snapshot_stats() const noexcept { return snapshot_stats_; }

    private:
        SharedMemorySegment cmd_if_segment_{SHM_HWI_CMD_IF, sizeof(ShmSnapshotBlock<HwiCommandIfBlock>)};
        SharedMemorySegment state_segment_{SHM_HWI_STATE, sizeof(ShmSnapshotBlock<HwiStateBlock>)};
        SharedMemorySegment state_debug_segment_{SHM_HWI_STATE_DEBUG, sizeof(ShmSnapshotBlock<HwiStateDebugBlock>)};
        SharedMemorySegment sensor_state_segment_{SHM_HWI_SENSOR_STATE, sizeof(ShmSnapshotBlock<HwiSensorStateBlock>)};
        SharedMemorySegment control_segment_{SHM_HSH_CTRL, sizeof(ShmSnapshotBlock<HshControlBlock>)};
        mutable SnapshotStats snapshot_stats_{};
    };

} // namespace hardware_ipc_common

#endif // SHM_HPP
