#ifndef LOCKFREE_BUFFERS_HPP
#define LOCKFREE_BUFFERS_HPP

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <new>
#include <array>
#include <type_traits>

namespace turtlebot4 {

// ============================================================================
// CACHE LINE SIZE - Prevent false sharing
// ============================================================================
#ifdef __cpp_lib_hardware_interference_size
    constexpr size_t CACHE_LINE_SIZE = std::hardware_destructive_interference_size;
#else
    constexpr size_t CACHE_LINE_SIZE = 64;
#endif

// ============================================================================
// 1. SPSC RING BUFFER - For /scan (High-frequency, large data)
// ============================================================================
// Single-Producer Single-Consumer Lock-Free Ring Buffer
// - Zero-Copy: Reader gets pointer to buffer slot
// - No locks, no waiting
// - Ideal for: 50 Hz scan data, 8 KB per message
//
// Usage:
//   Writer: auto* slot = buffer.prepare_write();
//           fill_data(slot);
//           buffer.commit_write();
//
//   Reader: const auto* data = buffer.read();
//           if (data) process(*data);
//           buffer.release_read();

template<typename T, size_t Capacity>
class SPSCRingBuffer {
    static_assert(Capacity > 0 && (Capacity & (Capacity - 1)) == 0,
                  "Capacity must be power of 2");
public:
    SPSCRingBuffer() : head_(0), tail_(0) {
        for (auto& slot : slots_) {
            slot.sequence.store(0, std::memory_order_relaxed);
        }
    }

    // === WRITER SIDE (Single Producer) ===

    // Get slot for writing (Zero-Copy). Returns nullptr if buffer full.
    T* prepare_write() noexcept {
        const size_t head = head_.load(std::memory_order_relaxed);
        const size_t next = (head + 1) & MASK;

        // Check if buffer is full
        if (next == tail_.load(std::memory_order_acquire)) {
            return nullptr;  // Buffer full
        }

        return &slots_[head].data;
    }

    // Commit the write, making data visible to reader
    void commit_write() noexcept {
        const size_t head = head_.load(std::memory_order_relaxed);
        slots_[head].sequence.store(head + 1, std::memory_order_release);
        head_.store((head + 1) & MASK, std::memory_order_release);
    }

    // Combined write for small types (copies data)
    bool try_write(const T& value) noexcept {
        T* slot = prepare_write();
        if (!slot) return false;
        *slot = value;
        commit_write();
        return true;
    }

    // === READER SIDE (Single Consumer) ===

    // Get pointer to data for reading (Zero-Copy). Returns nullptr if empty.
    const T* read() const noexcept {
        const size_t tail = tail_.load(std::memory_order_relaxed);

        // Check if buffer is empty
        if (tail == head_.load(std::memory_order_acquire)) {
            return nullptr;  // Buffer empty
        }

        // Wait for data to be committed
        const size_t expected_seq = tail + 1;
        if (slots_[tail].sequence.load(std::memory_order_acquire) != expected_seq) {
            return nullptr;  // Data not yet committed
        }

        return &slots_[tail].data;
    }

    // Release read slot, allowing writer to reuse it
    void release_read() noexcept {
        const size_t tail = tail_.load(std::memory_order_relaxed);
        tail_.store((tail + 1) & MASK, std::memory_order_release);
    }

    // Combined read for convenience (copies data)
    bool try_read(T& out) noexcept {
        const T* ptr = read();
        if (!ptr) return false;
        out = *ptr;
        release_read();
        return true;
    }

    // === STATUS ===

    size_t size() const noexcept {
        const size_t head = head_.load(std::memory_order_acquire);
        const size_t tail = tail_.load(std::memory_order_acquire);
        return (head - tail) & MASK;
    }

    bool empty() const noexcept {
        return head_.load(std::memory_order_acquire) ==
               tail_.load(std::memory_order_acquire);
    }

    bool full() const noexcept {
        const size_t head = head_.load(std::memory_order_acquire);
        const size_t tail = tail_.load(std::memory_order_acquire);
        return ((head + 1) & MASK) == tail;
    }

    static constexpr size_t capacity() noexcept { return Capacity; }

private:
    static constexpr size_t MASK = Capacity - 1;

    struct alignas(CACHE_LINE_SIZE) Slot {
        T data;
        std::atomic<size_t> sequence;
    };

    alignas(CACHE_LINE_SIZE) std::atomic<size_t> head_;
    alignas(CACHE_LINE_SIZE) std::atomic<size_t> tail_;
    std::array<Slot, Capacity> slots_;
};


// ============================================================================
// 2. SEQLOCK - For /odom (Small data, many readers)
// ============================================================================
// Sequence Lock for lock-free reading with single writer
// - Readers never block the writer
// - Readers retry if they catch a concurrent write
// - Zero-Copy optional: use read_into() for direct access
// - Ideal for: Small data (< 256 bytes), many concurrent readers
//
// Usage:
//   Writer: lock.write(new_odom);
//
//   Reader: Odom data = lock.read();  // Copies, may retry internally
//           // or for zero-copy:
//           lock.read_into(my_buffer);

template<typename T>
class SeqLock {
    static_assert(std::is_trivially_copyable_v<T>,
                  "SeqLock requires trivially copyable types");
public:
    SeqLock() : sequence_(0) {}

    // === WRITER (Single thread only!) ===

    void write(const T& value) noexcept {
        // Increment to odd (signals write in progress)
        sequence_.fetch_add(1, std::memory_order_release);
        std::atomic_thread_fence(std::memory_order_release);

        data_ = value;

        std::atomic_thread_fence(std::memory_order_release);
        // Increment to even (signals write complete)
        sequence_.fetch_add(1, std::memory_order_release);
    }

    // Write with callback for partial updates
    template<typename Func>
    void update(Func&& func) noexcept {
        sequence_.fetch_add(1, std::memory_order_release);
        std::atomic_thread_fence(std::memory_order_release);

        func(data_);

        std::atomic_thread_fence(std::memory_order_release);
        sequence_.fetch_add(1, std::memory_order_release);
    }

    // === READERS (Multiple threads safe) ===

    // Read with automatic retry (returns copy)
    T read() const noexcept {
        T result;
        while (!try_read(result)) {
            // Spin - in practice rarely happens with small data
        }
        return result;
    }

    // Try to read once (returns false if write in progress)
    bool try_read(T& out) const noexcept {
        const uint64_t seq1 = sequence_.load(std::memory_order_acquire);

        // Odd sequence = write in progress
        if (seq1 & 1) {
            return false;
        }

        std::atomic_thread_fence(std::memory_order_acquire);
        out = data_;
        std::atomic_thread_fence(std::memory_order_acquire);

        const uint64_t seq2 = sequence_.load(std::memory_order_acquire);

        // Sequence changed = write occurred during read
        return seq1 == seq2;
    }

    // Read into existing buffer (Zero-Copy for caller's perspective)
    void read_into(T& out) const noexcept {
        while (!try_read(out)) {
            // Spin
        }
    }

    // Get current sequence number (for change detection)
    uint64_t sequence() const noexcept {
        return sequence_.load(std::memory_order_acquire);
    }

private:
    alignas(CACHE_LINE_SIZE) std::atomic<uint64_t> sequence_;
    alignas(CACHE_LINE_SIZE) T data_;
};


// ============================================================================
// 3. DOUBLE BUFFER WITH PRIORITY - For /cmd_vel (Two writers)
// ============================================================================
// Two writers (Navigation + Teleop) with priority switching
// - Teleop has priority (safety override)
// - Lock-free reads
// - Atomic mode switching
//
// Usage:
//   Nav:    buffer.write_nav(cmd);
//   Teleop: buffer.write_teleop(cmd);  // Auto-activates teleop mode
//   Reader: auto cmd = buffer.read();
//   Mode:   buffer.set_mode(Mode::NAVIGATION);

template<typename T>
class PriorityDoubleBuffer {
    static_assert(std::is_trivially_copyable_v<T>,
                  "PriorityDoubleBuffer requires trivially copyable types");
public:
    enum class Mode : uint8_t {
        NAVIGATION = 0,
        TELEOP = 1
    };

    PriorityDoubleBuffer() : mode_(Mode::NAVIGATION), nav_seq_(0), teleop_seq_(0) {}

    // === WRITERS ===

    void write_nav(const T& value) noexcept {
        nav_lock_.write(value);
        nav_seq_.fetch_add(1, std::memory_order_release);
    }

    void write_teleop(const T& value) noexcept {
        teleop_lock_.write(value);
        teleop_seq_.fetch_add(1, std::memory_order_release);
        // Teleop auto-activates (safety override)
        mode_.store(Mode::TELEOP, std::memory_order_release);
    }

    // === READER ===

    T read() const noexcept {
        if (mode_.load(std::memory_order_acquire) == Mode::TELEOP) {
            return teleop_lock_.read();
        }
        return nav_lock_.read();
    }

    // Zero-Copy read into buffer
    void read_into(T& out) const noexcept {
        if (mode_.load(std::memory_order_acquire) == Mode::TELEOP) {
            teleop_lock_.read_into(out);
        } else {
            nav_lock_.read_into(out);
        }
    }

    // === MODE CONTROL ===

    void set_mode(Mode m) noexcept {
        mode_.store(m, std::memory_order_release);
    }

    Mode mode() const noexcept {
        return mode_.load(std::memory_order_acquire);
    }

    // Sequence numbers for change detection
    uint64_t nav_sequence() const noexcept {
        return nav_seq_.load(std::memory_order_acquire);
    }

    uint64_t teleop_sequence() const noexcept {
        return teleop_seq_.load(std::memory_order_acquire);
    }

    uint64_t active_sequence() const noexcept {
        return (mode() == Mode::TELEOP) ? teleop_sequence() : nav_sequence();
    }

private:
    SeqLock<T> nav_lock_;
    SeqLock<T> teleop_lock_;
    alignas(CACHE_LINE_SIZE) std::atomic<Mode> mode_;
    alignas(CACHE_LINE_SIZE) std::atomic<uint64_t> nav_seq_;
    alignas(CACHE_LINE_SIZE) std::atomic<uint64_t> teleop_seq_;
};


// ============================================================================
// 4. DOUBLE BUFFER WITH DIRTY FLAG - For /map (Large data, slow updates)
// ============================================================================
// Back-buffer pattern with swap
// - Writer fills back buffer, then swaps atomically
// - Reader always sees consistent front buffer
// - Dirty flag signals new data available
// - Zero-Copy: Reader gets pointer to front buffer
//
// Usage:
//   Writer: auto* back = buffer.back_buffer();
//           fill_map(back);
//           buffer.swap_buffers();
//
//   Reader: if (buffer.has_new_data()) {
//               const auto* map = buffer.front_buffer();
//               process(*map);
//               buffer.clear_dirty();
//           }

template<typename T>
class DoubleBufferDirty {
public:
    DoubleBufferDirty() : front_idx_(0), dirty_(false) {}

    // === WRITER ===

    // Get back buffer for writing (Zero-Copy)
    T* back_buffer() noexcept {
        return &buffers_[1 - front_idx_.load(std::memory_order_acquire)];
    }

    // Swap buffers and set dirty flag
    void swap_buffers() noexcept {
        const size_t old_front = front_idx_.load(std::memory_order_acquire);
        front_idx_.store(1 - old_front, std::memory_order_release);
        dirty_.store(true, std::memory_order_release);
        sequence_.fetch_add(1, std::memory_order_release);
    }

    // === READER ===

    // Get front buffer for reading (Zero-Copy)
    const T* front_buffer() const noexcept {
        return &buffers_[front_idx_.load(std::memory_order_acquire)];
    }

    // Check if new data available
    bool has_new_data() const noexcept {
        return dirty_.load(std::memory_order_acquire);
    }

    // Clear dirty flag after processing
    void clear_dirty() noexcept {
        dirty_.store(false, std::memory_order_release);
    }

    // Sequence for change detection
    uint64_t sequence() const noexcept {
        return sequence_.load(std::memory_order_acquire);
    }

    // Copy read (for small types or when copy needed)
    T read() const noexcept {
        return *front_buffer();
    }

private:
    std::array<T, 2> buffers_;
    alignas(CACHE_LINE_SIZE) std::atomic<size_t> front_idx_;
    alignas(CACHE_LINE_SIZE) std::atomic<bool> dirty_;
    alignas(CACHE_LINE_SIZE) std::atomic<uint64_t> sequence_;
};


// ============================================================================
// 5. ATOMIC FLAG WITH TIMESTAMP - For /bumper (Safety-critical)
// ============================================================================
// Ultra-fast atomic flag for safety events
// - Single atomic read/write
// - Timestamp for event timing
// - Debounce support
//
// Note: In ROS2/WebSocket context, true hardware interrupt latency (<1ms)
// is not achievable. This provides fastest possible software response.
//
// Usage:
//   ISR/Callback: bumper.trigger();
//   Control:      if (bumper.is_triggered()) emergency_stop();
//   Reset:        bumper.reset();

class AtomicBumper {
public:
    AtomicBumper() : state_(0) {}

    // === TRIGGER (from callback/ISR) ===

    void trigger() noexcept {
        // Pack: timestamp (high 32 bits) + triggered flag (bit 0)
        const uint64_t now = current_time_ms();
        const uint64_t packed = (now << 32) | 1;
        state_.store(packed, std::memory_order_release);
    }

    // === QUERY (from control loop) ===

    bool is_triggered() const noexcept {
        return (state_.load(std::memory_order_acquire) & 1) != 0;
    }

    uint32_t trigger_time_ms() const noexcept {
        return static_cast<uint32_t>(state_.load(std::memory_order_acquire) >> 32);
    }

    // Time since trigger in ms (0 if not triggered)
    uint32_t elapsed_ms() const noexcept {
        const uint64_t packed = state_.load(std::memory_order_acquire);
        if (!(packed & 1)) return 0;
        return static_cast<uint32_t>(current_time_ms() - (packed >> 32));
    }

    // === RESET ===

    void reset() noexcept {
        state_.store(0, std::memory_order_release);
    }

    // Reset only if enough time has passed (debounce)
    bool reset_if_elapsed(uint32_t min_ms) noexcept {
        if (elapsed_ms() >= min_ms) {
            reset();
            return true;
        }
        return false;
    }

private:
    alignas(CACHE_LINE_SIZE) std::atomic<uint64_t> state_;

    static uint64_t current_time_ms() noexcept {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        return static_cast<uint64_t>(ts.tv_sec) * 1000 +
               static_cast<uint64_t>(ts.tv_nsec) / 1000000;
    }
};


// ============================================================================
// 6. TRIPLE BUFFER (Alternative to Ring Buffer for scan)
// ============================================================================
// Classic triple buffer if you prefer it over ring buffer
// - Guaranteed no frame drops (latest always available)
// - Slightly higher latency than ring buffer
// - Simpler implementation

template<typename T>
class TripleBuffer {
public:
    TripleBuffer() : write_idx_(0), read_idx_(1), ready_idx_(2) {}

    // === WRITER ===

    T* write_buffer() noexcept {
        return &buffers_[write_idx_.load(std::memory_order_acquire)];
    }

    void publish() noexcept {
        // Swap write and ready buffers
        size_t old_ready = ready_idx_.exchange(
            write_idx_.load(std::memory_order_acquire),
            std::memory_order_acq_rel
        );
        write_idx_.store(old_ready, std::memory_order_release);
        sequence_.fetch_add(1, std::memory_order_release);
    }

    // === READER ===

    const T* read_buffer() noexcept {
        // Swap read and ready buffers
        size_t old_ready = ready_idx_.exchange(
            read_idx_.load(std::memory_order_acquire),
            std::memory_order_acq_rel
        );
        read_idx_.store(old_ready, std::memory_order_release);
        return &buffers_[read_idx_.load(std::memory_order_acquire)];
    }

    // Peek without consuming (may return stale data)
    const T* peek() const noexcept {
        return &buffers_[ready_idx_.load(std::memory_order_acquire)];
    }

    uint64_t sequence() const noexcept {
        return sequence_.load(std::memory_order_acquire);
    }

private:
    std::array<T, 3> buffers_;
    alignas(CACHE_LINE_SIZE) std::atomic<size_t> write_idx_;
    alignas(CACHE_LINE_SIZE) std::atomic<size_t> read_idx_;
    alignas(CACHE_LINE_SIZE) std::atomic<size_t> ready_idx_;
    alignas(CACHE_LINE_SIZE) std::atomic<uint64_t> sequence_;
};

} // namespace turtlebot4

#endif // LOCKFREE_BUFFERS_HPP
