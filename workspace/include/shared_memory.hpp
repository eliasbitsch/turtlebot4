#ifndef SHARED_MEMORY_HPP
#define SHARED_MEMORY_HPP

#include <atomic>
#include <string>
#include <stdexcept>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

namespace turtlebot4 {

// ============================================================================
// LOCK-FREE SHARED MEMORY WITH SEQLOCK (Zero-Copy + IPC)
// ============================================================================
//
// Features:
// - Lock-Free: No mutex, readers never block writer
// - Zero-Copy: read() returns pointer, no copying
// - IPC-Safe: Works across processes via POSIX shared memory
// - Multi-Reader: Multiple processes can read simultaneously
//
// Usage:
//   Writer: shm.write(data);
//           shm.update([](T& d) { d.x = 5; });
//
//   Reader: shm.read([](const T& d) { use(d); });  // Zero-Copy
//           T copy = shm.read_copy();              // If copy needed

template<typename T>
class SharedMemory {
    static_assert(std::is_trivially_copyable_v<T>,
                  "SharedMemory requires trivially copyable types");
public:
    // Shared memory layout (process-shared)
    struct SharedBlock {
        std::atomic<uint64_t> sequence;  // SeqLock sequence counter
        T data;

        SharedBlock() : sequence(0), data{} {}
    };

    SharedMemory(const std::string& name, bool create = false)
        : name_(name), fd_(-1), block_(nullptr), is_owner_(create) {

        if (create) {
            shm_unlink(name_.c_str());
            fd_ = shm_open(name_.c_str(), O_CREAT | O_RDWR, 0666);
            if (fd_ == -1) {
                throw std::runtime_error("Failed to create shared memory: " + name_);
            }

            if (ftruncate(fd_, sizeof(SharedBlock)) == -1) {
                close(fd_);
                shm_unlink(name_.c_str());
                throw std::runtime_error("Failed to set shared memory size: " + name_);
            }
        } else {
            fd_ = shm_open(name_.c_str(), O_RDWR, 0666);
            if (fd_ == -1) {
                throw std::runtime_error("Failed to open shared memory: " + name_);
            }
        }

        block_ = static_cast<SharedBlock*>(
            mmap(nullptr, sizeof(SharedBlock), PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0)
        );

        if (block_ == MAP_FAILED) {
            close(fd_);
            if (create) shm_unlink(name_.c_str());
            throw std::runtime_error("Failed to map shared memory: " + name_);
        }

        if (create) {
            // Initialize sequence and data
            new (&block_->sequence) std::atomic<uint64_t>(0);
            new (&block_->data) T{};
        }
    }

    ~SharedMemory() {
        if (block_ != nullptr && block_ != MAP_FAILED) {
            munmap(block_, sizeof(SharedBlock));
        }
        if (fd_ != -1) {
            close(fd_);
        }
        if (is_owner_) {
            shm_unlink(name_.c_str());
        }
    }

    // Non-copyable
    SharedMemory(const SharedMemory&) = delete;
    SharedMemory& operator=(const SharedMemory&) = delete;

    // Move semantics
    SharedMemory(SharedMemory&& other) noexcept
        : name_(std::move(other.name_)), fd_(other.fd_),
          block_(other.block_), is_owner_(other.is_owner_) {
        other.fd_ = -1;
        other.block_ = nullptr;
        other.is_owner_ = false;
    }

    // === WRITER (Single writer only!) ===

    // Write complete data (Lock-Free)
    void write(const T& data) noexcept {
        block_->sequence.fetch_add(1, std::memory_order_release);  // Odd = writing
        std::atomic_thread_fence(std::memory_order_release);

        block_->data = data;

        std::atomic_thread_fence(std::memory_order_release);
        block_->sequence.fetch_add(1, std::memory_order_release);  // Even = done
    }

    // Update with callback (Lock-Free, for partial updates)
    template<typename Func>
    void update(Func&& func) noexcept {
        block_->sequence.fetch_add(1, std::memory_order_release);
        std::atomic_thread_fence(std::memory_order_release);

        func(block_->data);

        std::atomic_thread_fence(std::memory_order_release);
        block_->sequence.fetch_add(1, std::memory_order_release);
    }

    // === READER (Multiple readers safe, Lock-Free) ===

    // Zero-Copy read with callback (preferred!)
    // Callback receives const reference, no copying
    template<typename Func>
    void read(Func&& func) const noexcept {
        for (;;) {
            uint64_t seq1 = block_->sequence.load(std::memory_order_acquire);
            if (seq1 & 1) continue;  // Write in progress, retry

            std::atomic_thread_fence(std::memory_order_acquire);
            func(block_->data);  // Zero-Copy: direct access
            std::atomic_thread_fence(std::memory_order_acquire);

            if (block_->sequence.load(std::memory_order_acquire) == seq1) break;
        }
    }

    // Read with copy (when you need to keep the data)
    T read_copy() const noexcept {
        T result;
        for (;;) {
            uint64_t seq1 = block_->sequence.load(std::memory_order_acquire);
            if (seq1 & 1) continue;

            std::atomic_thread_fence(std::memory_order_acquire);
            result = block_->data;
            std::atomic_thread_fence(std::memory_order_acquire);

            if (block_->sequence.load(std::memory_order_acquire) == seq1) break;
        }
        return result;
    }

    // Try read once (returns false if write in progress)
    template<typename Func>
    bool try_read(Func&& func) const noexcept {
        uint64_t seq1 = block_->sequence.load(std::memory_order_acquire);
        if (seq1 & 1) return false;

        std::atomic_thread_fence(std::memory_order_acquire);
        func(block_->data);
        std::atomic_thread_fence(std::memory_order_acquire);

        uint64_t seq2 = block_->sequence.load(std::memory_order_acquire);
        return seq1 == seq2;
    }

    // Get current sequence (for change detection)
    uint64_t sequence() const noexcept {
        return block_->sequence.load(std::memory_order_acquire);
    }

    const std::string& name() const { return name_; }

private:
    std::string name_;
    int fd_;
    SharedBlock* block_;
    bool is_owner_;
};

} // namespace turtlebot4

#endif // SHARED_MEMORY_HPP
