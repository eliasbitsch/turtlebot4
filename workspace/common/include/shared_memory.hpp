#ifndef SHARED_MEMORY_HPP
#define SHARED_MEMORY_HPP

#include <string>
#include <cstring>
#include <stdexcept>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>

namespace turtlebot4 {

// POSIX shared memory wrapper with mutex synchronization
template<typename T>
class SharedMemory {
public:
    struct SharedBlock {
        pthread_mutex_t mutex;
        T data;
    };

    SharedMemory(const std::string& name, bool create = false)
        : name_(name), fd_(-1), block_(nullptr), is_owner_(create) {

        if (create) {
            // Remove existing shared memory if it exists
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
            // Initialize mutex with process-shared attribute
            pthread_mutexattr_t attr;
            pthread_mutexattr_init(&attr);
            pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);
            pthread_mutex_init(&block_->mutex, &attr);
            pthread_mutexattr_destroy(&attr);

            // Zero-initialize data
            std::memset(&block_->data, 0, sizeof(T));
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

    void write(const T& data) {
        pthread_mutex_lock(&block_->mutex);
        block_->data = data;
        pthread_mutex_unlock(&block_->mutex);
    }

    T read() {
        pthread_mutex_lock(&block_->mutex);
        T data = block_->data;
        pthread_mutex_unlock(&block_->mutex);
        return data;
    }

    // Read with callback (avoids copy for large structs)
    template<typename Func>
    void read(Func&& func) {
        pthread_mutex_lock(&block_->mutex);
        func(block_->data);
        pthread_mutex_unlock(&block_->mutex);
    }

    // Write with callback (for partial updates)
    template<typename Func>
    void update(Func&& func) {
        pthread_mutex_lock(&block_->mutex);
        func(block_->data);
        pthread_mutex_unlock(&block_->mutex);
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
