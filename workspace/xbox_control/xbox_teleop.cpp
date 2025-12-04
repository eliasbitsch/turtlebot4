// Standalone Joystick -> Shared Memory Controller
// Writes to SeqLock-protected shared memory compatible with turtlebot4 bridge

typedef unsigned char  u8;
typedef unsigned short u16;
typedef unsigned int   u32;
typedef short          s16;
typedef int            s32;
typedef unsigned long  u64;

// --- Joystick Event ---
struct js_event {
    u32 time;
    s16 value;
    u8 type;
    u8 number;
};

int is_axis(u8 t) { return t & 2; }

// --- SharedCmdVel Struct (matches turtlebot4::SharedCmdVel) ---
struct SharedCmdVel {
    double linear_x;
    double linear_y;
    double linear_z;
    double angular_x;
    double angular_y;
    double angular_z;

    s32  timestamp_sec;
    u32  timestamp_nanosec;
    u64  sequence;
    bool new_command;
};

// --- SharedBlock with SeqLock (matches turtlebot4::SharedMemory<T>::SharedBlock) ---
struct SharedBlock {
    volatile u64 seqlock;  // SeqLock sequence counter (atomic)
    SharedCmdVel data;
};

// --- External syscalls ---
extern "C" int open(const char* path, int flags);
extern "C" int read(int fd, void* buf, unsigned int count);
extern "C" int printf(const char*, ...);
extern "C" int fflush(void*);
extern "C" int shm_open(const char* name, int oflag, int mode);
extern "C" int ftruncate(int fd, long length);
extern "C" void* mmap(void* addr, unsigned long length, int prot, int flags, int fd, long offset);
extern "C" int clock_gettime(int clk_id, void* ts);

// --- Constants ---
#define O_RDWR     2
#define O_CREAT    64
#define PROT_READ  1
#define PROT_WRITE 2
#define MAP_SHARED 1
#define CLOCK_REALTIME 0

// Memory barrier
inline void memory_fence() { __sync_synchronize(); }

int main() {
    // --- 1. Open Joystick ---
    int fd = open("/dev/input/js0", 0);
    if (fd < 0) {
        fd = open("/dev/input/js1", 0);
        if (fd < 0) {
            printf("Error: Cannot open joystick /dev/input/js0 or js1\n");
            return 1;
        }
        printf("Opened /dev/input/js1\n");
    } else {
        printf("Opened /dev/input/js0\n");
    }

    s16 axis[8] = {0};

    const float max_v = 0.4f;   // max linear velocity m/s
    const float max_w = 2.0f;   // max angular velocity rad/s
    const float norm  = 32767.0f;
    const int deadzone = 3000;

    // --- 2. Open Shared Memory (don't create, bridge creates it) ---
    int shm_fd = shm_open("/shm_cmd_vel", O_RDWR, 0666);
    if (shm_fd < 0) {
        printf("Error: Cannot open /shm_cmd_vel - is the bridge running?\n");
        return 1;
    }

    SharedBlock* block = (SharedBlock*)mmap(0, sizeof(SharedBlock), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (!block || block == (void*)-1) {
        printf("Error: mmap failed\n");
        return 1;
    }

    printf("Connected to Shared Memory: /shm_cmd_vel (SeqLock format)\n");
    printf("Controls: Left stick -> linear_x (up/down), angular_z (left/right)\n");
    printf("          max_v=%.2f m/s, max_w=%.2f rad/s, deadzone=%d\n\n", max_v, max_w, deadzone);

    u64 seq = 1;

    js_event e;
    while (1) {
        if (read(fd, &e, sizeof(js_event)) != sizeof(js_event)) continue;
        if (!is_axis(e.type)) continue;

        axis[e.number] = e.value;

        float raw_v = axis[1];  // left stick vertical
        float raw_w = axis[0];  // left stick horizontal

        // Apply deadzone
        if (raw_v < deadzone && raw_v > -deadzone) raw_v = 0;
        if (raw_w < deadzone && raw_w > -deadzone) raw_w = 0;

        float v = -(raw_v / norm) * max_v;  // invert: stick up = forward
        float w = -(raw_w / norm) * max_w;  // invert: stick left = turn left

        // --- 3. Write with SeqLock protocol ---
        // Increment sequence (odd = write in progress)
        block->seqlock++;
        memory_fence();

        // Write data
        block->data.linear_x  = v;
        block->data.linear_y  = 0;
        block->data.linear_z  = 0;
        block->data.angular_x = 0;
        block->data.angular_y = 0;
        block->data.angular_z = w;

        // Timestamp
        struct { long tv_sec; long tv_nsec; } ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        block->data.timestamp_sec = ts.tv_sec;
        block->data.timestamp_nanosec = ts.tv_nsec;

        block->data.sequence = seq++;
        block->data.new_command = true;

        memory_fence();
        // Increment sequence again (even = write complete)
        block->seqlock++;

        printf("\rv=%.3f w=%.3f seq=%lu    ", v, w, seq);
        fflush(0);
    }

    return 0;
}