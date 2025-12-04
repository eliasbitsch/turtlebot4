typedef unsigned char  u8;
typedef short          s16;
typedef unsigned int   u32;
typedef int            s32;
typedef unsigned long long u64;

struct js_event { u32 time; s16 value; u8 type; u8 number; };
int is_axis(u8 t) { return t & 2; }

struct SharedCmdVel {
    double linear_x;
    double linear_y;
    double linear_z;
    double angular_x;
    double angular_y;
    double angular_z;

    s32 timestamp_sec;
    u32 timestamp_nanosec;
    u64 sequence;
    bool new_command;

    SharedCmdVel() { char* p = (char*)this; for (int i = 0; i < (int)sizeof(*this); i++) p[i] = 0; }
};

// Systemaufrufe
extern "C" int open(const char*, int);
extern "C" int read(int, void*, unsigned int);
extern "C" int printf(const char*, ...);
extern "C" int fflush(void*);
extern "C" int shm_open(const char*, int, int);
extern "C" void* mmap(void*, unsigned long, int, int, int, long);
extern "C" int clock_gettime(int, void*);

// Konstanten
#define PROT_READ  1
#define PROT_WRITE 2
#define MAP_SHARED 1
#define CLOCK_REALTIME 0

int main() {
    int fd = open("/dev/input/js0", 0);
    if (fd < 0) return 1;

    int shm_fd = shm_open("/turtle_cmd_vel", 2, 0666);
    if (shm_fd < 0) return 1;
    SharedCmdVel* shm = (SharedCmdVel*)mmap(0, sizeof(SharedCmdVel), PROT_READ|PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (!shm) return 1;

    s16 axis[8] = {0};
    const float max_v = 0.4f;
    const float max_w = 2.0f;
    const float norm = 32767.0f;
    const int deadzone = 3000;
    u64 seq = 1;

    js_event e;
    struct { long tv_sec; long tv_nsec; } ts;

    while (1) {
        if (read(fd, &e, sizeof(js_event)) != sizeof(js_event)) continue;
        if (e.type & 0x80) continue;

        if (is_axis(e.type)) {
            axis[e.number] = e.value;

            float raw_v = axis[1]; // linker Stick vertikal
            float raw_w = axis[0]; // linker Stick horizontal
            if (raw_v < deadzone && raw_v > -deadzone) raw_v = 0;
            if (raw_w < deadzone && raw_w > -deadzone) raw_w = 0;

            float v = -(raw_v / norm) * max_v;
            float w =  (raw_w / norm) * max_w;

            // Shared Memory füllen
            shm->linear_x  = v;
            shm->linear_y  = 0;
            shm->linear_z  = 0;
            shm->angular_x = 0;
            shm->angular_y = 0;
            shm->angular_z = w;

            clock_gettime(CLOCK_REALTIME, &ts);
            shm->timestamp_sec = ts.tv_sec;
            shm->timestamp_nanosec = ts.tv_nsec;
            shm->sequence = seq++;
            shm->new_command = true;

            printf("v=%.3f w=%.3f seq=%llu\n", v, w, seq);
            fflush(0);
        }
    }
}
