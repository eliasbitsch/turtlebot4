#include <fcntl.h>      // O_RDONLY, O_RDWR
#include <sys/mman.h>   // mmap, MAP_FAILED
#include <time.h>       // clock_gettime
#include <unistd.h>     // read
#include <stdio.h>      // printf

typedef unsigned char  u8;
typedef short          s16;
typedef unsigned int   u32;
typedef int            s32;
typedef unsigned long long u64;

struct js_event {
    u32 time;
    s16 value;
    u8  type;
    u8  number;
};

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

    SharedCmdVel() {
        char* p = (char*)this;
        for (int i = 0; i < (int)sizeof(*this); i++) p[i] = 0;
    }
};

int main() {

    // -----------------------------
    // 1) Joystick öffnen
    // -----------------------------
    int fd = open("/dev/input/js0", O_RDONLY);
    if (fd < 0) {
        printf("ERROR: Kann /dev/input/js0 nicht öffnen!\n");
        return 1;
    }

    // -----------------------------
    // 2) Shared Memory öffnen
    // -----------------------------
    int shm_fd = shm_open("/cmd_vel", O_RDWR, 0666);
    if (shm_fd < 0) {
        printf("ERROR: shm_open(/cmd_vel) fehlgeschlagen!\n");
        return 1;
    }

    // -----------------------------
    // 3) Shared Memory mappen
    // -----------------------------
    void* p = mmap(
        NULL,
        sizeof(SharedCmdVel),
        PROT_READ | PROT_WRITE,
        MAP_SHARED,
        shm_fd,
        0
    );

    if (p == MAP_FAILED) {
        printf("ERROR: mmap() fehlgeschlagen!\n");
        return 1;
    }

    SharedCmdVel* shm = (SharedCmdVel*)p;

    // -----------------------------
    // 4) Variablen & Konstanten
    // -----------------------------
    s16 axis[8] = {0};
    const float max_v = 0.4f;
    const float max_w = 2.0f;
    const float norm = 32767.0f;
    const int deadzone = 3000;
    u64 seq = 1;

    js_event e;
    struct timespec ts;

    printf("xbox_teleop gestartet.\n");

    // -----------------------------
    // 5) Hauptloop
    // -----------------------------
    while (1) {

        int r = read(fd, &e, sizeof(js_event));
        if (r != sizeof(js_event)) continue;

        // Initialisierungsereignisse überspringen
        if (e.type & 0x80) continue;

        if (is_axis(e.type)) {

            axis[e.number] = e.value;

            float raw_v = axis[1]; // linker Stick vertikal
            float raw_w = axis[0]; // linker Stick horizontal

            // Deadzone
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

            shm->timestamp_sec     = ts.tv_sec;
            shm->timestamp_nanosec = ts.tv_nsec;
            shm->sequence          = seq++;
            shm->new_command       = true;

            printf("v=%.3f  w=%.3f  seq=%llu\n", v, w, shm->sequence);
            fflush(stdout);
        }
    }

    return 0;
}
