typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned int u32;
typedef short s16;

struct js_event {
    u32 time;
    s16 value;
    u8 type;
    u8 number;
};

int is_button(u8 t) { return t & 1; }
int is_axis(u8 t)   { return t & 2; }

extern "C" int open(const char*, int);
extern "C" int read(int, void*, unsigned int);
extern "C" int printf(const char*, ...);

int main() {
    int fd = open("/dev/input/js0", 0); // O_RDONLY = 0
    if (fd < 0) return 1;

    js_event e;

    while (1) {
        int r = read(fd, &e, sizeof(js_event));
        if (r != sizeof(js_event)) continue;

        if (is_button(e.type)) {
            printf("button %d: %d\n", e.number, (e.value != 0));
        }
        else if (is_axis(e.type)) {
            printf("axis %d: %d\n", e.number, e.value);
        }
    }
    return 0;
}
