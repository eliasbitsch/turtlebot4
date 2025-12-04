# Buffer-Strategien für TurtleBot4

## Übersicht

| Topic        | Strategie                   | Buffer-Typ            | Zero Copy | Vorteile / warum gewählt                                                               |
| ------------ | --------------------------- | --------------------- | --------- | -------------------------------------------------------------------------------------- |
| **/scan**    | SPSC Ring Buffer            | N Puffersätze         | ✅ Ja     | Hohe Frequenz (50 Hz), große Daten (8 KB), kein Frame-Drop, Lock-Free                  |
| **/odom**    | SeqLock                     | 1 Datensatz           | ✅ Ja     | Kleine Daten (144 B), viele Leser, lock-freies Lesen ohne Retry bei kleinen Daten     |
| **/cmd_vel** | Priority Double Buffer      | 2 SeqLocks            | ✅ Ja     | Zwei Writer (Nav + Teleop), Teleop-Priority für Safety-Override, Lock-Free            |
| **/map**     | Double Buffer + Dirty Flag  | 2 Puffersätze         | ✅ Ja     | Große Daten (4 MB), langsames Update, Reader blockiert nie, Swap ist O(1)             |
| **/bumper**  | Atomic Flag + Timestamp     | 1 Atomic<uint64>      | ✅ Ja     | 8 Bytes, Sicherheitskritisch, schnellstmögliche Reaktion, Debounce-Support            |

---

## Header-Datei

Alle Buffer sind implementiert in: `workspace/common/include/lockfree_buffers.hpp`

```cpp
#include "lockfree_buffers.hpp"

using namespace turtlebot4;

// Scan: Ring Buffer mit 8 Slots
SPSCRingBuffer<ParsedScan, 8> scan_buffer;

// Odom: SeqLock für lock-freies Lesen
SeqLock<SharedOdometry> odom_buffer;

// CmdVel: Priority Buffer mit Teleop-Override
PriorityDoubleBuffer<SharedCmdVel> cmd_vel_buffer;

// Map: Double Buffer mit Dirty Flag
DoubleBufferDirty<SharedMap> map_buffer;

// Bumper: Atomic Flag
AtomicBumper bumper;
```

---

# 1. SPSC Ring Buffer – /scan

### Warum Ring Buffer statt Triple Buffer?

| Aspekt          | Triple Buffer           | SPSC Ring Buffer        |
|-----------------|-------------------------|-------------------------|
| Frame-Drop      | Ja (überschreibt)       | Nein (bis Ring voll)    |
| Latenz          | Konstant                | Minimal variabel        |
| Speicher        | 3 × Datengröße          | N × Datengröße          |
| Burst-Handling  | Schlecht                | Gut (puffert N Frames)  |

Bei 50 Hz Scan-Rate und gelegentlichen Verarbeitungs-Spikes ist ein Ring Buffer mit 4-8 Slots ideal.

### Wie es funktioniert

```
Writer                              Reader
  │                                   │
  ▼                                   ▼
┌─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┐
│  0  │  1  │  2  │  3  │  4  │  5  │  6  │  7  │
└─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┘
       ▲                       ▲
      HEAD                    TAIL
       │                       │
       └── Writer schreibt hier
                               └── Reader liest hier
```

### Zero-Copy Usage

```cpp
SPSCRingBuffer<ParsedScan, 8> scan_buffer;

// === WRITER (Parser Thread) ===
void on_scan_parsed(const std::string& json) {
    // Zero-Copy: Direkt in Buffer-Slot schreiben
    ParsedScan* slot = scan_buffer.prepare_write();
    if (!slot) {
        // Buffer voll - Frame droppen oder warten
        return;
    }

    // Direkt in den Slot parsen (kein Kopieren!)
    parse_scan_into(json, *slot);

    scan_buffer.commit_write();
}

// === READER (Navigation Thread) ===
void process_scans() {
    // Zero-Copy: Pointer auf Daten im Buffer
    const ParsedScan* scan = scan_buffer.read();
    if (!scan) return;  // Kein neuer Scan

    // Direkt verarbeiten ohne Kopie
    update_costmap(*scan);

    scan_buffer.release_read();  // Slot freigeben
}
```

---

# 2. SeqLock – /odom

### Wie es funktioniert

```
Sequence: 0 (gerade = bereit)
          ↓
Writer:   seq++ → [WRITE DATA] → seq++
          1 (ungerade = schreibt)  2 (gerade = fertig)

Reader:   s1 = seq
          [READ DATA]
          s2 = seq
          if (s1 == s2 && s1 gerade) → Daten gültig
          else → Retry
```

### Warum SeqLock?

- **Viele Leser**: Navigation, Lokalisierung, Logging lesen gleichzeitig
- **Lock-Free Reads**: Reader blockieren nie den Writer
- **Kleine Daten**: 144 Bytes → Retry-Overhead vernachlässigbar

### Zero-Copy Usage

```cpp
SeqLock<SharedOdometry> odom_buffer;

// === WRITER (Parser Thread) ===
void on_odom_received(const SharedOdometry& odom) {
    odom_buffer.write(odom);
}

// === READER (Multiple Threads) ===
void navigation_loop() {
    SharedOdometry odom;

    // read_into vermeidet Return-Kopie
    odom_buffer.read_into(odom);

    // Oder mit Retry-Count für Debugging:
    if (!odom_buffer.try_read(odom)) {
        // Write in progress - nächsten Zyklus versuchen
        return;
    }

    plan_path(odom);
}
```

---

# 3. Priority Double Buffer – /cmd_vel

### Das Problem

Zwei unabhängige Quellen schreiben Bewegungsbefehle:
1. **Navigation**: Autonome Pfadplanung
2. **Teleop**: Manuelle Steuerung (Joystick/Keyboard)

Teleop muss **immer Priorität** haben (Safety Override).

### Wie es funktioniert

```
┌─────────────────┐     ┌─────────────────┐
│   Nav SeqLock   │     │ Teleop SeqLock  │
│   (Buffer A)    │     │   (Buffer B)    │
└────────┬────────┘     └────────┬────────┘
         │                       │
         │    ┌──────────────┐   │
         └────┤ Atomic Mode  ├───┘
              │  0=Nav       │
              │  1=Teleop    │
              └──────┬───────┘
                     │
                     ▼
              ┌──────────────┐
              │    Reader    │
              │ (Motor Ctrl) │
              └──────────────┘
```

### Zero-Copy Usage

```cpp
PriorityDoubleBuffer<SharedCmdVel> cmd_buffer;

// === NAVIGATION WRITER ===
void navigation_output(const SharedCmdVel& cmd) {
    cmd_buffer.write_nav(cmd);
}

// === TELEOP WRITER (auto-activates priority) ===
void teleop_callback(const SharedCmdVel& cmd) {
    cmd_buffer.write_teleop(cmd);  // Setzt automatisch TELEOP Mode
}

// === MODE CONTROL ===
void on_teleop_timeout() {
    // Nach 500ms ohne Teleop → zurück zu Navigation
    cmd_buffer.set_mode(PriorityDoubleBuffer<SharedCmdVel>::Mode::NAVIGATION);
}

// === READER (Motor Controller) ===
void motor_control_loop() {
    SharedCmdVel cmd;
    cmd_buffer.read_into(cmd);  // Zero-Copy

    send_to_motors(cmd);
}
```

---

# 4. Double Buffer + Dirty Flag – /map

### Warum Double Buffer?

- **Riesige Daten**: 1024×1024 = 1 MB+ pro Map
- **Langsame Updates**: Map ändert sich selten (1-5 Hz)
- **Lange Schreibzeit**: Map-Update kann 10-50ms dauern

### Wie es funktioniert

```
               WRITER                           READER
                 │                                │
                 ▼                                ▼
         ┌──────────────┐                ┌──────────────┐
         │ Back Buffer  │                │ Front Buffer │
         │  (writing)   │                │  (reading)   │
         └──────┬───────┘                └──────────────┘
                │
                │ swap_buffers()
                ▼
         ┌──────────────┐
         │ Dirty Flag   │ ──→ Reader prüft has_new_data()
         │   = true     │
         └──────────────┘
```

### Zero-Copy Usage

```cpp
DoubleBufferDirty<SharedMap> map_buffer;

// === WRITER (SLAM Thread) ===
void slam_update() {
    // Zero-Copy: Direkt in Back-Buffer schreiben
    SharedMap* back = map_buffer.back_buffer();

    // Map befüllen (kann lange dauern - blockiert Reader nicht!)
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            back->data[y * width + x] = compute_cell(x, y);
        }
    }
    back->sequence++;

    // Atomischer Swap - O(1), nur Index-Tausch
    map_buffer.swap_buffers();
}

// === READER (Navigation Thread) ===
void path_planning() {
    // Nur verarbeiten wenn neue Daten
    if (!map_buffer.has_new_data()) {
        return;  // Alte Map weiter nutzen
    }

    // Zero-Copy: Pointer auf Front-Buffer
    const SharedMap* map = map_buffer.front_buffer();

    // Map nutzen (Reader blockiert Writer nicht!)
    compute_path(*map);

    map_buffer.clear_dirty();
}
```

---

# 5. Atomic Flag + Timestamp – /bumper

### Das Problem

- Bumper ist **sicherheitskritisch**
- Reaktion muss **so schnell wie möglich** sein
- Im WebSocket-Kontext: Echte <1ms nicht möglich

### Realistische Latenz-Kette

```
Bumper → ROS2 → rosbridge → WebSocket → JSON Parse → Callback
         └─────────────── 5-50ms ───────────────────┘
```

### Was wir optimieren können

Die **Software-Reaktion** nach Empfang der Nachricht:
- Atomic Load: ~1 Nanosekunde
- Kein Lock, kein Mutex, kein Retry

### Usage

```cpp
AtomicBumper bumper;

// === CALLBACK (Parser Thread) ===
void on_bumper_message(bool is_pressed) {
    if (is_pressed) {
        bumper.trigger();  // Setzt Flag + Timestamp
    }
}

// === CONTROL LOOP (100Hz+) ===
void safety_check() {
    if (bumper.is_triggered()) {
        emergency_stop();

        // Optional: Nach 2 Sekunden zurücksetzen
        if (bumper.elapsed_ms() > 2000) {
            bumper.reset();
        }
    }
}

// === DEBOUNCE ===
void bumper_handling() {
    if (bumper.is_triggered()) {
        handle_collision();

        // Reset nur wenn mindestens 500ms vergangen
        bumper.reset_if_elapsed(500);
    }
}
```

---

# Vergleich: Alt vs. Neu

| Topic     | Alte Implementierung         | Neue Implementierung          | Verbesserung                    |
|-----------|------------------------------|-------------------------------|---------------------------------|
| /scan     | ThreadSafeQueue + Mutex      | SPSC Ring Buffer              | Lock-Free, Zero-Copy, kein Drop |
| /odom     | SharedMemory + pthread_mutex | SeqLock                       | Lock-Free Reads, Multi-Reader   |
| /cmd_vel  | SharedMemory + Mutex         | PriorityDoubleBuffer          | Priority-System, Lock-Free      |
| /map      | (nicht implementiert)        | DoubleBufferDirty             | Zero-Copy, Non-Blocking         |
| /bumper   | ThreadSafeQueue              | AtomicBumper                  | Minimal Latency, Timestamp      |

---

# Integration in bestehenden Code

### Parser Manager Update

```cpp
// parser_manager.hpp
#include "lockfree_buffers.hpp"

class ParserManager {
    // NEU: Lock-Free Buffer
    SPSCRingBuffer<ParsedScan, 8> scan_buffer_;
    SeqLock<SharedOdometry> odom_buffer_;
    AtomicBumper bumper_;

public:
    // Zero-Copy Zugriff
    SPSCRingBuffer<ParsedScan, 8>& scan_buffer() { return scan_buffer_; }
    SeqLock<SharedOdometry>& odom_buffer() { return odom_buffer_; }
    AtomicBumper& bumper() { return bumper_; }
};
```

### Message Router Update

```cpp
// message_router.hpp
#include "lockfree_buffers.hpp"

class MessageRouter {
    // NEU: Priority Buffer für cmd_vel
    PriorityDoubleBuffer<SharedCmdVel> cmd_vel_buffer_;

public:
    PriorityDoubleBuffer<SharedCmdVel>& cmd_vel() { return cmd_vel_buffer_; }
};
```

---

# Performance-Erwartungen

| Metrik                  | Mit Mutex          | Lock-Free          |
|-------------------------|--------------------|--------------------|
| Scan Read Latency       | 1-10 µs            | 10-100 ns          |
| Odom Read Latency       | 1-5 µs             | 10-50 ns           |
| CmdVel Write Contention | Möglich            | Unmöglich          |
| Map Update Blocking     | Reader wartet      | Reader läuft weiter|
| Bumper Check            | 100-500 ns         | 1-5 ns             |

**Hinweis**: Diese Werte sind Schätzungen. Tatsächliche Performance hängt von CPU, Cache, und Systemlast ab.
