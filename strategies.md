
| TurtleBot-Datum      | Strategie                   | Warum?                                    |
| -------------------- | --------------------------- | ----------------------------------------- |
| **LaserScan**        | Triple Buffer               | Keine Wartezeiten beim Scannen oder Lesen |
| **Odometrie**        | SeqLock                     | Viele Leser, wenig beschrieben   |
| **Bewegungsbefehle** | Double Buffer + Atomic Mode | Zwei Quellen → sichere Umschaltung, keine race condition        |
| **Karte (/map)**     | Double Buffer + Dirty Flag  | Großes Update → Reader blockiert nie      |
| **Bumper**           | Interrupt + Atomic          | Sicherheitsreaktion <1ms                  |


###

# 🟢 **LaserScan – Triple Buffer**

### **Wie es funktioniert**

Es existieren drei Puffer, sodass der Laser neue Daten schreiben kann, während ein anderer Prozess gleichzeitig die alten Daten liest.

### **Warum gewählt**

Damit der TurtleBot **niemals warten muss** – weder beim Scannen noch beim Verarbeiten.

### **C++ Beispiel**

```cpp
struct LaserScan { float ranges[360]; };

class TripleBuffer {
public:
    LaserScan buffers[3];
    std::atomic<int> writeIndex{0};
    std::atomic<int> readIndex{1};

    LaserScan& getWriteBuffer() {
        int wi = writeIndex.load();
        return buffers[wi];
    }

    void publish() {
        int wi = writeIndex.load();
        int next = (wi + 1) % 3;
        readIndex.store(wi);
        writeIndex.store(next);
    }

    const LaserScan& getReadBuffer() const {
        return buffers[readIndex.load()];
    }
};
```

---

# 🔵 **Odometrie – SeqLock**

### **Wie es funktioniert**

Beim Schreiben erhöht der TurtleBot eine Sequenznummer, und Leser prüfen diese, um sicherzustellen, dass sie keine halbfertigen Daten erwischen.

### **Warum gewählt**

Viele Module lesen die Odometrie gleichzeitig → SeqLock ermöglicht **ultraschnelles und lock-freies Lesen**.

### **C++ Beispiel**

```cpp
struct Odom {
    double x, y, theta;
};

class SeqLock {
public:
    std::atomic<uint64_t> seq{0};
    Odom data;

    void write(const Odom& newOdom) {
        seq.fetch_add(1);        // ungerade: Schreiben beginnt
        data = newOdom;          // kritischer Abschnitt
        seq.fetch_add(1);        // gerade: fertig
    }

    Odom read() {
        Odom copy;
        uint64_t s1, s2;

        do {
            s1 = seq.load();
            copy = data;
            s2 = seq.load();
        } while (s1 != s2 || (s1 & 1)); // Wiederholen bei Inkonsistenz

        return copy;
    }
};
```

---

# 🔴 **Bewegungsbefehle – Double Buffer + Atomic Mode**

### **Wie es funktioniert**

Zwei Writer (Navigation + Teleop) schreiben jeweils in ihren eigenen Buffer.
Ein atomarer Modus entscheidet, welcher Buffer aktuell verwendet wird.

### **Warum gewählt**

Damit Befehle aus zwei Quellen niemals vermischt oder halb geschrieben werden.

### **C++ Beispiel**

```cpp
struct CmdVel { float v, w; };

class CmdVelSwitcher {
public:
    CmdVel bufferNav;
    CmdVel bufferTeleop;

    std::atomic<int> mode{0}; 
    // 0 = Navigation, 1 = Teleop

    void writeNav(const CmdVel& cmd) {
        bufferNav = cmd;
    }

    void writeTeleop(const CmdVel& cmd) {
        bufferTeleop = cmd;
    }

    void setMode(int newMode) {
        mode.store(newMode, std::memory_order_release);
    }

    CmdVel get() {
        if (mode.load()) return bufferTeleop;
        return bufferNav;
    }
};
```

---

# 🟣 **Karte (/map) – Double Buffer + Dirty Flag**

### **Wie es funktioniert**

Die neue Karte wird im Hintergrund in einen zweiten Buffer geschrieben und erst nach Fertigstellung über ein „Dirty Flag“ freigegeben.

### **Warum gewählt**

Kartendaten sind sehr groß → Reader dürfen während Schreiben **nicht blockieren**.

### **C++ Beispiel**

```cpp
struct Map { std::vector<int> grid; };

class MapBuffer {
public:
    Map frontBuffer;   // wird gelesen
    Map backBuffer;    // wird beschrieben
    std::atomic<bool> dirty{false};

    Map& getBackBuffer() {
        return backBuffer;
    }

    void publish() {
        frontBuffer = backBuffer; // große Kopie, aber Reader blockiert nicht
        dirty.store(false);
    }

    const Map& get() {
        return frontBuffer;
    }
};
```

---

# 🟡 **Bumper – Interrupt + Atomic**

### **Wie es funktioniert**

Ein Hardware-Interrupt setzt sofort ein atomisches Flag, das von allen Threads eingesehen werden kann.

### **Warum gewählt**

Weil eine Kollision **in unter 1 ms** erkannt werden muss, Polling aber zu langsam wäre.

### **C++ Beispiel**

```cpp
std::atomic<bool> bumperHit{false};

// Interrupt-Service-Routine
void bumperISR() {
    bumperHit.store(true, std::memory_order_release);
}

void controlLoop() {
    if (bumperHit.load(std::memory_order_acquire)) {
        stopRobot(); // sofort reagiert
    }
}
```

---

Wenn du möchtest, kann ich dir diese Markdown-Datei auch **als fertiges PDF, DOCX oder GitHub-README** generieren.
