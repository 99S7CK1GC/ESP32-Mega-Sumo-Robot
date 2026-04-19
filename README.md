# 🤖 ESP32 Mega Sumo Robot

An autonomous competitive sumo robot built on the ESP32, using ToF distance sensors to hunt opponents and IR line sensors to avoid falling off the dohyo (arena).

> 🥉 **3rd place** at [Roboko](https://www.instagram.com/ieee.enp/), a robotics competition organized by the **IEEE ENP Alger student club** at École Nationale Polytechnique d'Alger.

---

## 🧠 How It Works

The robot runs a simple priority loop every 30ms:

1. **Line detection** — if any IR sensor sees the white edge of the dohyo, immediately reverse and turn back in
2. **Opponent detection** — read three VL53L0X ToF sensors; attack if something is close enough
3. **Search pattern** — if no opponent is found, wobble forward and periodically sweep left/right to scan the arena

---

## 📦 Hardware

| Component | Quantity | Notes |
|---|---|---|
| ESP32 (DevKit v1 or similar) | 1 | Main controller |
| VL53L0X ToF distance sensor | 3 | Left / Center / Right — I2C |
| IR line sensor (digital) | 3 | Left / Center / Right |
| DC gear motors | 2 | One per side |
| Motor driver (L298N or TB6612) | 1 | Drives both motors via PWM |
| LiPo battery (7.4V 2S) | 1 | Main power supply |
| Robot chassis | 1 | Sumo-style, low and wide |
| Jumper wires + breadboard | — | |

---

## 🔌 Wiring

### VL53L0X Sensors (I2C)

| Signal | ESP32 Pin |
|---|---|
| SDA | GPIO 21 |
| SCL | GPIO 22 |
| XSHUT Left | GPIO 15 |
| XSHUT Center | GPIO 2 |
| XSHUT Right | GPIO 4 |

All three sensors share the same SDA/SCL bus. XSHUT pins are used to power each one up individually and assign it a unique I2C address before the next one is initialized.

| Sensor | I2C Address |
|---|---|
| Left | `0x30` |
| Center | `0x39` |
| Right | `0x32` |

### Motor Driver

| Signal | ESP32 Pin |
|---|---|
| Motor A IN1 | GPIO 12 |
| Motor A IN2 | GPIO 14 |
| Motor B IN1 | GPIO 27 |
| Motor B IN2 | GPIO 26 |

PWM is set to **2 kHz, 8-bit resolution** (0–255).

### IR Line Sensors

| Sensor | ESP32 Pin |
|---|---|
| Left | GPIO 32 |
| Center | GPIO 33 |
| Right | GPIO 25 |

Sensors output `LOW` when white line is detected.

---

## ⚙️ Software Setup

### Requirements

- [Arduino IDE](https://www.arduino.cc/en/software) or [PlatformIO](https://platformio.org/)
- ESP32 board support package
- [Adafruit VL53L0X library](https://github.com/adafruit/Adafruit_VL53L0X)

### Install Library (Arduino IDE)

Go to **Sketch → Include Library → Manage Libraries**, search for `Adafruit VL53L0X` and install it.

### Flash

1. Clone this repo
2. Open `sumo_robot.ino` in Arduino IDE
3. Select your ESP32 board and port
4. Upload

---

## 🎛️ Tuning

All the important values are defined at the top of the file as `#define` constants — no need to dig through the logic to adjust behavior.

```cpp
// How close the opponent needs to be for the robot to attack (mm)
#define ATTACK_THRESHOLD  800   // center sensor
#define SIDE_THRESHOLD    570   // left/right sensors

// Motor speeds (0–255)
#define SPEED_FWD  230
#define SPEED_TURN 180
#define SPEED_REV  195

// Search sweep timing
#define SEARCH_INTERVAL_MS 1800   // how often to do a sweep
#define SWEEP_DURATION_MS   450   // how long each sweep lasts
```

---

## 🔍 Sensor Filtering

Raw ToF readings can be noisy, so each sensor value is smoothed with an **Exponential Moving Average (EMA)**:

```
ema = ema * (1 - alpha) + new_reading * alpha
```

`alpha = 0.8` — fast response, light smoothing. Lower this if readings feel jittery in your setup.

---

## 🤼 Robot Behavior Summary

```
Boot → blink LED 3x → init sensors → wait for match

Loop:
  ├─ Line detected?  → reverse + turn back in   (highest priority)
  ├─ Center sensor?  → charge forward
  ├─ Left sensor?    → spin left, then charge
  ├─ Right sensor?   → spin right, then charge
  └─ Nothing found?  → wobble forward + periodic sweep
```

---

## 📁 Repo Structure

```
sumo_robot/
├── sumo_robot.ino    # Main Arduino sketch
└── README.md
```

---

## 📜 License

MIT — do whatever you want with it.
