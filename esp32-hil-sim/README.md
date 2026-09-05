# ESP32 Hardware-in-the-Loop (HIL) Simulation

A Hardware-in-the-Loop (HIL) robotics simulation testbed connecting a physical ESP32 microcontroller with a 2D differential-drive robot simulator built in Pygame.

## Architecture Overview

```
+-------------------------------------------------------------+
|                     Pygame Simulator                        |
|                      (sim/robot.py)                         |
|  - Differential drive robot kinematics (x, y, theta)        |
|  - Forward distance sensor (raycasting)                     |
|  - Interactive waypoint selection & obstacle placement      |
+-------------------------------------------------------------+
                            |   ^
   Sensor Telemetry (Serial)|   | Actuator Commands (Serial)
   "TELEM,x,y,theta,..."    |   | "CMD,v_left,v_right\n"
   (115200 Baud)            v   |
+-------------------------------------------------------------+
|                      ESP32 Controller                       |
|                       (src/main.cpp)                        |
|  - Hardware UART Serial                                     |
|  - Waypoint tracking PID controller                         |
|  - Reactive obstacle avoidance                              |
|  - Failsafe watchdog timer (1000 ms timeout)                |
+-------------------------------------------------------------+
```

## Directory Structure

```
esp32-hil-sim/
├── platformio.ini       # PlatformIO ESP32 configuration
├── requirements.txt     # Python dependencies (pygame, pyserial)
├── README.md            # Documentation & setup guide
├── src/
│   └── main.cpp         # ESP32 C++ firmware (Arduino/PlatformIO)
└── sim/
    └── robot.py         # Pygame robot simulation & serial bridge
```

## Getting Started

### 1. Install Python Dependencies

```powershell
pip install -r requirements.txt
```

### 2. Microcontroller Setup (`src/main.cpp`)

You can flash the microcontroller using either **PlatformIO** or the **Arduino IDE**:

#### Option A: PlatformIO (Recommended)
```powershell
pio run -t upload
```

#### Option B: Arduino IDE
1. Open the Arduino IDE.
2. Select **ESP32 Dev Module** under Tools -> Board.
3. Open `src/main.cpp`.
4. Select your ESP32's COM port and click **Upload**.

### 3. Run the Simulation

#### Hardware Mode (ESP32 Connected via USB)
```powershell
# Auto-detects serial port:
python sim/robot.py

# Or specify your COM port explicitly:
python sim/robot.py --port COM3 --baud 115200
```

#### Wokwi Simulation Mode (Virtual ESP32 via RFC2217)
1. In VS Code, open the Command Palette (`F1` or `Ctrl+Shift+P`) and select:
   **`Wokwi: Start Simulator`**
2. In a terminal, run the Pygame robot simulator connected to Wokwi:
```powershell
python sim/robot.py --wokwi
```

#### Mock Mode (Software-in-the-Loop without hardware)
If your ESP32 is not plugged in, you can test the simulation directly with the built-in mock controller:
```powershell
python sim/robot.py --mock
```

## Simulator Controls

- **Left Mouse Click**: Set a new target waypoint for the robot.
- **Right Mouse Click** or **'O' Key**: Place a circular obstacle at the cursor.
- **'C' Key**: Clear all placed obstacles.
- **'R' Key**: Reset robot pose to origin and clear trajectory trail.
- **Close Window**: Gracefully disconnects and stops motors.

## Communication Protocol

- **Telemetry Packet (PC -> ESP32)**:
  ```text
  TELEM,<x>,<y>,<theta_radians>,<target_x>,<target_y>,<distance_front>\n
  ```
- **Actuator Command Packet (ESP32 -> PC)**:
  ```text
  CMD,<v_left>,<v_right>\n
  ```
- **Failsafe**: If the ESP32 does not receive a `TELEM` packet for 1000 ms, it automatically outputs `CMD,0.00,0.00` to prevent runaway behavior.

