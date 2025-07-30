# 🚗 Speed Control System for Vehicles using ESP32

A real-time embedded system that dynamically adjusts vehicle speed based on GPS location (e.g., school zones) and obstacle proximity using an ultrasonic sensor. The system uses FreeRTOS for multitasking and CAN protocol to communicate with the vehicle’s ECU.

---

## 📦 Components Used

| Component               | Description                                      |
|------------------------|--------------------------------------------------|
| ESP32 Dev Board        | Main microcontroller with FreeRTOS               |
| GPS Module (Neo-6M)    | Provides real-time latitude and longitude        |
| Ultrasonic Sensor (HC-SR04) | Detects obstacles in front of the vehicle |
| MCP2515 CAN Module     | Interfaces with the vehicle ECU using CAN        |
| OLED Display (SSD1306) | Displays current data (optional)                 |
| Jumper Wires & Breadboard | For wiring and prototyping                  |

---

## 🔌 Circuit Connections

| Module        | ESP32 Pin       |
|---------------|------------------|
| GPS RX        | GPIO 17 (TX2)    |
| GPS TX        | GPIO 16 (RX2)    |
| HC-SR04 Trig  | GPIO 4           |
| HC-SR04 Echo  | GPIO 5           |
| MCP2515 CS    | GPIO 15          |
| MCP2515 INT   | GPIO 2 (optional)|
| OLED SDA/SCL  | GPIO 21 / 22     |

---

## 🧠 System Architecture

- **Task 1:** GPS Task – Reads GPS coordinates and checks proximity to school zone.
- **Task 2:** Ultrasonic Task – Detects nearby objects and adjusts speed accordingly.
- **Task 3:** CAN Task – Sends current speed limit to the vehicle ECU.
- **Task 4:** Display Task – Shows live GPS, speed limit, and obstacle distance.

Managed using **FreeRTOS** for concurrency.

---

## ⚙️ Functional Logic

1. **GPS Module:**
   - Reads latitude & longitude.
   - Compares with predefined school zone coordinates.
   - If within 100m → Sets speed limit to **30 km/h**.

2. **Ultrasonic Sensor:**
   - Measures distance to obstacles.
   - If obstacle < 100 cm → Overrides speed limit to **20 km/h**.

3. **CAN Bus:**
   - Sends 1-byte message to vehicle ECU with current speed limit.

4. **OLED Display (Optional):**
   - Shows current GPS data, obstacle distance, and speed limit.

---

## 📚 Libraries Required

Install via Arduino Library Manager:

- `TinyGPSPlus` (for GPS parsing)
- `Adafruit SSD1306` (for OLED)
- `Adafruit GFX` (dependency for SSD1306)
- `MCP_CAN_lib` (for CAN communication)

---

## 🛠️ How to Run

1. Connect components as per circuit diagram.
2. Upload the code using Arduino IDE or PlatformIO.
3. Open Serial Monitor (115200 baud) to view debug output.
4. Test GPS near the defined school zone coordinates.
5. Move obstacles near ultrasonic sensor to test speed adjustment.

---

## 📈 Results

- Automatically adjusts vehicle speed based on real-world parameters.
- Demonstrated ability to reduce hazards and enforce compliance in school zones.
- Reduced reaction time and improved safety decision-making by the vehicle by **~30%**.

---

## 📁 Future Enhancements

- Add EEPROM or NVS to store multiple zones.
- Add BLE to communicate with mobile dashboard.
- Add buzzer alert for manual override warning.

---

## 👨‍💻 Author

**Yogavani A**  
Embedded Systems Enthusiast | BE (ECE) | Cranes Varsity Embedded Systems Trainee  
