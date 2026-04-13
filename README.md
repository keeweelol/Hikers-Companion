# Hiker’s Companion

## Overview
Hiker’s Companion is a portable hiking safety device designed for use in areas with little or no cellular coverage. It combines GPS tracking with LoRa communication to allow users to send location data and emergency SOS signals to nearby devices or gateways.

The goal is to provide a **low-cost (<$100), reliable, and energy-efficient** safety solution for recreational hikers.

---

## Features
- 📍 GPS location tracking (≈10 m accuracy)
- 🚨 SOS emergency signaling
- 📡 Long-range LoRa communication (no cellular needed)
- 🔋 24-hour battery life target
- 📱 Bluetooth setup via mobile device
- 🖥️ LCD status display
- 🧭 Real-time location updates for responders

---

## System Architecture

### Main Components
- **ESP32 Microcontroller**
  - Controls system logic and communication
- **GPS Module**
  - Provides real-time coordinates
- **LoRa Module (SX1262)**
  - Handles long-range wireless transmission
- **LCD Display**
  - Shows device status (sending, delivered, etc.)
- **Power System**
  - 3.7V battery → 3.3V regulated supply

---

## How It Works
1. User powers on the device  
2. GPS continuously collects location data  
3. System checks for button input  
4. When pressed:
   - SOS message + GPS data is transmitted via LoRa  
5. Gateway forwards message to emergency contacts  
6. Signal repeats for tracking movement  

---

## Design Constraints
- Cost: **< $100**
- Battery life: **≥ 24 hours**
- Range: **~1 km off-trail communication**
- Temperature: **-18°C to 38°C**
- Durability:
  - Drop resistant (1.5 m)
  - IP55 (dust + water resistant)

---

## Communication Choice
LoRa was selected over alternatives like NB-IoT, Satellite (Iridium), and Mioty due to:

- ✅ Low power consumption  
- ✅ Long range  
- ✅ Low cost  
- ❌ Requires mesh/gateway infrastructure  

---

## Power Analysis
- Estimated average current: **~46.93 mA**
- Daily consumption: **~1.126 Ah**
- Battery capacity: **3 Ah**

➡️ Device can comfortably last **24+ hours** on a single charge.

---

## Current Progress
- ✅ System design finalized  
- ✅ Component selection complete (ESP32-based boards)  
- ✅ Communication method chosen (LoRa)  
- ✅ Initial power calculations completed  

---

## Next Steps
- 🔧 Prototype development (Spring 2026)
- 🔌 Hardware integration and testing
- 🔋 Real-world power validation
- 🧱 Final enclosure design
- 🚀 Full system validation (Fall 2026)

---

## Team
- Ky’Juan Branch  
- Zavier Lowe  
- Kenneth Patrick  
- Taylor Snow  

North Carolina A&T State University  
ECEN 478 – Senior Design Project I  

---

## Summary
Hiker’s Companion aims to improve safety in remote hiking environments by providing a reliable, low-cost emergency communication system that works where traditional networks fail.

---
