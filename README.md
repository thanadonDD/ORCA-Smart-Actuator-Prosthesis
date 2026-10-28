# 17-DOF Prosthetic Hand & Smart Actuator (ORCA-Based)

A compact, cost-effective dexterous prosthetic hand built upon the open-source **ETH ORCA Hand v1** architecture. This project features custom-designed smart actuators, closed-loop position/torque feedback, and distributed serial communication.

<!-- แปะ GIF สั้นๆ โชว์มือขยับหรือตอนเซอร์โวหมุนตอบสนอง -->
![Demo](docs/demo.gif)

---

## 🚀 Key Highlights
- **Actuation System:** Modified standard MG995/MG996R servo casing with custom internal electronics.
- **Microcontroller:** ATmega328PB / STM32 running bare-metal C/C++ control loops.
- **Magnetic Feedback:** 12-bit AS5600 absolute magnetic encoder via I2C for high-resolution joint angle tracking.
- **Motor Driver:** Integrated DRV8833 H-bridge supporting bidirectional PWM driving.
- **Communication:** Daisy-chained RS-485 / TTL UART supporting multi-actuator bus control.

---

## 🛠️ Hardware & PCB Design

| 3D Render / Layout | Assembled Board |
| :---: | :---: |
| ![PCB Layout](docs/pcb_layout.jpg) | ![Assembled PCB](docs/pcb_assembled.jpg) |

- **Layer Count:** 2-layer compact circular/rectangular PCB fitting inside standard servo dimensions.
- **Design Considerations:** Dedicated power planes for inductive motor spikes, localized decoupling caps, and isolated analog/digital ground routing.
- **Schematic:** [View Schematic PDF](docs/schematic.pdf)

---

## 💻 Firmware & Control Architecture

The firmware implements a real-time closed-loop PID position tracking controller:

$$u(t) = K_p e(t) + K_i \int e(t)\,dt + K_d \frac{de(t)}{dt}$$

- **Sampling Rate:** 1 kHz position feedback loop.
- **Safety Features:** Software stall detection, thermal overcurrent thresholding, and soft-stop limits.

### Project Structure
- `/hardware`: KiCad schematic, layout, and manufacturing files.
- `/firmware`: Embedded C/C++ source code.
- `/mechanical`: 3D printable files for socket brackets and finger tendons.

---

## 📹 Video Demonstration
- Full assembly and motion test: [Watch on YouTube](https://youtu.be/xxxxxx)
