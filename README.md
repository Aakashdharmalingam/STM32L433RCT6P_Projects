# STM32L433RCT6P – Bare-Metal & HAL Peripheral Examples With Projects🚀

This folder contains a curated collection of **STM32L433RCT6P peripheral examples** written in **bare-metal** (register-level) and **HAL-based** embedded C. The goal is to learn and demonstrate the inner workings of STM32 microcontrollers without relying heavily on abstraction layers and with heavily on abstraction layers.

🧠 **Focus**: Low-level register manipulation, hardware understanding, and peripheral integration with projects using STM32CubeIDE and Nucleo-L433RC-P board.

---

## 🧩 Topics Covered

| No. | Folder Name        | Description |
|-----|--------------------|-------------|
| 1️⃣ | `adc`              | Read analog values using ADC (12-bit SAR) from thermistor or other sources |
| 2️⃣ | `adc_pwm`          | Control PWM duty cycle based on temperature read from ADC |
| 3️⃣ | `Dot Matrix ADC`   | Drive dot matrix display print the A to Z letters (bare-metal logic) |
| 4️⃣ | `dot_matrix_hal`   | Dot matrix display using STM32 HAL drivers |
| 5️⃣ | `dot_matrix_sepc`  | Access the specific address of display to print the letters (custom display format or characters) |
| 6️⃣ | `external interrupts` | Configure EXTI for push-button or signal input (bare-metal) |
| 7️⃣ | `i2c`              | I2C communication master setup to transmitte (register-level or HAL-based) |
| 8️⃣ | `i2c_Rec`          | I2C receiver implementation (master-slave setup) |
| 9️⃣ | `led-hal`          | LED blinking using HAL functions |
| 🔟 | `led_blink`         | LED blinking using register-level GPIO toggling |
| 1️⃣1️⃣ | `led_button`     | LED toggle using button input (GPIO + polling) |
| 1️⃣2️⃣ | `pwm`            | PWM signal generation using TIM2, controlling frequency and duty cycle |
| 1️⃣3️⃣ | `spi`            | SPI communication setup FULL - DUPLEX(e.g., dot matrix displays) |
| 1️⃣4️⃣ | `timer`          | Use of TIM2 for time delay and event generation |
| 1️⃣5️⃣ | `uart`           | UART transmit (polling or interrupt-based) |
| 1️⃣6️⃣ | `uart_rec`       | UART receive (polling, interrupt, or DMA based) |
| 1️⃣7️⃣ | `uart_timer`     | UART + Timer integration for time-based UART actions |
| 1️⃣8️⃣ | `external_interrupt_hal`     | Configure EXTI for push-button or signal input to toggle LED (Hardware Abtraction Layer) |
| 1️⃣9️⃣ | `timer_hal`     | Use of TIM2 with Interrupt concept to toggle led by Hardware Abstraction Layer |
| 2️⃣0️⃣ | `pwm_timer_hal`     | PWM signal generation using TIM1 , controlling frequency and duty cycle CCR 50% to channel 1 by Hardware Abstraction Layer |
| 2️⃣1️⃣ | `uart_hal_usb`     | UART by ST-link USB and Recieved data Led_on or led_of to toggle LED again transmitte same data to check (HAL) |
| 2️⃣2️⃣ | `spi_hal`     | SPI1 is used to transmitte the data of 8bit by HAL |
| 2️⃣3️⃣ | `adc_hal`     | Read analog values using ADC (12-bit SAR) from thermistor by HAL |
| 2️⃣4️⃣ | `i2c_hal`     | HALF DUPLEX of I2C communication done by HAL |

---
## 🧩 Projects Covered

## 📁 Smart Access Control System with Display Logging — STM32 Project

After completing the basic concepts of STM32 — like **GPIO handling**, **UART communication**, **PWM generation**, **EXTERNAL Interrupt**, **ADC reading**, **SPI communication** and **I2C interfacing** — I applied these skills to build this integrated project.

This **Smart Access Control System** combines multiple STM32 peripherals into a real-world application.  
The system detects a person, checks temperature, controls a door using a servo motor, and logs access data into EEPROM.  
It also displays the access status on a **Dot Matrix Display** and transmits logs via **UART**.

This project demonstrates how individual STM32 basic concepts can come together in a practical embedded system.

---

### ✅ Block Daigram:
!(code/IMAGE/PROJECT-2.png)

---

### ✅ Features Covered from Basics:
- GPIO control for buzzer
- ADC reading with thermistor (Grove Temperature Sensor)
- PWM generation for servo motor control
- I2C EEPROM read/write operations (AT24C)
- UART communication for access logging
- Real-time interrupt handling for IR sensor (using EXTI)
- Display on Dot Matrix 8x8 by SPI

---

**✅ This project is added as part of my STM32 Basics Series to show practical implementation after learning the fundamentals.**

## 🔧 Development Environment

- **MCU**: STM32L433RCT6P  
- **Board**: Nucleo-L433RC-P  
- **IDE**: STM32CubeIDE  
- **Debugger**: ST-Link  
- **Language**: Embedded C (Bare-Metal + HAL)

---

## 🧪 Tools Used

- ✅ **Live Expressions** – Observe ADC or variable values in real-time  
- ✅ **Logic Analyzer** – Visualize PWM, SPI, UART, and I2C signals  
- ✅ **TTL converter** – Monitor UART TX/RX over USB
- ✅ **Dot matrix display** – Print the Characters
- ✅ **Temperature Sensors** – Monitor Analog Signal

---

## 📂 Folder Structure
code/
├── adc/
├── adc_pwm/
├── Dot Matrix ADC/
├── dot_matrix_hal/
├── dot_matrix_sepc/
├── external interrupts/
├── i2c/
├── i2c_Rec/
├── led-hal/
├── led_blink/
├── led_button/
├── pwm/
├── spi/
├── timer/
├── uart/
├── uart_rec/
├── uart_timer/
├── spi_hal/
├── uart_hal_usb/
├── pwm_timer_hal/
├── timer_hal/
├── external_interrupt_hal/
├── adc_hal/
├── i2c_hal/
├── Smart_Access_Control_System_Project/
README.md


---

## 📚 References

- [STM32L4 Reference Manual (RM0394)](https://www.st.com/resource/en/reference_manual/dm00151940.pdf)  
- [STM32L433RCT6P Datasheet](https://www.st.com/resource/en/datasheet/stm32l433rct6.pdf)  
- [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html)

---

## 👨‍💻 Author

**Aakash Dharmalingam**  
 ECE Student | Embedded Systems | IoT Developer  
🔗 [LinkedIn](https://www.linkedin.com/in/aakash-dharmalingam-6a1455248/)

---

## 📜 License

This repository is licensed under the [MIT License](../../LICENSE).  
Feel free to use, modify, and contribute to support the embedded systems learning community.


