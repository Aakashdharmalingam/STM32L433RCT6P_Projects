# STM32L433RCT6P – Bare-Metal & HAL Peripheral Examples 🚀

This folder contains a curated collection of **STM32L433RCT6P peripheral examples** written in **bare-metal** (register-level) and **HAL-based** embedded C. The goal is to learn and demonstrate the inner workings of STM32 microcontrollers without relying heavily on abstraction layers.

🧠 **Focus**: Low-level register manipulation, hardware understanding, and peripheral integration using STM32CubeIDE and Nucleo-L433RC-P board.

---

## 🧩 Topics Covered

| No. | Folder Name        | Description |
|-----|--------------------|-------------|
| 1️⃣ | `code/adc`              | Read analog values using ADC (12-bit SAR) from thermistor or other sources |
| 2️⃣ | `code/adc_pwm`          | Control PWM duty cycle based on temperature read from ADC |
| 3️⃣ | `code/Dot Matrix ADC`   | Drive dot matrix display print the A to Z letters (bare-metal logic) |
| 4️⃣ | `code/dot_matrix_hal`   | Dot matrix display using STM32 HAL drivers |
| 5️⃣ | `code/dot_matrix_sepc`  | Access the specific address of display to print the letters (custom display format or characters) |
| 6️⃣ | `code/external interrupts` | Configure EXTI for push-button or signal input (bare-metal) |
| 7️⃣ | `code/i2c`              | I2C communication master setup to transmitte (register-level or HAL-based) |
| 8️⃣ | `code/i2c_Rec`          | I2C receiver implementation (master-slave setup) |
| 9️⃣ | `code/led-hal`          | LED blinking using HAL functions |
| 🔟 | `code/led_blink`         | LED blinking using register-level GPIO toggling |
| 1️⃣1️⃣ | `code/led_button`     | LED toggle using button input (GPIO + polling) |
| 1️⃣2️⃣ | `code/pwm`            | PWM signal generation using TIM2, controlling frequency and duty cycle |
| 1️⃣3️⃣ | `code/spi`            | SPI communication setup FULL - DUPLEX(e.g., dot matrix displays) |
| 1️⃣4️⃣ | `code/timer`          | Use of TIM2 for time delay and event generation |
| 1️⃣5️⃣ | `code/uart`           | UART transmit (polling or interrupt-based) |
| 1️⃣6️⃣ | `code/uart_rec`       | UART receive (polling, interrupt, or DMA based) |
| 1️⃣7️⃣ | `code/uart_timer`     | UART + Timer integration for time-based UART actions |
| 1️⃣8️⃣| `code/uart_hal_usb`     | UART + USB using HAL function to blink the LED |

---

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


