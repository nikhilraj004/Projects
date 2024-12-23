# Interactive Air Mouse for Virtual Displays

## Table of Contents
1. [Title](#1-title)
2. [Introduction](#2-introduction)
3. [Overview](#3-overview)
4. [Components Required with Bill of Materials](#4-components-required-with-bill-of-materials)
5. [Table for Pin Connections](#5-table-for-pin-connections)
6. [Pinout Diagram](#6-pinout-diagram)
7. [Working Code](#7-working-code)
8. [Demo Video](#8-demo-video)
9. [Acknowledgements](#9-acknowledgements)

## 1. Title
Interactive Air Mouse for Virtual Displays

## 2. Introduction
This project aims to develop an Interactive Air Pen that lets users control a cursor on virtual displays through hand movements. Utilizing an Arduino Nano and an MPU6050 integrated accelerometer and gyroscope, the system translates physical gestures into virtual interactions, eliminating the need for physical touch-based tools. This affordable and hardware-minimalist solution offers a seamless user experience for interacting with presentations, digital whiteboards, and other projected content.

## 3. Overview
Traditional input methods for virtual displays, such as touchpads and expensive smart screens, limit user interaction and require complex hardware integration. Users need a more intuitive, cost-effective solution for controlling cursors on virtual screens.

## 4. Components Required with Bill of Materials
- Arduino Nano: INR 340
- MPU6050 accelerometer and gyroscopic sensor: INR 160
- Zero Breadboard and wires: INR 140
- Push button: INR 10
- **Total Estimated Budget:** INR 650

## 5. Table for Pin Connections
| Component   | Arduino Nano Pin | Connection Details                          |
|-------------|-------------------|---------------------------------------------|
| MPU6050     |                   |                                             |
| - VCC       | 5V              | Power supply for the accelerometer          |
| - GND       | GND               | Ground                                      |
| - SDA       | GPIO21            | I2C Data (Connect to SDA pin of ADXL345)    |
| - SCL       | GPIO22            | I2C Clock (Connect to SCL pin of ADXL345)   |
| Push Button |                   |                                             |
| - One Side  | GPIO3             | Digital pin 2 (Connect to one terminal of the push button) |
| - Other Side| GND               | Ground (Connect to the other terminal of the push button)  |
| - One Side  | GPIO4             | Digital pin 2 (Connect to one terminal of the push button) |
| - Other Side| GND               | Ground (Connect to the other terminal of the push button)  |

## 6. Pinout Diagram

![image](https://github.com/user-attachments/assets/c17fd0a7-89e6-48a7-b60c-fa908fffacbc)


## 7. Working Code
```cpp
#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

const int leftButtonPin = 3;
const int rightButtonPin = 4;
const unsigned long longPressDuration = 1000; // 1 second in milliseconds

bool leftClickHeld = false;

unsigned long leftButtonPressTime = 0;
unsigned long rightButtonPressTime = 0;

float positionX = 0;
float positionY = 0;

int16_t ax_min = 32767, ax_max = -32768;
int16_t ay_min = 32767, ay_max = -32768;

void setup() {
  pinMode(leftButtonPin, INPUT_PULLUP);
  pinMode(rightButtonPin, INPUT_PULLUP);
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("No MPU6050 detected");
    while (1);
  }
}

void loop() {
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);

  // Update dynamic range for x and y axes
  if (ax < ax_min) ax_min = ax;
  if (ax > ax_max) ax_max = ax;
  if (ay < ay_min) ay_min = ay;
  if (ay > ay_max) ay_max = ay;

  // Convert raw accelerometer data to g
  float ax_g = ax / 16384.0;
  float ay_g = ay / 16384.0;
  float az_g = az / 16384.0;

  // Calibrate and map accelerometer data to cursor position
  positionX = map(ax, ax_min, ax_max, -5, 5); // Adjust these values
  positionY = map(ay, ay_min, ay_max, -5, 5); // Adjust these values

  Serial.print(positionX);
  Serial.print(",");
  Serial.println(positionY);

  // Check left button state
  if (digitalRead(leftButtonPin) == LOW) {
    if (leftButtonPressTime == 0) {
      leftButtonPressTime = millis();
    } else if (millis() - leftButtonPressTime > longPressDuration) {
      hold_release_left_click();
      leftButtonPressTime = 0;
      delay(200); // Debounce delay
    }
  } else if (leftButtonPressTime > 0) {
    if (millis() - leftButtonPressTime < longPressDuration) {
      click_mouse(1); // 1 represents left click
    }
    leftButtonPressTime = 0;
    delay(50); // Debounce delay
  }

  // Check right button state
  if (digitalRead(rightButtonPin) == LOW) {
    if (rightButtonPressTime == 0) {
      rightButtonPressTime = millis();
    } else if (millis() - rightButtonPressTime > longPressDuration) {
      click_mouse(2); // 2 represents right click
      rightButtonPressTime = 0;
      delay(200); // Debounce delay
    }
  } else if (rightButtonPressTime > 0) {
    if (millis() - rightButtonPressTime < longPressDuration) {
      click_mouse(2); // 2 represents right click
    }
    rightButtonPressTime = 0;
    delay(50); // Debounce delay
  }

  delay(50); // Adjust for smoother readings
}

void click_mouse(uint8_t button) {
  Serial.print("CLICK ");
  if (button == 1) {
    Serial.println("LEFT");
  } else if (button == 2) {
    Serial.println("RIGHT");
  }
}

void hold_release_left_click() {
  if (!leftClickHeld) {
    Serial.println("HOLD LEFT");
    leftClickHeld = true;
  } else {
    Serial.println("RELEASE LEFT");
    leftClickHeld = false;
  }
}

```

```python
import serial
import asyncio
from pynput.mouse import Button, Controller
import pygetwindow as gw

# Configure the serial port
ser = serial.Serial('COM3', 9600)  # Update 'COM3' to your actual serial port

# Calibration values to adjust sensitivity and range
position_calibration = 7  # Adjust this value for position sensitivity

# Smoothing factor for exponential smoothing (0 < alpha < 1)
smoothing_factor = 0.7

# Threshold for detecting significant changes
sudden_change_threshold = 5.0

mouse = Controller()
left_click_held = False

# Define the application window title
window_title = "Untitled - Paint"  # Replace this with your application's window title

# Initialize smoothed position values
smoothed_x = 0
smoothed_y = 0

def get_window_bounds(window_title):
    window = gw.getWindowsWithTitle(window_title)
    if window:
        win = window[0]
        return win.left, win.top, win.right, win.bottom
    return None

async def read_serial():
    global left_click_held, smoothed_x, smoothed_y
    while True:
        if ser.in_waiting > 0:
            try:
                data = ser.readline().decode('utf-8', errors='ignore').strip()
                if data == "CLICK LEFT":
                    click_mouse(Button.left)
                elif data == "CLICK RIGHT":
                    click_mouse(Button.right)
                elif data == "HOLD LEFT":
                    if not left_click_held and is_cursor_in_window():
                        hold_left_click()
                        left_click_held = True
                elif data == "RELEASE LEFT":
                    if left_click_held:
                        release_left_click()
                        left_click_held = False
                else:
                    try:
                        x, y = map(float, data.split(','))
                        smoothed_x = smooth_position(smoothed_x, x)
                        smoothed_y = smooth_position(smoothed_y, y)
                        move_cursor(smoothed_x, smoothed_y)
                    except ValueError:
                        # Handle data parsing error
                        continue
            except UnicodeDecodeError:
                # Handle decoding error
                continue
        await asyncio.sleep(0.01)  # Adjust for smoothness

def is_cursor_in_window():
    bounds = get_window_bounds(window_title)
    if bounds:
        left, top, right, bottom = bounds
        current_x, current_y = mouse.position
        return left <= current_x <= right and top <= current_y <= bottom
    return False

def move_cursor(x, y):
    current_x, current_y = mouse.position
    new_x = current_x + x * position_calibration
    new_y = current_y + y * position_calibration
    mouse.position = (new_x, new_y)

def smooth_position(smoothed_value, new_value):
    if abs(new_value - smoothed_value) > sudden_change_threshold:
        return new_value
    return smoothed_value + smoothing_factor * (new_value - smoothed_value)

def click_mouse(button):
    mouse.click(button, 1)

def hold_left_click():
    mouse.press(Button.left)

def release_left_click():
    mouse.release(Button.left)

async def main():
    await asyncio.gather(
        read_serial(),
        # Add other asyncio tasks here if needed
    )

try:
    asyncio.run(main())
except KeyboardInterrupt:
    if left_click_held:
        release_left_click()  # Release left click if held
    ser.close()
    print("Exiting...")


```

## 8. Demo Video

https://drive.google.com/file/d/152Atm3FGCmY0dMZv9dF8FAHpPrbdi3Mn/view?usp=sharing

## 9. Acknowledgements
This project is inspired by the need for innovative and cost-effective solutions for interacting with virtual displays—special thanks to the contributors and developers of the Arduino Nano and MPU6050 libraries.
# Interactive Air Mouse
