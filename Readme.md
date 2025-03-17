# ESP32 Arduino Obstacles

## Introduction

ESP32 Arduino Obstacles is an exciting physics-based project designed for the **ESP32-S3R2** microcontroller. This project utilizes **TFT\_eSPI** library to render smooth, flicker-free animations using **Sprites**, ensuring a seamless graphical experience.

The main objective of this project is to simulate real-world physics, including gravity, obstacle detection, and avoidance, making it interactive and dynamic. The addition of a **gyroscope** makes it even more engaging, as movement can be influenced by external forces.

## Hardware & Software Requirements

### **Tested On:**

- **ESP32-S3R2** microcontroller

### **Dependencies:**

- **ESP32 Board Version:** 2.0.12 (Ensure compatibility with your TFT display)
- **TFT\_eSPI LCD Library:** v2.5.34
- **Arduino IDE** (Latest version recommended)

## Features

- **Smooth Animation**: Utilizes **Sprites** from the **TFT\_eSPI** library to eliminate flickering.
- **Physics Engine**: Implements basic gravity and damping effects.
- **Obstacle Detection & Avoidance**: Objects interact dynamically within the set boundaries.
- **Customizable Parameters**: Users can modify object properties such as size, number, and texture.
- **Gyroscope Integration**: Adds an extra layer of interaction and realism.

## Configuration & Controls

The following parameters in the code allow customization of object properties:

```cpp
const int maxObjects = 25;  // Maximum number of objects
myObject objects[maxObjects]; // Array to store objects

int objSizeMin = 8;
int objSizeMax = 8;
int objTextureMin = 0;
int objTextureMax = 1;

float dampingFactor = 1;  // Adjust as needed for damping effect
const float dampingFactorD = 0.90;
float mygravity = 0.0;  // Initial gravity (turned off)
float gravityStartTime = 5000; // Time (in milliseconds) to turn on gravity
unsigned long gravityChangeInterval = 5000; // Time interval to change gravity direction
unsigned long lastGravityChangeTime = 0;
bool gravityEnabled = false;
unsigned long startTime = 0;
float gravityG = 0.5;
float boundaryRadius = 120.0 - 1.0;
int backgroundColor = TFT_BLACK;
```

Users can modify these parameters to fine-tune the behavior of the physics simulation.

## Future Enhancements

- Improved collision detection
- Additional object textures and behaviors
- Enhanced gyroscope-based interactions
- Touchscreen support (if applicable)

## Getting Started

1. Install **Arduino IDE** and configure **ESP32 board version 2.0.12**.
2. Install the **TFT\_eSPI** library (v2.5.34).
3. Clone this repository and upload the code to your ESP32-S3R2 board.
4. Customize the parameters in the code as needed.
5. Enjoy the physics simulation!

## Demo Image

![Project Screenshot](https://github.com/mbbutt/Esp32_Arduino_Obsticals/tree/main/media/1742254307498.jpg)


[Watch Demo Video](https://github.com/mbbutt/Esp32_Arduino_Obsticals/tree/main/media/1742253668261.mp4)

## Contributing

This project is still in its **early stages**. Contributions, improvements, and suggestions are welcome!

## License

This project is open-source under the **MIT License**.

---

Feel free to update this **README** as the project evolves!

