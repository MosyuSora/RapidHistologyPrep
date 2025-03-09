# Rapid Histology Prep System

An Arduino-based system integrating precise temperature control (PID algorithm), stepper motor actuation, and automated fluid management.

## Hardware Components
- **Main Controller**: Arduino Nano
- **Temperature Sensor**: 10KΩ NTC Thermistor
- **Heating Module**: PWM-controlled heater
- **Actuation System**:
  - Self developed step-motor pump
- **Indicators**:
  - RGB LED (Red/Green/Blue)
  - Tactile control button
- **Power Supply**: 12V 3A DC

## Pin Configuration
| Component      | Pins    |
| -------------- | ------- |
| Inlet Stepper  | 4,5,6   |
| Outlet Stepper | 7,15,16 |
| Heater PWM     | 10      |
| Cooler PWM     | 11      |
| Thermistor     | A3      |
| Control Button | 2       |
| Blue LED       | 8       |
| Green LED      | 12      |
| Red LED        | 13      |

## Core Features
###   Temperature Control System
- PID Parameters: Kp=15, Ki=0.02, Kd=0.5
- Dual Temperature Setpoints: 40°C (fluid phase)/60°C (paraffin phase)
- 15-point median filtering for noise reduction
- Operational Phases:`Heat to 40°C → Fluid Cycles → Heat to 60°C → Maintain 20min → Cool to 40°C`

### Status Indicators

- **Green LED**: Power status (always on)
- **Red LED**:
  - Slow blink (1Hz): 40°C mode
  - Fast blink (5Hz): 60°C mode
- **Blue LED**: Temperature setpoint reached

## Operation Guide

1. Connect components per pin configuration
2. Green LED illuminates at power-on indicating readiness
3. Press button to initiate 40°C heating sequence
4. Automatic process flow:
   - 40°C achieved → 8-stage fluid handling
   - Fluid cycles completed → Heat to 60°C for 20min
   - Auto-cool to 40°C for standby

## Software Architecture

### Main Loop

├── Button polling
├── Temperature PID control
├── Fluid state machine
├── Real-time motor control
└── LED status update

### Core Classes:

- `MedianFilter`: Analog signal conditioning
- `PIDController`: Thermal regulation
- `stepMotor`: Stepper driver abstraction

### Flow Chart:

![Image](https://private-user-images.githubusercontent.com/42824664/420695755-0922a720-4b5a-43d4-99d2-0f9a3a8b227b.png?jwt=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3NDE1NDYwODIsIm5iZiI6MTc0MTU0NTc4MiwicGF0aCI6Ii80MjgyNDY2NC80MjA2OTU3NTUtMDkyMmE3MjAtNGI1YS00M2Q0LTk5ZDItMGY5YTNhOGIyMjdiLnBuZz9YLUFtei1BbGdvcml0aG09QVdTNC1ITUFDLVNIQTI1NiZYLUFtei1DcmVkZW50aWFsPUFLSUFWQ09EWUxTQTUzUFFLNFpBJTJGMjAyNTAzMDklMkZ1cy1lYXN0LTElMkZzMyUyRmF3czRfcmVxdWVzdCZYLUFtei1EYXRlPTIwMjUwMzA5VDE4NDMwMlomWC1BbXotRXhwaXJlcz0zMDAmWC1BbXotU2lnbmF0dXJlPWNkOGYxZDNhMTkyYmRmMzU2ZmUwZjdlMDdjODRmNmNkZTMyZTNhZDFkNjA2YjE3MzY5OGNlYjM2ZmViYzA1MjgmWC1BbXotU2lnbmVkSGVhZGVycz1ob3N0In0.lfazNF6d1S_Dou9dXL8pwee2thFvY0vDKspEp9guWRg)



## License

MIT License - Free for modification/use with original author attribution