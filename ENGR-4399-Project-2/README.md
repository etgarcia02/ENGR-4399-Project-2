# ENGR 4399 Project 2: Smart Water Distribution and Pump Control for Greenhouse

Smart Water Distribution and Pump Control, a distributed and resilient cyber-physical system.
Three ESP32 nodes talk over ESP-NOW (no Wi-Fi network, API, or cloud). If a node goes quiet for 6 seconds,
the others stop the pump, close the valve, sound the buzzer, and name the affected zone on the serial dashboard.

Course: ENGR 4399 ST, Cyber-Physical and IoT Systems, Fall 2026
Group: Ethan Garcia; Esai Uribe
Wokwi links: Node A https://wokwi.com/projects/480534892117182465 | Node B https://wokwi.com/projects/480534953746211841 | Node C https://wokwi.com/projects/480535007096733697

## Nodes

| Node | Zone | Sensors | Outputs |
| --- | --- | --- | --- |
| A | Source and pump | Water sensor (GPIO 35) | L293D pump driver (GPIO 13, 14), ACK button (GPIO 27) |
| B | Distribution | Tank level (GPIO 34), leak (GPIO 35) | Servo valve (GPIO 18) |
| C | Gateway and monitor | DHT11 (GPIO 4) | Buzzer (GPIO 23), mute button (GPIO 27), serial dashboard |

## Folders

- `hardware/`: the sketches that run on the real boards (one per node).
- `wokwi/`: one Wokwi project per node. Wokwi cannot run ESP-NOW between boards, so the other two nodes are emulated.

## How to run

1. Install the Arduino ESP32 board package and the libraries: ArduinoJson (v7), ESP32Servo (Node B), DHT sensor library and Adafruit Unified Sensor (Node C).
2. Flash each board with its own sketch. Set the Serial Monitor to 115200 baud.
3. Node C prints the dashboard. In `node_c.ino`, set `SIM_NODE_A` to `false` when the real Node A is running.
4. To leave SAFE on Node A, press the ACK button or type `a` in the Serial Monitor.

## Message format

Each node sends a JSON heartbeat every 2 seconds:

```json
{"id":"B","zone":"Distribution","seq":42,"up":184,"ttl":2,"via":"B","lvl":24,"leak":0,"valve":"OPEN","st":"NORMAL","why":""}
```

## Notes

- No secrets are stored in this code. The Wi-Fi channel is a placeholder and encryption keys are not set in shared code.
- Node A is based on the partner's original sketch. Lines marked ADDED or CHANGED are the fail-safe additions.
