# usv-telemetry

Código do TCC I (PUCRS — Ciência da Computação). O projeto implementa um sistema de controle e telemetria para um USV tipo airboat do LSA/PUCRS, usando ROS 2 Humble, MAVLink sobre TCP e uma GCS feita em PyQt6.

Durante o desenvolvimento, tudo rodou em Docker no Windows. A ideia é depois mover para o Raspberry Pi 4 embarcado no barco.

---

## O que tem aqui

```
├── gcs/
│   ├── gcs.py            # interface gráfica (PyQt6 + Leaflet.js)
│   └── requirements.txt
└── ros2_ws/src/usv_telemetry/
    ├── gps_node.py            # lê NMEA do u-blox M8N
    ├── imu_node.py            # lê MPU-6050 via I²C
    ├── mavlink_bridge_node.py # ponte ROS 2 ↔ MAVLink/TCP
    └── control_node.py        # converte Twist em PWM
```

---

## Rodando com Docker (simulação)

Testei no Windows 11 com Docker Desktop + WSL2.

### 1. Subir o contêiner

```bash
docker run -it --rm \
  --name ros2_usv \
  -p 14550:14550 \
  -v "${PWD}/ros2_ws:/tcc/ros2_ws" \
  ros:humble
```

### 2. Compilar o pacote

```bash
cd /tcc/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 3. Iniciar os nós

```bash
ros2 run usv_telemetry gps_node &
ros2 run usv_telemetry imu_node &
ros2 run usv_telemetry mavlink_bridge_node &
ros2 run usv_telemetry control_node
```

Os nós geram dados simulados e o `mavlink_bridge_node` fica escutando na porta **14550**.

---

## GCS (Windows)

```bash
pip install -r gcs/requirements.txt
python gcs/gcs.py
```

Conecta em `127.0.0.1:14550`. Clique em **Conectar** depois que o contêiner estiver rodando.

---

## Como funciona

| Nó | Publica | Assina | Função |
|---|---|---|---|
| `gps_node` | `/fix` | — | parseia NMEA do u-blox M8N |
| `imu_node` | `/imu/data_raw` | — | lê MPU-6050 via I²C |
| `imu_filter_node` | `/imu/data_filtered` | `/imu/data_raw` | filtro de Madgwick |
| `mavlink_bridge_node` | `/cmd_vel` | `/fix`, `/imu/data_filtered` | ponte MAVLink/TCP |
| `control_node` | — | `/cmd_vel` | gera PWM para ESC e servo |

A GCS recebe `GLOBAL_POSITION_INT` e `ATTITUDE` via MAVLink e mostra no mapa + gráficos. Os sliders geram `RC_CHANNELS_OVERRIDE`, que o bridge republica como `Twist` no `/cmd_vel`.

---

## Hardware (produção)

- Raspberry Pi 4 (4 GB)
- GPS u-blox M8N (UART)
- IMU MPU-6050 (I²C 0x68)
- Cube Orange com ArduRover
- ESC 80 A + motor brushless
- Servomotor de direção (PWM 1000–2000 µs)

---

## Autora

Thaysa Roberta da Silva — PUCRS / Ciência da Computação
