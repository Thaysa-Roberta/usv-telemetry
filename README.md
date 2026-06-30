# USV Telemetry — Sistema de Controle e Telemetria para USVs

Sistema de controle e telemetria para Veículos de Superfície Não Tripulados (USVs) desenvolvido como TCC I no [LSA/PUCRS](https://lsa.pucrs.br/).

A arquitetura utiliza **ROS 2 Humble** para aquisição e processamento de dados dos sensores embarcados (GPS u-blox M8N e IMU MPU-6050), **MAVLink sobre TCP** para transmissão de telemetria, e uma **GCS em PyQt6** para monitoramento e controle remoto.

---

## Estrutura do Repositório

```
usv-telemetry/
├── gcs/
│   ├── gcs.py            # Ground Control Station (PyQt6)
│   └── requirements.txt  # Dependências Python da GCS
└── ros2_ws/
    └── src/
        └── usv_telemetry/
            ├── package.xml
            ├── setup.py
            └── usv_telemetry/
                ├── gps_node.py            # Leitura do receptor GPS (u-blox M8N / NMEA)
                ├── imu_node.py            # Leitura da IMU MPU-6050 via I²C
                ├── mavlink_bridge_node.py # Ponte ROS 2 ↔ MAVLink/TCP
                └── control_node.py        # Tradução de Twist → PWM (motor + servo)
```

---

## Pré-requisitos

### Para rodar em simulação (Windows/Linux com Docker)

- [Docker Desktop](https://www.docker.com/products/docker-desktop/) (Windows com WSL2 ou Hyper-V)
- Python 3.10+ com pip

### Para rodar no hardware real

- Raspberry Pi 4 (4 GB) com Ubuntu 22.04 Server
- ROS 2 Humble instalado ([instruções oficiais](https://docs.ros.org/en/humble/Installation.html))

---

## Como Rodar — Simulação com Docker

### 1. Iniciar o contêiner ROS 2

```bash
docker run -it --rm \
  --name ros2_usv \
  -p 14550:14550 \
  -v "$(pwd)/ros2_ws:/tcc/ros2_ws" \
  ros:humble
```

> No Windows (PowerShell), substitua `$(pwd)` por `${PWD}`.

### 2. Dentro do contêiner — compilar o pacote

```bash
cd /tcc/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 3. Dentro do contêiner — iniciar os nós

```bash
ros2 run usv_telemetry gps_node &
ros2 run usv_telemetry imu_node &
ros2 run usv_telemetry mavlink_bridge_node &
ros2 run usv_telemetry control_node
```

Os quatro nós gerarão dados simulados e o `mavlink_bridge_node` ficará aguardando conexão TCP na porta **14550**.

---

## Como Rodar — GCS (Windows)

### 1. Instalar dependências

```bash
pip install -r gcs/requirements.txt
```

### 2. Iniciar a GCS

```bash
python gcs/gcs.py
```

A GCS se conectará automaticamente a `127.0.0.1:14550` (quando o contêiner Docker estiver rodando). Clique em **Conectar** na interface.

---

## Arquitetura

```
┌─────────────────────────────────────────────┐
│  Docker / Raspberry Pi 4                    │
│                                             │
│  gps_node ──────────────────────────────┐  │
│  imu_node ──► imu_filter_node           │  │
│                    │                    ▼  │
│              control_node     mavlink_bridge_node
│                    │                    │  │
└────────────────────│────────────────────│──┘
                     │  PWM               │ TCP :14550
                     ▼                    ▼
                  Atuadores          GCS (Windows)
               (Motor + Servo)      PyQt6 + Leaflet.js
```

### Nós ROS 2

| Nó | Tópico publicado | Tópico subscrito | Descrição |
|---|---|---|---|
| `gps_node` | `/fix` (`NavSatFix`) | — | Lê NMEA do u-blox M8N via serial |
| `imu_node` | `/imu/data_raw` (`Imu`) | — | Lê MPU-6050 via I²C |
| `imu_filter_node` | `/imu/data_filtered` (`Imu`) | `/imu/data_raw` | Filtro de Madgwick |
| `mavlink_bridge_node` | `/cmd_vel` (`Twist`) | `/fix`, `/imu/data_filtered` | Ponte MAVLink/TCP |
| `control_node` | — | `/cmd_vel` | Gera sinais PWM para ESC e servo |

### Mensagens MAVLink

| Mensagem | Direção | Conteúdo |
|---|---|---|
| `GLOBAL_POSITION_INT` | USV → GCS | Posição GPS (lat/lon/alt) |
| `ATTITUDE` | USV → GCS | Roll, Pitch, Yaw |
| `HEARTBEAT` | USV → GCS | Status de conexão (1 Hz) |
| `RC_CHANNELS_OVERRIDE` | GCS → USV | Comandos de velocidade e direção |

---

## Hardware Alvo

| Componente | Especificação |
|---|---|
| Processador | Raspberry Pi 4 — 4 GB RAM |
| GPS | u-blox M8N (UART/USB, protocolo NMEA) |
| IMU | MPU-6050 (I²C, endereço 0x68) |
| Controlador de voo | Cube Orange (ArduRover) |
| ESC | 80 A (motor brushless) |
| Servomotor de direção | Sinal PWM 1000–2000 µs |

---

## Dependências Python (GCS)

```
PyQt6>=6.4
PyQt6-WebEngine>=6.4
pyqtgraph>=0.13
pymavlink>=2.4
```

---

## Autora

**Thaysa Roberta da Silva**  
Ciência da Computação — PUCRS  
Orientador: Prof. Anderson Roberto Pinheiro Domingues  
Laboratório de Sistemas Autônomos (LSA/PUCRS)

---

## Licença

MIT License — veja [LICENSE](LICENSE) para detalhes.
