"""
imu_node.py
-----------
Nó ROS 2 responsável pela leitura da IMU e publicação dos dados
de orientação e aceleração no tópico /imu/data.

Em modo SIMULADO (padrão), publica dados com pequena variação
aleatória para testes sem hardware.
Em modo REAL, lê o MPU-6050 via I2C usando a biblioteca smbus2.

Uso:
    ros2 run usv_telemetry imu_node
    ros2 run usv_telemetry imu_node --ros-args -p simulated:=false
"""

import math
import random

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3


# Endereço I2C padrão do MPU-6050
MPU6050_ADDR        = 0x68

# Registradores do MPU-6050
REG_PWR_MGMT_1      = 0x6B
REG_ACCEL_XOUT_H    = 0x3B
REG_GYRO_XOUT_H     = 0x43

# Fatores de escala padrão (±2g / ±250°/s)
ACCEL_SCALE         = 16384.0   # LSB/g
GYRO_SCALE          = 131.0     # LSB/(°/s)
G_MS2               = 9.80665   # m/s²
DEG_TO_RAD          = math.pi / 180.0


class ImuNode(Node):
    """Publica dados brutos da IMU no tópico /imu/data."""

    def __init__(self):
        super().__init__('imu_node')

        # ------------------------------------------------------------------
        # Parâmetros configuráveis
        # ------------------------------------------------------------------
        self.declare_parameter('simulated', True)
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', MPU6050_ADDR)
        self.declare_parameter('publish_rate_hz', 50.0)

        self.simulated   = self.get_parameter('simulated').value
        self.i2c_bus     = self.get_parameter('i2c_bus').value
        self.i2c_addr    = self.get_parameter('i2c_address').value
        rate_hz          = self.get_parameter('publish_rate_hz').value

        # ------------------------------------------------------------------
        # Publisher
        # ------------------------------------------------------------------
        self.publisher_ = self.create_publisher(Imu, '/imu/data', 10)

        # ------------------------------------------------------------------
        # Timer
        # ------------------------------------------------------------------
        self.timer = self.create_timer(1.0 / rate_hz, self.publish_imu)

        # Estado interno para simulação de deriva lenta
        self._sim_roll  = 0.0
        self._sim_pitch = 0.0
        self._sim_yaw   = 0.0

        # ------------------------------------------------------------------
        # Inicialização do modo real (I2C)
        # ------------------------------------------------------------------
        self.bus = None
        if not self.simulated:
            self._init_i2c()

        mode = 'SIMULADO' if self.simulated else f'REAL (I2C bus {self.i2c_bus}, addr 0x{self.i2c_addr:02X})'
        self.get_logger().info(f'imu_node iniciado em modo {mode} a {rate_hz} Hz')

    # ------------------------------------------------------------------
    def _init_i2c(self):
        """Inicializa o barramento I2C e acorda o MPU-6050."""
        try:
            import smbus2
            self.bus = smbus2.SMBus(self.i2c_bus)
            # Desativa o modo sleep do MPU-6050
            self.bus.write_byte_data(self.i2c_addr, REG_PWR_MGMT_1, 0x00)
            self.get_logger().info('MPU-6050 inicializado via I2C')
        except Exception as e:
            self.get_logger().error(
                f'Falha ao inicializar I2C: {e}. Alternando para modo simulado.'
            )
            self.simulated = True

    # ------------------------------------------------------------------
    def publish_imu(self):
        """Callback do timer: lê ou simula dados IMU e publica a mensagem."""
        msg = Imu()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        if self.simulated:
            self._fill_simulated(msg)
        else:
            self._fill_real(msg)

        self.publisher_.publish(msg)

    # ------------------------------------------------------------------
    def _fill_simulated(self, msg: Imu):
        """Preenche a mensagem com dados simulados (veículo parado + ruído)."""
        # Deriva angular lenta para simular movimento
        self._sim_yaw   += random.gauss(0, 0.001)
        self._sim_pitch += random.gauss(0, 0.0005)
        self._sim_roll  += random.gauss(0, 0.0005)

        # Converte ângulos de Euler para quaternion
        roll, pitch, yaw = self._sim_roll, self._sim_pitch, self._sim_yaw
        cy, sy = math.cos(yaw * 0.5),   math.sin(yaw * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cr, sr = math.cos(roll * 0.5),  math.sin(roll * 0.5)

        msg.orientation.w = cr * cp * cy + sr * sp * sy
        msg.orientation.x = sr * cp * cy - cr * sp * sy
        msg.orientation.y = cr * sp * cy + sr * cp * sy
        msg.orientation.z = cr * cp * sy - sr * sp * cy

        # Covariância de orientação (diagonal ~0.01 rad²)
        msg.orientation_covariance = [
            0.01, 0.0,  0.0,
            0.0,  0.01, 0.0,
            0.0,  0.0,  0.01
        ]

        # Velocidade angular (girômetro) com ruído
        msg.angular_velocity.x = random.gauss(0.0, 0.002)
        msg.angular_velocity.y = random.gauss(0.0, 0.002)
        msg.angular_velocity.z = random.gauss(0.0, 0.002)
        msg.angular_velocity_covariance = [
            0.0004, 0.0,    0.0,
            0.0,    0.0004, 0.0,
            0.0,    0.0,    0.0004
        ]

        # Aceleração linear: gravitação + ruído (veículo parado)
        msg.linear_acceleration.x = random.gauss(0.0,    0.05)
        msg.linear_acceleration.y = random.gauss(0.0,    0.05)
        msg.linear_acceleration.z = random.gauss(G_MS2,  0.05)
        msg.linear_acceleration_covariance = [
            0.0025, 0.0,    0.0,
            0.0,    0.0025, 0.0,
            0.0,    0.0,    0.0025
        ]

    # ------------------------------------------------------------------
    def _fill_real(self, msg: Imu):
        """Lê registradores do MPU-6050 via I2C e preenche a mensagem."""
        try:
            # Lê 14 bytes a partir do registrador de aceleração
            raw = self.bus.read_i2c_block_data(self.i2c_addr, REG_ACCEL_XOUT_H, 14)

            def to_int16(high, low):
                val = (high << 8) | low
                return val - 65536 if val >= 32768 else val

            ax_raw = to_int16(raw[0],  raw[1])
            ay_raw = to_int16(raw[2],  raw[3])
            az_raw = to_int16(raw[4],  raw[5])
            gx_raw = to_int16(raw[8],  raw[9])
            gy_raw = to_int16(raw[10], raw[11])
            gz_raw = to_int16(raw[12], raw[13])

            # Converte para unidades físicas
            ax = (ax_raw / ACCEL_SCALE) * G_MS2
            ay = (ay_raw / ACCEL_SCALE) * G_MS2
            az = (az_raw / ACCEL_SCALE) * G_MS2
            gx = (gx_raw / GYRO_SCALE) * DEG_TO_RAD
            gy = (gy_raw / GYRO_SCALE) * DEG_TO_RAD
            gz = (gz_raw / GYRO_SCALE) * DEG_TO_RAD

            msg.linear_acceleration  = Vector3(x=ax, y=ay, z=az)
            msg.angular_velocity     = Vector3(x=gx, y=gy, z=gz)

            # Covariâncias especificadas pelo fabricante (MPU-6050 datasheet)
            msg.linear_acceleration_covariance = [
                0.0025, 0.0,    0.0,
                0.0,    0.0025, 0.0,
                0.0,    0.0,    0.0025
            ]
            msg.angular_velocity_covariance = [
                0.0004, 0.0,    0.0,
                0.0,    0.0004, 0.0,
                0.0,    0.0,    0.0004
            ]
            # Orientação não calculada aqui — use imu_filter_madgwick
            msg.orientation_covariance[0] = -1.0  # indica "não disponível"

        except Exception as e:
            self.get_logger().warn(f'Erro na leitura I2C: {e}')


# ------------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
