"""
gps_node.py
-----------
Nó ROS 2 responsável pela leitura do receptor GPS e publicação
da posição geográfica no tópico /gps/fix.

Em modo SIMULADO (padrão), publica uma posição fixa com pequena
variação aleatória para testes sem hardware.
Em modo REAL, lê sentenças NMEA 0183 de uma porta serial (UART)
e faz o parsing com a biblioteca pynmea2.

Uso:
    ros2 run usv_telemetry gps_node
    ros2 run usv_telemetry gps_node --ros-args -p simulated:=false -p serial_port:=/dev/ttyUSB0
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus

import math
import random


class GpsNode(Node):
    """Publica dados de posição GPS no tópico /gps/fix."""

    def __init__(self):
        super().__init__('gps_node')

        # ------------------------------------------------------------------
        # Parâmetros configuráveis via linha de comando ou launch file
        # ------------------------------------------------------------------
        self.declare_parameter('simulated', True)
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 9600)
        self.declare_parameter('publish_rate_hz', 1.0)

        # Posição base para simulação (LSA/PUCRS – Porto Alegre)
        self.declare_parameter('sim_latitude',  -30.0595)
        self.declare_parameter('sim_longitude', -51.1732)
        self.declare_parameter('sim_altitude',   46.0)

        self.simulated    = self.get_parameter('simulated').value
        self.serial_port  = self.get_parameter('serial_port').value
        self.baud_rate    = self.get_parameter('baud_rate').value
        rate_hz           = self.get_parameter('publish_rate_hz').value
        self.sim_lat      = self.get_parameter('sim_latitude').value
        self.sim_lon      = self.get_parameter('sim_longitude').value
        self.sim_alt      = self.get_parameter('sim_altitude').value

        # ------------------------------------------------------------------
        # Publisher
        # ------------------------------------------------------------------
        self.publisher_ = self.create_publisher(NavSatFix, '/gps/fix', 10)

        # ------------------------------------------------------------------
        # Timer de publicação
        # ------------------------------------------------------------------
        period = 1.0 / rate_hz
        self.timer = self.create_timer(period, self.publish_gps)

        # ------------------------------------------------------------------
        # Inicialização do modo real (serial)
        # ------------------------------------------------------------------
        self.serial_conn = None
        if not self.simulated:
            self._init_serial()

        mode = 'SIMULADO' if self.simulated else f'REAL ({self.serial_port})'
        self.get_logger().info(f'gps_node iniciado em modo {mode} a {rate_hz} Hz')

    # ------------------------------------------------------------------
    def _init_serial(self):
        """Abre a conexão serial com o receptor GPS."""
        try:
            import serial  # pyserial
            self.serial_conn = serial.Serial(
                self.serial_port,
                self.baud_rate,
                timeout=1.0
            )
            self.get_logger().info(
                f'Porta serial {self.serial_port} aberta a {self.baud_rate} baud'
            )
        except Exception as e:
            self.get_logger().error(
                f'Falha ao abrir porta serial: {e}. Alternando para modo simulado.'
            )
            self.simulated = True

    # ------------------------------------------------------------------
    def publish_gps(self):
        """Callback do timer: lê ou simula dados GPS e publica a mensagem."""
        msg = NavSatFix()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps_link'

        if self.simulated:
            msg = self._simulated_fix(msg)
        else:
            msg = self._real_fix(msg)

        self.publisher_.publish(msg)
        self.get_logger().debug(
            f'GPS publicado: lat={msg.latitude:.6f}  lon={msg.longitude:.6f}  '
            f'alt={msg.altitude:.1f} m'
        )

    # ------------------------------------------------------------------
    def _simulated_fix(self, msg: NavSatFix) -> NavSatFix:
        """Gera uma posição simulada com pequena variação aleatória."""
        noise = 0.00005  # ~5 m de ruído
        msg.latitude  = self.sim_lat + random.gauss(0, noise)
        msg.longitude = self.sim_lon + random.gauss(0, noise)
        msg.altitude  = self.sim_alt + random.gauss(0, 0.5)

        msg.status.status  = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS

        # Covariância diagonal (variância ~2.5 m²)
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        cov = 2.5 ** 2
        msg.position_covariance = [cov, 0.0, 0.0,
                                   0.0, cov, 0.0,
                                   0.0, 0.0, cov * 4]
        return msg

    # ------------------------------------------------------------------
    def _real_fix(self, msg: NavSatFix) -> NavSatFix:
        """Lê uma sentença NMEA da porta serial e preenche a mensagem."""
        try:
            import pynmea2
            line = self.serial_conn.readline().decode('ascii', errors='replace').strip()
            if line.startswith('$GPGGA') or line.startswith('$GNGGA'):
                parsed = pynmea2.parse(line)
                msg.latitude  = parsed.latitude
                msg.longitude = parsed.longitude
                msg.altitude  = float(parsed.altitude) if parsed.altitude else 0.0

                fix_quality = int(parsed.gps_qual) if parsed.gps_qual else 0
                msg.status.status = (NavSatStatus.STATUS_FIX
                                     if fix_quality > 0
                                     else NavSatStatus.STATUS_NO_FIX)
                msg.status.service = NavSatStatus.SERVICE_GPS
                msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        except Exception as e:
            self.get_logger().warn(f'Erro na leitura serial: {e}')
            msg.status.status = NavSatStatus.STATUS_NO_FIX
        return msg


# ------------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = GpsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
