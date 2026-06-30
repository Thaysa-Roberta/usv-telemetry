"""
mavlink_bridge_node.py
----------------------
Nó ROS 2 que atua como ponte entre o grafo de tópicos ROS 2 e uma
estação de controle remota (GCS) usando o protocolo MAVLink sobre TCP.

Direção USV → GCS (telemetria):
    /gps/fix        → MAVLink GLOBAL_POSITION_INT
    /imu/data       → MAVLink ATTITUDE
    /motors/status  → MAVLink STATUSTEXT

Direção GCS → USV (comandos):
    MAVLink RC_CHANNELS_OVERRIDE → /cmd_vel (geometry_msgs/Twist)
    MAVLink COMMAND_LONG (MOTOR_TEST) → /cmd_vel

Uso:
    ros2 run usv_telemetry mavlink_bridge_node
    ros2 run usv_telemetry mavlink_bridge_node --ros-args -p gcs_host:=192.168.1.100 -p gcs_port:=14550
"""

import math
import socket
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import String

try:
    from pymavlink import mavutil
    MAVLINK_AVAILABLE = True
except ImportError:
    MAVLINK_AVAILABLE = False


# ID do sistema e componente MAVLink do USV
MAV_SYSTEM_ID    = 1
MAV_COMPONENT_ID = 1

# Limites de stick RC → velocidade
RC_MIN, RC_MID, RC_MAX = 1000, 1500, 2000


class MavlinkBridgeNode(Node):
    """Ponte bidirecional ROS 2 ↔ MAVLink sobre TCP."""

    def __init__(self):
        super().__init__('mavlink_bridge_node')

        # ------------------------------------------------------------------
        # Parâmetros
        # ------------------------------------------------------------------
        self.declare_parameter('gcs_host', '0.0.0.0')   # aceita conexões de qualquer IP
        self.declare_parameter('gcs_port', 14550)
        self.declare_parameter('heartbeat_rate_hz', 1.0)
        self.declare_parameter('telemetry_rate_hz', 4.0)

        self.gcs_host    = self.get_parameter('gcs_host').value
        self.gcs_port    = self.get_parameter('gcs_port').value
        hb_rate          = self.get_parameter('heartbeat_rate_hz').value
        telem_rate       = self.get_parameter('telemetry_rate_hz').value

        # ------------------------------------------------------------------
        # Estado dos sensores (atualizado pelos subscribers)
        # ------------------------------------------------------------------
        self._gps_msg: NavSatFix = None
        self._imu_msg: Imu       = None
        self._motor_status: str  = ''
        self._lock = threading.Lock()

        # ------------------------------------------------------------------
        # Subscribers ROS 2
        # ------------------------------------------------------------------
        self.create_subscription(NavSatFix, '/gps/fix',       self._gps_cb,    10)
        self.create_subscription(Imu,       '/imu/data',      self._imu_cb,    10)
        self.create_subscription(String,    '/motors/status', self._motors_cb, 10)

        # ------------------------------------------------------------------
        # Publisher ROS 2
        # ------------------------------------------------------------------
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ------------------------------------------------------------------
        # Conexão MAVLink
        # ------------------------------------------------------------------
        self._mav = None
        self._connected = False

        if not MAVLINK_AVAILABLE:
            self.get_logger().warn(
                'pymavlink não encontrado. Instale com: pip install pymavlink\n'
                'Rodando em modo de log apenas.'
            )
        else:
            self._start_tcp_server()

        # ------------------------------------------------------------------
        # Timers
        # ------------------------------------------------------------------
        self.create_timer(1.0 / hb_rate,    self._send_heartbeat)
        self.create_timer(1.0 / telem_rate, self._send_telemetry)

        self.get_logger().info(
            f'mavlink_bridge_node iniciado — aguardando GCS em '
            f'{self.gcs_host}:{self.gcs_port} (TCP)'
        )

    # ------------------------------------------------------------------
    # Callbacks dos subscribers
    # ------------------------------------------------------------------
    def _gps_cb(self, msg: NavSatFix):
        with self._lock:
            self._gps_msg = msg

    def _imu_cb(self, msg: Imu):
        with self._lock:
            self._imu_msg = msg

    def _motors_cb(self, msg: String):
        with self._lock:
            self._motor_status = msg.data

    # ------------------------------------------------------------------
    # Servidor TCP para conexão da GCS
    # ------------------------------------------------------------------
    def _start_tcp_server(self):
        """Inicia thread que aceita uma conexão TCP da GCS."""
        t = threading.Thread(target=self._accept_loop, daemon=True)
        t.start()

    def _accept_loop(self):
        """Aguarda conexão da GCS usando pymavlink em modo servidor (tcpin)."""
        try:
            self.get_logger().info(f'MAVLink TCP server listening on port {self.gcs_port}')
            self._mav = mavutil.mavlink_connection(
                f'tcpin:{self.gcs_host}:{self.gcs_port}',
                source_system=MAV_SYSTEM_ID,
                source_component=MAV_COMPONENT_ID
            )
            self.get_logger().info('GCS conectada via MAVLink TCP')
            self._connected = True
            # Inicia thread de recepção de comandos
            threading.Thread(target=self._receive_loop, daemon=True).start()
        except Exception as e:
            self.get_logger().error(f'Erro no servidor TCP: {e}')

    # ------------------------------------------------------------------
    # Loop de recepção de mensagens MAVLink da GCS
    # ------------------------------------------------------------------
    def _receive_loop(self):
        """Recebe mensagens MAVLink e converte em comandos ROS 2."""
        while self._connected and rclpy.ok():
            try:
                msg = self._mav.recv_match(blocking=True, timeout=1.0)
                if msg is None:
                    continue
                self._process_mavlink_msg(msg)
            except Exception as e:
                self.get_logger().warn(f'Erro na recepção MAVLink: {e}')
                break

    def _process_mavlink_msg(self, msg):
        """Interpreta mensagem MAVLink recebida e publica no tópico correto."""
        msg_type = msg.get_type()

        if msg_type == 'RC_CHANNELS_OVERRIDE':
            # Canal 3 = aceleração (throttle) → velocidade linear
            # Canal 4 = guinada (rudder/yaw)   → velocidade angular
            ch3 = getattr(msg, 'chan3_raw', RC_MID)
            ch4 = getattr(msg, 'chan4_raw', RC_MID)

            linear_x  = self._rc_to_vel(ch3, max_val=1.0)
            angular_z = self._rc_to_vel(ch4, max_val=1.57)

            twist = Twist()
            twist.linear.x  = linear_x
            twist.angular.z = angular_z
            self.cmd_pub.publish(twist)

            self.get_logger().debug(
                f'RC → linear={linear_x:.2f} m/s  angular={angular_z:.2f} rad/s'
            )

        elif msg_type == 'HEARTBEAT':
            self.get_logger().debug('Heartbeat recebido da GCS')

    @staticmethod
    def _rc_to_vel(rc_value: int, max_val: float) -> float:
        """Converte valor de canal RC [1000–2000] para velocidade [-max, +max]."""
        normalized = (rc_value - RC_MID) / (RC_MAX - RC_MID)
        normalized = max(-1.0, min(1.0, normalized))
        return normalized * max_val

    # ------------------------------------------------------------------
    # Envio de mensagens MAVLink para a GCS
    # ------------------------------------------------------------------
    def _send_heartbeat(self):
        """Envia HEARTBEAT periódico para manter a conexão ativa."""
        if not self._connected or self._mav is None:
            return
        try:
            self._mav.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_SURFACE_BOAT,
                mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
                mavutil.mavlink.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED,
                0,
                mavutil.mavlink.MAV_STATE_ACTIVE
            )
        except Exception as e:
            self.get_logger().warn(f'Erro ao enviar heartbeat: {e}')

    def _send_telemetry(self):
        """Envia dados de telemetria (GPS + IMU) para a GCS."""
        if not MAVLINK_AVAILABLE:
            self._log_telemetry_sim()
            return

        if not self._connected or self._mav is None:
            return

        with self._lock:
            gps = self._gps_msg
            imu = self._imu_msg

        now_ms = int(time.time() * 1000) & 0xFFFFFFFF

        # --- GPS → GLOBAL_POSITION_INT ---
        if gps is not None:
            try:
                self._mav.mav.global_position_int_send(
                    now_ms,
                    int(gps.latitude  * 1e7),   # degE7
                    int(gps.longitude * 1e7),   # degE7
                    int(gps.altitude  * 1000),  # mm acima do MSL
                    0,                           # altitude relativa (mm)
                    0, 0, 0,                     # vx, vy, vz (cm/s)
                    0                            # hdg (desconhecido)
                )
            except Exception as e:
                self.get_logger().warn(f'Erro ao enviar GPS: {e}')

        # --- IMU → ATTITUDE ---
        if imu is not None:
            try:
                roll, pitch, yaw = self._quat_to_euler(imu.orientation)
                self._mav.mav.attitude_send(
                    now_ms,
                    roll, pitch, yaw,
                    imu.angular_velocity.x,
                    imu.angular_velocity.y,
                    imu.angular_velocity.z
                )
            except Exception as e:
                self.get_logger().warn(f'Erro ao enviar IMU: {e}')

    def _log_telemetry_sim(self):
        """Registra telemetria no log quando pymavlink não está disponível."""
        with self._lock:
            gps = self._gps_msg
            imu = self._imu_msg

        if gps:
            self.get_logger().info(
                f'[TELEM] GPS: lat={gps.latitude:.6f} lon={gps.longitude:.6f} '
                f'alt={gps.altitude:.1f}m'
            )
        if imu:
            roll, pitch, yaw = self._quat_to_euler(imu.orientation)
            self.get_logger().info(
                f'[TELEM] IMU: roll={math.degrees(roll):.1f}° '
                f'pitch={math.degrees(pitch):.1f}° '
                f'yaw={math.degrees(yaw):.1f}°'
            )

    # ------------------------------------------------------------------
    # Utilitários
    # ------------------------------------------------------------------
    @staticmethod
    def _quat_to_euler(q) -> tuple:
        """Converte quaternion (geometry_msgs) para ângulos de Euler (rad)."""
        x, y, z, w = q.x, q.y, q.z, q.w
        roll  = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
        pitch = math.asin( max(-1.0, min(1.0, 2*(w*y - z*x))))
        yaw   = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        return roll, pitch, yaw


# ------------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = MavlinkBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
