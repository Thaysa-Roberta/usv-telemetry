"""
control_node.py
---------------
Nó ROS 2 responsável por receber comandos de velocidade do tópico
/cmd_vel e convertê-los em sinais PWM para os dois motores do USV
via interface serial (UART) com o microcontrolador.

O USV usa propulsão diferencial: dois motores independentes.
A velocidade linear e angular do Twist é mapeada para velocidade
de cada motor (left/right) pela cinemática diferencial.

Em modo SIMULADO (padrão), imprime os valores de PWM calculados
sem enviar para hardware.

Uso:
    ros2 run usv_telemetry control_node
    ros2 run usv_telemetry control_node --ros-args -p simulated:=false -p serial_port:=/dev/ttyACM0
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


# Limites de PWM (microssegundos) — ajustar conforme os ESCs do USV
PWM_NEUTRO = 1500   # us — sem movimento
PWM_MIN    = 1100   # us — ré máxima
PWM_MAX    = 1900   # us — frente máxima


class ControlNode(Node):
    """Converte /cmd_vel em sinais PWM e envia ao microcontrolador via serial."""

    def __init__(self):
        super().__init__('control_node')

        # ------------------------------------------------------------------
        # Parâmetros configuráveis
        # ------------------------------------------------------------------
        self.declare_parameter('simulated', True)
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)

        # Geometria do veículo
        self.declare_parameter('wheel_base', 0.35)   # distância entre motores (m)
        self.declare_parameter('max_linear_vel', 1.0)   # m/s
        self.declare_parameter('max_angular_vel', 1.57)  # rad/s (~90°/s)

        self.simulated       = self.get_parameter('simulated').value
        self.serial_port     = self.get_parameter('serial_port').value
        self.baud_rate       = self.get_parameter('baud_rate').value
        self.wheel_base      = self.get_parameter('wheel_base').value
        self.max_linear      = self.get_parameter('max_linear_vel').value
        self.max_angular     = self.get_parameter('max_angular_vel').value

        # ------------------------------------------------------------------
        # Subscriber: recebe comandos de velocidade
        # ------------------------------------------------------------------
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self._cmd_vel_callback,
            10
        )

        # ------------------------------------------------------------------
        # Publisher: publica estado dos motores (para debug / GCS)
        # ------------------------------------------------------------------
        self.status_pub = self.create_publisher(String, '/motors/status', 10)

        # ------------------------------------------------------------------
        # Watchdog: para os motores se não receber comandos em 1 s
        # ------------------------------------------------------------------
        self._last_cmd_time = self.get_clock().now()
        self.create_timer(0.5, self._watchdog_callback)

        # ------------------------------------------------------------------
        # Interface serial
        # ------------------------------------------------------------------
        self.serial_conn = None
        if not self.simulated:
            self._init_serial()

        mode = 'SIMULADO' if self.simulated else f'REAL ({self.serial_port})'
        self.get_logger().info(f'control_node iniciado em modo {mode}')
        self.get_logger().info(
            f'Aguardando comandos em /cmd_vel  '
            f'(max_v={self.max_linear} m/s, max_w={self.max_angular} rad/s)'
        )

    # ------------------------------------------------------------------
    def _init_serial(self):
        """Abre a conexão serial com o microcontrolador."""
        try:
            import serial
            self.serial_conn = serial.Serial(
                self.serial_port, self.baud_rate, timeout=0.1
            )
            self.get_logger().info(f'Serial {self.serial_port} aberta')
        except Exception as e:
            self.get_logger().error(
                f'Falha ao abrir serial: {e}. Alternando para modo simulado.'
            )
            self.simulated = True

    # ------------------------------------------------------------------
    def _cmd_vel_callback(self, msg: Twist):
        """Recebe Twist e calcula PWM para cada motor."""
        self._last_cmd_time = self.get_clock().now()

        v = msg.linear.x   # velocidade linear  (m/s)
        w = msg.angular.z  # velocidade angular (rad/s)

        # Cinemática diferencial: v_left / v_right em m/s
        v_left  = v - (w * self.wheel_base / 2.0)
        v_right = v + (w * self.wheel_base / 2.0)

        # Normaliza [-1, +1]
        v_max   = max(self.max_linear, abs(v_left), abs(v_right))
        v_left  = v_left  / v_max if v_max > 0 else 0.0
        v_right = v_right / v_max if v_max > 0 else 0.0

        # Mapeia para PWM em microssegundos
        pwm_left  = self._vel_to_pwm(v_left)
        pwm_right = self._vel_to_pwm(v_right)

        self._send_pwm(pwm_left, pwm_right)

    # ------------------------------------------------------------------
    def _vel_to_pwm(self, normalized: float) -> int:
        """Converte velocidade normalizada [-1, +1] para PWM (us)."""
        normalized = max(-1.0, min(1.0, normalized))
        if normalized >= 0:
            pwm = int(PWM_NEUTRO + normalized * (PWM_MAX - PWM_NEUTRO))
        else:
            pwm = int(PWM_NEUTRO + normalized * (PWM_NEUTRO - PWM_MIN))
        return pwm

    # ------------------------------------------------------------------
    def _send_pwm(self, pwm_left: int, pwm_right: int):
        """Envia comando PWM ao microcontrolador ou simula."""
        # Protocolo serial simples: "L<valor>R<valor>\n"
        # Ex.: "L1600R1400\n" → motor esq. 1600 us, motor dir. 1400 us
        command = f'L{pwm_left}R{pwm_right}\n'

        if self.simulated:
            self.get_logger().info(
                f'[SIM] PWM → esquerdo={pwm_left} us  direito={pwm_right} us'
            )
        else:
            try:
                self.serial_conn.write(command.encode('ascii'))
            except Exception as e:
                self.get_logger().error(f'Erro ao enviar PWM: {e}')

        # Publica status para o tópico de debug
        status_msg = String()
        status_msg.data = f'L={pwm_left}us R={pwm_right}us'
        self.status_pub.publish(status_msg)

    # ------------------------------------------------------------------
    def _watchdog_callback(self):
        """Para os motores se nenhum comando for recebido em 1 segundo."""
        elapsed = (self.get_clock().now() - self._last_cmd_time).nanoseconds / 1e9
        if elapsed > 1.0:
            self._send_pwm(PWM_NEUTRO, PWM_NEUTRO)


# ------------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Garante parada dos motores ao encerrar
        node._send_pwm(PWM_NEUTRO, PWM_NEUTRO)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
