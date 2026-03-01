# ===============================================
# СКРИПТ ДЛЯ ЗАПУСКА АВТОСКАНИРОВАНИЯ
# ===============================================
#
# Запускает интерактивное меню для управления автосканированием.
# Подключается к работающей ROS2 системе (launch_all.py должен быть запущен).
#
# Использование:
#   python3 python/pj_lidar/start_scan.py
#
# Команды в меню:
#   start  — запустить автосканирование
#   stop   — остановить сканирование
#   exit   — выход
#

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import sys
import threading


class ScanStarter(Node):
    """Узел для управления автосканированием."""

    def __init__(self):
        super().__init__('scan_starter')

        # Издатель команд → scan_state_machine
        self.cmd_pub = self.create_publisher(String, 'scan_command', 10)

        # Подписчик состояния робота
        self.state_sub = self.create_subscription(
            String, 'robot_state', self._state_callback, 10)

        self.last_state = 'IDLE'
        self.get_logger().info('Scan Starter инициализирован')

    def _state_callback(self, msg: String):
        state = msg.data or ''
        if state != self.last_state:
            self.last_state = state
            print(f'  [Состояние] {state}')

    def send(self, command: str):
        msg = String()
        msg.data = command
        self.cmd_pub.publish(msg)

    def start_scan(self):
        self.send('START_SCAN')
        self.get_logger().info('▶️  START_SCAN отправлен')

    def stop_scan(self):
        self.send('STOP_SCAN')
        self.get_logger().info('⏹  STOP_SCAN отправлен')


def main():
    rclpy.init()
    node = ScanStarter()

    # Крутим узел в фоне, чтобы callbacks работали
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Ждём, пока издатель подключится к подписчикам
    print('\n⏳ Подключение к системе...')
    time.sleep(1.5)

    print('\n' + '=' * 50)
    print('🤖  ESP32 LiDAR Robot — Auto Scan Control')
    print('=' * 50)
    print('  start  — запустить автосканирование')
    print('  stop   — остановить сканирование')
    print('  exit   — выход')
    print('=' * 50 + '\n')

    try:
        while True:
            try:
                cmd = input('Команда: ').strip().lower()
            except EOFError:
                break

            if cmd == 'start':
                node.start_scan()
                print('✓ Сканирование запущено! (следи за состоянием выше)')

            elif cmd == 'stop':
                node.stop_scan()
                print('⏹ Сканирование остановлено')

            elif cmd in ('exit', 'quit', 'q'):
                print('� Выход...')
                break

            elif cmd == '':
                pass  # пустая строка — игнорируем

            else:
                print(f'❌ Неизвестная команда: {cmd!r}  (start / stop / exit)')

    except KeyboardInterrupt:
        print('\n⏹ Прерывание...')
    finally:
        node.stop_scan()
        time.sleep(0.2)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
