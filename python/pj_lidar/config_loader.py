# ===============================================
# КОНФИГ ЛОАДЕР
# ===============================================
#
# ЗАДАЧА:
# Загружает параметры из YAML файла вместо hardcode.
# Позволяет пользователю легко менять IP, порты и параметры.
# При отсутствии конфига запускается интерактивная настройка.
#

try:
    import yaml
    YAML_AVAILABLE = True
except ImportError:
    YAML_AVAILABLE = False
    print("⚠️ PyYAML не установлен!")
    print("   Установить: pip install pyyaml")

import os
import json
from pathlib import Path

class ConfigLoader:
    """Загрузчик конфигурации из YAML или JSON."""
    
    _config = None
    _config_file = None
    
    @staticmethod
    def load(config_path=None):
        """Загрузить конфиг - сначала ищет YAML, затем JSON."""
        if ConfigLoader._config is not None:
            return ConfigLoader._config
        
        # Поиск конфига
        if config_path is None:
            # Попробовать найти через ament_index (ROS2)
            try:
                from ament_index_python import get_package_share_directory
                pkg_share = get_package_share_directory('pj_lidar')
                ros_config_path = os.path.join(pkg_share, 'config.yaml')
            except:
                ros_config_path = None
            
            possible_paths = [
                "config.yaml",
                "config.json",
                os.path.join(os.path.dirname(__file__), "config.yaml"),
                os.path.join(os.path.dirname(__file__), "config.json"),
                ros_config_path,
                os.path.expanduser("~/.robot_config.yaml"),
                os.path.expanduser("~/.robot_config.json"),
            ]
            
            for path in possible_paths:
                if os.path.exists(path):
                    config_path = path
                    break
        
        # Если конфига нет — создать интерактивно
        if config_path is None or not os.path.exists(config_path or ""):
            print("\n" + "="*60)
            print("⚙️  ПЕРВАЯ НАСТРОЙКА СИСТЕМЫ")
            print("="*60)
            ConfigLoader._create_interactive_config()
            config_path = "config.yaml"
        
        # Загрузить конфиг
        try:
            if config_path.endswith('.yaml') and YAML_AVAILABLE:
                with open(config_path, 'r', encoding='utf-8') as f:
                    ConfigLoader._config = yaml.safe_load(f)
            elif config_path.endswith('.json'):
                with open(config_path, 'r', encoding='utf-8') as f:
                    ConfigLoader._config = json.load(f)
            else:
                raise ValueError(f"Неподдерживаемый формат: {config_path}")
            
            ConfigLoader._config_file = config_path
            print(f"\n✓ Конфиг загружен: {config_path}\n")
        except Exception as e:
            raise ValueError(f"Ошибка загрузки конфига: {str(e)}")
        
        return ConfigLoader._config
    
    @staticmethod
    def _create_interactive_config():
        """Интерактивное создание конфига."""
        if not YAML_AVAILABLE:
            print("⚠️ PyYAML не установлен, используется JSON формат")
        
        config = {}
        
        print("\n📡 TCP КОНФИГУРАЦИЯ (подключение к ESP32)")
        print("-" * 60)
        
        ip = input("  IP адрес ESP32 [192.168.4.2]: ").strip() or "192.168.4.2"
        port = input("  TCP порт [3333]: ").strip() or "3333"
        timeout = input("  Timeout [5.0]: ").strip() or "5.0"
        
        config['esp32_tcp_client'] = {
            'robot_ip': ip,
            'tcp_port': int(port),
            'timeout': float(timeout)
        }
        
        print("\n📶 UDP КОНФИГУРАЦИЯ (приём LiDAR)")
        print("-" * 60)
        
        udp_port = input("  UDP порт [8888]: ").strip() or "8888"
        range_min = input("  Min дистанция [0.15]: ").strip() or "0.15"
        range_max = input("  Max дистанция [12.0]: ").strip() or "12.0"
        
        config['lidar_udp_server'] = {
            'udp_port': int(udp_port),
            'range_min': float(range_min),
            'range_max': float(range_max),
            'frame_id': 'laser'
        }
        
        print("\n🤖 АВТОПИЛОТ КОНФИГУРАЦИЯ")
        print("-" * 60)
        
        obstacle_dist = input("  Дистанция обнаружения [0.5]: ").strip() or "0.5"
        forward_speed = input("  Макс скорость вперед [0.3]: ").strip() or "0.3"
        rotation_speed = input("  Скорость вращения [0.5]: ").strip() or "0.5"
        
        config['scan_state_machine'] = {
            'obstacle_distance': float(obstacle_dist),
            'max_forward_speed': float(forward_speed),
            'rotation_speed': float(rotation_speed),
            'enable_auto_mode': False
        }
        
        print("\n🗺️  SLAM КОНФИГУРАЦИЯ")
        print("-" * 60)
        
        map_path = input("  Папка для карт [~/maps]: ").strip() or "~/maps"
        
        config['slam_toolbox'] = {
            'map_save_path': map_path,
            'auto_save': True
        }
        
        # Сохранение
        try:
            with open("config.yaml", 'w', encoding='utf-8') as f:
                yaml.dump(config, f, allow_unicode=True, default_flow_style=False)
            
            print("\n" + "="*60)
            print("✓ Конфиг сохранён: config.yaml")
            print("="*60)
            print("\nПараметры:")
            print(f"  ESP32:        {config['esp32_tcp_client']['robot_ip']}:{config['esp32_tcp_client']['tcp_port']}")
            print(f"  UDP:          порт {config['lidar_udp_server']['udp_port']}")
            print(f"  Дистанция:    {config['lidar_udp_server']['range_min']}м - {config['lidar_udp_server']['range_max']}м")
            print(f"  Скорость:     {config['scan_state_machine']['max_forward_speed']} м/с")
            print(f"  Карты:        {config['slam_toolbox']['map_save_path']}")
            print("\n✓ Готово! Можно запускать систему.\n")
            
        except Exception as e:
            print(f"✗ Ошибка сохранения: {str(e)}")
    
    @staticmethod
    def get(key, default=None):
        """
        Получить значение из конфига.
        
        Args:
            key (str): ключ (например "esp32_tcp_client.robot_ip")
            default: значение по умолчанию
            
        Returns:
            Значение из конфига или default
        """
        if ConfigLoader._config is None:
            ConfigLoader.load()
        
        # Поддержка вложенных ключей ("section.key")
        keys = key.split('.')
        value = ConfigLoader._config
        
        for k in keys:
            if isinstance(value, dict) and k in value:
                value = value[k]
            else:
                return default
        
        return value
    
    @staticmethod
    def get_tcp_config():
        """Получить конфиг TCP."""
        return ConfigLoader.get('esp32_tcp_client', {})
    
    @staticmethod
    def get_udp_config():
        """Получить конфиг UDP."""
        return ConfigLoader.get('lidar_udp_server', {})
    
    @staticmethod
    def get_state_machine_config():
        """Получить конфиг State Machine."""
        return ConfigLoader.get('scan_state_machine', {})
    
    @staticmethod
    def reconfig():
        """Пересоздать конфиг (для изменения параметров)."""
        ConfigLoader._config = None
        ConfigLoader._config_file = None
        ConfigLoader.load()
