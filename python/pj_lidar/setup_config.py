# ===============================================
# ИНТЕРАКТИВНАЯ НАСТРОЙКА КОНФИГА
# ===============================================
#
# ЗАПУСК: python setup_config.py
# Пользователь отвечает на вопросы и config.yaml создаётся автоматически
#

from pj_lidar.config_loader import ConfigLoader

if __name__ == "__main__":
    print("\n" + "="*70)
    print("🤖 ESP32 LiDAR Robot - ПЕРВИЧНАЯ НАСТРОЙКА")
    print("="*70)
    
    ConfigLoader.reconfig()
    
    print("\n" + "="*70)
    print("✓ НАСТРОЙКА ЗАВЕРШЕНА!")
    print("="*70)
    print("\nТеперь можно запускать систему:")
    print("  ros2 launch pj_lidar launch_all.py")
    print("\nДля изменения параметров отредактируйте config.yaml\n")
