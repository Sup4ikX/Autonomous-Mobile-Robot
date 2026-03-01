import pytest
import json
import os
import tempfile
from pathlib import Path

# Импортировать ConfigLoader
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from pj_lidar.config_loader import ConfigLoader

class TestConfigLoader:
    """Тесты для ConfigLoader."""
    
    @pytest.fixture(autouse=True)
    def setup(self):
        """Сброс ConfigLoader перед каждым тестом."""
        ConfigLoader._config = None
        ConfigLoader._config_file = None
        yield
        ConfigLoader._config = None
        ConfigLoader._config_file = None
    
    def test_load_json_config(self):
        """Тест загрузки JSON конфига."""
        with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
            config_data = {
                'esp32_tcp_client': {
                    'robot_ip': '192.168.1.100',
                    'tcp_port': 3333,
                    'timeout': 5.0
                }
            }
            json.dump(config_data, f)
            f.flush()
            
            config = ConfigLoader.load(f.name)
            
            assert config is not None
            assert config['esp32_tcp_client']['robot_ip'] == '192.168.1.100'
            assert config['esp32_tcp_client']['tcp_port'] == 3333
            
            os.unlink(f.name)
    
    def test_get_nested_key(self):
        """Тест получения вложенного ключа."""
        with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
            config_data = {
                'esp32_tcp_client': {
                    'robot_ip': '192.168.1.100',
                    'tcp_port': 3333
                }
            }
            json.dump(config_data, f)
            f.flush()
            
            ConfigLoader.load(f.name)
            
            ip = ConfigLoader.get('esp32_tcp_client.robot_ip')
            assert ip == '192.168.1.100'
            
            port = ConfigLoader.get('esp32_tcp_client.tcp_port')
            assert port == 3333
            
            os.unlink(f.name)
    
    def test_default_value(self):
        """Тест возврата значения по умолчанию."""
        with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
            json.dump({'test': 'value'}, f)
            f.flush()
            
            ConfigLoader.load(f.name)
            
            result = ConfigLoader.get('nonexistent.key', 'default')
            assert result == 'default'
            
            os.unlink(f.name)
