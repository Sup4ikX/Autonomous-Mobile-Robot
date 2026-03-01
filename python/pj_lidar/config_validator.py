import json
from typing import Dict, Any, Optional

class ConfigValidator:
    """Валидатор конфигурации."""
    
    # JSON Schema для валидации
    SCHEMA = {
        "esp32_tcp_client": {
            "required": ["robot_ip", "tcp_port"],
            "properties": {
                "robot_ip": {"type": "string", "pattern": r"^\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3}$"},
                "tcp_port": {"type": "integer", "minimum": 1, "maximum": 65535},
                "timeout": {"type": "number", "minimum": 0.1, "maximum": 60}
            }
        },
        "lidar_udp_server": {
            "required": ["udp_port"],
            "properties": {
                "udp_port": {"type": "integer", "minimum": 1024, "maximum": 65535},
                "range_min": {"type": "number", "minimum": 0},
                "range_max": {"type": "number", "minimum": 0}
            }
        },
        "scan_state_machine": {
            "properties": {
                "obstacle_distance": {"type": "number", "minimum": 0.1},
                "max_forward_speed": {"type": "number", "minimum": 0},
                "rotation_speed": {"type": "number", "minimum": 0}
            }
        }
    }
    
    @classmethod
    def validate(cls, config: Dict[str, Any]) -> tuple[bool, Optional[str]]:
        """
        Валидировать конфиг.
        
        Args:
            config: словарь конфигурации
            
        Returns:
            (is_valid, error_message)
        """
        try:
            for section, schema in cls.SCHEMA.items():
                if section not in config:
                    return False, f"Missing section: {section}"
                
                section_config = config[section]
                
                # Проверить обязательные поля
                for required_field in schema.get("required", []):
                    if required_field not in section_config:
                        return False, f"Missing field: {section}.{required_field}"
                
                # Проверить типы данных
                for field, field_schema in schema.get("properties", {}).items():
                    if field not in section_config:
                        continue
                    
                    value = section_config[field]
                    field_type = field_schema.get("type")
                    
                    # Проверка типа
                    if field_type == "string":
                        if not isinstance(value, str):
                            return False, f"Invalid type for {section}.{field}: expected string"
                        
                        # Проверить паттерн IP
                        if "pattern" in field_schema:
                            import re
                            if not re.match(field_schema["pattern"], value):
                                return False, f"Invalid pattern for {section}.{field}: {value}"
                    
                    elif field_type == "integer":
                        if not isinstance(value, int):
                            return False, f"Invalid type for {section}.{field}: expected integer"
                        
                        if "minimum" in field_schema and value < field_schema["minimum"]:
                            return False, f"Value too small: {section}.{field} < {field_schema['minimum']}"
                        
                        if "maximum" in field_schema and value > field_schema["maximum"]:
                            return False, f"Value too large: {section}.{field} > {field_schema['maximum']}"
                    
                    elif field_type == "number":
                        if not isinstance(value, (int, float)):
                            return False, f"Invalid type for {section}.{field}: expected number"
                        
                        if "minimum" in field_schema and value < field_schema["minimum"]:
                            return False, f"Value too small: {section}.{field} < {field_schema['minimum']}"
            
            return True, None
        
        except Exception as e:
            return False, str(e)
