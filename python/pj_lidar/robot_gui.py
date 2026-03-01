# ===============================================
# МОДУЛЬ: ROBOT GUI
# ===============================================
#
# ЗАДАЧА:
# Графический интерфейс для управления ESP32 роботом через TCP.
# Позволяет отправлять команды, мониторить статус и визуально управлять роботом.
#
# ВХОДНЫЕ ДАННЫЕ:
# - robot_ip, tcp_port из config.yaml (user-configurable)
#
# ВЫХОДНЫЕ ДАННЫЕ:
# - TCP команды на ESP32
# - Визуализация статуса робота
#
# ЦВЕТОВАЯ СХЕМА:
# - #05386B (темно-синий) - фон
# - #379683 (морской) - заголовок
# - #5CDB95 (зелёный) - активные элементы
# - #8EE4AF (светло-зелёный) - акценты
# - #EDF5E1 (кремовый) - текст
#

import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import socket
import threading
import time
from datetime import datetime
import subprocess

# Импортировать конфиг-лоадер с fallback
try:
    from pj_lidar.config_loader import ConfigLoader
    CONFIG_AVAILABLE = True
except (ImportError, ModuleNotFoundError):
    CONFIG_AVAILABLE = False
    print("[!] ConfigLoader не найдена - используются значения по умолчанию")

class RobotGUI:
    """Графический интерфейс для управления роботом."""
    
    # Цветовая схема
    COLOR_DARK_BLUE = "#05386B"
    COLOR_TEAL = "#379683"
    COLOR_GREEN = "#5CDB95"
    COLOR_LIGHT_GREEN = "#8EE4AF"
    COLOR_CREAM = "#EDF5E1"
    
    def __init__(self, root):
        self.root = root
        self.root.title("ESP32 LiDAR Robot Controller")
        self.root.geometry("1000x700")
        self.root.configure(bg=self.COLOR_DARK_BLUE)
        
        # TCP переменные
        self.socket = None
        self.connected = False
        self.recv_buffer = ""
        self.lock = threading.Lock()
        
        # Загрузить параметры подключения из config.yaml (user-configurable)
        if CONFIG_AVAILABLE:
            try:
                ConfigLoader.load()
                tcp_config = ConfigLoader.get_tcp_config()
                # PARAMETERS LOADED FROM USER CONFIG (config.yaml)
                self.robot_ip = tcp_config.get('robot_ip', 'auto')   # <-- from config.yaml
                self.tcp_port = tcp_config.get('tcp_port', 3333)    # <-- from config.yaml
            except:
                self.robot_ip = "auto"
                self.tcp_port = 3333
        else:
            self.robot_ip = "auto"
            self.tcp_port = 3333
        
        # Поток приема данных
        self.receive_thread = None
        self.stop_receive = False
        
        # Создание интерфейса
        self.create_ui()
    
    def create_ui(self):
        """Создать главный интерфейс."""
        
        # ===== ЗАГОЛОВОК =====
        header_frame = tk.Frame(self.root, bg=self.COLOR_TEAL, height=60)
        header_frame.pack(fill=tk.X, padx=0, pady=0)
        header_frame.pack_propagate(False)
        
        title = tk.Label(
            header_frame,
            text="🤖 ESP32 LiDAR Robot Controller",
            font=("Arial", 20, "bold"),
            bg=self.COLOR_TEAL,
            fg=self.COLOR_CREAM
        )
        title.pack(pady=10)
        
        # ===== ГЛАВНОЕ СОДЕРЖИМОЕ =====
        main_frame = tk.Frame(self.root, bg=self.COLOR_DARK_BLUE)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # ===== ЛЕВАЯ ПАНЕЛЬ: ПОДКЛЮЧЕНИЕ =====
        left_frame = tk.Frame(main_frame, bg=self.COLOR_DARK_BLUE)
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=False, padx=5)
        
        self.create_connection_panel(left_frame)
        self.create_motor_control_panel(left_frame)
        self.create_lidar_control_panel(left_frame)
        self.create_system_panel(left_frame)
        
        # ===== ПРАВАЯ ПАНЕЛЬ: СТАТУС И ЛОГИ =====
        right_frame = tk.Frame(main_frame, bg=self.COLOR_DARK_BLUE)
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=5)
        
        self.create_status_panel(right_frame)
        self.create_log_panel(right_frame)
    
    def create_connection_panel(self, parent):
        """Панель подключения."""
        frame = tk.LabelFrame(
            parent,
            text="📡 Подключение",
            bg=self.COLOR_DARK_BLUE,
            fg=self.COLOR_GREEN,
            font=("Arial", 11, "bold"),
            relief=tk.RAISED,
            bd=2
        )
        frame.pack(fill=tk.X, padx=5, pady=5)
        
        # IP адрес - загружается из config.yaml (user-configurable)
        ip_frame = tk.Frame(frame, bg=self.COLOR_DARK_BLUE)
        ip_frame.pack(fill=tk.X, padx=10, pady=5)
        
        tk.Label(ip_frame, text="IP:", bg=self.COLOR_DARK_BLUE, fg=self.COLOR_CREAM).pack(side=tk.LEFT)
        self.ip_entry = tk.Entry(ip_frame, width=15, bg=self.COLOR_CREAM, fg=self.COLOR_DARK_BLUE)
        self.ip_entry.insert(0, self.robot_ip)  # <-- from config or default
        self.ip_entry.pack(side=tk.LEFT, padx=5)
        
        # Порт - загружается из config.yaml (user-configurable)
        tk.Label(ip_frame, text="Порт:", bg=self.COLOR_DARK_BLUE, fg=self.COLOR_CREAM).pack(side=tk.LEFT, padx=(10,0))
        self.port_entry = tk.Entry(ip_frame, width=8, bg=self.COLOR_CREAM, fg=self.COLOR_DARK_BLUE)
        self.port_entry.insert(0, str(self.tcp_port))  # <-- from config or default
        self.port_entry.pack(side=tk.LEFT, padx=5)
        
        # Кнопки подключения
        btn_frame = tk.Frame(frame, bg=self.COLOR_DARK_BLUE)
        btn_frame.pack(fill=tk.X, padx=10, pady=5)
        
        self.connect_btn = tk.Button(
            btn_frame,
            text="✓ Подключить",
            bg=self.COLOR_GREEN,
            fg=self.COLOR_DARK_BLUE,
            font=("Arial", 10, "bold"),
            command=self.connect,
            relief=tk.RAISED,
            bd=2
        )
        self.connect_btn.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)
        
        self.disconnect_btn = tk.Button(
            btn_frame,
            text="✗ Отключить",
            bg="#E74C3C",
            fg=self.COLOR_CREAM,
            font=("Arial", 10, "bold"),
            command=self.disconnect,
            relief=tk.RAISED,
            bd=2,
            state=tk.DISABLED
        )
        self.disconnect_btn.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)
        
        # Статус подключения
        status_frame = tk.Frame(frame, bg=self.COLOR_DARK_BLUE)
        status_frame.pack(fill=tk.X, padx=10, pady=5)
        
        tk.Label(status_frame, text="Статус:", bg=self.COLOR_DARK_BLUE, fg=self.COLOR_CREAM).pack(side=tk.LEFT)
        self.status_label = tk.Label(
            status_frame,
            text="● Не подключено",
            bg=self.COLOR_DARK_BLUE,
            fg="#E74C3C",
            font=("Arial", 10, "bold")
        )
        self.status_label.pack(side=tk.LEFT, padx=5)
    
    def create_motor_control_panel(self, parent):
        """Панель управления моторами."""
        frame = tk.LabelFrame(
            parent,
            text="⚙️ Управление моторами",
            bg=self.COLOR_DARK_BLUE,
            fg=self.COLOR_GREEN,
            font=("Arial", 11, "bold"),
            relief=tk.RAISED,
            bd=2
        )
        frame.pack(fill=tk.X, padx=5, pady=5)
        
        # Скорость движения
        speed_frame = tk.Frame(frame, bg=self.COLOR_DARK_BLUE)
        speed_frame.pack(fill=tk.X, padx=10, pady=10)
        
        tk.Label(speed_frame, text="Скорость:", bg=self.COLOR_DARK_BLUE, fg=self.COLOR_CREAM).pack(anchor=tk.W)
        
        self.speed_slider = tk.Scale(
            speed_frame,
            from_=-100, to=100,
            orient=tk.HORIZONTAL,
            bg=self.COLOR_LIGHT_GREEN,
            fg=self.COLOR_DARK_BLUE,
            troughcolor=self.COLOR_TEAL,
            activebackground=self.COLOR_GREEN
        )
        self.speed_slider.set(0)
        self.speed_slider.pack(fill=tk.X, pady=5)
        
        # Кнопки движения
        btn_frame = tk.Frame(frame, bg=self.COLOR_DARK_BLUE)
        btn_frame.pack(fill=tk.X, padx=10, pady=10)
        
        tk.Button(
            btn_frame,
            text="⬆ Вперед",
            bg=self.COLOR_GREEN,
            fg=self.COLOR_DARK_BLUE,
            font=("Arial", 10, "bold"),
            command=self.move_forward,
            relief=tk.RAISED,
            bd=2
        ).grid(row=0, column=1, padx=2, pady=2, sticky="ew")
        
        tk.Button(
            btn_frame,
            text="⬅ Влево",
            bg=self.COLOR_LIGHT_GREEN,
            fg=self.COLOR_DARK_BLUE,
            font=("Arial", 10, "bold"),
            command=self.move_left,
            relief=tk.RAISED,
            bd=2
        ).grid(row=1, column=0, padx=2, pady=2, sticky="ew")
        
        tk.Button(
            btn_frame,
            text="⏹ Стоп",
            bg="#E74C3C",
            fg=self.COLOR_CREAM,
            font=("Arial", 10, "bold"),
            command=self.stop,
            relief=tk.RAISED,
            bd=2
        ).grid(row=1, column=1, padx=2, pady=2, sticky="ew")
        
        tk.Button(
            btn_frame,
            text="➡ Вправо",
            bg=self.COLOR_LIGHT_GREEN,
            fg=self.COLOR_DARK_BLUE,
            font=("Arial", 10, "bold"),
            command=self.move_right,
            relief=tk.RAISED,
            bd=2
        ).grid(row=1, column=2, padx=2, pady=2, sticky="ew")
        
        tk.Button(
            btn_frame,
            text="⬇ Назад",
            bg=self.COLOR_GREEN,
            fg=self.COLOR_DARK_BLUE,
            font=("Arial", 10, "bold"),
            command=self.move_backward,
            relief=tk.RAISED,
            bd=2
        ).grid(row=2, column=1, padx=2, pady=2, sticky="ew")
        
        # Равные веса колонок
        for i in range(3):
            btn_frame.grid_columnconfigure(i, weight=1)
    
    def create_lidar_control_panel(self, parent):
        """Панель управления LiDAR."""
        frame = tk.LabelFrame(
            parent,
            text="🔴 Управление LiDAR",
            bg=self.COLOR_DARK_BLUE,
            fg=self.COLOR_GREEN,
            font=("Arial", 11, "bold"),
            relief=tk.RAISED,
            bd=2
        )
        frame.pack(fill=tk.X, padx=5, pady=5)
        
        # PWM LiDAR
        pwm_frame = tk.Frame(frame, bg=self.COLOR_DARK_BLUE)
        pwm_frame.pack(fill=tk.X, padx=10, pady=10)
        
        tk.Label(pwm_frame, text="PWM (0-255):", bg=self.COLOR_DARK_BLUE, fg=self.COLOR_CREAM).pack(anchor=tk.W)
        
        pwm_input_frame = tk.Frame(pwm_frame, bg=self.COLOR_DARK_BLUE)
        pwm_input_frame.pack(fill=tk.X, pady=5)
        
        self.pwm_entry = tk.Entry(pwm_input_frame, width=12, bg=self.COLOR_CREAM, fg=self.COLOR_DARK_BLUE)
        self.pwm_entry.insert(0, "128")
        self.pwm_entry.pack(side=tk.LEFT)
        
        tk.Button(
            pwm_input_frame,
            text="Установить",
            bg=self.COLOR_GREEN,
            fg=self.COLOR_DARK_BLUE,
            font=("Arial", 9, "bold"),
            command=self.set_lidar_pwm,
            relief=tk.RAISED,
            bd=2
        ).pack(side=tk.LEFT, padx=5)
        
        # Кнопки LiDAR
        btn_frame = tk.Frame(frame, bg=self.COLOR_DARK_BLUE)
        btn_frame.pack(fill=tk.X, padx=10, pady=5)
        
        tk.Button(
            btn_frame,
            text="✓ Вкл LiDAR",
            bg=self.COLOR_GREEN,
            fg=self.COLOR_DARK_BLUE,
            font=("Arial", 9, "bold"),
            command=self.enable_lidar,
            relief=tk.RAISED,
            bd=2
        ).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)
        
        tk.Button(
            btn_frame,
            text="✗ Выкл LiDAR",
            bg="#E74C3C",
            fg=self.COLOR_CREAM,
            font=("Arial", 9, "bold"),
            command=self.disable_lidar,
            relief=tk.RAISED,
            bd=2
        ).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)
    
    def create_system_panel(self, parent):
        """Панель системных команд."""
        frame = tk.LabelFrame(
            parent,
            text="⚡ Система",
            bg=self.COLOR_DARK_BLUE,
            fg=self.COLOR_GREEN,
            font=("Arial", 11, "bold"),
            relief=tk.RAISED,
            bd=2
        )
        frame.pack(fill=tk.X, padx=5, pady=5)
        
        btn_frame = tk.Frame(frame, bg=self.COLOR_DARK_BLUE)
        btn_frame.pack(fill=tk.X, padx=10, pady=10)
        
        tk.Button(
            btn_frame,
            text="📊 Статус",
            bg=self.COLOR_LIGHT_GREEN,
            fg=self.COLOR_DARK_BLUE,
            font=("Arial", 9, "bold"),
            command=self.request_status,
            relief=tk.RAISED,
            bd=2
        ).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)
        
        tk.Button(
            btn_frame,
            text="🔄 Перезагрузка",
            bg="#F39C12",
            fg=self.COLOR_CREAM,
            font=("Arial", 9, "bold"),
            command=self.reset_esp32,
            relief=tk.RAISED,
            bd=2
        ).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)
    
    def create_status_panel(self, parent):
        """Панель статуса."""
        frame = tk.LabelFrame(
            parent,
            text="📈 Статус робота",
            bg=self.COLOR_DARK_BLUE,
            fg=self.COLOR_GREEN,
            font=("Arial", 11, "bold"),
            relief=tk.RAISED,
            bd=2
        )
        frame.pack(fill=tk.X, padx=5, pady=5)
        
        # Батарея
        bat_frame = tk.Frame(frame, bg=self.COLOR_DARK_BLUE)
        bat_frame.pack(fill=tk.X, padx=10, pady=5)
        
        tk.Label(bat_frame, text="🔋 Батарея:", bg=self.COLOR_DARK_BLUE, fg=self.COLOR_CREAM).pack(side=tk.LEFT)
        self.battery_label = tk.Label(
            bat_frame,
            text="-- V",
            bg=self.COLOR_DARK_BLUE,
            fg=self.COLOR_LIGHT_GREEN,
            font=("Arial", 11, "bold")
        )
        self.battery_label.pack(side=tk.LEFT, padx=10)
        
        # Левый мотор
        left_frame = tk.Frame(frame, bg=self.COLOR_DARK_BLUE)
        left_frame.pack(fill=tk.X, padx=10, pady=5)
        
        tk.Label(left_frame, text="⬅ Левый:", bg=self.COLOR_DARK_BLUE, fg=self.COLOR_CREAM).pack(side=tk.LEFT)
        self.left_speed_label = tk.Label(
            left_frame,
            text="-- PWM",
            bg=self.COLOR_DARK_BLUE,
            fg=self.COLOR_GREEN,
            font=("Arial", 11, "bold")
        )
        self.left_speed_label.pack(side=tk.LEFT, padx=10)
        
        # Правый мотор
        right_frame = tk.Frame(frame, bg=self.COLOR_DARK_BLUE)
        right_frame.pack(fill=tk.X, padx=10, pady=5)
        
        tk.Label(right_frame, text="➡ Правый:", bg=self.COLOR_DARK_BLUE, fg=self.COLOR_CREAM).pack(side=tk.LEFT)
        self.right_speed_label = tk.Label(
            right_frame,
            text="-- PWM",
            bg=self.COLOR_DARK_BLUE,
            fg=self.COLOR_GREEN,
            font=("Arial", 11, "bold")
        )
        self.right_speed_label.pack(side=tk.LEFT, padx=10)
        
        # LiDAR RPM
        lidar_frame = tk.Frame(frame, bg=self.COLOR_DARK_BLUE)
        lidar_frame.pack(fill=tk.X, padx=10, pady=5)
        
        tk.Label(lidar_frame, text="🔴 LiDAR:", bg=self.COLOR_DARK_BLUE, fg=self.COLOR_CREAM).pack(side=tk.LEFT)
        self.lidar_rpm_label = tk.Label(
                                    
            lidar_frame,
            text="-- RPM",
            bg=self.COLOR_DARK_BLUE,
            fg=self.COLOR_GREEN,
            font=("Arial", 11, "bold")
        )
        self.lidar_rpm_label.pack(side=tk.LEFT, padx=10)
    
    def create_log_panel(self, parent):
        """Панель логов."""
        frame = tk.LabelFrame(
            parent,
            text="📋 Логи",
            bg=self.COLOR_DARK_BLUE,
            fg=self.COLOR_GREEN,
            font=("Arial", 11, "bold"),
            relief=tk.RAISED,
            bd=2
        )
        frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Текстовое поле логов
        self.log_text = scrolledtext.ScrolledText(
            frame,
            height=15,
            bg=self.COLOR_DARK_BLUE,
            fg=self.COLOR_CREAM,
            font=("Courier", 9),
            relief=tk.SUNKEN,
            bd=1
        )
        self.log_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Кнопка очистки
        tk.Button(
            frame,
            text="🗑 Очистить логи",
            bg=self.COLOR_TEAL,
            fg=self.COLOR_CREAM,
            font=("Arial", 9, "bold"),
            command=self.clear_logs,
            relief=tk.RAISED,
            bd=2
        ).pack(pady=5)
    
    # ===== МЕТОДЫ ПОДКЛЮЧЕНИЯ =====
    
    def _try_connect(self, ip, port):
        """Попытка подключения к IP:port."""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(0.5)
            result = sock.connect_ex((ip, port))
            sock.close()
            return result == 0
        except:
            return False

    def _get_local_ip(self):
        """Получить локальный IP адрес."""
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.settimeout(1.0)
            s.connect(('8.8.8.8', 80))
            local_ip = s.getsockname()[0]
            s.close()
            return local_ip
        except:
            return None

    def discover_esp32(self, port=3333):
        """Автоматическое обнаружение ESP32 (параллельное сканирование)."""
        self.log("🔍 Поиск ESP32 в сети...")

        # Сначала проверяем наиболее вероятные адреса
        common_ips = [
            '192.168.0.23', '192.168.0.100', '192.168.0.101', '192.168.0.102',
            '192.168.1.23', '192.168.1.100', '192.168.1.101',
            '192.168.4.1',  '192.168.4.2',   '192.168.4.10',
            '192.168.8.1',  '192.168.8.2',
        ]
        for ip in common_ips:
            if self._try_connect(ip, port):
                self.log(f"✓ ESP32 найден: {ip}")
                return ip

        # Параллельное сканирование локальной подсети
        local_ip = self._get_local_ip()
        if local_ip:
            subnet = local_ip.rsplit('.', 1)[0]
            self.log(f"🔍 Сканирование {subnet}.0/24 (параллельно)...")
            import concurrent.futures
            found = [None]
            with concurrent.futures.ThreadPoolExecutor(max_workers=32) as ex:
                futs = {ex.submit(self._try_connect, f"{subnet}.{i}", port): i
                        for i in range(1, 255)}
                for f in concurrent.futures.as_completed(futs):
                    if f.result() and found[0] is None:
                        found[0] = f"{subnet}.{futs[f]}"
            if found[0]:
                self.log(f"✓ ESP32 найден: {found[0]}")
                return found[0]

        self.log("❌ ESP32 не найден в сети")
        return None

    def connect(self):
        """Подключиться к ESP32 (в фоновом потоке, чтобы не блокировать GUI)."""
        # Заблокировать кнопку чтобы не кликали дважды
        self.connect_btn.config(state=tk.DISABLED, text="⏳ Подключение...")
        self.status_label.config(text="● Подключение...", fg="#F39C12")
        self.root.update()

        t = threading.Thread(target=self._connect_worker, daemon=True)
        t.start()

    def _connect_worker(self):
        """Фоновый поток подключения."""
        robot_ip = self.ip_entry.get().strip()

        # Авто-обнаружение
        if robot_ip.lower() == 'auto':
            self.log("🔧 Автоопределение IP...")
            try:
                port = int(self.port_entry.get())
            except Exception:
                port = 3333
            found = self.discover_esp32(port)
            if found:
                robot_ip = found
                # Обновить поле IP из основного потока
                self.root.after(0, lambda: (
                    self.ip_entry.delete(0, tk.END),
                    self.ip_entry.insert(0, robot_ip)
                ))
            else:
                self.root.after(0, lambda: (
                    messagebox.showerror(
                        "ESP32 не найден",
                        "Не удалось найти ESP32 автоматически.\n"
                        "Введите IP-адрес вручную и нажмите Подключить."),
                    self.connect_btn.config(state=tk.NORMAL, text="✓ Подключить"),
                    self.status_label.config(text="● Не подключено", fg="#E74C3C")
                ))
                return

        try:
            tcp_port = int(self.port_entry.get())
        except Exception:
            self.root.after(0, lambda: (
                messagebox.showerror("Ошибка", "Неверный номер порта!"),
                self.connect_btn.config(state=tk.NORMAL, text="✓ Подключить"),
                self.status_label.config(text="● Не подключено", fg="#E74C3C")
            ))
            return

        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(8.0)
            sock.connect((robot_ip, tcp_port))

            # Успех — обновляем состояние из основного потока
            self.socket = sock
            self.robot_ip = robot_ip
            self.tcp_port = tcp_port
            self.connected = True

            self.stop_receive = False
            self.receive_thread = threading.Thread(target=self.receive_loop, daemon=True)
            self.receive_thread.start()

            self.root.after(0, self._on_connect_success)

        except socket.timeout:
            self.root.after(0, lambda: self._on_connect_error(
                f"Соединение с {robot_ip}:{tcp_port} прервано по таймауту.\n"
                "Проверьте:\n"
                "  • ESP32 включён и подключён к Wi-Fi\n"
                "  • Компьютер в той же сети\n"
                "  • IP-адрес верный"
            ))
        except ConnectionRefusedError:
            self.root.after(0, lambda: self._on_connect_error(
                f"Соединение отклонено ({robot_ip}:{tcp_port}).\n"
                "ESP32 включён, но TCP-сервер не отвечает."
            ))
        except Exception as e:
            self.root.after(0, lambda: self._on_connect_error(str(e)))

    def _on_connect_success(self):
        """Вызывается в основном потоке после успешного подключения."""
        self.status_label.config(text="● Подключено", fg=self.COLOR_GREEN)
        self.connect_btn.config(state=tk.DISABLED, text="✓ Подключить")
        self.disconnect_btn.config(state=tk.NORMAL)
        self.ip_entry.config(state=tk.DISABLED)
        self.port_entry.config(state=tk.DISABLED)
        self.log(f"✓ Подключено к ESP32 ({self.robot_ip}:{self.tcp_port})")

    def _on_connect_error(self, message: str):
        """Вызывается в основном потоке при ошибке подключения."""
        self.connect_btn.config(state=tk.NORMAL, text="✓ Подключить")
        self.status_label.config(text="● Не подключено", fg="#E74C3C")
        self.log(f"✗ Ошибка: {message.splitlines()[0]}")
        messagebox.showerror("Ошибка подключения", message)
    
    def disconnect(self):
        """Отключиться от ESP32."""
        try:
            self.stop_receive = True
            if self.socket:
                self.socket.close()
            
            self.connected = False
            self.status_label.config(text="● Не подключено", fg="#E74C3C")
            self.connect_btn.config(state=tk.NORMAL)
            self.disconnect_btn.config(state=tk.DISABLED)
            self.ip_entry.config(state=tk.NORMAL)
            self.port_entry.config(state=tk.NORMAL)
            
            self.log("✗ Отключено от ESP32")
        except Exception as e:
            self.log(f"Ошибка отключения: {str(e)}")
    
    def send_command(self, command):
        """Отправить команду на ESP32."""
        if not self.connected:
            messagebox.showwarning("Предупреждение", "Не подключено к роботу!")
            return False
        
        try:
            with self.lock:
                msg = command + '\n'
                self.socket.sendall(msg.encode('utf-8'))
                self.log(f"→ Отправлено: {command}")
            return True
        except Exception as e:
            self.log(f"✗ Ошибка отправки: {str(e)}")
            self.connected = False
            return False
    
    def receive_loop(self):
        """Цикл приема данных."""
        while self.connected and not self.stop_receive:
            try:
                data = self.socket.recv(1024)
                if not data:
                    break
                
                with self.lock:
                    self.recv_buffer += data.decode('utf-8')
                
                while '\n' in self.recv_buffer:
                    line, self.recv_buffer = self.recv_buffer.split('\n', 1)
                    if line:
                        self.process_response(line.strip())
                        
            except socket.timeout:
                continue
            except Exception as e:
                self.log(f"✗ Ошибка приема: {str(e)}")
                break
    
    def process_response(self, response):
        """Обработать ответ от ESP32."""
        self.log(f"← Получено: {response}")
        
        # Парсинг статуса
        if response.startswith("STATUS:"):
            self.parse_status(response)
    
    def parse_status(self, status_str):
        """Распарсить статус."""
        try:
            data = status_str[7:]
            params = data.split(',')
            
            for param in params:
                if '=' in param:
                    key, value = param.split('=', 1)
                    # Accept several possible keys from different firmwares
                    if key in ('LEFT_SPEED', 'L_PWM', 'L_TARGET', 'L_PWM_CMD'):
                        self.left_speed_label.config(text=f"{value} PWM")
                    elif key in ('RIGHT_SPEED', 'R_PWM', 'R_TARGET', 'R_PWM_CMD'):
                        self.right_speed_label.config(text=f"{value} PWM")
                    elif key == 'BATTERY':
                        # приведем значение к числу (удалим 'V' если есть)
                        try:
                            volt_str = value.strip().upper().replace('V','')
                            voltage = float(volt_str)
                            # параметры для расчета процента
                            const_full = 12.6   # напряжение полностью заряженной батареи
                            const_empty = 10.5  # напряжение разряженной батареи
                            pct = int(round((voltage - const_empty) / (const_full - const_empty) * 100))
                            if pct < 0: pct = 0
                            if pct > 100: pct = 100
                            self.battery_label.config(text=f"{voltage:.2f} V ({pct}%)")
                        except:
                            # если парсинг не удался — показать как есть
                            self.battery_label.config(text=value)
                    elif key == 'LIDAR_RPM':
                        self.lidar_rpm_label.config(text=f"{value} RPM")
        except:
            pass
    
    # ===== КОМАНДЫ УПРАВЛЕНИЯ =====
    
    def move_forward(self):
        """Движение вперед."""
        speed = self.speed_slider.get()
        pwm = int((abs(speed) / 100.0) * 255)
        self.send_command(f'set_motor_left:{pwm}')
        self.send_command(f'set_motor_right:{pwm}')
    
    def move_backward(self):
        """Движение назад."""
        speed = self.speed_slider.get()
        pwm = -int((abs(speed) / 100.0) * 255)
        self.send_command(f'set_motor_left:{pwm}')
        self.send_command(f'set_motor_right:{pwm}')
    
    def move_left(self):
        """Вращение влево."""
        speed = self.speed_slider.get()
        pwm = int((abs(speed) / 100.0) * 255)
        self.send_command(f'set_motor_left:{-pwm}')
        self.send_command(f'set_motor_right:{pwm}')
    
    def move_right(self):
        """Вращение вправо."""
        speed = self.speed_slider.get()
        pwm = int((abs(speed) / 100.0) * 255)
        self.send_command(f'set_motor_left:{pwm}')
        self.send_command(f'set_motor_right:{-pwm}')
    
    def stop(self):
        """Остановка."""
        self.send_command('disable_motors')
        self.speed_slider.set(0)
    
    def set_lidar_pwm(self):
        """Установить PWM LiDAR."""
        try:
            pwm = int(self.pwm_entry.get())
            if 0 <= pwm <= 255:
                self.send_command(f'set_lidar_pwm:{pwm}')
            else:
                messagebox.showerror("Ошибка", "PWM должен быть 0-255!")
        except:
            messagebox.showerror("Ошибка", "Неверное значение PWM!")
    
    def enable_lidar(self):
        """Включить LiDAR."""
        self.set_lidar_pwm()
    
    def disable_lidar(self):
        """Отключить LiDAR."""
        self.send_command('set_lidar_pwm:0')
    
    def request_status(self):
        """Запросить статус."""
        self.send_command('get_battery')
        self.send_command('get_motor_speed')
        self.send_command('get_wifi_status')
    
    def reset_esp32(self):
        """Перезагрузить ESP32."""
        if messagebox.askyesno("Подтверждение", "Перезагрузить ESP32?"):
            self.send_command('reset')
            self.disconnect()
    
    def log(self, message):
        """Добавить сообщение в логи."""
        timestamp = datetime.now().strftime("%H:%M:%S")
        log_message = f"[{timestamp}] {message}\n"
        
        self.log_text.insert(tk.END, log_message)
        self.log_text.see(tk.END)
        self.root.update()
    
    def clear_logs(self):
        """Очистить логи."""
        self.log_text.delete(1.0, tk.END)


def main():
    """Главная функция."""
    root = tk.Tk()
    gui = RobotGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
