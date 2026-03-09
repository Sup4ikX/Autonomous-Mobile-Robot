import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import socket
import threading
import queue
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import math

# Максимальное количество точек в одном секторе (из вашего кода)
MAX_POINTS_PER_SECTOR = 52

class UdpReceiver(threading.Thread):
    """Поток для приёма UDP-пакетов (без проверки CRC для лидара)."""
    def __init__(self, port, queue, stop_event):
        super().__init__()
        self.port = port
        self.queue = queue
        self.stop_event = stop_event
        self.sock = None
        self.daemon = True

    def run(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.5)
        try:
            self.sock.bind(('0.0.0.0', self.port))
        except Exception as e:
            self.queue.put(('error', f"Не удалось открыть порт {self.port}: {e}"))
            return
        print(f"UDP receiver started on port {self.port}")

        while not self.stop_event.is_set():
            try:
                data, addr = self.sock.recvfrom(65535)
                if len(data) < 1:
                    continue

                marker = data[0]

                # ----- Пакет телеметрии (маркер 0xBB) -----
                if marker == 0xBB and len(data) >= 15:
                    ip_bytes = data[1:5]
                    port_bytes = data[5:7]
                    left_bytes = data[7:11]
                    right_bytes = data[11:15]

                    ip_addr = '.'.join(str(b) for b in ip_bytes)
                    tcp_port = int.from_bytes(port_bytes, 'big')
                    left = int.from_bytes(left_bytes, 'little')
                    right = int.from_bytes(right_bytes, 'little')

                    self.queue.put(('telemetry', (ip_addr, tcp_port, left, right)))

                # ----- Пакет лидара (маркер 0xAA) -----
                elif marker == 0xAA:
                    # Минимальная длина: заголовок 8 байт + минимум данных (5) + CRC 2 = 15
                    if len(data) < 15:
                        self.queue.put(('lidar_error', 'Пакет слишком короткий'))
                        continue

                    # Длина пакета, включая CRC (байты 1-2, big-endian)
                    pkt_len = int.from_bytes(data[1:3], 'big')
                    # Проверяем, что пришло достаточно байт (включая CRC)
                    if len(data) < pkt_len:
                        self.queue.put(('lidar_error', 'Пакет неполный'))
                        continue

                    # Версия, тип, индикатор
                    version = data[3]
                    pkt_type = data[4]
                    indicator = data[5]

                    # Проверка типа пакета (должен быть 0x61)
                    if pkt_type != 0x61:
                        self.queue.put(('lidar_error', f'Неверный тип пакета: {hex(pkt_type)}'))
                        continue

                    # Проверка индикатора данных (должен быть 0xAD)
                    if indicator != 0xAD:
                        self.queue.put(('lidar_error', f'Неверный индикатор данных: {hex(indicator)}'))
                        continue

                    # Длина полезных данных (байты 6-7, big-endian)
                    data_len = int.from_bytes(data[6:8], 'big')

                    # CRC не проверяем – доверяем принятым данным

                    # Извлечение полей
                    speed_raw = data[8]
                    angle_offset_raw = int.from_bytes(data[9:11], 'big')
                    start_angle_raw = int.from_bytes(data[11:13], 'big')

                    # Количество измерений в пакете
                    remaining = data_len - 5
                    if remaining < 0 or remaining % 3 != 0:
                        self.queue.put(('lidar_error', f'Некорректная длина данных: {data_len}'))
                        continue
                    num_points = remaining // 3

                    # Извлекаем измерения (не выходим за пределы данных до CRC)
                    points = []
                    offset = 13  # начало первого измерения
                    for i in range(num_points):
                        if offset + 2 >= pkt_len - 2:  # защита от выхода за границы
                            break
                        signal = data[offset]
                        dist_raw = int.from_bytes(data[offset+1:offset+3], 'big')
                        # Угол пока без учёта сектора – его вычислим позже в GUI с учётом текущих настроек
                        # Передаём сырые данные: start_angle_raw и номер точки в пакете
                        points.append((i, signal, start_angle_raw, dist_raw))
                        offset += 3

                    packet_info = {
                        'speed': speed_raw * 0.05,
                        'angle_offset': angle_offset_raw * 0.01,
                        'start_angle_raw': start_angle_raw,
                        'num_points': num_points,
                        'points': points
                    }
                    self.queue.put(('lidar', packet_info))

                else:
                    # Неизвестный маркер
                    pass

            except socket.timeout:
                continue
            except Exception as e:
                print(f"UDP receive error: {e}")

        if self.sock:
            self.sock.close()
        print("UDP receiver stopped")


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("UDP Telemetry & Lidar Viewer")
        self.geometry("1200x800")
        self.protocol("WM_DELETE_WINDOW", self.on_close)

        self.data_queue = queue.Queue()
        self.stop_event = threading.Event()
        self.receiver_thread = None
        self.current_port = 4444

        # Параметры лидара
        self.sectors = 16                # количество секторов (может меняться)
        self.sector_angle = 360.0 / self.sectors
        self.reset_accumulation()

        self.create_widgets()
        self.start_receiver()
        self.process_queue()

        # Таймер для обновления графика (раз в 100 мс)
        self.update_graph_timer()

    def reset_accumulation(self):
        """Сброс накопленных данных при изменении числа секторов."""
        self.lidar_points = []          # список всех точек за текущий оборот (x,y)
        self.sector_received = [False] * self.sectors
        self.sector_data = [None] * self.sectors  # для хранения точек по секторам
        self.last_sector = -1

    def create_widgets(self):
        # Панель управления
        control_frame = tk.Frame(self)
        control_frame.pack(fill=tk.X, padx=5, pady=5)

        tk.Label(control_frame, text="UDP порт:").pack(side=tk.LEFT)
        self.port_entry = tk.Entry(control_frame, width=6)
        self.port_entry.insert(0, str(self.current_port))
        self.port_entry.pack(side=tk.LEFT, padx=5)

        self.apply_port_btn = tk.Button(control_frame, text="Применить порт", command=self.apply_port)
        self.apply_port_btn.pack(side=tk.LEFT, padx=5)

        tk.Label(control_frame, text="Секторов:").pack(side=tk.LEFT, padx=(20,5))
        self.sectors_spinbox = tk.Spinbox(control_frame, from_=1, to=32, width=4, command=self.on_sectors_change)
        self.sectors_spinbox.delete(0, tk.END)
        self.sectors_spinbox.insert(0, str(self.sectors))
        self.sectors_spinbox.pack(side=tk.LEFT)

        self.start_button = tk.Button(control_frame, text="Стоп", command=self.toggle_receiver)
        self.start_button.pack(side=tk.LEFT, padx=20)

        # Переключатель типа графика
        tk.Label(control_frame, text="Тип графика:").pack(side=tk.LEFT, padx=(20,5))
        self.graph_type = tk.StringVar(value="polar")
        tk.Radiobutton(control_frame, text="Полярный", variable=self.graph_type, value="polar", command=self.switch_graph_type).pack(side=tk.LEFT)
        tk.Radiobutton(control_frame, text="Декартов", variable=self.graph_type, value="cartesian", command=self.switch_graph_type).pack(side=tk.LEFT)

        # Основная панель с вкладками
        self.notebook = ttk.Notebook(self)
        self.notebook.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # Вкладка 1: Телеметрия и таблица лидара
        self.tab_table = ttk.Frame(self.notebook)
        self.notebook.add(self.tab_table, text="Таблица")
        self.create_table_tab()

        # Вкладка 2: График лидара
        self.tab_graph = ttk.Frame(self.notebook)
        self.notebook.add(self.tab_graph, text="График")
        self.create_graph_tab()

        # Метка для сообщений об ошибках
        self.error_var = tk.StringVar()
        error_label = tk.Label(self, textvariable=self.error_var, fg="red", anchor=tk.W)
        error_label.pack(fill=tk.X, padx=5, pady=2)

    def on_sectors_change(self):
        """Обработка изменения числа секторов."""
        try:
            new_sectors = int(self.sectors_spinbox.get())
            if new_sectors == self.sectors:
                return
            if new_sectors < 1 or new_sectors > 32:
                raise ValueError
            self.sectors = new_sectors
            self.sector_angle = 360.0 / self.sectors
            self.reset_accumulation()
            # Очищаем график
            if hasattr(self, 'scatter'):
                self.scatter.set_offsets([])
                self.canvas.draw_idle()
            self.error_var.set(f"Число секторов изменено на {self.sectors}")
        except ValueError:
            messagebox.showerror("Ошибка", "Введите число секторов от 1 до 32")

    def create_table_tab(self):
        """Вкладка с телеметрией и таблицей точек лидара."""
        main_panel = tk.PanedWindow(self.tab_table, orient=tk.HORIZONTAL, sashrelief=tk.RAISED, sashwidth=5)
        main_panel.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # Левая панель: телеметрия
        tele_frame = tk.LabelFrame(main_panel, text="Телеметрия", font=('Arial', 12, 'bold'))
        main_panel.add(tele_frame, width=300)

        self.tele_vars = {}
        labels = [("IP-адрес", "ip"),
                  ("TCP-порт", "port"),
                  ("Счётчик left", "left"),
                  ("Счётчик right", "right")]
        for i, (caption, key) in enumerate(labels):
            lbl = tk.Label(tele_frame, text=caption + ":", anchor=tk.W)
            lbl.grid(row=i, column=0, sticky=tk.W, padx=5, pady=2)
            var = tk.StringVar(value="---")
            self.tele_vars[key] = var
            val_lbl = tk.Label(tele_frame, textvariable=var, anchor=tk.W, width=20)
            val_lbl.grid(row=i, column=1, sticky=tk.W, padx=5, pady=2)

        # Правая панель: таблица лидара
        lidar_frame = tk.LabelFrame(main_panel, text="Данные лидара (последний пакет)", font=('Arial', 12, 'bold'))
        main_panel.add(lidar_frame, width=700)

        # Поля для параметров лидара
        param_frame = tk.Frame(lidar_frame)
        param_frame.pack(fill=tk.X, padx=5, pady=5)

        self.lidar_vars = {}
        params = [("Скорость (об/с)", "speed"),
                  ("Смещение угла (°)", "offset"),
                  ("Начальный угол (сырой)", "start_angle_raw"),
                  ("Точек в пакете", "num_points")]
        for i, (caption, key) in enumerate(params):
            lbl = tk.Label(param_frame, text=caption + ":", anchor=tk.W)
            lbl.grid(row=0, column=i*2, sticky=tk.W, padx=5)
            var = tk.StringVar(value="---")
            self.lidar_vars[key] = var
            val_lbl = tk.Label(param_frame, textvariable=var, anchor=tk.W, width=12)
            val_lbl.grid(row=0, column=i*2+1, sticky=tk.W, padx=5)

        # Текстовое поле для вывода точек
        self.lidar_text = scrolledtext.ScrolledText(lidar_frame, wrap=tk.WORD, height=25, font=('Courier', 9))
        self.lidar_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

    def create_graph_tab(self):
        """Вкладка с графиком лидара."""
        self.fig = Figure(figsize=(8, 8), dpi=80)
        self.create_graph_axes()
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.tab_graph)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self.scatter = self.ax.scatter([], [], s=2, c='blue', alpha=0.7)
        self.ax.set_title("Данные лидара")
        if self.graph_type.get() == 'polar':
            self.ax.set_ylim(0, 5000)
        else:
            self.ax.set_xlim(-5000, 5000)
            self.ax.set_ylim(-5000, 5000)
            self.ax.set_aspect('equal')
            self.ax.grid(True)
        self.canvas.draw()

    def create_graph_axes(self):
        self.fig.clear()
        if self.graph_type.get() == 'polar':
            self.ax = self.fig.add_subplot(111, projection='polar')
            self.ax.set_theta_zero_location('N')
            self.ax.set_theta_direction(-1)
            self.ax.set_rlim(0, 5000)
            self.ax.set_rlabel_position(90)
        else:
            self.ax = self.fig.add_subplot(111)
            self.ax.set_xlim(-5000, 5000)
            self.ax.set_ylim(-5000, 5000)
            self.ax.set_aspect('equal')
            self.ax.grid(True)
        self.ax.set_title("Данные лидара")

    def switch_graph_type(self):
        self.create_graph_axes()
        self.scatter = self.ax.scatter([], [], s=2, c='blue', alpha=0.7)
        if self.lidar_points:
            self.update_graph()
        self.canvas.draw()

    def update_graph(self):
        if not hasattr(self, 'scatter') or not self.lidar_points:
            return
        points = np.array(self.lidar_points)
        # Проверяем, что массив двумерный и имеет хотя бы два столбца
        if points.ndim != 2 or points.shape[1] < 2:
            return
        if self.graph_type.get() == 'polar':
            angles = np.arctan2(points[:, 1], points[:, 0])
            radii = np.hypot(points[:, 0], points[:, 1])
            self.scatter.set_offsets(np.c_[angles, radii])
        else:
            self.scatter.set_offsets(points)
        self.canvas.draw_idle()

    def apply_port(self):
        try:
            new_port = int(self.port_entry.get())
            if new_port < 1 or new_port > 65535:
                raise ValueError
            if new_port == self.current_port:
                return
            self.current_port = new_port
            self.stop_receiver()
            self.start_receiver()
            self.error_var.set(f"Порт изменён на {new_port}")
        except ValueError:
            messagebox.showerror("Ошибка", "Введите корректный номер порта (1-65535)")

    def start_receiver(self):
        if self.receiver_thread is None or not self.receiver_thread.is_alive():
            self.stop_event.clear()
            self.receiver_thread = UdpReceiver(self.current_port, self.data_queue, self.stop_event)
            self.receiver_thread.start()
            self.start_button.config(text="Стоп")

    def stop_receiver(self):
        self.stop_event.set()
        if self.receiver_thread:
            self.receiver_thread.join(timeout=1.0)
        self.start_button.config(text="Старт")

    def toggle_receiver(self):
        if self.stop_event.is_set():
            self.start_receiver()
        else:
            self.stop_receiver()

    def process_queue(self):
        try:
            while True:
                msg_type, data = self.data_queue.get_nowait()
                if msg_type == 'telemetry':
                    self.update_telemetry(data)
                elif msg_type == 'lidar':
                    self.process_lidar_packet(data)
                elif msg_type == 'lidar_error':
                    self.error_var.set(f"Ошибка лидара: {data}")
                elif msg_type == 'error':
                    self.error_var.set(f"Ошибка: {data}")
        except queue.Empty:
            pass
        self.after(100, self.process_queue)

    def update_telemetry(self, data):
        ip, port, left, right = data
        self.tele_vars['ip'].set(ip)
        self.tele_vars['port'].set(str(port))
        self.tele_vars['left'].set(str(left))
        self.tele_vars['right'].set(str(right))

    def process_lidar_packet(self, info):
        """Обработка пакета лидара."""
        # Обновляем параметры в таблице
        self.lidar_vars['speed'].set(f"{info['speed']:.2f}")
        self.lidar_vars['offset'].set(f"{info['angle_offset']:.2f}")
        self.lidar_vars['start_angle_raw'].set(str(info['start_angle_raw']))
        self.lidar_vars['num_points'].set(str(info['num_points']))

        # Вычисляем номер сектора на основе начального угла и текущего числа секторов
        start_angle = info['start_angle_raw'] * 0.01
        sector = int(round(start_angle / self.sector_angle)) % self.sectors

        # Данные пакета: список (idx, signal, start_angle_raw, dist_raw)
        points_raw = info['points']
        num_points = len(points_raw)

        # Если сектор уже получен в текущем обороте – начинаем новый оборот
        if self.sector_received[sector]:
            # Сброс накопленных секторов
            self.sector_received = [False] * self.sectors
            self.sector_data = [None] * self.sectors
            self.lidar_points = []

        # Вычисляем шаг угла внутри сектора
        step_angle = self.sector_angle / num_points if num_points > 0 else 0

        # Преобразуем точки в углы и расстояния
        points_for_display = []  # для таблицы (idx, signal, angle, dist_mm)
        points_for_graph = []    # для графика (x, y)
        for i, (idx, signal, start_raw, dist_raw) in enumerate(points_raw):
            angle = start_angle + i * step_angle
            dist_mm = dist_raw * 0.25
            points_for_display.append((idx, signal, angle, dist_mm))
            # Для графика: декартовы координаты
            rad = math.radians(angle)
            x = dist_mm * math.cos(rad)
            y = dist_mm * math.sin(rad)
            points_for_graph.append((x, y))

        # Сохраняем в массив сектора
        self.sector_data[sector] = points_for_graph
        self.sector_received[sector] = True

        # Обновляем таблицу последнего пакета (для наглядности)
        self.lidar_text.delete(1.0, tk.END)
        self.lidar_text.insert(tk.END, f"{'№':>3} {'Сигнал':>6} {'Угол (°)':>10} {'Дистанция (мм)':>15}\n")
        self.lidar_text.insert(tk.END, "-" * 40 + "\n")
        for (idx, signal, angle, dist) in points_for_display:
            self.lidar_text.insert(tk.END, f"{idx+1:3d} {signal:6d} {angle:10.2f} {dist:15.1f}\n")
        self.lidar_text.see(tk.END)

        # Очищаем ошибки (если были)
        self.error_var.set("")

    def update_graph_timer(self):
        """Периодически обновляет график на основе накопленных секторов."""
        # Собираем все точки из полученных секторов
        all_points = []
        for sec in range(self.sectors):
            if self.sector_received[sec] and self.sector_data[sec] is not None:
                all_points.extend(self.sector_data[sec])
        if all_points:
            self.lidar_points = all_points
            self.update_graph()
        self.after(100, self.update_graph_timer)

    def on_close(self):
        self.stop_receiver()
        self.destroy()


if __name__ == "__main__":
    app = App()
    app.mainloop()