#!/usr/bin/env python3
"""
GUI программа для прослушивания UDP порта
Помогает диагностировать приходят ли данные от ESP32
"""

import socket
import threading
import tkinter as tk
from tkinter import scrolledtext, ttk
import time

class UDPListenerGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("UDP Listener Test")
        self.root.geometry("700x500")
        
        self.running = False
        self.sock = None
        self.packet_count = 0
        self.start_time = None
        
        # Настройка UI
        self.setup_ui()
        
    def setup_ui(self):
        # Заголовок
        title = tk.Label(self.root, text="UDP Listener Test", font=("Arial", 16, "bold"))
        title.pack(pady=10)
        
        # Настройки порта
        frame = ttk.Frame(self.root)
        frame.pack(pady=5)
        
        tk.Label(frame, text="Порт:").grid(row=0, column=0, padx=5)
        self.port_entry = ttk.Entry(frame, width=10)
        self.port_entry.insert(0, "4444")
        self.port_entry.grid(row=0, column=1, padx=5)
        
        # Кнопки
        btn_frame = ttk.Frame(self.root)
        btn_frame.pack(pady=5)
        
        self.start_btn = ttk.Button(btn_frame, text="Старт", command=self.start_listening)
        self.start_btn.grid(row=0, column=0, padx=5)
        
        self.stop_btn = ttk.Button(btn_frame, text="Стоп", command=self.stop_listening, state=tk.DISABLED)
        self.stop_btn.grid(row=0, column=1, padx=5)
        
        self.clear_btn = ttk.Button(btn_frame, text="Очистить", command=self.clear_log)
        self.clear_btn.grid(row=0, column=2, padx=5)
        
        # Статистика
        stats_frame = ttk.Frame(self.root)
        stats_frame.pack(pady=5, fill=tk.X, padx=10)
        
        tk.Label(stats_frame, text="Статус:").grid(row=0, column=0, sticky="w")
        self.status_label = tk.Label(stats_frame, text="Остановлен", foreground="red")
        self.status_label.grid(row=0, column=1, sticky="w")
        
        tk.Label(stats_frame, text="Пакетов:").grid(row=1, column=0, sticky="w")
        self.packet_label = tk.Label(stats_frame, text="0")
        self.packet_label.grid(row=1, column=1, sticky="w")
        
        # Лог
        tk.Label(self.root, text="Лог:").pack(pady=5)
        
        self.log_text = scrolledtext.ScrolledText(self.root, width=80, height=20, font=("Courier", 9))
        self.log_text.pack(pady=5, padx=10, fill=tk.BOTH, expand=True)
        
    def log(self, message):
        self.log_text.insert(tk.END, message + "\n")
        self.log_text.see(tk.END)
        
    def clear_log(self):
        self.log_text.delete(1.0, tk.END)
        
    def start_listening(self):
        try:
            port = int(self.port_entry.get())
        except ValueError:
            self.log("Ошибка: неверный номер порта")
            return
            
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock.bind(('0.0.0.0', port))
            self.sock.settimeout(0.5)
            
            self.running = True
            self.packet_count = 0
            self.start_time = time.time()
            
            # Запуск потока
            self.thread = threading.Thread(target=self.listen_loop, daemon=True)
            self.thread.start()
            
            self.start_btn.config(state=tk.DISABLED)
            self.stop_btn.config(state=tk.NORMAL)
            self.port_entry.config(state=tk.DISABLED)
            self.status_label.config(text="Слушаю...", foreground="green")
            self.log(f"=== Старт прослушивания порта {port} ===")
            
        except Exception as e:
            self.log(f"Ошибка: {e}")
            
    def stop_listening(self):
        self.running = False
        if self.sock:
            try:
                self.sock.close()
            except:
                pass
                
        self.start_btn.config(state=tk.NORMAL)
        self.stop_btn.config(state=tk.DISABLED)
        self.port_entry.config(state=tk.NORMAL)
        self.status_label.config(text="Остановлен", foreground="red")
        self.log("=== Остановлено ===")
        
    def listen_loop(self):
        while self.running:
            try:
                data, addr = self.sock.recvfrom(4096)
                self.packet_count += 1
                current_time = time.time()
                elapsed = current_time - self.start_time
                
                msg = f"[{elapsed:.2f}s] От {addr}: {len(data)} байт"
                self.log(msg)
                
                # Определяем тип
                if len(data) >= 1:
                    if data[0] == 0xAA:
                        self.log(f"  >>> LiDAR пакет (0xAA)")
                    elif data[0] == 0xBB:
                        self.log(f"  >>> Telemetry (0xBB)")
                        if len(data) >= 23:
                            try:
                                ip = ".".join(str(b) for b in data[1:5])
                                tcp_port = (data[5] << 8) | data[6]
                                self.log(f"      IP: {ip}, TCP: {tcp_port}")
                            except:
                                pass
                    else:
                        self.log(f"  >>> Неизвестный тип: 0x{data[0]:02X}")
                        
                self.log(f"      Hex: {data[:30].hex()}...")
                self.packet_label.config(text=str(self.packet_count))
                
            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    self.log(f"Ошибка: {e}")
                break
                
        self.root.after(0, self.stop_listening)

def main():
    root = tk.Tk()
    app = UDPListenerGUI(root)
    root.mainloop()

if __name__ == '__main__':
    main()
