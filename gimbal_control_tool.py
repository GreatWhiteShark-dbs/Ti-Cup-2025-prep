"""云台数据包测试例程"""

import tkinter as tk
from tkinter import ttk, messagebox
import serial
import struct
import threading
import time
from serial.tools import list_ports

class GimbalControlTool:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("云台控制工具")
        self.root.geometry("800x1000")  # 增大窗口尺寸
        self.root.resizable(True, True)  # 允许调整窗口大小
        
        # 串口配置
        self.serial_port = None
        self.is_connected = False
        
        # 云台状态
        self.current_pitch = 0.0
        self.current_yaw = 0.0
        
        # 创建界面
        self.create_widgets()
        
        # 接收线程
        self.receive_thread = None
        self.running = False
        
    def create_widgets(self):
        """创建界面组件"""
        # 主框架
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # 串口连接区域
        self.create_connection_frame(main_frame)
        
        # 云台控制区域
        self.create_control_frame(main_frame)
        
        # 状态显示区域
        self.create_status_frame(main_frame)
        
        # 日志区域
        self.create_log_frame(main_frame)
        
    def create_connection_frame(self, parent):
        """创建串口连接区域"""
        conn_frame = ttk.LabelFrame(parent, text="串口连接", padding="5")
        conn_frame.grid(row=0, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(0, 10))
        
        # 串口选择
        ttk.Label(conn_frame, text="串口:").grid(row=0, column=0, padx=(0, 5))
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(conn_frame, textvariable=self.port_var, width=15)
        self.port_combo.grid(row=0, column=1, padx=(0, 10))
        
        # 波特率选择
        ttk.Label(conn_frame, text="波特率:").grid(row=0, column=2, padx=(0, 5))
        self.baud_var = tk.StringVar(value="9600")
        baud_combo = ttk.Combobox(conn_frame, textvariable=self.baud_var, width=10)
        baud_combo['values'] = ('9600', '115200', '460800')
        baud_combo.grid(row=0, column=3, padx=(0, 10))
        
        # 刷新和连接按钮
        ttk.Button(conn_frame, text="刷新", command=self.refresh_ports).grid(row=0, column=4, padx=(0, 5))
        self.connect_btn = ttk.Button(conn_frame, text="连接", command=self.toggle_connection)
        self.connect_btn.grid(row=0, column=5)
        
        # 连接状态指示
        self.status_label = ttk.Label(conn_frame, text="未连接", foreground="red")
        self.status_label.grid(row=0, column=6, padx=(10, 0))
        
        # 初始化串口列表
        self.refresh_ports()
        
    def create_control_frame(self, parent):
        """创建云台控制区域"""
        control_frame = ttk.LabelFrame(parent, text="云台控制", padding="5")
        control_frame.grid(row=1, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(0, 10))
        
        # 角度输入区域
        angle_frame = ttk.Frame(control_frame)
        angle_frame.grid(row=0, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=(0, 10))
        
        # 俯仰角控制
        ttk.Label(angle_frame, text="俯仰角:").grid(row=0, column=0, padx=(0, 5))
        self.pitch_var = tk.DoubleVar(value=0.0)
        pitch_scale = ttk.Scale(angle_frame, from_=-360, to=360, variable=self.pitch_var, 
                               orient=tk.HORIZONTAL, length=200)
        pitch_scale.grid(row=0, column=1, padx=(0, 10))
        self.pitch_entry = ttk.Entry(angle_frame, textvariable=self.pitch_var, width=8)
        self.pitch_entry.grid(row=0, column=2, padx=(0, 5))
        ttk.Label(angle_frame, text="°").grid(row=0, column=3)
        
        # 偏航角控制
        ttk.Label(angle_frame, text="偏航角:").grid(row=1, column=0, padx=(0, 5), pady=(5, 0))
        self.yaw_var = tk.DoubleVar(value=0.0)
        yaw_scale = ttk.Scale(angle_frame, from_=-360, to=360, variable=self.yaw_var, 
                             orient=tk.HORIZONTAL, length=200)
        yaw_scale.grid(row=1, column=1, padx=(0, 10), pady=(5, 0))
        self.yaw_entry = ttk.Entry(angle_frame, textvariable=self.yaw_var, width=8)
        self.yaw_entry.grid(row=1, column=2, padx=(0, 5), pady=(5, 0))
        ttk.Label(angle_frame, text="°").grid(row=1, column=3, pady=(5, 0))
        
        # 控制按钮区域
        btn_frame = ttk.Frame(control_frame)
        btn_frame.grid(row=1, column=0, columnspan=3, pady=(10, 0))
        
        ttk.Button(btn_frame, text="设置角度", command=self.set_angle).grid(row=0, column=0, padx=(0, 10))
        ttk.Button(btn_frame, text="归零", command=self.reset_gimbal).grid(row=0, column=1, padx=(0, 10))
        ttk.Button(btn_frame, text="停止", command=self.stop_gimbal).grid(row=0, column=2)
        
        # 快捷控制按钮
        quick_frame = ttk.LabelFrame(control_frame, text="快捷控制", padding="5")
        quick_frame.grid(row=2, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=(10, 0))
        
        # 小角度控制
        small_frame = ttk.Frame(quick_frame)
        small_frame.grid(row=0, column=0, columnspan=3, pady=(0, 5))
        ttk.Label(small_frame, text="小角度(15°):").grid(row=0, column=0, padx=(0, 10))
        ttk.Button(small_frame, text="上仰", command=lambda: self.quick_move(15, 0)).grid(row=0, column=1)
        ttk.Button(small_frame, text="左转", command=lambda: self.quick_move(0, 15)).grid(row=1, column=0)  # 改为正值
        ttk.Button(small_frame, text="归零", command=self.reset_gimbal).grid(row=1, column=1)
        ttk.Button(small_frame, text="右转", command=lambda: self.quick_move(0, -15)).grid(row=1, column=2)  # 改为负值
        ttk.Button(small_frame, text="下俯", command=lambda: self.quick_move(-15, 0)).grid(row=2, column=1)
        
        # 大角度控制
        large_frame = ttk.Frame(quick_frame)
        large_frame.grid(row=1, column=0, columnspan=3, pady=(5, 0))
        ttk.Label(large_frame, text="大角度(90°):").grid(row=0, column=0, padx=(0, 10))
        ttk.Button(large_frame, text="上仰", command=lambda: self.quick_move(90, 0)).grid(row=0, column=1)
        ttk.Button(large_frame, text="左转", command=lambda: self.quick_move(0, 90)).grid(row=1, column=0)  # 改为正值
        ttk.Button(large_frame, text="180°", command=lambda: self.quick_move(0, 180)).grid(row=1, column=1)
        ttk.Button(large_frame, text="右转", command=lambda: self.quick_move(0, -90)).grid(row=1, column=2)  # 改为负值
        ttk.Button(large_frame, text="下俯", command=lambda: self.quick_move(-90, 0)).grid(row=2, column=1)
        
    def create_status_frame(self, parent):
        """创建状态显示区域"""
        status_frame = ttk.LabelFrame(parent, text="云台状态", padding="5")
        status_frame.grid(row=2, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(0, 10))
        
        # 当前角度显示
        ttk.Label(status_frame, text="当前俯仰角:").grid(row=0, column=0, padx=(0, 5))
        self.current_pitch_label = ttk.Label(status_frame, text="0.0°", foreground="blue")
        self.current_pitch_label.grid(row=0, column=1, padx=(0, 20))
        
        ttk.Label(status_frame, text="当前偏航角:").grid(row=0, column=2, padx=(0, 5))
        self.current_yaw_label = ttk.Label(status_frame, text="0.0°", foreground="blue")
        self.current_yaw_label.grid(row=0, column=3)
        
    def create_log_frame(self, parent):
        """创建日志区域"""
        log_frame = ttk.LabelFrame(parent, text="通信日志", padding="5")
        log_frame.grid(row=3, column=0, columnspan=2, sticky=(tk.W, tk.E, tk.N, tk.S), pady=(0, 10))
        
        # 日志文本框 - 增大尺寸
        self.log_text = tk.Text(log_frame, height=15, width=90)  # 增大高度和宽度
        scrollbar = ttk.Scrollbar(log_frame, orient=tk.VERTICAL, command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=scrollbar.set)
        
        self.log_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        scrollbar.grid(row=0, column=1, sticky=(tk.N, tk.S))
        
        # 清除日志按钮
        ttk.Button(log_frame, text="清除日志", command=self.clear_log).grid(row=1, column=0, pady=(5, 0))
        
    def refresh_ports(self):
        """刷新串口列表"""
        ports = [port.device for port in list_ports.comports()]
        self.port_combo['values'] = ports
        if ports:
            self.port_combo.set(ports[0])
            
    def toggle_connection(self):
        """切换串口连接状态"""
        if not self.is_connected:
            self.connect_serial()
        else:
            self.disconnect_serial()
            
    def connect_serial(self):
        """连接串口"""
        try:
            port = self.port_var.get()
            baud = int(self.baud_var.get())
            
            if not port:
                messagebox.showerror("错误", "请选择串口")
                return
                
            self.serial_port = serial.Serial(port, baud, timeout=1)
            self.is_connected = True
            
            # 更新界面
            self.connect_btn.config(text="断开")
            self.status_label.config(text="已连接", foreground="green")
            
            # 启动接收线程
            self.running = True
            self.receive_thread = threading.Thread(target=self.receive_data)
            self.receive_thread.daemon = True
            self.receive_thread.start()
            
            self.log_message(f"串口连接成功: {port} @ {baud}")
            
        except Exception as e:
            messagebox.showerror("连接错误", f"无法连接串口: {str(e)}")
            self.log_message(f"连接失败: {str(e)}")
            
    def disconnect_serial(self):
        """断开串口连接"""
        try:
            self.running = False
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()
            
            self.is_connected = False
            self.connect_btn.config(text="连接")
            self.status_label.config(text="未连接", foreground="red")
            
            self.log_message("串口连接已断开")
            
        except Exception as e:
            self.log_message(f"断开连接错误: {str(e)}")
            
    def send_gimbal_packet(self, command, pitch_angle, yaw_angle):
        """发送云台控制数据包"""
        if not self.is_connected or not self.serial_port:
            messagebox.showwarning("警告", "请先连接串口")
            return False
            
        try:
            # 构建数据包 (小端序)
            start_flag = 0xCC77
            
            # 打包数据
            data = struct.pack('<HBff', start_flag, command, pitch_angle, yaw_angle)
            
            # 计算校验和
            checksum = command
            pitch_bytes = struct.pack('<f', pitch_angle)
            yaw_bytes = struct.pack('<f', yaw_angle)
            for b in pitch_bytes + yaw_bytes:
                checksum += b
            checksum = checksum & 0xFF
            
            # 添加校验和
            packet = data + struct.pack('B', checksum)
            
            # 发送数据包
            self.serial_port.write(packet)
            
            self.log_message(f"发送: 命令=0x{command:02X}, 俯仰={pitch_angle:.1f}°, 偏航={yaw_angle:.1f}°, 校验和=0x{checksum:02X}")
            return True
            
        except Exception as e:
            self.log_message(f"发送错误: {str(e)}")
            return False
            
    def set_angle(self):
        """设置云台角度"""
        pitch = self.pitch_var.get()
        yaw = self.yaw_var.get()
        
        # 限制角度范围到-360到360度
        pitch = max(-360, min(360, pitch))
        yaw = max(-360, min(360, yaw))
        
        if self.send_gimbal_packet(0x01, pitch, yaw):  # GIMBAL_CMD_SET_ANGLE
            self.current_pitch = pitch
            self.current_yaw = yaw
            self.update_status_display()
            
    def reset_gimbal(self):
        """云台归零"""
        if self.send_gimbal_packet(0x04, 0.0, 0.0):  # GIMBAL_CMD_RESET
            self.current_pitch = 0.0
            self.current_yaw = 0.0
            self.pitch_var.set(0.0)
            self.yaw_var.set(0.0)
            self.update_status_display()
            
    def stop_gimbal(self):
        """停止云台"""
        self.send_gimbal_packet(0x03, 0.0, 0.0)  # GIMBAL_CMD_STOP
        
    def quick_move(self, pitch_delta, yaw_delta):
        """快捷移动"""
        new_pitch = self.current_pitch + pitch_delta
        new_yaw = self.current_yaw + yaw_delta
        
        # 限制角度范围到-360到360度
        new_pitch = max(-360, min(360, new_pitch))
        new_yaw = max(-360, min(360, new_yaw))
        
        self.pitch_var.set(new_pitch)
        self.yaw_var.set(new_yaw)
        self.set_angle()
        
    def update_status_display(self):
        """更新状态显示"""
        self.current_pitch_label.config(text=f"{self.current_pitch:.1f}°")
        self.current_yaw_label.config(text=f"{self.current_yaw:.1f}°")
        
    def receive_data(self):
        """接收数据线程"""
        while self.running and self.serial_port and self.serial_port.is_open:
            try:
                if self.serial_port.in_waiting > 0:
                    data = self.serial_port.read(self.serial_port.in_waiting)
                    if data:
                        try:
                            # 尝试用UTF-8解码接收到的数据
                            text_data = data.decode('utf-8', errors='replace')
                            self.log_message(f"接收: {text_data}")
                        except UnicodeDecodeError:
                            # 如果UTF-8解码失败，显示16进制作为备选
                            hex_data = ' '.join([f'{b:02X}' for b in data])
                            self.log_message(f"接收(HEX): {hex_data}")
                    
                time.sleep(0.1)
                
            except Exception as e:
                if self.running:  # 只在运行时记录错误
                    self.log_message(f"接收错误: {str(e)}")
                break
                
    def log_message(self, message):
        """添加日志消息"""
        timestamp = time.strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] {message}\n"
        
        # 在主线程中更新GUI
        self.root.after(0, self._update_log, log_entry)
        
    def _update_log(self, message):
        """更新日志显示"""
        self.log_text.insert(tk.END, message)
        self.log_text.see(tk.END)
        
        # 限制日志行数
        lines = self.log_text.get("1.0", tk.END).split('\n')
        if len(lines) > 100:
            self.log_text.delete("1.0", "10.0")
            
    def clear_log(self):
        """清除日志"""
        self.log_text.delete("1.0", tk.END)
        
    def on_closing(self):
        """程序关闭时的清理"""
        self.running = False
        if self.is_connected:
            self.disconnect_serial()
        self.root.destroy()
        
    def run(self):
        """运行程序"""
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.root.mainloop()

if __name__ == "__main__":
    app = GimbalControlTool()
    app.run()