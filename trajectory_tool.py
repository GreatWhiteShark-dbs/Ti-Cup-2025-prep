import cv2
import numpy as np
import serial
import threading
import time
import struct
from collections import deque

class TrajectoryTool:
    def __init__(self):
        # 串口配置
        self.serial_port = None
        self.port_name = "COM7"
        self.baud_rate = 9600
        
        # 画布配置
        self.canvas_size = (800, 600)  # 画布大小
        self.canvas = np.zeros((self.canvas_size[1], self.canvas_size[0], 3), dtype=np.uint8)
        self.canvas.fill(255)  # 白色背景
        
        # 轨迹点配置
        self.trajectory_range = 1000  # 轨迹点范围
        self.scale_x = self.canvas_size[0] / self.trajectory_range  # X轴缩放比例
        self.scale_y = self.canvas_size[1] / self.trajectory_range  # Y轴缩放比例
        
        # 鼠标绘制相关
        self.drawing = False
        self.last_point = None
        self.last_send_time = 0
        self.send_interval = 0.1  # 0.3秒发送间隔（降低发送速度）
        
        # 接收轨迹点
        self.received_points = deque(maxlen=10000)
        self.receiving_mode = False
        
        # 线程控制
        self.running = True
        self.receive_thread = None
        
        # 指令模式
        self.command_mode = False
        
    def init_serial(self):
        """初始化串口"""
        try:
            self.serial_port = serial.Serial(
                port=self.port_name,
                baudrate=self.baud_rate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1
            )
            print(f"串口 {self.port_name} 初始化成功")
            return True
        except Exception as e:
            print(f"串口初始化失败: {e}")
            return False
    
    def close_serial(self):
        """关闭串口"""
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            print("串口已关闭")
    
    def send_command(self, command):
        """发送单字符指令"""
        if not self.serial_port or not self.serial_port.is_open:
            print("串口未连接")
            return False
        
        try:
            # 发送单字符指令
            self.serial_port.write(command.encode('ascii'))
            print(f"发送指令: {command}")
            return True
        except Exception as e:
            print(f"发送指令失败: {e}")
            return False
    
    def canvas_to_trajectory(self, x, y):
        """将画布坐标转换为轨迹坐标"""
        # 反转X坐标：画布左边对应轨迹坐标1000，画布右边对应轨迹坐标0
        traj_x = 1000 - int(x / self.scale_x)
        traj_y = int(y / self.scale_y)
        # 确保坐标在有效范围内
        traj_x = max(0, min(1000, traj_x))
        traj_y = max(0, min(1000, traj_y))
        return traj_x, traj_y
    
    def trajectory_to_canvas(self, traj_x, traj_y):
        """将轨迹坐标转换为画布坐标"""
        # 反转X坐标：轨迹坐标1000对应画布左边，轨迹坐标0对应画布右边
        x = int((1000 - traj_x) * self.scale_x)
        y = int(traj_y * self.scale_y)
        return x, y
    
    def send_trajectory_point(self, x, y):
        """发送轨迹点（通过云台控制数据包）"""
        if not self.serial_port or not self.serial_port.is_open:
            return
        
        try:
            # 将轨迹坐标转换为云台角度
            # X坐标范围: 0-1000 -> 偏航角度范围: -45° 到 +45° (修正方向)
            yaw_angle = -45.0 + (float(x) * 90.0 / 1000.0)
            
            # Y坐标范围: 0-1000 -> 俯仰角度范围: +30° 到 -30° (上正下负)
            pitch_angle = 30.0 - (float(y) * 60.0 / 1000.0)
            
            # 限制角度范围
            if yaw_angle > 45.0:
                yaw_angle = 45.0
            if yaw_angle < -45.0:
                yaw_angle = -45.0
            if pitch_angle > 30.0:
                pitch_angle = 30.0
            if pitch_angle < -30.0:
                pitch_angle = -30.0
            
            # 云台控制数据包格式: 起始标志(2字节) + 命令(1字节) + 俯仰角(4字节) + 偏航角(4字节) + 校验和(1字节)
            start_flag = 0xCC77  # 云台控制数据包起始标志
            command = 0x01       # GIMBAL_CMD_TRAJECTORY_TRACK
            
            # 打包数据
            data = struct.pack('<HBff', start_flag, command, pitch_angle, yaw_angle)
            
            # 计算校验和（命令字节 + 角度数据的字节）
            checksum = command
            pitch_bytes = struct.pack('<f', pitch_angle)
            yaw_bytes = struct.pack('<f', yaw_angle)
            for b in pitch_bytes + yaw_bytes:
                checksum += b
            checksum = checksum & 0xFF
            
            # 添加校验和
            data += struct.pack('<B', checksum)
            
            self.serial_port.write(data)
            
            print(f"发送轨迹点: ({x}, {y}) -> 俯仰角: {pitch_angle:.1f}°, 偏航角: {yaw_angle:.1f}°, 校验和: {checksum}")
            
        except Exception as e:
            print(f"发送数据失败: {e}")
    
    def receive_trajectory_points(self):
        """接收轨迹点线程（解析云台控制数据包）"""
        buffer = bytearray()
        
        while self.running:
            if not self.serial_port or not self.serial_port.is_open:
                time.sleep(0.1)
                continue
            
            try:
                # 读取数据
                data = self.serial_port.read(32)
                if data:
                    buffer.extend(data)
                    
                    # 查找数据包
                    while len(buffer) >= 12:  # 云台控制数据包长度
                        # 查找起始标志 0xCC77
                        start_idx = -1
                        for i in range(len(buffer) - 1):
                            if buffer[i] == 0x77 and buffer[i + 1] == 0xCC:
                                start_idx = i
                                break
                        
                        if start_idx == -1:
                            buffer.clear()
                            break
                        
                        # 移除起始标志之前的数据
                        if start_idx > 0:
                            buffer = buffer[start_idx:]
                        
                        # 检查是否有完整数据包
                        if len(buffer) >= 12:
                            try:
                                # 解析数据包
                                start_flag, command, pitch_angle, yaw_angle, checksum = struct.unpack('<HBffB', buffer[:12])
                                
                                # 验证校验和
                                calculated_checksum = command
                                pitch_bytes = struct.pack('<f', pitch_angle)
                                yaw_bytes = struct.pack('<f', yaw_angle)
                                for b in pitch_bytes + yaw_bytes:
                                    calculated_checksum += b
                                calculated_checksum = calculated_checksum & 0xFF
                                
                                if calculated_checksum == checksum and command == 0x02:  # GIMBAL_CMD_TRAJECTORY_TRACK
                                    # 将角度转换回坐标进行显示
                                    x = int((yaw_angle + 45.0) * 1000.0 / 90.0)
                                    y = int((30.0 - pitch_angle) * 1000.0 / 60.0)
                                    
                                    # 确保坐标在有效范围内
                                    x = max(0, min(1000, x))
                                    y = max(0, min(1000, y))
                                    
                                    # 添加到接收队列
                                    self.received_points.append((x, y))
                                    print(f"接收轨迹点: 俯仰角{pitch_angle:.1f}°, 偏航角{yaw_angle:.1f}° -> 坐标({x}, {y})")
                                
                                # 移除已处理的数据包
                                buffer = buffer[12:]
                                
                            except struct.error:
                                buffer = buffer[1:]  # 移除一个字节继续查找
                        else:
                            break
                            
            except Exception as e:
                print(f"接收数据错误: {e}")
                time.sleep(0.1)
    
    def mouse_callback(self, event, x, y, flags, param):
        """鼠标回调函数"""
        if self.receiving_mode or self.command_mode:
            return  # 接收模式或指令模式下不能绘制
        
        current_time = time.time()
        
        if event == cv2.EVENT_LBUTTONDOWN:
            self.drawing = True
            self.last_point = (x, y)
            
            # 发送起始点
            traj_x, traj_y = self.canvas_to_trajectory(x, y)
            self.send_trajectory_point(traj_x, traj_y)
            self.last_send_time = current_time
            
        elif event == cv2.EVENT_MOUSEMOVE and self.drawing:
            # 绘制线条
            if self.last_point:
                cv2.line(self.canvas, self.last_point, (x, y), (0, 0, 255), 2)
                self.last_point = (x, y)
                
                # 检查是否需要发送轨迹点
                if current_time - self.last_send_time >= self.send_interval:
                    traj_x, traj_y = self.canvas_to_trajectory(x, y)
                    self.send_trajectory_point(traj_x, traj_y)
                    self.last_send_time = current_time
                    
        elif event == cv2.EVENT_LBUTTONUP:
            self.drawing = False
            self.last_point = None
    
    def clear_canvas(self):
        """清空画布"""
        self.canvas.fill(255)
        self.received_points.clear()
    
    def draw_received_points(self):
        """绘制接收到的轨迹点"""
        if len(self.received_points) < 2:
            return
        
        points = list(self.received_points)
        for i in range(1, len(points)):
            x1, y1 = self.trajectory_to_canvas(points[i-1][0], points[i-1][1])
            x2, y2 = self.trajectory_to_canvas(points[i][0], points[i][1])
            
            # 确保坐标在画布范围内
            x1 = max(0, min(self.canvas_size[0]-1, x1))
            y1 = max(0, min(self.canvas_size[1]-1, y1))
            x2 = max(0, min(self.canvas_size[0]-1, x2))
            y2 = max(0, min(self.canvas_size[1]-1, y2))
            
            cv2.line(self.canvas, (x1, y1), (x2, y2), (0, 255, 0), 2)
    
    def display_command_help(self, canvas):
        """显示指令帮助信息"""
        help_text = [
            "Command Mode - Available Commands:",
            "Car Control:",
            "  a - Forward    b - Backward   c - Turn Left",
            "  d - Turn Right e - Rotate L   f - Rotate R",
            "  g - Move 100mm h - Back 100mm i - Left 90deg",
            "  j - Right 90deg s - Stop",
            "Gimbal Control:",
            "  k - Pitch Up   l - Pitch Down n - Yaw Left",
            "  m - Yaw Right  o - Reset",
            "Trajectory Sync:",
            "  T - Start Sync S - Stop Sync",
            "Status Query:",
            "  A - Car Status B - Gimbal Status C - Reset Pos",
            "",
            "Press any key to send command..."
        ]
        
        y_offset = 50
        for i, text in enumerate(help_text):
            color = (0, 0, 255) if "Command Mode" in text else (50, 50, 50)
            font_scale = 0.7 if "Command Mode" in text else 0.5
            cv2.putText(canvas, text, (10, y_offset + i * 25), 
                       cv2.FONT_HERSHEY_SIMPLEX, font_scale, color, 1)
    
    def send_circle_trajectory(self, duration=10.0):
        """发送圆形轨迹点
        Args:
            duration: 画圆总时长(秒)
        """
        if not self.serial_port or not self.serial_port.is_open:
            print("串口未连接，无法发送圆形轨迹")
            return
    
        center_x, center_y = 500, 500  # 圆心坐标(0-1000范围)
        radius = 30  # 半径缩小一半(原400)
        
        # 发送第一个点(圆心右侧起始点)
        first_x = center_x + radius
        first_y = center_y
        self.send_trajectory_point(int(first_x), int(first_y))
        print("已发送起始点，等待5秒...")
        time.sleep(5)  # 等待5秒
        
        start_time = time.time()
        points_sent = 1  # 已发送1个点
        
        print(f"开始绘制圆形轨迹，总时长: {duration}秒")
        
        while time.time() - start_time < duration:
            # 计算当前角度(0-2π)
            progress = (time.time() - start_time) / duration
            angle = 2 * np.pi * progress
            
            # 计算圆上的点坐标
            x = center_x + radius * np.cos(angle)
            y = center_y + radius * np.sin(angle)
            
            # 发送轨迹点
            self.send_trajectory_point(int(x), int(y))
            points_sent += 1
            
            # 控制发送频率(约20Hz)
            time.sleep(0.05)
        
        print(f"圆形轨迹发送完成，共发送{points_sent}个点")

    def run(self):
        """主运行函数"""
        # 初始化串口
        if not self.init_serial():
            print("无法初始化串口，程序退出")
            return
        
        # 启动接收线程
        self.receive_thread = threading.Thread(target=self.receive_trajectory_points)
        self.receive_thread.daemon = True
        self.receive_thread.start()
        
        # 创建窗口
        cv2.namedWindow('Trajectory Tool', cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback('Trajectory Tool', self.mouse_callback)
        
        print("轨迹工具启动成功!")
        print("操作说明:")
        print("- 鼠标左键拖拽: 绘制并发送轨迹")
        print("- 按 'c': 清空画布")
        print("- 按 'o': 发送圆形轨迹(5秒)")  # 新增说明
        print("- 按 'r': 切换到接收模式")
        print("- 按 's': 切换到发送模式")
        print("- 按 'x': 切换到指令模式")
        print("- 按 'q': 退出程序")
        print("- 指令模式下按任意键发送对应指令")
        
        while True:
            # 创建显示画布
            display_canvas = self.canvas.copy()
            
            # 根据模式显示不同内容
            if self.command_mode:
                # 指令模式 - 显示指令帮助
                display_canvas.fill(240)  # 浅灰色背景
                self.display_command_help(display_canvas)
                
            elif self.receiving_mode:
                # 接收模式 - 绘制接收到的轨迹点
                self.draw_received_points()
                display_canvas = self.canvas.copy()
                
                # 显示模式信息
                cv2.putText(display_canvas, "Receiving Mode", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            else:
                # 发送模式 - 显示模式信息
                cv2.putText(display_canvas, "Sending Mode", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            
            # 显示坐标范围信息（非指令模式）
            if not self.command_mode:
                cv2.putText(display_canvas, f"Range: 0-{self.trajectory_range}", (10, 60), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 100, 100), 1)
            
            cv2.imshow('Trajectory Tool', display_canvas)
            
            # 处理按键
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('o') and not self.command_mode:  # 新增圆形轨迹发送
                self.send_circle_trajectory()
            elif key == ord('c') and not self.command_mode:
                self.clear_canvas()
                print("画布已清空")
            elif key == ord('r') and not self.command_mode:
                self.receiving_mode = True
                self.command_mode = False
                self.clear_canvas()
                print("切换到接收模式")
            elif key == ord('s') and not self.command_mode:
                self.receiving_mode = False
                self.command_mode = False
                self.clear_canvas()
                print("切换到发送模式")
            elif key == ord('x'):
                self.command_mode = not self.command_mode
                self.receiving_mode = False
                if self.command_mode:
                    print("切换到指令模式")
                else:
                    print("退出指令模式")
            elif self.command_mode and key != 255:
                # 指令模式下发送按键对应的指令
                command_char = chr(key)
                if command_char.isalnum() or command_char in 'ABCDEFGHIJKLMNOPQRSTUVWXYZ':
                    self.send_command(command_char)
        
        # 清理资源
        self.running = False
        if self.receive_thread:
            self.receive_thread.join(timeout=1)
        self.close_serial()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    tool = TrajectoryTool()
    tool.run()