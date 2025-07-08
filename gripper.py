#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
电爪控制程序 - 纯串口实现ModBus RTU协议
只依赖pyserial库，避免pymodbus冲突
"""

import serial
import struct
import time
import sys

class ElectricGripperController:
    """电爪控制器 - 使用纯串口实现ModBus RTU"""
    
    def __init__(self, port='COM3', baudrate=115200, slave_id=1):
        """
        初始化夹爪控制器
        :param port: 串口号
        :param baudrate: 波特率  
        :param slave_id: 从机地址
        """
        self.port = port
        self.baudrate = baudrate
        self.slave_id = slave_id
        self.ser = None
        
        # 寄存器地址定义（根据文档）
        self.REGISTERS = {
            'COMM_ADDR': 0x001F,        # 31: 修改通讯地址
            'SAVE_COMM': 0x0019,        # 25: 修改通讯保存
            'CONTROL_MODE': 0x0035,     # 53: 控制方式
            'RELEASE_CMD': 0x0036,      # 54: 串口控制松开
            'GRIP_CMD': 0x0037,         # 55: 串口控制夹紧
            'RELEASE_POS': 0x0038,      # 56-57: 松开位置 (双寄存器)
            'GRIP_POS': 0x003A,         # 58-59: 夹紧位置 (双寄存器)
            'STATUS': 0x0040,           # 64: 电爪状态
            'CURRENT_READ': 0x0041,     # 65: 夹爪力矩电流(只读)
            'SPEED_READ': 0x0042,       # 66-67: 夹爪速度(只读)
            'GRIP_CURRENT': 0x0044,     # 68: 夹爪力矩基本电流
            'GRIP_SPEED': 0x0045,       # 69-70: 夹爪速度
            'SAVE_CONFIG': 0x0047,      # 71: 保存配置
            'AUTO_FIND': 0x0051,        # 81: 上电是否自动找行程
            'RESET_CMD': 0x0052,        # 82: 复位找行程指令
        }
        
        # 状态码说明
        self.STATUS_CODES = {
            0: "空闲状态",
            1: "夹紧夹到物体", 
            2: "夹紧没有夹到物体",
            4: "开始夹紧",
            5: "开始松开",
            6: "松开到位"
        }
        
    def crc16_modbus(self, data):
        """计算ModBus RTU CRC16校验码"""
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc & 0xFFFF
    
    def connect(self):
        """连接串口"""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=8,
                parity='N',
                stopbits=1,
                timeout=1.0
            )
            print(f"✓ 串口 {self.port} 连接成功")
            return True
        except Exception as e:
            print(f"✗ 串口连接失败: {e}")
            return False
    
    def disconnect(self):
        """断开串口连接"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("串口已断开")
    
    def read_holding_registers(self, reg_addr, count=1):
        """
        读取保持寄存器 (功能码03)
        :param reg_addr: 寄存器地址
        :param count: 寄存器数量
        :return: 读取的数据列表
        """
        if not self.ser or not self.ser.is_open:
            print("串口未连接")
            return None
        
        # 构建请求帧
        frame = bytearray([
            self.slave_id,                  # 从机地址
            0x03,                          # 功能码
            (reg_addr >> 8) & 0xFF,        # 起始地址高字节
            reg_addr & 0xFF,               # 起始地址低字节
            (count >> 8) & 0xFF,           # 寄存器数量高字节
            count & 0xFF                   # 寄存器数量低字节
        ])
        
        # 计算并添加CRC
        crc = self.crc16_modbus(frame)
        frame.extend([crc & 0xFF, (crc >> 8) & 0xFF])
        
        try:
            # 清空缓冲区
            self.ser.reset_input_buffer()
            
            # 发送请求
            self.ser.write(frame)
            
            # 接收响应
            response = self.ser.read(5 + count * 2)
            
            if len(response) < 5:
                print("响应数据不完整")
                return None
            
            # 验证响应
            if response[0] != self.slave_id or response[1] != 0x03:
                print("响应帧错误")
                return None
            
            # 提取数据
            data_len = response[2]
            if data_len != count * 2:
                print("数据长度不匹配")
                return None
            
            data_bytes = response[3:3+data_len]
            
            # 解析16位寄存器值
            values = []
            for i in range(0, len(data_bytes), 2):
                value = (data_bytes[i] << 8) | data_bytes[i+1]
                values.append(value)
            
            return values
            
        except Exception as e:
            print(f"读取寄存器失败: {e}")
            return None
    
    def write_single_register(self, reg_addr, value):
        """
        写入单个寄存器 (功能码06)
        :param reg_addr: 寄存器地址
        :param value: 写入的值
        :return: 是否成功
        """
        if not self.ser or not self.ser.is_open:
            print("串口未连接")
            return False
        
        # 确保值在16位范围内
        value = value & 0xFFFF
        
        # 构建请求帧
        frame = bytearray([
            self.slave_id,                  # 从机地址
            0x06,                          # 功能码
            (reg_addr >> 8) & 0xFF,        # 寄存器地址高字节
            reg_addr & 0xFF,               # 寄存器地址低字节
            (value >> 8) & 0xFF,           # 数据高字节
            value & 0xFF                   # 数据低字节
        ])
        
        # 计算并添加CRC
        crc = self.crc16_modbus(frame)
        frame.extend([crc & 0xFF, (crc >> 8) & 0xFF])
        
        try:
            # 清空缓冲区
            self.ser.reset_input_buffer()
            
            # 发送请求
            self.ser.write(frame)
            
            # 接收响应（写命令会回显）
            response = self.ser.read(8)
            
            # 验证响应（应该与请求相同）
            if len(response) == 8 and response == frame:
                return True
            else:
                print("写入响应验证失败")
                return False
                
        except Exception as e:
            print(f"写入寄存器失败: {e}")
            return False
    
    def write_double_register(self, reg_addr, value):
        """
        写入双寄存器（32位值，按文档的Little-endian swap格式）
        :param reg_addr: 起始寄存器地址
        :param value: 32位值
        :return: 是否成功
        """
        # 按照文档说明的Little-endian swap格式
        low_word = value & 0xFFFF
        high_word = (value >> 16) & 0xFFFF
        
        # 先写低字寄存器，再写高字寄存器
        success1 = self.write_single_register(reg_addr, low_word)
        time.sleep(0.01)  # 短暂延时
        success2 = self.write_single_register(reg_addr + 1, high_word)
        
        return success1 and success2
    
    def set_control_mode(self, mode):
        """
        设置控制方式
        :param mode: 0=IO控制, 1=串口控制
        """
        if mode not in [0, 1]:
            print("控制模式必须是0（IO控制）或1（串口控制）")
            return False
        
        success = self.write_single_register(self.REGISTERS['CONTROL_MODE'], mode)
        if success:
            mode_text = "串口控制" if mode == 1 else "IO控制"
            print(f"✓ 控制方式设置为: {mode_text}")
        return success
    
    def grip(self):
        """夹紧操作"""
        success = self.write_single_register(self.REGISTERS['GRIP_CMD'], 1)
        if success:
            print("✓ 发送夹紧命令")
        return success
    
    def release(self):
        """松开操作"""
        success = self.write_single_register(self.REGISTERS['RELEASE_CMD'], 1)
        if success:
            print("✓ 发送松开命令")
        return success
    
    def set_grip_current(self, current):
        """
        设置夹持力电流
        :param current: 电流值 (500-2000 mA)
        """
        if not (500 <= current <= 2000):
            print("❌ 夹持力电流必须在500-2000 mA范围内")
            return False
        
        success = self.write_single_register(self.REGISTERS['GRIP_CURRENT'], current)
        if success:
            print(f"✓ 夹持力电流设置为: {current} mA")
        return success
    
    def set_grip_speed(self, speed):
        """
        设置夹持速度
        :param speed: 速度值 (10-3000)
        """
        if not (10 <= speed <= 3000):
            print("❌ 夹持速度必须在10-3000范围内")
            return False
        
        success = self.write_double_register(self.REGISTERS['GRIP_SPEED'], speed)
        if success:
            print(f"✓ 夹持速度设置为: {speed}")
        return success
    
    def get_status(self):
        """获取电爪状态"""
        values = self.read_holding_registers(self.REGISTERS['STATUS'], 1)
        if values:
            status_code = values[0]
            status_text = self.STATUS_CODES.get(status_code, f"未知状态({status_code})")
            print(f"电爪状态: {status_text} (代码: {status_code})")
            return status_code, status_text
        return None, None
    
    def get_current(self):
        """获取当前夹爪力矩电流"""
        values = self.read_holding_registers(self.REGISTERS['CURRENT_READ'], 1)
        if values:
            current = values[0]
            print(f"当前力矩电流: {current} mA")
            return current
        return None
    
    def save_config(self):
        """保存配置到设备"""
        success = self.write_single_register(self.REGISTERS['SAVE_CONFIG'], 1)
        if success:
            print("✓ 发送配置保存命令")
            # 等待保存完成
            for i in range(10):  # 最多等待1秒
                time.sleep(0.1)
                values = self.read_holding_registers(self.REGISTERS['SAVE_CONFIG'], 1)
                if values and values[0] == 0:
                    print("✓ 配置保存完成")
                    return True
            print("⚠ 配置保存状态未确认")
        return success
    
    def test_connection(self):
        """测试连接"""
        print("测试设备连接...")
        status = self.get_status()
        if status:
            print('设备连接成功')
        else:
            print('设备连接失败')
        return status[0] is not None if status else False

def main():
    """主程序"""
    print("=" * 50)
    print("        电爪控制程序 v1.1")
    print("=" * 50)
    
    # 创建控制器实例

    gripper = ElectricGripperController(port='COM3', baudrate=115200, slave_id=1)
    
    # 连接设备
    if not gripper.connect():
        print("程序退出")
        return
    
    try:
        # 测试连接
        if not gripper.test_connection():
            print("❌ 设备连接测试失败，请检查:")
            print("  1. 串口号是否正确")
            print("  2. 设备是否正常连接")
            print("  3. 波特率和从机地址是否匹配")
            return
        
        print("✓ 设备连接正常")
        
        # 初始化设置
        print("\n初始化设备设置...")
        gripper.set_control_mode(1)          # 设置为串口控制
        gripper.set_grip_current(550)       # 设置夹持力
        gripper.set_grip_speed(2000)      # 设置夹持速度
        gripper.save_config()                # 保存设置
        
        # 主控制循环
        print("\n" + "=" * 50)
        print("控制菜单:")
        print("1 - 夹紧")
        print("2 - 松开") 
        print("3 - 查看状态")
        print("4 - 查看电流")
        print("5 - 设置夹持力")
        print("6 - 设置夹持速度")
        print("7 - 保存配置")
        print("0 - 退出")
        print("=" * 50)
        
        while True:
            try:
                choice = input("\n请选择操作 (0-7): ").strip()
                
                if choice == '1':
                    gripper.grip()
                elif choice == '2':
                    gripper.release()
                elif choice == '3':
                    gripper.get_status()
                elif choice == '4':
                    gripper.get_current()
                elif choice == '5':
                    try:
                        current = int(input("请输入夹持力电流 (500-2000 mA): "))
                        if gripper.set_grip_current(current):
                            gripper.save_config()
                    except ValueError:
                        print("❌ 请输入有效的数字")
                elif choice == '6':
                    try:
                        speed = int(input("请输入夹持速度 (10-3000): "))
                        if gripper.set_grip_speed(speed):
                            gripper.save_config()
                    except ValueError:
                        print("❌ 请输入有效的数字")
                elif choice == '7':
                    gripper.save_config()
                elif choice == '0':
                    print("退出程序")
                    break
                else:
                    print("❌ 无效选择，请输入0-7")
                
                time.sleep(0.1)  # 短暂延时
                
            except KeyboardInterrupt:
                print("\n程序被中断")
                break
    
    except Exception as e:
        print(f"程序运行出错: {e}")
    
    finally:
        gripper.disconnect()

if __name__ == "__main__":
    main()