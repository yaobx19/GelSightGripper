import time
from collections import deque
from gripper import ElectricGripperController
from gelsightmini import DisplacementTracker
from GSmini import GSmini


def main():
      pass

def set_state(new_state):
    if new_state in VALID_STATES:
        global state
        state = new_state
        print('='*60)
        print(f'current state: {state}')
        print('='*60)
    else:
        raise ValueError(f"无效状态：{new_state}")

if __name__ == '__main__':
      # 创建触觉实例并初始化
      gsmini = GSmini()
      gsmini.initialize()

      # 创建控制器实例
      initial_force = 1000  # 设置初始夹持力
      gripper = ElectricGripperController(port='COM3', baudrate=115200, slave_id=1)
      gripper.connect()
      gripper.test_connection()
      gripper.set_control_mode(1)          # 设置为串口控制
      gripper.set_grip_current(initial_force)       # 设置夹持力
      gripper.set_grip_speed(2000)         # 设置夹持速度
      gripper.release()                    # 松开夹持
      gripper.save_config()                # 保存设置
      

      # 维护状态变量
      gripping = False
      contact_stable_count = 0  # 稳定接触计数
      no_contact_stable_count = 0  # 无接触稳定计数
      slip_recovery_count = 0  # 滑移恢复计数
      weight_monitor_count = 0  # 液量监测计数器
      STABLE_THRESHOLD = 3  # 需要连续2帧稳定才改变状态
      RECOVERY_THRESHOLD = 5  # 滑移后恢复的等待帧数
      GRIP_VALIDATION_FRAMES = 15  # 夹紧后验证帧数
      WEIGHT_MONITOR_FRAMES = 30  # 液量监测间隔帧数(约1秒)
      VALID_STATES = {"WAITING", "GRIPPING"}
      state = "WAITING"
      last_slip_time = 0  # 记录上次滑移时间
      y_slip_warning_count = 0
      MAX_Y_SLIP_WARNING = 5  # 最大警告次数，防止频繁打印
      MAX_DISTURBANCE_WARNING = 5  # 最大警告次数，防止频繁打印
      MAX_ROLLING_WARNING = 5  # 最大警告次数，防止频繁打印

      # 读取位移，根据位移大小决定控制器输出
      while True:
            gsmini.get_frame()

            if state == "WAITING":
                  has_contact = gsmini.judge_contact()

                  if has_contact:
                        contact_stable_count += 1
                        no_contact_stable_count = 0
                        
                        # 连续检测到接触且当前未夹紧时才开始夹紧
                        # 增加条件：距离上次滑移要有足够的恢复时间
                        if (contact_stable_count >= STABLE_THRESHOLD and 
                        not gripping and 
                        slip_recovery_count >= RECOVERY_THRESHOLD):
                        
                              # 执行夹紧动作
                              gripping = True
                              gripper.grip()
                              time.sleep(1)  # 等待夹紧稳定
                              
                              # 夹紧后验证是否成功夹到物体
                              grip_success = False
                              for validation_frame in range(GRIP_VALIDATION_FRAMES):
                                    gsmini.get_frame()  # 获取新的传感器数据
                                    current_contact = gsmini.judge_contact()
                                    
                                    if current_contact:
                                          grip_success = True
                                          break
                                    time.sleep(0.1)  # 短暂等待下一帧
                              
                              if grip_success:
                                    # 夹紧成功，切换到GRIPPING状态
                                    set_state("GRIPPING")
                                    contact_stable_count = 0
                                    slip_recovery_count = 0
                                    weight_monitor_count = 0  # 重置液量监测计数器
                                    print("Grip successful - switching to GRIPPING state")
                              else:
                                    # 夹紧失败，松开夹爪并保持在WAITING状态
                                    print("Grip failed - nothing detected, releasing gripper")
                                    gripping = False
                                    gripper.release()
                                    contact_stable_count = 0
                                    slip_recovery_count = 0
                                    time.sleep(0.5)  # 等待松开稳定
                                    
                  else:
                        no_contact_stable_count += 1
                        contact_stable_count = 0
                        
                        # 连续检测到无接触且当前已夹紧时才松开
                        if no_contact_stable_count >= STABLE_THRESHOLD and gripping:
                              gripping = False
                              gripper.release()
                              no_contact_stable_count = 0
                              print("No contact detected - releasing gripper")
                        
                  # 在WAITING状态时增加恢复计数
                  if slip_recovery_count < RECOVERY_THRESHOLD:
                        slip_recovery_count += 1

            elif state == "GRIPPING":
                  is_rolling = gsmini.detect_scroll()
                  rolling_warning_count = 0
                  if is_rolling:
                        if rolling_warning_count <= MAX_ROLLING_WARNING:
                              print(f"[IS ROLLING] Waiting for stability!")
                              rolling_warning_count = rolling_warning_count + 1
                              continue

                  x_direction_slip, y_direction_slip = gsmini.detect_slip()
            
                  # 连续检测到x方向滑移且当前夹紧时才开始放松
                  if x_direction_slip and gripping:
                        gripping = False
                        gripper.release()
                        set_state("WAITING")
                        # 重置计数器，防止立即重新夹紧
                        contact_stable_count = 0
                        slip_recovery_count = 0
                        last_slip_time = time.time()
                        y_slip_warning_count = 0  # 重置y方向滑移警告计数
                        print("X-direction slip detected - releasing and returning to WAITING")
                        time.sleep(2)  # 等待稳定
                  
                  elif y_direction_slip:
                        # 检测到y方向滑移，但首先判断是否为扰动
                        is_disturbance = gsmini.identify_disturbance()
                        
                        if is_disturbance:
                              print(f"[IS DISTURBANCE] Detected disturbance during y-slip, waiting for stability!")
                              y_slip_warning_count = 0  # 重置警告计数，因为是扰动
                        else:
                              # 真正的滑移，需要警告，但限制频率
                              y_slip_warning_count += 1
                              if y_slip_warning_count <= MAX_Y_SLIP_WARNING:
                                    print(f'[WARNING] Slipping! STOP ADDING WATER (Warning {y_slip_warning_count}/{MAX_Y_SLIP_WARNING})')
                              elif y_slip_warning_count == MAX_Y_SLIP_WARNING + 1:
                                    print('[WARNING] Continuous slipping detected, suppressing further warnings...')
                                    # 将下次的夹持力增大50，将会再下一次松开再夹住的时候生效
                                    gripper.set_grip_current(initial_force+50) 
                                    print(f'[Set Gripper Current] Current Gripper Force: {initial_force+50}')
                                    initial_force = initial_force + 50
                        continue
                  
                  else:
                        # 没有检测到滑移，重置警告计数器
                        y_slip_warning_count = 0
                        
                        # 检查是否有扰动
                        is_disturbance = gsmini.identify_disturbance()
                        
                        if is_disturbance:
                              print(f"[IS DISTURBANCE] Waiting for stability!")
                              continue
                        else: 
                              # 当没有扰动，滑移或者滚动等紧急情况时，进行液量监测
                              weight_monitor_count += 1
                              if weight_monitor_count >= WEIGHT_MONITOR_FRAMES:
                                    # 每30帧（约1秒）监测一次液量
                                    current_weight = gsmini.perceive_weight()
                                    print(f"[LIQUID MONITOR] Current liquid level: {current_weight}")
                                    weight_monitor_count = 0  # 重置计数器
            else:
                  pass