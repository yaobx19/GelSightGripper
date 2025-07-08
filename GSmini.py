import cv2
import numpy as np
from gelsightmini import DisplacementTracker
from collections import deque


class GSmini:
      def __init__(self):
            self.displacement_tracker_l = DisplacementTracker(device_num=3)
            self.displacement_history_l = deque(maxlen=30)  # Store data for 1 seconds.
            self.displacement_tracker_r = DisplacementTracker(device_num=0)
            self.displacement_history_r = deque(maxlen=30)  # Store data for 1 seconds.
            self.initialized = False

      def initialize(self):
            frame_l = self.displacement_tracker_l.cam_stream.update(1 / 30.0)
            self.displacement_tracker_l.initialize(frame_l)
            frame_r = self.displacement_tracker_r.cam_stream.update(1 / 30.0)
            self.displacement_tracker_r.initialize(frame_r)

            for i in range(3):
                  self.get_frame()
            self.initialized = True
            print('initialize success')
      
      def get_frame(self):
            """
            Get frame and restore it.
            """
            frame = self.displacement_tracker_l.cam_stream.update(1 / 30.0)
            self.displacement_history_l.append(frame)

            frame = self.displacement_tracker_r.cam_stream.update(1 / 30.0)
            self.displacement_history_r.append(frame)

            pass
      
      def judge_contact(self):
            """
            Determine whether there has been contact, 
            which will be used to assess whether it is necessary to initiate gripping 
            and whether to grip the water cup.
            """
            # 确保有足够的历史数据
            if len(self.displacement_history_l) < 2 or len(self.displacement_history_r) < 2:
                  return 0
            
            # 计算左手最近2帧的平均位移
            recent_l_displacements = []
            for i in [1, 2]:  # 访问最新的两帧：-1和-2
                  disp_i = self.displacement_tracker_l.get_average_displacement(
                        self.displacement_history_l[-i]
                  )
                  recent_l_displacements.append(disp_i)
            avg_disp_l = np.mean(recent_l_displacements)

            if avg_disp_l > 0.3:
                  return 1

            # 计算右手最近2帧的平均位移
            recent_r_displacements = []
            for i in [1, 2]:
                  disp_i = self.displacement_tracker_r.get_average_displacement(
                        self.displacement_history_r[-i]
                  )
                  recent_r_displacements.append(disp_i)
            avg_disp_r = np.mean(recent_r_displacements)

            if avg_disp_r > 0.3:
                  return 1
            
            return 0

      def detect_slip(self):
            """
            The detection will determine whether there is a trend of sliding, 
            which is used to evaluate the stability of the grip 
            and whether water cup is being extracted.

            Returns:
                  tuple: (x_direction_slip, y_direction_slip)
                        x_direction_slip: 是否在x方向发生滑移
                        y_direction_slip: 是否在y方向发生滑移
            """
            # 确保有足够的历史数据进行5帧检测
            if len(self.displacement_history_l) < 5 or len(self.displacement_history_r) < 5:
                  return False, False

            # 检查x方向滑移：左右手都连续5帧x位移大于0.5
            x_direction_slip = True

            # 检查左手最近5帧的x方向位移
            for i in range(1, 6):  # 访问最新的5帧：-1, -2, -3, -4, -5
                  avg_l_dx, avg_l_dy = self.displacement_tracker_l.get_average_directional_displacement(
                        self.displacement_history_l[-i]
                  )
                  if abs(avg_l_dx) <= 0.5:
                        x_direction_slip = False
                        break

            # 如果左手满足条件，再检查右手
            if x_direction_slip:
                  for i in range(1, 6):  # 访问最新的5帧：-1, -2, -3, -4, -5
                        avg_r_dx, avg_r_dy = self.displacement_tracker_r.get_average_directional_displacement(
                        self.displacement_history_r[-i]
                        )
                        if abs(avg_r_dx) <= 0.5:
                              x_direction_slip = False
                              break

            # 检查y方向滑移：左右手都连续5帧y位移大于1.0
            y_direction_slip = True

            # 检查左手最近5帧的y方向位移
            for i in range(1, 6):  # 访问最新的5帧：-1, -2, -3, -4, -5
                  avg_l_dx, avg_l_dy = self.displacement_tracker_l.get_average_directional_displacement(
                        self.displacement_history_l[-i]
                  )
                  if abs(avg_l_dy) <= 1.0:
                        y_direction_slip = False
                        break

            # 如果左手满足条件，再检查右手
            if y_direction_slip:
                  for i in range(1, 6):  # 访问最新的5帧：-1, -2, -3, -4, -5
                        avg_r_dx, avg_r_dy = self.displacement_tracker_r.get_average_directional_displacement(
                        self.displacement_history_r[-i]
                        )
                        if abs(avg_r_dy) <= 1.0:
                              y_direction_slip = False
                              break

            return x_direction_slip, y_direction_slip

      def perceive_weight(self):
            """
            The weight of the water cup (water volume) is perceived, 
            calculated based on the linear relationship 
            between the y-direction displacement of the left and right hands
            """
            # 获取左手最近帧的y方向位移（dy）
            _, avg_l_dy = self.displacement_tracker_l.get_average_directional_displacement(
                  self.displacement_history_l[-1])
            
            # 获取右手最近帧的y方向位移（dy）
            _, avg_r_dy = self.displacement_tracker_r.get_average_directional_displacement(
                  self.displacement_history_r[-1])
            
            # 根据给定公式计算水量
            liquid = 0.5 * (
                  (-87.558 * avg_r_dy - 6.5394) + 
                  (-102.07 * avg_l_dy + 30.634)
            ) - 29.198 + 0.7
            
            return liquid

      def identify_disturbance(self, threshold=0.5, n_frames=5):
            """
            Identify if the water bottle is being disturbed.

            Args:
                  threshold: 扰动阈值，超过此值认为不是扰动
                  n_frames: 用于计算平均位移场的历史帧数

            Returns:
                  bool: True表示检测到扰动，False表示没有扰动
            """
            if len(self.displacement_history_l) < n_frames + 1:
                  return False

            # 当前帧位移
            current_l_dx, current_l_dy = self.displacement_tracker_l.get_directional_displacement_every_point(
                  self.displacement_history_l[-1])
            current_r_dx, current_r_dy = self.displacement_tracker_r.get_directional_displacement_every_point(
                  self.displacement_history_r[-1])

            # 检查是否成功计算当前帧位移
            if current_l_dx is None or current_r_dx is None:
                  return False

            # 累积历史位移
            valid_count = 0
            sum_l_dx = np.zeros_like(current_l_dx)
            sum_l_dy = np.zeros_like(current_l_dy)
            sum_r_dx = np.zeros_like(current_r_dx)
            sum_r_dy = np.zeros_like(current_r_dy)

            for i in range(2, n_frames + 2):  # 取过去 n_frames 帧
                  past_l_dx, past_l_dy = self.displacement_tracker_l.get_directional_displacement_every_point(
                        self.displacement_history_l[-i])
                  past_r_dx, past_r_dy = self.displacement_tracker_r.get_directional_displacement_every_point(
                        self.displacement_history_r[-i])

                  # 跳过无效帧
                  if past_l_dx is None or past_r_dx is None:
                        continue

                  sum_l_dx += past_l_dx
                  sum_l_dy += past_l_dy
                  sum_r_dx += past_r_dx
                  sum_r_dy += past_r_dy
                  valid_count += 1

            # 若有效帧不足，无法判断
            if valid_count == 0:
                  return False

            # 平均位移
            avg_l_dx = sum_l_dx / valid_count
            avg_l_dy = sum_l_dy / valid_count
            avg_r_dx = sum_r_dx / valid_count
            avg_r_dy = sum_r_dy / valid_count

            # 计算欧式距离差异
            diff_l = np.sqrt((current_l_dx - avg_l_dx) ** 2 + (current_l_dy - avg_l_dy) ** 2)
            diff_r = np.sqrt((current_r_dx - avg_r_dx) ** 2 + (current_r_dy - avg_r_dy) ** 2)

            # 判断是否有超过21个点超过扰动阈值
            over_threshold_l = np.sum(diff_l > threshold)
            over_threshold_r = np.sum(diff_r > threshold)

            if over_threshold_l > 21 or over_threshold_r > 21:
                  return True  # 不是扰动
            else:
                  return False   # 是扰动

      def detect_scroll(self):
            """
            Determine whether the water cup is rolling 
            (i.e., when the bottle cap is being twisted).
            """
            # 确保有足够的历史数据进行5帧检测
            if len(self.displacement_history_l) < 5 or len(self.displacement_history_r) < 5:
                  return False
            
            # 检查y方向滑移：左右手都连续5帧y位移大于1.0
            is_rolling = True
            
            # 检查左手最近5帧的y方向位移
            for i in range(1, 6):  # 访问最新的5帧：-1, -2, -3, -4, -5
                  avg_l_dx, avg_l_dy = self.displacement_tracker_l.get_average_directional_displacement(
                        self.displacement_history_l[-i]
                  )
                  if abs(avg_l_dx) <= 0.5:
                        is_rolling = False
                        break

            for i in range(1, 6):  # 访问最新的5帧：-1, -2, -3, -4, -5
                        avg_r_dx, avg_r_dy = self.displacement_tracker_r.get_average_directional_displacement(
                        self.displacement_history_r[-i]
                        )
                        if abs(avg_r_dx) <= 0.5:
                              is_rolling = False
                              break
            
            # 如果平移方向相反，那么就认为在滚动；否则不是
            if avg_l_dx * avg_r_dx < 0:
                  is_rolling = False
            
            return is_rolling