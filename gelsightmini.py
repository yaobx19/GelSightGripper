from utilities.gelsightmini import GelSightMini
import cv2
import numpy as np
from utilities.marker_tracker import MarkerTracker
import argparse
from config import GSConfig
import matplotlib.pyplot as plt
import os

class DisplacementTracker:
    def __init__(self, device_num):
        self.markertracker = None
        self.Ox = None
        self.Oy = None
        self.nct = 0
        self.old_gray = None
        self.lk_params = None
        self.p0 = None
        self.initialized = False

        # 初始化相机
        parser = argparse.ArgumentParser(
        description="Run the Gelsight Mini Viewer with an optional config file."
    )
        parser.add_argument(
            "--gs-config",
            type=str,
            default=None,
            help="Path to the JSON configuration file. If not provided, default config is used.",
        )

        args = parser.parse_args()

        gs_config = GSConfig(args.gs_config).config

        # 初始化相机流
        self.cam_stream = GelSightMini(
            target_width=gs_config.camera_width,
            target_height=gs_config.camera_height,
            border_fraction=gs_config.border_fraction,
        )
        self.cam_stream.select_device(device_num)  # 选择设备0
        self.cam_stream.start()
        
        # 存储位移历史数据
        self.displacement_history = []
        self.current_displacements = None

    def initialize(self, frame: np.ndarray):
        # 将帧转换为浮点数格式
        img = np.float32(frame) / 255.0
        
        # 创建MarkerTracker实例
        self.markertracker = MarkerTracker(img)
        
        # 获取初始标记点中心
        marker_centers = self.markertracker.initial_marker_center
        self.Ox = marker_centers[:, 1]  # x坐标
        self.Oy = marker_centers[:, 0]  # y坐标
        self.nct = len(marker_centers)  # 标记点数量
        
        # 转换为灰度图像用于光流追踪
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        self.old_gray = frame_gray
        
        # Lucas-Kanade光流参数
        self.lk_params = dict(
            winSize=(15, 15),
            maxLevel=2,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03),
        )
        
        # 准备追踪点
        self.p0 = np.array([[self.Ox[0], self.Oy[0]]], np.float32).reshape(-1, 1, 2)
        for i in range(self.nct - 1):
            new_point = np.array([[self.Ox[i + 1], self.Oy[i + 1]]], np.float32).reshape(-1, 1, 2)
            self.p0 = np.append(self.p0, new_point, axis=0)
        
        self.initialized = True
        print(f"初始化完成，检测到 {self.nct} 个标记点")

    def calculate_displacements(self, current_points: np.ndarray):
        """计算每个标记点的位移"""
        if not self.initialized or len(current_points) != self.nct:
            return None
        
        # 计算每个点相对于初始位置的位移
        displacements = []
        for i in range(len(current_points)):
            dx = current_points[i][0] - self.Ox[i]
            dy = current_points[i][1] - self.Oy[i]
            displacement = np.sqrt(dx**2 + dy**2)
            displacements.append(displacement)
        
        return np.array(displacements)

    def get_average_displacement(self, frame: np.ndarray):
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        
        # 使用Lucas-Kanade光流追踪
        p1, st, err = cv2.calcOpticalFlowPyrLK(
            self.old_gray, frame_gray, self.p0, None, **self.lk_params
        )
        
        # 选择成功追踪的点
        good_new = p1[st == 1]
        good_old = self.p0[st == 1]
        
        avg_disp = 0.0
        # 更新追踪点
        if len(good_new) >= self.nct:
            self.p0 = good_new.reshape(-1, 1, 2)
            
            # 计算位移
            current_points = good_new.reshape(-1, 2)
            current_displacements = self.calculate_displacements(current_points)
            avg_disp = np.mean(current_displacements)

        # 更新灰度图像
        self.old_gray = frame_gray.copy()
        
        return avg_disp
    
    def calculate_directional_displacements(self, current_points: np.ndarray):
        """计算每个标记点在x和y方向的位移"""
        if not self.initialized or len(current_points) != self.nct:
            return None, None
        
        # 计算每个点相对于初始位置的x和y方向位移
        dx_list = []
        dy_list = []
        
        for i in range(len(current_points)):
            dx = current_points[i][0] - self.Ox[i]  # x方向位移
            dy = current_points[i][1] - self.Oy[i]  # y方向位移
            dx_list.append(dx)
            dy_list.append(dy)
        
        return np.array(dx_list), np.array(dy_list)
    
    def get_directional_displacement_every_point(self, frame: np.ndarray):
        """获取每个点的x和y方向的位移"""
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        
        # 使用Lucas-Kanade光流追踪
        p1, st, err = cv2.calcOpticalFlowPyrLK(
            self.old_gray, frame_gray, self.p0, None, **self.lk_params
        )
        
        # 选择成功追踪的点
        good_new = p1[st == 1]
        good_old = self.p0[st == 1]
        
        dx_displacements_list = []
        dy_displacements_list = []
        
        # 更新追踪点
        if len(good_new) >= self.nct:
            self.p0 = good_new.reshape(-1, 1, 2)
            
            # 计算方向性位移
            current_points = good_new.reshape(-1, 2)
            dx_displacements, dy_displacements = self.calculate_directional_displacements(current_points)
            if dx_displacements is not None and dy_displacements is not None:
                dx_displacements_list.append(dx_displacements)
                dy_displacements_list.append(dy_displacements)
        
        # 更新灰度图像
        self.old_gray = frame_gray.copy()
        
        return dx_displacements_list, dy_displacements_list


    def get_average_directional_displacement(self, frame: np.ndarray):
        """获取x和y方向的平均位移"""
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        
        # 使用Lucas-Kanade光流追踪
        p1, st, err = cv2.calcOpticalFlowPyrLK(
            self.old_gray, frame_gray, self.p0, None, **self.lk_params
        )
        
        # 选择成功追踪的点
        good_new = p1[st == 1]
        good_old = self.p0[st == 1]
        
        avg_dx = 0.0
        avg_dy = 0.0
        
        # 更新追踪点
        if len(good_new) >= self.nct:
            self.p0 = good_new.reshape(-1, 1, 2)
            
            # 计算方向性位移
            current_points = good_new.reshape(-1, 2)
            dx_displacements, dy_displacements = self.calculate_directional_displacements(current_points)
            
            if dx_displacements is not None and dy_displacements is not None:
                avg_dx = np.mean(dx_displacements)  # x方向平均位移
                avg_dy = np.mean(dy_displacements)  # y方向平均位移

        # 更新灰度图像
        self.old_gray = frame_gray.copy()
        
        return avg_dx, avg_dy

    def update_marker_view(self, frame: np.ndarray):
        if not self.initialized:
            return None
            
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        
        # 使用Lucas-Kanade光流追踪
        p1, st, err = cv2.calcOpticalFlowPyrLK(
            self.old_gray, frame_gray, self.p0, None, **self.lk_params
        )
        
        # 选择成功追踪的点
        good_new = p1[st == 1]
        good_old = self.p0[st == 1]
        
        # 更新追踪点
        if len(good_new) >= self.nct:
            self.p0 = good_new.reshape(-1, 1, 2)
            
            # 计算位移
            current_points = good_new.reshape(-1, 2)
            self.current_displacements = self.calculate_displacements(current_points)
            
            if self.current_displacements is not None:
                # 保存位移历史
                self.displacement_history.append(self.current_displacements.copy())
                
                # 打印当前帧的位移统计
                self.print_displacement_stats(frame_count=len(self.displacement_history))
        
        # 更新灰度图像
        self.old_gray = frame_gray.copy()
        return good_new  # 返回当前标记点位置


    def get_comprehensive_displacement(self, frame: np.ndarray):
        """获取x、y方向平均位移以及总平均位移"""
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        
        # 使用Lucas-Kanade光流追踪
        p1, st, err = cv2.calcOpticalFlowPyrLK(
            self.old_gray, frame_gray, self.p0, None, **self.lk_params
        )
        
        # 选择成功追踪的点
        good_new = p1[st == 1]
        good_old = self.p0[st == 1]
        
        avg_dx = 0.0
        avg_dy = 0.0
        avg_total = 0.0
        
        # 更新追踪点
        if len(good_new) >= self.nct:
            self.p0 = good_new.reshape(-1, 1, 2)
            
            # 计算位移
            current_points = good_new.reshape(-1, 2)
            dx_displacements, dy_displacements = self.calculate_directional_displacements(current_points)
            
            if dx_displacements is not None and dy_displacements is not None:
                avg_dx = np.mean(dx_displacements)  # x方向平均位移
                avg_dy = np.mean(dy_displacements)  # y方向平均位移
                
                # 计算总位移（欧几里得距离）
                total_displacements = np.sqrt(dx_displacements**2 + dy_displacements**2)
                avg_total = np.mean(total_displacements)

        # 更新灰度图像
        self.old_gray = frame_gray.copy()
        
        return avg_dx, avg_dy, avg_total

    
    def get_displacement_field(self, frame: np.ndarray):
        if not self.initialized:
            return None
            
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        
        # 使用Lucas-Kanade光流追踪
        p1, st, err = cv2.calcOpticalFlowPyrLK(
            self.old_gray, frame_gray, self.p0, None, **self.lk_params
        )
        
        # 选择成功追踪的点
        good_new = p1[st == 1]
        good_old = self.p0[st == 1]
        
        displacements = np.zeros((7, 9, 2), dtype=np.float32)
        # 更新追踪点
        if len(good_new) >= self.nct:
            self.p0 = good_new.reshape(-1, 1, 2)
            
            # 计算位移
            current_points = good_new.reshape(-1, 2)
            if len(current_points) != self.nct:
                print(f'Warning: (current_points) != self.nct')
                return None
            
            # 计算每个点的位移向量
            displacements = np.stack([
                current_points[:, 0] - self.Ox,
                current_points[:, 1] - self.Oy
            ], axis=1)  # shape: (nct, 2)
            displacements.reshape((7, 9, 2))
        
        # 更新灰度图像
        self.old_gray = frame_gray.copy()

        return displacements

    def print_displacement_stats(self, frame_count: int):
        """打印位移统计信息"""
        if self.current_displacements is None:
            return
        
        max_disp = np.max(self.current_displacements)
        min_disp = np.min(self.current_displacements)
        avg_disp = np.mean(self.current_displacements)
        
        print(f"帧 {frame_count:4d} - 最大位移: {max_disp:6.2f}px, "
              f"最小位移: {min_disp:6.2f}px, "
              f"平均位移: {avg_disp:6.2f}px")

    def print_overall_stats(self):
        """打印整体统计信息"""
        if not self.displacement_history:
            print("没有位移数据")
            return
        
        all_displacements = np.concatenate(self.displacement_history)
        
        overall_max = np.max(all_displacements)
        overall_min = np.min(all_displacements)
        overall_avg = np.mean(all_displacements)
        
        print("\n=== 整体位移统计 ===")
        print(f"总体最大位移: {overall_max:.2f}px")
        print(f"总体最小位移: {overall_min:.2f}px")
        print(f"总体平均位移: {overall_avg:.2f}px")
        print(f"总帧数: {len(self.displacement_history)}")
        print(f"标记点数量: {self.nct}")

    def show_displacement_field(self, displacement_field, frame_idx):
        import os
        import matplotlib.pyplot as plt
        import numpy as np

        output_dir = "displacement_field_images"
        os.makedirs(output_dir, exist_ok=True)

        if displacement_field is not None:
            # 位移场形状为 (7, 9, 2)
            h, w = displacement_field.shape[:2]

            # 定义一个固定原始网格起始点，例如从 (x0, y0) 均匀分布
            x0, y0 = 50, 50  # 原点偏移值，可以根据画布大小调整
            spacing = 20     # 点与点之间的间距（像素）

            X, Y = np.meshgrid(np.arange(w) * spacing + x0,
                            np.arange(h) * spacing + y0)

            U = displacement_field[:, :, 0]
            V = displacement_field[:, :, 1]

            # 绘图并保存
            plt.figure(figsize=(6, 6))
            plt.quiver(X, Y, U, V, color='red', angles='xy', scale_units='xy', scale=1)
            plt.xlim(0, x0 + spacing * (w + 1))
            plt.ylim(y0 + spacing * (h + 1), 0)  # y轴倒置以匹配图像坐标系统
            plt.title(f"Displacement Vector Field - Frame {frame_idx}")
            plt.axis('off')

            filename = os.path.join(output_dir, f"frame_{frame_idx:04d}.png")
            plt.savefig(filename, bbox_inches='tight')
            plt.close()



def main():
    # 创建位移跟踪器实例
    displacement_tracker_l = DisplacementTracker(device_num=3)
    displacement_tracker_r = DisplacementTracker(device_num=0)
    frame_count = 0
    
    try:

        while True:
            # 获取帧
            frame_l = displacement_tracker_l.cam_stream.update(1 / 30.0)
            if frame_l is None:
                continue
            # 初始化（第一帧）
            if not displacement_tracker_l.initialized:
                displacement_tracker_l.initialize(frame_l)
                continue

            # 获取帧 
            frame_r = displacement_tracker_r.cam_stream.update(1 / 30.0)
            if frame_r is None:
                continue
            # 初始化（第一帧）
            if not displacement_tracker_r.initialized:
                displacement_tracker_r.initialize(frame_r)
                continue
                
            frame_count += 1

            # 获取x，y方向位移
            avg_dx_l, avg_dy_l, _ = displacement_tracker_l.get_comprehensive_displacement(frame_l)
            avg_dx_r, avg_dy_r, _ = displacement_tracker_r.get_comprehensive_displacement(frame_r)


            if frame_count % 90 ==0:  # 每3秒打印一次
                print(f'Print Num: {frame_count/90}')
                print('='*80)
                print(f'avg_dx_l: {avg_dx_l}')
                print(f'avg_dy_l: {avg_dy_l}')
                print('='*80)
                print(f'avg_dx_r: {avg_dx_r}')
                print(f'avg_dy_r: {avg_dy_r}')
                print('='*80)
                
    except KeyboardInterrupt:
        print("\n程序被用户中断")

if __name__ == "__main__":
    main()