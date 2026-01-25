"""
支持保存GIF的动画更新器
"""

import numpy as np

from pyauto.simulation.animation_updater import AdvancedAnimationUpdater


class GifAnimationUpdater(AdvancedAnimationUpdater):
    """
    扩展AdvancedAnimationUpdater，添加GIF保存功能
    """

    def __init__(self, path_x, path_y, config, draw_car_func=None, gif_path=None):
        """
        初始化GIF动画更新器

        Args:
            path_x: 参考路径x坐标
            path_y: 参考路径y坐标
            config: 配置对象
            draw_car_func: 绘制车辆的函数，默认为draw.draw_car
            gif_path: GIF保存路径
        """
        super().__init__(path_x, path_y, config, draw_car_func)
        self.gif_path = gif_path
        self.frames = []  # 存储帧数据

    def update_frame(self, node, cx, cy, target_ind, steer):
        """
        更新动画帧并捕获用于GIF
        """
        # 调用父类更新
        result = super().update_frame(node, cx, cy, target_ind, steer)

        # 捕获帧
        if result:  # 只在成功更新时捕获
            self._capture_frame()

        return result

    def _capture_frame(self):
        """捕获当前帧"""
        try:
            # 将当前图形转换为numpy数组
            self.fig.canvas.draw()
            # 使用tostring_rgb或buffer_rgba()方法
            if hasattr(self.fig.canvas, "tostring_rgb"):
                buf = np.frombuffer(self.fig.canvas.tostring_rgb(), dtype=np.uint8)
            elif hasattr(self.fig.canvas, "buffer_rgba"):
                buf = np.frombuffer(self.fig.canvas.buffer_rgba(), dtype=np.uint8)
                # RGBA转RGB
                w, h = self.fig.canvas.get_width_height()
                buf = buf.reshape(h, w, 4)[:, :, :3]  # 去掉alpha通道
                buf = buf.reshape(-1)
            else:
                # 备用方案：使用renderer
                self.fig.canvas.draw()
                buf = np.frombuffer(self.fig.canvas.renderer.buffer_rgba(), dtype=np.uint8)
                w, h = self.fig.canvas.get_width_height()
                buf = buf.reshape(h, w, 4)[:, :, :3]
                buf = buf.reshape(-1)

            w, h = self.fig.canvas.get_width_height()
            buf = buf.reshape(h, w, 3)
            self.frames.append(buf.copy())
        except Exception as e:
            # 静默失败，避免中断仿真
            if len(self.frames) == 0:  # 只在第一次失败时打印
                print(f"捕获帧时出错: {e}")
                print("将尝试其他方法...")

    def save_to_gif(self, output_path=None, fps=20, skip_frames=5):
        """
        将录制的帧保存为GIF

        Args:
            output_path: GIF输出路径，如果为None则使用初始化时的路径
            fps: 帧率
            skip_frames: 跳帧数（每N帧保存一次，减小文件大小）
        """
        if len(self.frames) == 0:
            print("没有可保存的帧")
            return

        try:
            from PIL import Image
        except ImportError:
            print("需要安装Pillow库: pip install Pillow")
            return

        save_path = output_path or self.gif_path
        if save_path is None:
            save_path = "animation.gif"

        print(f"正在保存GIF... 总帧数: {len(self.frames)}")

        # 跳帧以减小文件大小
        selected_frames = self.frames[::skip_frames]

        # 转换为PIL图像
        pil_frames = [Image.fromarray(frame) for frame in selected_frames]

        # 保存为GIF
        duration = int(1000 / fps * skip_frames)  # 毫秒
        pil_frames[0].save(
            save_path, save_all=True, append_images=pil_frames[1:], duration=duration, loop=0, optimize=True
        )

        print(f"GIF已保存到: {save_path}")
        print(f"保存帧数: {len(selected_frames)}, 每帧间隔: {duration}ms")

    def close(self):
        """关闭动画窗口并保存GIF"""
        # 如果设置了路径，自动保存
        if self.gif_path is not None and len(self.frames) > 0:
            self.save_to_gif()
        super().close()
