#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os
import warnings
import subprocess
import threading
import signal
import re
import math


# 在导入任何ROS包之前设置ROS环境变量
def setup_ros_environment():
    """设置ROS环境变量"""
    ros_distro = "noetic"  # 根据你的ROS版本修改
    ros_root = f"/opt/ros/{ros_distro}"

    # 设置基本的ROS环境变量
    os.environ['ROS_DISTRO'] = ros_distro
    os.environ['ROS_ROOT'] = f"{ros_root}/share/ros"
    os.environ['ROS_PACKAGE_PATH'] = f"{ros_root}/share"
    os.environ['ROS_MASTER_URI'] = "http://localhost:11311"
    os.environ['ROS_VERSION'] = "1"

    # 添加ROS Python路径到sys.path
    ros_python_path = f"{ros_root}/lib/python3/dist-packages"
    if os.path.exists(ros_python_path) and ros_python_path not in sys.path:
        sys.path.insert(0, ros_python_path)
        print(f"添加 ROS Python 路径: {ros_python_path}")

    # 设置PYTHONPATH环境变量
    current_pythonpath = os.environ.get('PYTHONPATH', '')
    if ros_python_path not in current_pythonpath:
        if current_pythonpath:
            os.environ['PYTHONPATH'] = f"{ros_python_path}:{current_pythonpath}"
        else:
            os.environ['PYTHONPATH'] = ros_python_path
        print(f"更新 PYTHONPATH: {os.environ['PYTHONPATH']}")

    # 打印环境变量设置
    for key in ['ROS_MASTER_URI', 'ROS_ROOT', 'ROS_PACKAGE_PATH', 'ROS_DISTRO']:
        print(f"设置环境变量: {key}={os.environ.get(key)}")


# 在导入其他模块之前设置ROS环境
setup_ros_environment()

warnings.filterwarnings('ignore', category=DeprecationWarning)

import numpy as np
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QPushButton, QLabel, QLineEdit,
                             QGroupBox, QFileDialog, QMessageBox, QTextEdit)
from PyQt5.QtCore import Qt, pyqtSignal, QThread, pyqtSlot, QTimer
from PyQt5.QtGui import QFont, QTextCursor, QColor
import pyqtgraph as pg
from pyqtgraph import Vector
import pyqtgraph.opengl as gl

# ROS相关导入
try:
    import rospy
    from geometry_msgs.msg import PoseStamped

    HAS_ROS = True
    print("ROS 包导入成功")
except ImportError as e:
    HAS_ROS = False
    print(f"警告: 未安装ROS相关包或环境配置错误，位姿信息将使用模拟数据: {e}")

# 如果安装了open3d，使用open3d读取pcd文件
try:
    import open3d as o3d

    HAS_OPEN3D = True
except ImportError:
    HAS_OPEN3D = False
    print("警告: 未安装open3d库，将使用模拟数据。请安装: pip install open3d")


class ROSSubscriberThread(QThread):
    """ROS订阅线程"""
    pose_signal = pyqtSignal(float, float, float, float, float, float)  # x, y, z, roll, pitch, yaw
    error_signal = pyqtSignal(str)
    connected_signal = pyqtSignal()  # 连接成功信号

    def __init__(self, topic_name="/kalman_filtered_pose"):
        super().__init__()
        self.topic_name = topic_name
        self.is_running = False
        self.subscriber = None

    def run(self):
        """在后台线程中运行ROS节点"""
        if not HAS_ROS:
            self.error_signal.emit("ROS未安装或环境配置错误，无法订阅话题")
            return

        try:
            # 检查ROS Master是否运行
            try:
                import rosgraph
                master = rosgraph.Master('/rostopic')
                master.getPid()
                print("ROS Master 连接成功")
            except Exception as e:
                self.error_signal.emit(f"无法连接到ROS Master: {str(e)}。请确保运行了 'roscore'")
                return

            # 初始化ROS节点
            if not rospy.get_node_uri():
                rospy.init_node('pcd_viewer_pose_subscriber', anonymous=True, disable_signals=True)
                print("ROS节点初始化成功")

            # 创建订阅者
            self.subscriber = rospy.Subscriber(self.topic_name, PoseStamped, self.pose_callback, queue_size=10)

            self.is_running = True
            self.connected_signal.emit()  # 发送连接成功信号
            print(f"开始订阅话题: {self.topic_name}")

            # 保持节点运行
            rate = rospy.Rate(30)  # 30Hz
            while self.is_running and not rospy.is_shutdown():
                try:
                    rate.sleep()
                except rospy.ROSInterruptException:
                    break

        except Exception as e:
            self.error_signal.emit(f"ROS订阅失败: {str(e)}")
            print(f"ROS订阅失败: {e}")
        finally:
            self.is_running = False
            if self.subscriber:
                self.subscriber.unregister()

    def pose_callback(self, msg):
        """位姿话题回调函数"""
        try:
            # 提取位置信息
            x = msg.pose.position.x
            y = msg.pose.position.y
            z = msg.pose.position.z

            # 根据用户描述，orientation的xyz对应roll, pitch, yaw
            roll = msg.pose.orientation.x
            pitch = msg.pose.orientation.y
            yaw = msg.pose.orientation.z

            # 发送信号到主线程更新UI
            self.pose_signal.emit(x, y, z, roll, pitch, yaw)

        except Exception as e:
            self.error_signal.emit(f"解析位姿数据失败: {str(e)}")

    def stop(self):
        """停止ROS订阅"""
        self.is_running = False
        if self.subscriber:
            try:
                self.subscriber.unregister()
            except:
                pass

        # 不要在这里调用rospy.signal_shutdown()，因为可能影响其他ROS操作
        self.quit()


class ROSLauncherThread(QThread):
    """ROS启动线程"""
    output_signal = pyqtSignal(str)
    error_signal = pyqtSignal(str)
    finished_signal = pyqtSignal(bool)

    def __init__(self, ndt_path="/media/dzt/pym/NDT"):
        super().__init__()
        self.ndt_path = ndt_path
        self.process = None
        self.is_running = False

    def run(self):
        """在后台线程中运行ROS命令"""
        try:
            # 检查路径是否存在
            if not os.path.exists(self.ndt_path):
                self.error_signal.emit(f"错误: NDT路径不存在: {self.ndt_path}")
                self.finished_signal.emit(False)
                return

            # 构建命令 - 设置环境变量避免一些警告
            cmd = f"""cd {self.ndt_path} && \
                     source devel/setup.bash && \
                     export ROSCONSOLE_FORMAT='[${{severity}}] [${{time}}]: ${{message}}' && \
                     roslaunch ndt_localizer ndt_localizer.launch"""

            self.output_signal.emit(f"正在启动NDT定位器...\n路径: {self.ndt_path}\n")
            self.output_signal.emit("执行命令:\n" + cmd.replace("\\", "").strip() + "\n")

            # 使用subprocess执行命令，设置环境变量
            env = os.environ.copy()
            env['PYTHONUNBUFFERED'] = '1'  # 确保Python输出不缓冲

            self.process = subprocess.Popen(
                cmd,
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,  # 将stderr重定向到stdout
                text=True,
                executable='/bin/bash',
                preexec_fn=os.setsid,  # 创建新的进程组
                env=env,
                bufsize=1,  # 行缓冲
                universal_newlines=True
            )

            self.is_running = True

            # 实时读取输出
            for line in iter(self.process.stdout.readline, ''):
                if not self.is_running:
                    break

                if line:
                    # 清理ANSI转义序列和其他控制字符
                    cleaned_line = self.clean_ansi_codes(line.strip())
                    if cleaned_line:
                        # 检测是否是错误信息
                        if '[ERROR]' in cleaned_line or '[WARN' in cleaned_line:
                            self.error_signal.emit(cleaned_line)
                        else:
                            self.output_signal.emit(cleaned_line)

                # 检查进程是否结束
                if self.process.poll() is not None:
                    break

            # 等待进程完全结束
            if self.process:
                self.process.wait()

            self.finished_signal.emit(self.process.returncode == 0 if self.process else False)

        except Exception as e:
            self.error_signal.emit(f"启动失败: {str(e)}")
            self.finished_signal.emit(False)
        finally:
            self.is_running = False

    def clean_ansi_codes(self, text):
        """清除ANSI转义序列和其他控制字符"""
        import re
        # 移除ANSI颜色代码
        ansi_escape = re.compile(r'\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])')
        text = ansi_escape.sub('', text)
        # 移除其他控制字符
        text = re.sub(r'\]2;.*?\x07', '', text)  # 移除终端标题设置
        text = re.sub(r'\[[\d;]*m', '', text)  # 移除颜色代码
        return text.strip()

    def stop(self):
        """停止ROS进程"""
        self.is_running = False
        if self.process:
            try:
                # 终止整个进程组
                os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
                self.output_signal.emit("\n定位系统已停止")
                # 等待进程结束
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                # 如果SIGTERM不起作用，使用SIGKILL
                try:
                    os.killpg(os.getpgid(self.process.pid), signal.SIGKILL)
                except:
                    pass
            except Exception as e:
                self.error_signal.emit(f"停止进程时出错: {str(e)}")


class TerminalDialog(QWidget):
    """终端输出对话框"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("ROS终端输出")
        self.setGeometry(100, 100, 800, 600)

        # 设置为独立窗口，避免自动显示
        self.setWindowFlags(Qt.Window)

        layout = QVBoxLayout()

        # 终端输出文本框
        self.terminal_output = QTextEdit()
        self.terminal_output.setReadOnly(True)
        self.terminal_output.setStyleSheet("""
            QTextEdit {
                background-color: #1e1e1e;
                color: #00ff00;
                font-family: 'Courier New', monospace;
                font-size: 12px;
                border: 2px solid #333;
            }
        """)

        # 按钮布局
        button_layout = QHBoxLayout()

        self.clear_button = QPushButton("清空输出")
        self.clear_button.clicked.connect(self.clear_output)

        self.close_button = QPushButton("关闭")
        self.close_button.clicked.connect(self.hide)

        button_layout.addWidget(self.clear_button)
        button_layout.addStretch()
        button_layout.addWidget(self.close_button)

        layout.addWidget(QLabel("终端输出:"))
        layout.addWidget(self.terminal_output)
        layout.addLayout(button_layout)

        self.setLayout(layout)

    def append_output(self, text, is_error=False):
        """添加输出文本"""
        if not text or not text.strip():
            return

        cursor = self.terminal_output.textCursor()
        cursor.movePosition(QTextCursor.End)
        self.terminal_output.setTextCursor(cursor)

        # 根据内容类型设置颜色
        if is_error or '[ERROR]' in text or '[WARN' in text:
            # 错误和警告信息用黄色/橙色显示
            if '[ERROR]' in text:
                self.terminal_output.setTextColor(Qt.red)
            else:
                self.terminal_output.setTextColor(QColor(255, 165, 0))  # 橙色
        elif '[INFO]' in text or 'started' in text.lower() or 'ready' in text.lower():
            # 信息用亮绿色显示
            self.terminal_output.setTextColor(QColor(0, 255, 0))
        else:
            # 正常信息用绿色显示
            self.terminal_output.setTextColor(Qt.green)

        self.terminal_output.insertPlainText(text + "\n")

        # 滚动到底部
        scrollbar = self.terminal_output.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

        # 限制输出行数，防止内存占用过大
        document = self.terminal_output.document()
        if document.lineCount() > 1000:
            cursor = self.terminal_output.textCursor()
            cursor.movePosition(QTextCursor.Start)
            cursor.movePosition(QTextCursor.Down, QTextCursor.KeepAnchor, 100)
            cursor.removeSelectedText()

    def clear_output(self):
        """清空输出"""
        self.terminal_output.clear()


class PCDViewer(QMainWindow):
    def __init__(self):
        super().__init__()
        self.point_cloud_data = None
        self.default_pcd_path = "/media/dzt/pym/tingchechang.pcd"  # 默认PCD文件路径
        self.gps_file_path = "/media/dzt/pym/gps_coordinates.txt"  # GPS坐标保存路径
        self.ndt_path = "/media/dzt/pym/NDT"  # NDT路径
        self.max_display_points = 20000000  # 默认最大显示点数

        # ROS启动相关
        self.ros_launcher = None
        self.is_localization_running = False
        self.terminal_dialog = None

        # ROS订阅相关
        self.ros_subscriber = None
        self.is_pose_subscription_active = False

        self.initUI()

        # 自动启动ROS位姿订阅
        self.start_pose_subscription()

    def initUI(self):
        """初始化用户界面"""
        self.setWindowTitle('PCD点云可视化工具')
        self.setGeometry(100, 100, 1200, 800)

        # 创建中央控件
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        # 创建主布局（垂直布局）
        main_layout = QVBoxLayout(central_widget)

        # 创建上部分布局（水平布局）
        top_layout = QHBoxLayout()

        # 左侧控制面板
        left_panel = self.create_left_panel()
        top_layout.addWidget(left_panel, 1)  # 比例为1

        # 右侧3D视图
        self.gl_widget = self.create_3d_view()
        top_layout.addWidget(self.gl_widget, 4)  # 比例为4

        # 将上部分添加到主布局
        main_layout.addLayout(top_layout, 5)  # 占据主要空间

        # 创建下部分姿态显示面板
        bottom_panel = self.create_bottom_panel()
        main_layout.addWidget(bottom_panel, 1)  # 占据较小空间

        # 创建终端对话框（但不显示）
        self.terminal_dialog = TerminalDialog(self)
        # 确保终端窗口默认隐藏
        self.terminal_dialog.hide()

        # 尝试加载默认PCD文件，如果不存在则加载示例点云
        self.load_default_or_sample()

    def create_left_panel(self):
        """创建左侧控制面板"""
        panel = QGroupBox("控制面板")
        panel.setMaximumWidth(250)

        layout = QVBoxLayout()

        # 定位启动按钮
        self.btn_start_localization = QPushButton("🎯 定位启动")
        self.btn_start_localization.setMinimumHeight(50)
        self.btn_start_localization.setStyleSheet("""
            QPushButton {
                background-color: #4CAF50;
                color: white;
                font-size: 16px;
                font-weight: bold;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #45a049;
            }
            QPushButton:pressed {
                background-color: #3d8b40;
            }
            QPushButton:disabled {
                background-color: #cccccc;
            }
        """)
        self.btn_start_localization.clicked.connect(self.start_localization)

        # 停止定位按钮
        self.btn_stop_localization = QPushButton("⏹ 停止定位")
        self.btn_stop_localization.setMinimumHeight(50)
        self.btn_stop_localization.setEnabled(False)
        self.btn_stop_localization.setStyleSheet("""
            QPushButton {
                background-color: #f44336;
                color: white;
                font-size: 16px;
                font-weight: bold;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #da190b;
            }
            QPushButton:pressed {
                background-color: #c00000;
            }
            QPushButton:disabled {
                background-color: #cccccc;
            }
        """)
        self.btn_stop_localization.clicked.connect(self.stop_localization)

        # 显示终端按钮
        self.btn_show_terminal = QPushButton("🖥 显示终端")
        self.btn_show_terminal.setMinimumHeight(40)
        self.btn_show_terminal.setStyleSheet("""
            QPushButton {
                background-color: #9C27B0;
                color: white;
                font-size: 14px;
                font-weight: bold;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #7B1FA2;
            }
        """)
        self.btn_show_terminal.clicked.connect(self.show_terminal)

        # 位姿订阅控制按钮
        self.btn_pose_subscription = QPushButton("📡 重启位姿订阅")
        self.btn_pose_subscription.setMinimumHeight(40)
        self.btn_pose_subscription.setStyleSheet("""
            QPushButton {
                background-color: #795548;
                color: white;
                font-size: 14px;
                font-weight: bold;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #5D4037;
            }
        """)
        self.btn_pose_subscription.clicked.connect(self.restart_pose_subscription)

        # 加载地图按钮
        self.btn_load_map = QPushButton("🗺 加载地图")
        self.btn_load_map.setMinimumHeight(50)
        self.btn_load_map.setStyleSheet("""
            QPushButton {
                background-color: #2196F3;
                color: white;
                font-size: 16px;
                font-weight: bold;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #1976D2;
            }
            QPushButton:pressed {
                background-color: #1565C0;
            }
        """)
        self.btn_load_map.clicked.connect(self.load_map)

        # 重置视角按钮
        self.btn_reset_view = QPushButton("🔄 重置视角")
        self.btn_reset_view.setMinimumHeight(40)
        self.btn_reset_view.setStyleSheet("""
            QPushButton {
                background-color: #FF9800;
                color: white;
                font-size: 14px;
                font-weight: bold;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #F57C00;
            }
        """)
        self.btn_reset_view.clicked.connect(self.reset_view)

        # 定位状态标签
        self.localization_status = QLabel("定位状态: 未启动")
        self.localization_status.setStyleSheet("""
            QLabel {
                padding: 10px;
                background-color: #f0f0f0;
                border-radius: 3px;
                color: #666;
                font-weight: bold;
            }
        """)

        # 位姿订阅状态标签
        self.pose_status = QLabel("位姿订阅: 正在连接...")
        self.pose_status.setStyleSheet("""
            QLabel {
                padding: 10px;
                background-color: #fff3cd;
                border-radius: 3px;
                color: #856404;
                font-weight: bold;
            }
        """)

        # 添加文件路径显示
        self.file_label = QLabel("未加载文件")
        self.file_label.setWordWrap(True)
        self.file_label.setStyleSheet("""
            QLabel {
                padding: 10px;
                background-color: #f0f0f0;
                border-radius: 3px;
                color: #666;
            }
        """)

        # 添加点云信息显示
        self.info_label = QLabel("点云信息：\n点数：0")
        self.info_label.setStyleSheet("""
            QLabel {
                padding: 10px;
                background-color: #f8f8f8;
                border-radius: 3px;
                color: #333;
            }
        """)

        # GPS坐标设置
        gps_group = QGroupBox("GPS坐标与姿态设置")
        gps_layout = QVBoxLayout()

        # 经度输入
        lon_layout = QHBoxLayout()
        lon_label = QLabel("经度:")
        lon_label.setMinimumWidth(50)
        self.longitude_input = QLineEdit("")
        self.longitude_input.setPlaceholderText("输入经度 (如: 116.397128)")
        lon_layout.addWidget(lon_label)
        lon_layout.addWidget(self.longitude_input)

        # 纬度输入
        lat_layout = QHBoxLayout()
        lat_label = QLabel("纬度:")
        lat_label.setMinimumWidth(50)
        self.latitude_input = QLineEdit("")
        self.latitude_input.setPlaceholderText("输入纬度 (如: 39.916527)")
        lat_layout.addWidget(lat_label)
        lat_layout.addWidget(self.latitude_input)

        # 高度输入
        alt_layout = QHBoxLayout()
        alt_label = QLabel("高度:")
        alt_label.setMinimumWidth(50)
        self.altitude_input = QLineEdit("")
        self.altitude_input.setPlaceholderText("输入高度 (米)")
        alt_layout.addWidget(alt_label)
        alt_layout.addWidget(self.altitude_input)

        # Roll输入
        roll_layout = QHBoxLayout()
        roll_label = QLabel("Roll:")
        roll_label.setMinimumWidth(50)
        self.roll_input = QLineEdit("")
        self.roll_input.setPlaceholderText("输入Roll角度 (度)")
        roll_layout.addWidget(roll_label)
        roll_layout.addWidget(self.roll_input)

        # Pitch输入
        pitch_layout = QHBoxLayout()
        pitch_label = QLabel("Pitch:")
        pitch_label.setMinimumWidth(50)
        self.pitch_input = QLineEdit("")
        self.pitch_input.setPlaceholderText("输入Pitch角度 (度)")
        pitch_layout.addWidget(pitch_label)
        pitch_layout.addWidget(self.pitch_input)

        # Yaw输入
        yaw_layout = QHBoxLayout()
        yaw_label = QLabel("Yaw:")
        yaw_label.setMinimumWidth(50)
        self.yaw_input = QLineEdit("")
        self.yaw_input.setPlaceholderText("输入Yaw角度 (度)")
        yaw_layout.addWidget(yaw_label)
        yaw_layout.addWidget(self.yaw_input)

        # 保存按钮
        self.btn_save_gps = QPushButton("💾 保存GPS坐标与姿态")
        self.btn_save_gps.setMinimumHeight(35)
        self.btn_save_gps.setStyleSheet("""
            QPushButton {
                background-color: #607D8B;
                color: white;
                font-size: 14px;
                font-weight: bold;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #455A64;
            }
            QPushButton:pressed {
                background-color: #37474F;
            }
        """)
        self.btn_save_gps.clicked.connect(self.save_gps_coordinates)

        # 加载已保存的GPS坐标
        self.load_gps_coordinates()

        gps_layout.addLayout(lon_layout)
        gps_layout.addLayout(lat_layout)
        gps_layout.addLayout(alt_layout)
        gps_layout.addLayout(roll_layout)
        gps_layout.addLayout(pitch_layout)
        gps_layout.addLayout(yaw_layout)
        gps_layout.addWidget(self.btn_save_gps)
        gps_group.setLayout(gps_layout)

        layout.addWidget(self.btn_start_localization)
        layout.addWidget(self.btn_stop_localization)
        layout.addWidget(self.btn_show_terminal)
        layout.addWidget(self.btn_pose_subscription)
        layout.addWidget(self.localization_status)
        layout.addWidget(self.pose_status)
        layout.addWidget(self.btn_load_map)
        layout.addWidget(self.btn_reset_view)
        layout.addWidget(QLabel("当前文件："))
        layout.addWidget(self.file_label)
        layout.addWidget(QLabel(""))  # 空白间隔
        layout.addWidget(self.info_label)
        layout.addWidget(gps_group)
        layout.addStretch()

        panel.setLayout(layout)
        return panel

    def create_3d_view(self):
        """创建3D视图控件"""
        # 创建3D图形窗口
        w = gl.GLViewWidget()
        w.setCameraPosition(distance=100, elevation=30, azimuth=45)
        w.opts['center'] = Vector(0, 0, 0)  # 设置初始中心点
        w.setBackgroundColor('w')

        # 添加网格
        grid = gl.GLGridItem()
        grid.scale(10, 10, 1)
        w.addItem(grid)

        # 添加坐标轴
        axis = gl.GLAxisItem()
        axis.setSize(20, 20, 20)
        w.addItem(axis)

        # 保存为属性以便后续使用
        self.view_widget = w
        self.scatter_plot = None

        return w

    def create_bottom_panel(self):
        """创建底部姿态显示面板"""
        panel = QGroupBox("位姿信息")
        panel.setMaximumHeight(120)

        layout = QHBoxLayout()

        # 创建6个输入框用于显示x, y, z, roll, pitch, yaw
        self.pose_inputs = {}
        labels = ['X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw']
        units = ['m', 'm', 'm', '°', '°', '°']

        for i, (label, unit) in enumerate(zip(labels, units)):
            # 创建每个参数的垂直布局
            param_layout = QVBoxLayout()

            # 标签
            lbl = QLabel(f"{label} ({unit})")
            lbl.setAlignment(Qt.AlignCenter)
            lbl.setStyleSheet("font-weight: bold; color: #333;")

            # 输入框
            input_field = QLineEdit("0.00")
            input_field.setAlignment(Qt.AlignCenter)
            input_field.setReadOnly(True)  # 设置为只读
            input_field.setStyleSheet("""
                QLineEdit {
                    background-color: #f5f5f5;
                    border: 2px solid #ddd;
                    border-radius: 5px;
                    padding: 8px;
                    font-size: 14px;
                    font-weight: bold;
                    color: #2196F3;
                }
            """)

            self.pose_inputs[label.lower()] = input_field

            param_layout.addWidget(lbl)
            param_layout.addWidget(input_field)

            # 添加到主布局
            layout.addLayout(param_layout)

        panel.setLayout(layout)
        return panel

    def start_pose_subscription(self):
        """启动位姿话题订阅"""
        if self.is_pose_subscription_active:
            return

        if not HAS_ROS:
            self.pose_status.setText("位姿订阅: ROS未安装或配置错误")
            self.pose_status.setStyleSheet("""
                QLabel {
                    padding: 10px;
                    background-color: #f8d7da;
                    border-radius: 3px;
                    color: #721c24;
                    font-weight: bold;
                }
            """)
            return

        try:
            # 创建ROS订阅线程
            self.ros_subscriber = ROSSubscriberThread("/kalman_filtered_pose")

            # 连接信号
            self.ros_subscriber.pose_signal.connect(self.update_pose_from_ros, Qt.QueuedConnection)
            self.ros_subscriber.error_signal.connect(self.on_pose_error, Qt.QueuedConnection)
            self.ros_subscriber.connected_signal.connect(self.on_pose_connected, Qt.QueuedConnection)

            # 启动线程
            self.ros_subscriber.start()
            self.is_pose_subscription_active = True

            self.pose_status.setText("位姿订阅: 正在连接...")
            self.pose_status.setStyleSheet("""
                QLabel {
                    padding: 10px;
                    background-color: #fff3cd;
                    border-radius: 3px;
                    color: #856404;
                    font-weight: bold;
                }
            """)

            print("位姿话题订阅已启动")

        except Exception as e:
            self.pose_status.setText(f"位姿订阅: 连接失败")
            self.pose_status.setStyleSheet("""
                QLabel {
                    padding: 10px;
                    background-color: #f8d7da;
                    border-radius: 3px;
                    color: #721c24;
                    font-weight: bold;
                }
            """)
            print(f"启动位姿订阅失败: {e}")

    def restart_pose_subscription(self):
        """重启位姿订阅"""
        # 先停止现有订阅
        if self.ros_subscriber:
            self.ros_subscriber.stop()
            self.ros_subscriber.wait()
            self.ros_subscriber = None

        self.is_pose_subscription_active = False

        # 重新启动
        self.start_pose_subscription()

    @pyqtSlot()
    def on_pose_connected(self):
        """位姿订阅连接成功"""
        self.pose_status.setText("位姿订阅: 已连接")
        self.pose_status.setStyleSheet("""
            QLabel {
                padding: 10px;
                background-color: #d4edda;
                border-radius: 3px;
                color: #155724;
                font-weight: bold;
            }
        """)

    @pyqtSlot(float, float, float, float, float, float)
    def update_pose_from_ros(self, x, y, z, roll, pitch, yaw):
        """从ROS话题更新位姿显示"""
        try:
            self.pose_inputs['x'].setText(f"{x:.3f}")
            self.pose_inputs['y'].setText(f"{y:.3f}")
            self.pose_inputs['z'].setText(f"{z:.3f}")
            self.pose_inputs['roll'].setText(f"{math.degrees(roll):.2f}")  # 转换为角度
            self.pose_inputs['pitch'].setText(f"{math.degrees(pitch):.2f}")  # 转换为角度
            self.pose_inputs['yaw'].setText(f"{math.degrees(yaw):.2f}")  # 转换为角度

        except Exception as e:
            print(f"更新位姿显示失败: {e}")

    @pyqtSlot(str)
    def on_pose_error(self, error_msg):
        """处理位姿订阅错误"""
        self.pose_status.setText("位姿订阅: 连接断开")
        self.pose_status.setStyleSheet("""
            QLabel {
                padding: 10px;
                background-color: #f8d7da;
                border-radius: 3px;
                color: #721c24;
                font-weight: bold;
            }
        """)
        print(f"位姿订阅错误: {error_msg}")

    def load_default_or_sample(self):
        """尝试加载默认PCD文件或示例数据"""
        if os.path.exists(self.default_pcd_path):
            print(f"找到默认PCD文件：{self.default_pcd_path}")
            self.load_pcd_file(self.default_pcd_path)
        else:
            print(f"默认PCD文件不存在：{self.default_pcd_path}")
            self.load_sample_pointcloud()

    def load_sample_pointcloud(self):
        """加载示例点云数据"""
        # 生成示例点云数据（一个立方体）
        n_points = 1000
        points = np.random.randn(n_points, 3) * 10

        # 添加一些结构
        # 地面
        ground = np.random.randn(500, 3)
        ground[:, 2] = -5 + np.random.randn(500) * 0.5
        ground[:, :2] *= 20

        # 墙壁
        wall = np.random.randn(300, 3)
        wall[:, 0] = 15 + np.random.randn(300) * 0.5
        wall[:, 1] *= 10
        wall[:, 2] = wall[:, 2] * 3

        points = np.vstack([points, ground, wall])

        self.display_pointcloud(points)
        self.update_info(len(points), len(points))
        self.file_label.setText("示例数据")

    def save_gps_coordinates(self):
        """保存GPS坐标和姿态到文本文件"""
        try:
            longitude = self.longitude_input.text().strip()
            latitude = self.latitude_input.text().strip()
            altitude = self.altitude_input.text().strip()
            roll = self.roll_input.text().strip()
            pitch = self.pitch_input.text().strip()
            yaw = self.yaw_input.text().strip()

            # 验证输入
            if not longitude or not latitude or not altitude:
                QMessageBox.warning(self, "警告", "请至少填写经度、纬度和高度！")
                return

            # 验证数值
            try:
                lon_val = float(longitude)
                lat_val = float(latitude)
                alt_val = float(altitude)
                roll_val = float(roll) if roll else 0.0
                pitch_val = float(pitch) if pitch else 0.0
                yaw_val = float(yaw) if yaw else 0.0
            except ValueError:
                QMessageBox.warning(self, "警告", "请输入有效的数值！")
                return

            # 保存到文件
            import datetime
            timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

            # 确保目录存在
            save_dir = os.path.dirname(self.gps_file_path)
            if not os.path.exists(save_dir):
                os.makedirs(save_dir, exist_ok=True)

            # 写入文件（追加模式）
            with open(self.gps_file_path, 'a', encoding='utf-8') as f:
                f.write(f"{timestamp}\t{longitude}\t{latitude}\t{altitude}\t{roll_val}\t{pitch_val}\t{yaw_val}\n")

            # 同时保存一个最新的GPS坐标文件（覆盖模式）
            latest_file = os.path.join(save_dir, "latest_gps.txt")
            with open(latest_file, 'w', encoding='utf-8') as f:
                f.write(f"经度: {longitude}\n")
                f.write(f"纬度: {latitude}\n")
                f.write(f"高度: {altitude}\n")
                f.write(f"Roll: {roll_val}\n")
                f.write(f"Pitch: {pitch_val}\n")
                f.write(f"Yaw: {yaw_val}\n")
                f.write(f"时间: {timestamp}\n")

            QMessageBox.information(self, "成功",
                                    f"GPS坐标与姿态已保存！\n"
                                    f"经度: {longitude}\n"
                                    f"纬度: {latitude}\n"
                                    f"高度: {altitude} 米\n"
                                    f"Roll: {roll_val}°\n"
                                    f"Pitch: {pitch_val}°\n"
                                    f"Yaw: {yaw_val}°\n"
                                    f"保存路径: {self.gps_file_path}")

            print(f"GPS坐标已保存: 经度={longitude}, 纬度={latitude}, 高度={altitude}, "
                  f"Roll={roll_val}, Pitch={pitch_val}, Yaw={yaw_val}")

        except Exception as e:
            QMessageBox.critical(self, "错误", f"保存GPS坐标失败：\n{str(e)}")
            print(f"保存GPS坐标失败: {e}")

    def load_gps_coordinates(self):
        """加载最近保存的GPS坐标和姿态"""
        try:
            latest_file = os.path.join(os.path.dirname(self.gps_file_path), "latest_gps.txt")
            if os.path.exists(latest_file):
                with open(latest_file, 'r', encoding='utf-8') as f:
                    lines = f.readlines()
                    for line in lines:
                        if line.startswith("经度:"):
                            self.longitude_input.setText(line.split(":")[1].strip())
                        elif line.startswith("纬度:"):
                            self.latitude_input.setText(line.split(":")[1].strip())
                        elif line.startswith("高度:"):
                            self.altitude_input.setText(line.split(":")[1].strip())
                        elif line.startswith("Roll:"):
                            self.roll_input.setText(line.split(":")[1].strip())
                        elif line.startswith("Pitch:"):
                            self.pitch_input.setText(line.split(":")[1].strip())
                        elif line.startswith("Yaw:"):
                            self.yaw_input.setText(line.split(":")[1].strip())
                print("已加载上次保存的GPS坐标和姿态")
        except Exception as e:
            print(f"加载GPS坐标失败: {e}")

    def display_pointcloud(self, points, auto_adjust=True):
        """显示点云数据 - 修复版本"""
        # 检查数据有效性
        if points is None or len(points) == 0:
            print("警告: 点云数据为空！")
            return

        print(f"准备显示 {len(points)} 个点")

        # 使用固定的最大显示点数
        max_points = self.max_display_points

        # 如果点云太大，进行下采样
        display_points = points
        if len(points) > max_points:
            print(f"点云过大，下采样到 {max_points} 个点")
            indices = np.random.choice(len(points), max_points, replace=False)
            display_points = points[indices]

        # 移除旧的点云
        if self.scatter_plot is not None:
            self.view_widget.removeItem(self.scatter_plot)

        # 确保数据类型正确
        display_points = display_points.astype(np.float32)

        # 创建颜色（根据高度着色）
        z = display_points[:, 2]
        if z.max() - z.min() > 0:
            z_norm = (z - z.min()) / (z.max() - z.min())  # 归一化到0-1
        else:
            z_norm = np.zeros_like(z)

        # 创建彩虹色映射
        colors = np.zeros((len(display_points), 4))
        colors[:, 0] = z_norm  # R
        colors[:, 1] = 1 - z_norm * 0.5  # G
        colors[:, 2] = 1 - z_norm  # B
        colors[:, 3] = 1.0  # Alpha

        # 创建散点图 - 使用更大的点尺寸和不同的渲染方式
        self.scatter_plot = gl.GLScatterPlotItem(
            pos=display_points,
            color=colors.astype(np.float32),
            size=3,  # 增大点的尺寸
            pxMode=True,  # 使用像素模式
            glOptions='opaque'  # 使用不透明渲染
        )

        # 添加到视图
        self.view_widget.addItem(self.scatter_plot)
        self.point_cloud_data = points  # 保存原始数据

        # 自动调整视角
        if auto_adjust:
            self.auto_adjust_view(display_points)

        # 强制更新显示
        self.view_widget.update()

        print(f"成功显示点云，实际显示 {len(display_points)} 个点")

    def auto_adjust_view(self, points):
        """自动调整视角以适应点云 - 改进版本"""
        if len(points) == 0:
            return

        # 计算点云中心和范围
        center = np.mean(points, axis=0)
        min_vals = np.min(points, axis=0)
        max_vals = np.max(points, axis=0)
        ranges = max_vals - min_vals
        max_range = np.max(ranges)

        # 设置相机距离为最大范围的1.5倍
        distance = max_range * 1.5

        # 更新相机位置
        self.view_widget.setCameraPosition(
            distance=distance,
            elevation=20,  # 降低仰角
            azimuth=45
        )

        # 设置视图中心点
        self.view_widget.opts['center'] = Vector(float(center[0]), float(center[1]), float(center[2]))

        # 更新网格
        for item in self.view_widget.items[:]:  # 创建副本以避免迭代时修改
            if isinstance(item, gl.GLGridItem):
                self.view_widget.removeItem(item)

        # 添加新的网格
        grid = gl.GLGridItem()
        grid.setSize(max_range * 2, max_range * 2, 1)
        grid.setSpacing(max_range / 10, max_range / 10, 1)
        grid.translate(center[0], center[1], min_vals[2] - 1)  # 将网格放在点云下方
        self.view_widget.addItem(grid)

        print(f"视角已调整 - 中心: {center}, 距离: {distance}, 范围: {ranges}")

    def reset_view(self):
        """重置视角"""
        if self.point_cloud_data is not None:
            # 重新调整到当前点云
            max_points = self.max_display_points

            display_points = self.point_cloud_data
            if len(self.point_cloud_data) > max_points:
                indices = np.random.choice(len(self.point_cloud_data), max_points, replace=False)
                display_points = self.point_cloud_data[indices]

            # 重新显示点云
            self.display_pointcloud(display_points, auto_adjust=True)
        else:
            # 默认视角
            self.view_widget.setCameraPosition(distance=100, elevation=30, azimuth=45)
            self.view_widget.opts['center'] = Vector(0, 0, 0)

    def update_info(self, total_points, display_points):
        """更新点云信息显示"""
        self.info_label.setText(f"点云信息：\n总点数：{total_points:,}\n显示点数：{display_points:,}")

        if self.point_cloud_data is not None:
            min_vals = np.min(self.point_cloud_data, axis=0)
            max_vals = np.max(self.point_cloud_data, axis=0)
            self.info_label.setText(
                f"点云信息：\n"
                f"总点数：{total_points:,}\n"
                f"显示点数：{display_points:,}\n"
                f"X范围：[{min_vals[0]:.2f}, {max_vals[0]:.2f}]\n"
                f"Y范围：[{min_vals[1]:.2f}, {max_vals[1]:.2f}]\n"
                f"Z范围：[{min_vals[2]:.2f}, {max_vals[2]:.2f}]"
            )

    def start_localization(self):
        """启动定位功能 - 执行ROS命令"""
        if self.is_localization_running:
            QMessageBox.information(self, "提示", "定位系统已在运行中！")
            return

        reply = QMessageBox.question(self, '确认启动',
                                     f'将在以下路径执行ROS命令:\n{self.ndt_path}\n\n确定要启动NDT定位系统吗？',
                                     QMessageBox.Yes | QMessageBox.No,
                                     QMessageBox.No)

        if reply == QMessageBox.Yes:
            # 创建并启动ROS线程
            self.ros_launcher = ROSLauncherThread(self.ndt_path)

            # 确保使用Qt的信号槽机制，避免线程问题
            self.ros_launcher.output_signal.connect(self.on_ros_output, Qt.QueuedConnection)
            self.ros_launcher.error_signal.connect(self.on_ros_error, Qt.QueuedConnection)
            self.ros_launcher.finished_signal.connect(self.on_ros_finished, Qt.QueuedConnection)

            # 设置线程优先级，避免阻塞主线程
            self.ros_launcher.setPriority(QThread.LowPriority)

            # 启动线程
            self.ros_launcher.start()

            # 更新UI状态
            self.is_localization_running = True
            self.btn_start_localization.setEnabled(False)
            self.btn_stop_localization.setEnabled(True)
            self.localization_status.setText("定位状态: 正在启动...")
            self.localization_status.setStyleSheet("""
                QLabel {
                    padding: 10px;
                    background-color: #fff3cd;
                    border-radius: 3px;
                    color: #856404;
                    font-weight: bold;
                }
            """)

            # 显示终端窗口（只在点击定位启动时才显示）
            self.show_terminal()

            # 确保主界面保持响应
            QApplication.processEvents()

    def stop_localization(self):
        """停止定位系统"""
        if not self.is_localization_running:
            return

        reply = QMessageBox.question(self, '确认停止',
                                     '确定要停止NDT定位系统吗？',
                                     QMessageBox.Yes | QMessageBox.No,
                                     QMessageBox.No)

        if reply == QMessageBox.Yes:
            if self.ros_launcher:
                self.ros_launcher.stop()

            # 更新UI状态
            self.is_localization_running = False
            self.btn_start_localization.setEnabled(True)
            self.btn_stop_localization.setEnabled(False)
            self.localization_status.setText("定位状态: 已停止")
            self.localization_status.setStyleSheet("""
                QLabel {
                    padding: 10px;
                    background-color: #f8d7da;
                    border-radius: 3px;
                    color: #721c24;
                    font-weight: bold;
                }
            """)

    def show_terminal(self):
        """显示终端窗口"""
        if self.terminal_dialog:
            self.terminal_dialog.show()
            self.terminal_dialog.raise_()
            self.terminal_dialog.activateWindow()

    @pyqtSlot(str)
    def on_ros_output(self, text):
        """处理ROS输出"""
        if self.terminal_dialog:
            self.terminal_dialog.append_output(text, is_error=False)

        # 检查是否成功启动
        if "started" in text.lower() or "ready" in text.lower() or "load" in text.lower():
            self.localization_status.setText("定位状态: 运行中")
            self.localization_status.setStyleSheet("""
                QLabel {
                    padding: 10px;
                    background-color: #d4edda;
                    border-radius: 3px;
                    color: #155724;
                    font-weight: bold;
                }
            """)

        # 检查是否正在加载地图
        if "load" in text.lower() and ".pcd" in text.lower():
            self.localization_status.setText("定位状态: 正在加载地图...")

    @pyqtSlot(str)
    def on_ros_error(self, text):
        """处理ROS错误输出"""
        if self.terminal_dialog:
            # 过滤掉一些不重要的警告
            if "disk usage" in text.lower() or "rosclean" in text.lower():
                # 这只是磁盘空间警告，不是真正的错误
                self.terminal_dialog.append_output(text, is_error=False)
            elif "jsk_rviz_plugin" in text:
                # 这是插件警告，通常不影响功能
                self.terminal_dialog.append_output(text, is_error=False)
            else:
                self.terminal_dialog.append_output(text, is_error=True)

    @pyqtSlot(bool)
    def on_ros_finished(self, success):
        """处理ROS进程结束"""
        if success:
            QMessageBox.information(self, "提示", "定位系统已正常结束")
        else:
            QMessageBox.warning(self, "警告", "定位系统异常退出，请检查终端输出")

        # 重置状态
        self.is_localization_running = False
        self.btn_start_localization.setEnabled(True)
        self.btn_stop_localization.setEnabled(False)
        self.localization_status.setText("定位状态: 未启动")
        self.localization_status.setStyleSheet("""
            QLabel {
                padding: 10px;
                background-color: #f0f0f0;
                border-radius: 3px;
                color: #666;
                font-weight: bold;
            }
        """)

    def load_pcd_file(self, file_path):
        """加载PCD文件 - 修复版本"""
        try:
            points = None

            if HAS_OPEN3D and file_path.endswith(('.pcd', '.ply')):
                # 使用Open3D读取PCD文件
                print(f"使用Open3D读取文件：{file_path}")
                pcd = o3d.io.read_point_cloud(file_path)
                points = np.asarray(pcd.points)

                # 确保点云数据是float32类型
                points = points.astype(np.float32)

                print(f"成功读取 {len(points)} 个点")
                print(f"点云范围：X[{points[:, 0].min():.2f}, {points[:, 0].max():.2f}], "
                      f"Y[{points[:, 1].min():.2f}, {points[:, 1].max():.2f}], "
                      f"Z[{points[:, 2].min():.2f}, {points[:, 2].max():.2f}]")

            elif file_path.endswith(('.txt', '.xyz')):
                # 读取文本格式的点云文件
                print(f"读取文本文件：{file_path}")
                points = np.loadtxt(file_path)
                if points.ndim == 2 and points.shape[1] >= 3:
                    points = points[:, :3].astype(np.float32)  # 只取前三列(x,y,z)
                else:
                    raise ValueError("文本文件格式不正确")
            else:
                raise ValueError(f"不支持的文件格式：{file_path}")

            if points is not None and len(points) > 0:
                # 显示点云
                self.display_pointcloud(points, auto_adjust=True)

                # 更新信息
                max_display = self.max_display_points

                actual_display = min(len(points), max_display)
                self.update_info(len(points), actual_display)

                # 更新文件路径显示
                filename = os.path.basename(file_path)
                self.file_label.setText(filename)

                print(f"点云加载成功：总共 {len(points):,} 个点")
                # 静默显示加载信息，不弹出对话框
            else:
                raise ValueError("点云数据为空")

        except Exception as e:
            print(f"加载文件失败：{str(e)}")
            QMessageBox.critical(self, "错误", f"加载文件失败：\n{str(e)}")
            # 加载示例数据
            self.load_sample_pointcloud()

    def load_map(self):
        """加载PCD地图文件"""
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "选择PCD文件",
            os.path.dirname(self.default_pcd_path) if os.path.exists(os.path.dirname(self.default_pcd_path)) else "",
            "Point Cloud Files (*.pcd *.ply *.xyz *.txt);;All Files (*)"
        )

        if file_path:
            self.load_pcd_file(file_path)

    def keyPressEvent(self, event):
        """处理键盘事件"""
        if event.key() == Qt.Key_R:
            # 按R键重置视角
            self.reset_view()
        elif event.key() == Qt.Key_Escape:
            self.close()

    def closeEvent(self, event):
        """窗口关闭事件"""
        # 停止ROS进程
        if self.is_localization_running and self.ros_launcher:
            reply = QMessageBox.question(self, '确认关闭',
                                         '定位系统正在运行，是否停止并关闭？',
                                         QMessageBox.Yes | QMessageBox.No,
                                         QMessageBox.No)

            if reply == QMessageBox.Yes:
                self.ros_launcher.stop()
                event.accept()
            else:
                event.ignore()
                return

        # 停止位姿订阅
        if self.ros_subscriber:
            self.ros_subscriber.stop()
            self.ros_subscriber.wait()

        event.accept()


def main():
    try:
        app = QApplication(sys.argv)
        app.setStyle('Fusion')  # 使用Fusion风格，更现代
        viewer = PCDViewer()
        viewer.show()
        sys.exit(app.exec_())
    except Exception as e:
        print(f"程序运行错误: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()