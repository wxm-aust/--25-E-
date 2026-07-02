#!/usr/bin/env python3
import sys
from PyQt5.QtWidgets import (QApplication, QWidget, QPushButton, 
                             QGridLayout, QScrollArea, QVBoxLayout,
                             QHBoxLayout, QLabel, QFrame, QGroupBox)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QFont
import rospy
from std_msgs.msg import Int32, Float32

class MyWindow(QWidget):
    def __init__(self):
        super().__init__()

        # 初始化ROS节点和发布者
        rospy.init_node('menu_node', anonymous=True)
        self.pub = rospy.Publisher('button_clicks', Int32, queue_size=10)
        
        # 创建订阅者
        rospy.Subscriber('current_value', Float32, self.current_callback)
        rospy.Subscriber('power_value', Float32, self.power_callback)
        rospy.Subscriber('d_value', Float32, self.d_callback)
        rospy.Subscriber('x_value', Float32, self.x_callback)

        # 设置窗口标题和大小
        self.setWindowTitle('Control Panel')
        self.setGeometry(500, 500, 500, 500)

        # 创建主布局
        main_layout = QVBoxLayout()
        main_layout.setContentsMargins(8, 8, 8, 8)
        main_layout.setSpacing(8)
        
        # 创建数值显示区域
        display_group = QGroupBox("实时监测")
        display_layout = QGridLayout(display_group)
        display_layout.setSpacing(5)
        
        # 创建四个显示窗口
        self.display_widgets = []
        display_titles = ["CURRENT (A)", "POWER (W)", "D", "x"]
        display_styles = [
            "color: blue; background-color: #f0f0ff;", 
            "color: red; background-color: #fff0f0;", 
            "color: green; background-color: #f0fff0;", 
            "color: purple; background-color: #f8f0ff;" 
        ]
        
        for i in range(4):
            frame = QFrame()
            frame.setFrameShape(QFrame.Box)
            frame.setLineWidth(1)
            frame.setStyleSheet("border: 1px solid #888; border-radius: 5px;")
            
            layout = QVBoxLayout(frame)
            layout.setContentsMargins(5, 5, 5, 5)
            
            title = QLabel(display_titles[i])
            title.setAlignment(Qt.AlignCenter)
            title.setFont(QFont("Arial", 8))
            
            value_label = QLabel("0.00" if i < 2 else "0")
            value_label.setAlignment(Qt.AlignCenter)
            value_label.setFont(QFont("Arial", 10, QFont.Bold))
            value_label.setStyleSheet(display_styles[i])
            
            layout.addWidget(title)
            layout.addWidget(value_label)
            
            # 设置固定尺寸
            frame.setFixedSize(110, 60)
            display_layout.addWidget(frame, i // 2, i % 2)
            self.display_widgets.append(value_label)
        
        # 添加到主布局
        main_layout.addWidget(display_group)
        
        # 创建按钮区域
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setMinimumHeight(350)
        
        # 创建内容部件
        content_widget = QWidget()
        self.grid_layout = QGridLayout(content_widget)
        self.grid_layout.setSpacing(4)
        
        # 定义20个按钮的名称（按您的要求）
        button_names = [
            "1", "2", "3", "4", "5", "6", "7", "8", "9", "0",
            "square", "triangle", "circle", "muti_square", "fold_square",
            "number_square", "angle_square", "test1", "test2"
        ]
        
        # 添加第20个按钮（您只提供了19个名称，所以添加一个占位符）
        button_names.append("btn20")  # 添加第20个按钮名称
        
        # 创建20个按钮
        self.buttons = []
        for i, name in enumerate(button_names):
            # 简化长名称的显示
            display_name = name
            if name == "muti_square":
                display_name = "m_square"
            elif name == "fold_square":
                display_name = "f_square"
            elif name == "number_square":
                display_name = "n_square"
            elif name == "angle_square":
                display_name = "a_square"
                
            button = QPushButton(display_name, self)
            button.setMinimumHeight(35)
            button.setMaximumHeight(35)
            button.setStyleSheet("""
                QPushButton {
                    font-size: 9pt;
                    background-color: #e0e0e0;
                    border: 1px solid #888;
                    border-radius: 4px;
                }
                QPushButton:hover {
                    background-color: #d0d0ff;
                }
            """)
            
            # 设置工具提示显示完整名称
            if display_name != name:
                button.setToolTip(name)
            
            # 绑定点击事件
            button.clicked.connect(lambda checked, n=i+1: self.button_click(n))
            self.buttons.append(button)
            
            # 添加到网格布局 (5列x4行)
            row = i // 5
            col = i % 5
            self.grid_layout.addWidget(button, row, col)
        
        # 设置滚动区域的内容部件
        scroll_area.setWidget(content_widget)
        main_layout.addWidget(scroll_area)
        
        # 设置主窗口的布局
        self.setLayout(main_layout)

        # 定时器
        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(self.check_ros_status)
        self.ros_timer.start(1000)
        
        # 模拟数据更新定时器
        self.data_timer = QTimer(self)
        # self.data_timer.timeout.connect(self.update_simulated_data)
        self.data_timer.start(500)

        # 初始化模拟数据
        self.sim_current = 0.0
        self.sim_power = 0.0
        self.sim_d = 0
        self.sim_x = 0

    def button_click(self, number):
        """当按钮被点击时调用此方法"""
        # 获取按钮的实际名称
        button_name = self.buttons[number-1].text()
        if self.buttons[number-1].toolTip():
            button_name = self.buttons[number-1].toolTip()
            
        message = f"Button {number} ({button_name}) was clicked"
        print(message)
        self.pub.publish(Int32(data=number))

    def current_callback(self, msg):
        self.display_widgets[0].setText(f"{msg.data:.2f}")

    def power_callback(self, msg):
        self.display_widgets[1].setText(f"{msg.data:.2f}")

    def d_callback(self, msg):
        self.display_widgets[2].setText(f"{(msg.data):.2f}")

    def x_callback(self, msg):
        self.display_widgets[3].setText(f"{(msg.data):.2f}")

    def update_simulated_data(self):
        self.sim_current += 0.1
        if self.sim_current > 5.0:
            self.sim_current = 0.0
        self.sim_power = self.sim_current * 12.0
        self.sim_d = (self.sim_d + 1) % 100
        self.sim_x = (self.sim_x + 2) % 100
        
        self.display_widgets[0].setText(f"{self.sim_current:.2f}")
        self.display_widgets[1].setText(f"{self.sim_power:.2f}")
        self.display_widgets[2].setText(f"{self.sim_d}")
        self.display_widgets[3].setText(f"{self.sim_x}")

    def check_ros_status(self):
        if rospy.is_shutdown():
            print("ROS has been shutdown. Exiting...")
            QApplication.quit()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MyWindow()
    window.show()
    try:
        sys.exit(app.exec_())
    except rospy.ROSInterruptException:
        pass