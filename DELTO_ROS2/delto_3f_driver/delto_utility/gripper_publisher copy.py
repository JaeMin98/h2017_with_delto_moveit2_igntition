import sys
import time
import math
from PyQt5.QtWidgets import QApplication, QSlider, QVBoxLayout, QWidget, QLabel, QHBoxLayout, QPushButton
from PyQt5.QtCore import Qt, QThread, QTimer
import rclpy
from std_msgs.msg import Float32MultiArray

class SliderPublisher(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.target_joint_state = [0.0]*12
        self.current_joint_state = [0.0]*12
        self.publisher = self.node.create_publisher(Float32MultiArray, 'gripper/target_joint', 10)
        
        # 목표 위치들 (라디안)
        self.goal_positions = {
            'reset': [0.0, 0.0, 0.0, 0.0,
                     0.0, 0.0, 0.0, 0.0,
                     0.0, 0.0, 0.0, 0.0],
            'finger1': [0.0, -0.017453288659453392, 0.9948374629020691, 1.2566368579864502, 
                       0.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, 0.0, 0.0],
            'finger2': [0.0, 0.0, 0.0, 0.0,
                       -0.6283184289932251, -0.08726644515991211, 1.029744029045105, 
                       1.2042769193649292,
                       0.0, 0.0, 0.0, 0.0],
            'finger3': [0.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, 0.0, 0.0,
                       0.9424775838851929, -0.034906577318906784, 
                       0.8901177048683167, 1.117010474205017],
            'all_fingers': [0.0, -0.017453288659453392, 0.9948374629020691, 1.2566368579864502, 
                           -0.6283184289932251, -0.08726644515991211, 1.029744029045105, 
                           1.2042769193649292, 0.9424775838851929, -0.034906577318906784, 
                           0.8901177048683167, 1.117010474205017]
        }
        
        # 현재 목표 위치 (기본값: 모든 핑거)
        self.current_goal = self.goal_positions['all_fingers']
        
        self.initUI()

    def initUI(self):
        self.sliders = []
        self.labels = []
        layout = QVBoxLayout()

        # 궤적 이동 버튼들 추가
        button_layout = QHBoxLayout()
        
        reset_button = QPushButton('Reset (All Zero)', self)
        reset_button.clicked.connect(lambda: self.start_trajectory_to_goal('reset'))
        button_layout.addWidget(reset_button)
        
        finger1_button = QPushButton('Finger 1 Only', self)
        finger1_button.clicked.connect(lambda: self.start_trajectory_to_goal('finger1'))
        button_layout.addWidget(finger1_button)
        
        finger2_button = QPushButton('Finger 2 Only', self)
        finger2_button.clicked.connect(lambda: self.start_trajectory_to_goal('finger2'))
        button_layout.addWidget(finger2_button)
        
        finger3_button = QPushButton('Finger 3 Only', self)
        finger3_button.clicked.connect(lambda: self.start_trajectory_to_goal('finger3'))
        button_layout.addWidget(finger3_button)
        
        layout.addLayout(button_layout)
        
        all_fingers_button = QPushButton('All Fingers (Full Grasp)', self)
        all_fingers_button.clicked.connect(lambda: self.start_trajectory_to_goal('all_fingers'))
        layout.addWidget(all_fingers_button)
        
        manual_reset_button = QPushButton('Manual Reset to Zero', self)
        manual_reset_button.clicked.connect(self.reset_to_zero)
        layout.addWidget(manual_reset_button)

        for i in range(12):
            slider = QSlider(Qt.Horizontal, self)
            slider.setMinimum(-180)  # Set the minimum value
            slider.setMaximum(180)  # Set the maximum value
            slider.setValue(0) 
            label = QLabel('0', self)
            slider.valueChanged[int].connect(lambda value, i=i, label=label: self.make_pub(value, i+1, label))
            
            h_layout = QHBoxLayout()
            h_layout.addWidget(QLabel(f'Joint {i+1}:'))
            h_layout.addWidget(slider)
            h_layout.addWidget(label)
            layout.addLayout(h_layout)
            self.sliders.append(slider)
            self.labels.append(label)

        self.setLayout(layout)
        self.setWindowTitle('Delto Joint Controller')
        self.show()

    def make_pub(self, value, i, label):
        self.target_joint_state[i-1] = float(value* 3.141592 / 180.0)
        self.current_joint_state[i-1] = self.target_joint_state[i-1]
        
        print(self.target_joint_state)
        msg = Float32MultiArray()
        msg.data = self.target_joint_state
        label.setText(str(value))
        self.node.get_logger().info('Publishing: "%s"' % msg.data)
        self.publisher.publish(msg)
    
    def start_trajectory_to_goal(self, goal_name):
        """지정된 목표 위치로 즉시 이동"""
        self.current_goal = self.goal_positions[goal_name]
        
        # 목표 위치로 즉시 설정
        self.current_joint_state = self.current_goal.copy()
        self.target_joint_state = self.current_goal.copy()
        
        # 슬라이더와 라벨 업데이트
        for i in range(12):
            angle_deg = int(self.current_joint_state[i] * 180.0 / 3.141592)
            self.sliders[i].setValue(angle_deg)
            self.labels[i].setText(str(angle_deg))
        
        # ROS 메시지 발행
        msg = Float32MultiArray()
        msg.data = self.current_joint_state.copy()
        self.publisher.publish(msg)
        
        goal_descriptions = {
            'reset': 'all joints to zero',
            'finger1': 'finger 1 only',
            'finger2': 'finger 2 only', 
            'finger3': 'finger 3 only',
            'all_fingers': 'all fingers (full grasp)'
        }
        
        self.node.get_logger().info(f'Moved to {goal_descriptions[goal_name]} position: {self.current_joint_state}')
    
    def start_trajectory_movement(self):
        """기존 목표 위치로 즉시 이동 (하위 호환성)"""
        self.start_trajectory_to_goal('all_fingers')
    
    def reset_to_zero(self):
        """모든 관절을 0으로 리셋"""
        for i in range(12):
            self.sliders[i].setValue(0)
            self.labels[i].setText('0')
            self.target_joint_state[i] = 0.0
            self.current_joint_state[i] = 0.0
        
        msg = Float32MultiArray()
        msg.data = self.target_joint_state.copy()
        self.publisher.publish(msg)
        
        self.node.get_logger().info('Reset all joints to zero')

class ROSThread(QThread):
    def __init__(self, node):
        QThread.__init__(self)
        self.node = node

    def run(self):
        rclpy.spin(self.node)

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('slider_publisher')
    ros_thread = ROSThread(node)
    ros_thread.start()

    app = QApplication(sys.argv)
    ex = SliderPublisher(node)
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()