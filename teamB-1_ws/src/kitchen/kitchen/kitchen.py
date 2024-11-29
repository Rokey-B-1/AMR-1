import sys
import time
import functools
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QApplication, QMainWindow, QMessageBox
from PySide2.QtCore import *

import rclpy
import rclpy.action
from rclpy.node import Node
from system_interfaces.msg import OrderStatus
from system_interfaces.srv import OrderDetail
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from nav2_msgs.srv import SetInitialPose
from geometry_msgs.msg import Point, Quaternion
# from std_msgs.msg import String
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

INIT = "안녕하세요"
PREPARING = "Preparing..."
CANCELED = "Canceled..."
DELIVERING = "Delivering..."

coordinates = {
    1 : [2.845625400543213, 1.3294998407363892, 0.0],
    2 : [2.845625400543213, 0.1905091106891632, 0.0],
    3 : [2.845625400543213, -0.8839163184165955, 0.0],
    4 : [1.8414881229400635, 1.3294998407363892, 0.0],
    5 : [1.8414881229400635, 0.1905091106891632, 0.0],
    6 : [1.8414881229400635, -0.8839163184165955, 0.0],
    "Kitchen" : [0.9438039660453796, 1.268632173538208, 0.0], # Kitchen
    "Home" : [0.0, 0.0, 0.0] # Home position
}

class KitchenNone(Node) :
    def __init__(self):
        Node.__init__(self, "kitchen_node") # Node의 init
        self.callback_group = ReentrantCallbackGroup()
        
        """ 변수 """
        self.where_robot = "로봇이 Home에 있습니다."  # 주방 모니터 왼쪽 상단 로봇 location
        self.wait_time = 5                           # 테이블 체류 시간 (GUI만 구현 / 로직 미구현)
        self.traversal_list = []                     # 테이블 순회 리스트 (GUI만 구현 / 로직 미구현)
        
        # 주문 상태 관리 (테이블 6개의 현재 상태 관리)
        self.order_status_list = {1 : INIT, 2 : INIT, 3 : INIT,
                                  4 : INIT, 5 : INIT, 6 : INIT}
        
        # 테이블마다의 타이머 관리 (CANCELED 상태를 3초 뒤에 INIT으로 만들 타이머)
        self.timer_list = {1 : None, 2 : None, 3 : None,
                       4 : None, 5 : None, 6 : None}
        
        # 6개 테이블의 상태 퍼블리쉬를 순회하기 위한 변수
        self.current_index = 0
        
        # 주방 시스템이 관리하는 주문 목록
        self.order_list = {1 : None, 2 : None, 3 : None,
                           4 : None, 5 : None, 6 : None}
        
        self.last_table = "N"                   # 최근 주문 문구 업데이트용
        self.goal_handle = None                 # goal cancel을 위해 goal_handle 저장
        self.init_pose = [0.0, 0.0, 0.0, 1.0]   
        
        # 주문 서비스
        self.order_service = self.create_service(
            OrderDetail,
            'order_detail',
            self.handle_order,
            callback_group=self.callback_group
        )
        
        # 주문 상태 퍼블리셔
        self.status_publisher = self.create_publisher(
            OrderStatus,
            'order_status',
            10,
            callback_group=self.callback_group
        )
        self.timer = self.create_timer(0.25, self.publish_status)
        
        # 로봇 액션 클라이언트   
        self.nav_client = ActionClient(
            self, 
            NavigateToPose, 
            "navigate_to_pose",
            callback_group=self.callback_group
        )
        
        # 초기 위치 설정 서비스 (클라이언트 -> 서버(nav2))
        self.set_initial_pose_service_client = self.create_client(
            SetInitialPose,
            '/set_initial_pose',
            callback_group=self.callback_group
        )
        
        # setting 2D position estimate
        while not self.set_initial_pose_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /set_initial_pose not available, waiting again...')
        self.set_initial_pose(*self.init_pose)
        
    def generate_order_summary(self, n, l, q, p):
        summary = f"[ {n}번 테이블 ]\n\n"

        for menu, quantity in zip(l, q):
            summary += f"{menu}    x{quantity}\n"

        summary += f"\n금액 : {p} 원"

        return summary

    def handle_order(self, request, response):
        """주문 요청 처리"""
        try:
            # 서비스 request로 받은 데이터
            table_number = request.table_id        # 정수
            menu_list = request.menu_items         # 리스트
            menu_quantities = request.quantities   # 리스트
            total_price = request.total_price      # 정수
            
            # Qt 업데이트를 위한 값 추출
            self.last_table = table_number # 상단 문구 업데이트를 위한 테이블 넘버
            
            """ 터미널 로그 확인용 """
            what = ", ".join([f"{item} {quantity}개" for item, quantity in zip(menu_list, menu_quantities)])
            self.get_logger().info(f"{table_number}번 테이블 주문접수 / {what} / 총 금액 {total_price}원")
            
            # 노드에서 관리하는 주문목록(order_list는 딕셔너리로 저장 및 관리)
            # 기존 주문이 있는지 확인하고, 주문 목록 업데이트
            if (table_number in self.order_list) and (self.order_list[table_number] is not None):
            # 기존 주문이 있다면 메뉴와 수량을 업데이트
                existing_order = self.order_list[table_number]
                existing_menu_dict = dict(zip(existing_order['menu_list'], existing_order['menu_quantities']))
            
                # 새로운 주문 추가 및 수량 업데이트
                for item, quantity in zip(menu_list, menu_quantities):
                    if item in existing_menu_dict:
                        existing_menu_dict[item] += quantity
                    else:
                        existing_menu_dict[item] = quantity
                        
                # 총 금액 업데이트
                updated_total_price = existing_order['total_price'] + total_price

                # 업데이트된 주문 정보 저장
                self.order_list[table_number] = {
                    'menu_list': list(existing_menu_dict.keys()),
                    'menu_quantities': list(existing_menu_dict.values()),
                    'total_price': updated_total_price
                }
                
            else:
                # 기존 주문이 없으면 새로운 주문으로 추가
                self.order_list[table_number] = {
                    'menu_list': menu_list,
                    'menu_quantities': menu_quantities,
                    'total_price': total_price
                }
                
            self.change_order_status(table_number, PREPARING)

            # 응답 데이터 설정
            response.success = True
            response.message = "주문이 성공적으로 접수 되었습니다."

        except Exception as e:
            self.get_logger().error(f"Error processing order: {str(e)}") # 에러 로그
            response.success = False
            response.message = "주문에 실패 하였습니다. 관리자에게 문의하세요."
        
        return response
        
    def change_order_status(self, table_id, new_status):
        """주문 상태 변경""" # 테이블 마다 현재 주문 상태를 가짐 (Preparing..., Delivering...)
        self.order_status_list[table_id] = new_status
        
        # 상태가 "Canceled..."일 경우 3초 뒤에 "안녕하세요"로 복원하는 타이머 설정
        if new_status == CANCELED:
            self.timer_list[table_id] = self.create_timer(3.0, lambda: self.reset_order_status(table_id))
        
    def reset_order_status(self, table_id):
        """주문 상태를 "안녕하세요"로 복원"""
        self.order_status_list[table_id] = INIT
        self.timer_list[table_id] = None
        
    def publish_status(self):
        """현재 테이블 상태를 순차적으로 발행"""
        try:
            # 현재 발행할 테이블의 상태를 가져옴
            current_status = self.order_status_list[self.current_index+1]

            # ROS2 메시지 생성
            msg = OrderStatus()
            msg.table_numbers = self.current_index+1
            msg.order_states = current_status

            # 메시지 발행
            self.status_publisher.publish(msg)
            self.get_logger().info(f"Published status : Table {self.current_index+1}, State: {current_status}")

            # 다음 테이블로 이동
            self.current_index = (self.current_index + 1) % len(self.order_status_list)
                
        except Exception as e:
            self.get_logger().error(f"Error publishing status: {str(e)}")

    # 2D position estimation (Kitchen 노드 실행시 호출)
    def set_initial_pose(self, x, y, z, w):
        req = SetInitialPose.Request()
        
        req.pose.header.frame_id = 'map'
        req.pose.pose.pose.position = Point(x=x, y=y, z=0.0)
        req.pose.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=z, w=w)
        req.pose.pose.covariance = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.02]

        future = self.set_initial_pose_service_client.call_async(req)

    def create_goal(self, x, y, theta):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.z = theta  # 단순하게 z를 설정하지만, 필요시 쿼터니언 변환 사용
        return goal

    ###### Action Logic ######
    def send_goal(self, table, x, y, theta):
        self.goal_table = table
        goal_msg = NavigateToPose.Goal()
        
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = theta
        goal_msg.pose.pose.orientation.w = 1.0
        
        if isinstance(table, int) :
            self.where_robot = f"로봇이 {table}번 테이블로 이동 중..."
            self.change_order_status(table, DELIVERING)
        else :
            self.where_robot = f"로봇이 {table}으로 이동 중..."
        
        self.nav_client.wait_for_server() # 서버 연결 대기
        
        self.get_logger().info(f'Goal을 전송하였습니다. 로봇이 이동 중입니다..\n(x={x}, y={y}, theta={theta})')
        
        self.send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_response_callback)
        
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # 피드백은 터미널에서 Log로만 확인 / 실제 GUI에 띄우지는 않음
        self.get_logger().info(f"로봇의 현재 위치 : {feedback.current_pose}")
    
    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        
        if not self.goal_handle.accepted:
            self.get_logger().warn('Goal rejected :(')
            return
        
        self.get_logger().info('Goal accepted :)')
        self.action_result_future = self.goal_handle.get_result_async()
        self.action_result_future.add_done_callback(self.get_result_callback)
        
    def get_result_callback(self, future):
        result = future.result().result

        self.get_logger().debug(f'{self.where_robot}에 도착하였습니다.')
        self.where_robot = "로봇이 목표지점에 도착, 대기 중"
        self.change_order_status(self.goal_table, INIT)

    # 로봇 정지 (로봇 멈춤 / 순회 리스트 초기화 / qt에서도 순회 리스트 초기화)
    def cancel_goal(self):
        if self.goal_handle is not None:
            self.get_logger().info('Cancelling the goal...')
            self.cancel_future = self.goal_handle.cancel_goal_async()
            self.cancel_future.add_done_callback(self.cancel_done_callback)
        else:
            self.get_logger().info('No active goal to cancel.')

    def cancel_done_callback(self, future):
        cancel_result = future.result()
        
        self.traversal_list = [] # 순회 리스트 초기화
        self.where_robot = "로봇이 정지 하였습니다. 로봇 작동을 재개하세요."
        self.get_logger().info('Goal successfully cancelled.')

###############################################################################3
class kitchen_program(object) :
    def __init__(self, node) :
        self.node = node
        
        """ 변수 """
        # 6개의 테이블 (setupUi 에서 업데이트)
        self.tables = []
        # 선택한 테이블
        self.selected_table = -1
        # 순회할 테이블 리스트
        self.traversal_list = []
        
        self.How_to_pay = "카드" # 카드결제가 default
        self.current_goal_index = 0
        
        # QTimer를 사용하여 ROS2 콜백 실행
        self.timer = QTimer()
        self.timer.timeout.connect(self.spin_once)
        self.timer.start(250)  # 250ms마다 ROS2 콜백 처리
        
        
    def spin_once(self):
        """QTimer로 호출되는 ROS2 콜백 처리"""
        rclpy.spin_once(self.node, timeout_sec=0.1)
        self.update_order_status_msg() # 주문 정보 문구 업데이트
        self.update_summary()  # 주문 목록 업데이트
        self.update_robot_position() # 로봇 위치 문구 업데이트
    
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(958, 623)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.cancel_btn = QtWidgets.QPushButton(self.centralwidget)
        self.cancel_btn.setGeometry(QtCore.QRect(420, 490, 251, 91))
        self.cancel_btn.setObjectName("cancel_btn")
        self.orderStatus = QtWidgets.QLabel(self.centralwidget)
        self.orderStatus.setGeometry(QtCore.QRect(430, 50, 491, 41))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.orderStatus.setFont(font)
        self.orderStatus.setAlignment(QtCore.Qt.AlignCenter)
        self.orderStatus.setObjectName("orderStatus")
        self.go_btn = QtWidgets.QPushButton(self.centralwidget)
        self.go_btn.setGeometry(QtCore.QRect(680, 490, 251, 91))
        self.go_btn.setStyleSheet("background-color: rgb(53, 132, 228);")
        self.go_btn.setObjectName("go_btn")
        self.gridLayoutWidget = QtWidgets.QWidget(self.centralwidget)
        self.gridLayoutWidget.setGeometry(QtCore.QRect(420, 100, 511, 381))
        self.gridLayoutWidget.setObjectName("gridLayoutWidget")
        self.gridLayout = QtWidgets.QGridLayout(self.gridLayoutWidget)
        self.gridLayout.setContentsMargins(0, 0, 0, 0)
        self.gridLayout.setObjectName("gridLayout")
        self.table4 = QtWidgets.QPushButton(self.gridLayoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.table4.sizePolicy().hasHeightForWidth())
        self.table4.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setPointSize(10)
        self.table4.setFont(font)
        self.table4.setAutoDefault(False)
        self.table4.setDefault(False)
        self.table4.setFlat(False)
        self.table4.setObjectName("table4")
        self.gridLayout.addWidget(self.table4, 1, 0, 1, 1)
        self.table1 = QtWidgets.QPushButton(self.gridLayoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.table1.sizePolicy().hasHeightForWidth())
        self.table1.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setPointSize(10)
        self.table1.setFont(font)
        self.table1.setAutoDefault(False)
        self.table1.setDefault(False)
        self.table1.setFlat(False)
        self.table1.setObjectName("table1")
        self.gridLayout.addWidget(self.table1, 0, 0, 1, 1)
        self.table5 = QtWidgets.QPushButton(self.gridLayoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.table5.sizePolicy().hasHeightForWidth())
        self.table5.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setPointSize(10)
        self.table5.setFont(font)
        self.table5.setAutoDefault(False)
        self.table5.setDefault(False)
        self.table5.setFlat(False)
        self.table5.setObjectName("table5")
        self.gridLayout.addWidget(self.table5, 1, 1, 1, 1)
        self.table6 = QtWidgets.QPushButton(self.gridLayoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.table6.sizePolicy().hasHeightForWidth())
        self.table6.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setPointSize(10)
        self.table6.setFont(font)
        self.table6.setAutoDefault(False)
        self.table6.setDefault(False)
        self.table6.setFlat(False)
        self.table6.setObjectName("table6")
        self.gridLayout.addWidget(self.table6, 1, 2, 1, 1)
        self.table3 = QtWidgets.QPushButton(self.gridLayoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.table3.sizePolicy().hasHeightForWidth())
        self.table3.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setPointSize(10)
        self.table3.setFont(font)
        self.table3.setAutoDefault(False)
        self.table3.setDefault(False)
        self.table3.setFlat(False)
        self.table3.setObjectName("table3")
        self.gridLayout.addWidget(self.table3, 0, 2, 1, 1)
        self.table2 = QtWidgets.QPushButton(self.gridLayoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.table2.sizePolicy().hasHeightForWidth())
        self.table2.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setPointSize(10)
        self.table2.setFont(font)
        self.table2.setAutoDefault(False)
        self.table2.setDefault(False)
        self.table2.setFlat(False)
        self.table2.setObjectName("table2")
        self.gridLayout.addWidget(self.table2, 0, 1, 1, 1)
        self.horizontalLayoutWidget = QtWidgets.QWidget(self.centralwidget)
        self.horizontalLayoutWidget.setGeometry(QtCore.QRect(40, 470, 361, 109))
        self.horizontalLayoutWidget.setObjectName("horizontalLayoutWidget")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.horizontalLayoutWidget)
        self.horizontalLayout.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.go_to_kitchen_btn = QtWidgets.QPushButton(self.horizontalLayoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.go_to_kitchen_btn.sizePolicy().hasHeightForWidth())
        self.go_to_kitchen_btn.setSizePolicy(sizePolicy)
        self.go_to_kitchen_btn.setObjectName("go_to_kitchen_btn")
        self.horizontalLayout.addWidget(self.go_to_kitchen_btn)
        self.stop_btn = QtWidgets.QPushButton(self.horizontalLayoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.stop_btn.sizePolicy().hasHeightForWidth())
        self.stop_btn.setSizePolicy(sizePolicy)
        self.stop_btn.setStyleSheet("background-color: rgb(237, 51, 59);")
        self.stop_btn.setObjectName("stop_btn")
        self.horizontalLayout.addWidget(self.stop_btn)
        self.go_to_home_btn = QtWidgets.QPushButton(self.horizontalLayoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.go_to_home_btn.sizePolicy().hasHeightForWidth())
        self.go_to_home_btn.setSizePolicy(sizePolicy)
        self.go_to_home_btn.setObjectName("go_to_home_btn")
        self.horizontalLayout.addWidget(self.go_to_home_btn)
        self.robot_location = QtWidgets.QLabel(self.centralwidget)
        self.robot_location.setGeometry(QtCore.QRect(40, 30, 311, 31))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.robot_location.setFont(font)
        self.robot_location.setObjectName("robot_location")
        self.wait_time_btn = QtWidgets.QPushButton(self.centralwidget)
        self.wait_time_btn.setGeometry(QtCore.QRect(280, 110, 121, 31))
        self.wait_time_btn.setObjectName("wait_time_btn")
        self.wait_time_txt = QtWidgets.QLabel(self.centralwidget)
        self.wait_time_txt.setGeometry(QtCore.QRect(40, 80, 400, 31))
        self.wait_time_txt.setObjectName("wait_time_txt")
        self.serving_list_txt = QtWidgets.QLabel(self.centralwidget)
        self.serving_list_txt.setGeometry(QtCore.QRect(40, 140, 110, 31))
        self.serving_list_txt.setObjectName("serving_list_txt")
        self.wait_time = QtWidgets.QLineEdit(self.centralwidget)
        self.wait_time.setGeometry(QtCore.QRect(40, 110, 231, 31))
        self.wait_time.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.wait_time.setAlignment(QtCore.Qt.AlignCenter)
        self.wait_time.setObjectName("wait_time")
        self.serving_list_btn = QtWidgets.QPushButton(self.centralwidget)
        self.serving_list_btn.setGeometry(QtCore.QRect(280, 170, 121, 61))
        self.serving_list_btn.setObjectName("serving_list_btn")
        self.serving_list = QtWidgets.QListWidget(self.centralwidget)
        self.serving_list.setGeometry(QtCore.QRect(40, 170, 231, 61))
        self.serving_list.setObjectName("serving_list")
        self.verticalLayoutWidget = QtWidgets.QWidget(self.centralwidget)
        self.verticalLayoutWidget.setGeometry(QtCore.QRect(250, 250, 151, 201))
        self.verticalLayoutWidget.setObjectName("verticalLayoutWidget")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.verticalLayoutWidget)
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout.setObjectName("verticalLayout")
        self.check_cash_btn = QtWidgets.QPushButton(self.verticalLayoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.check_cash_btn.sizePolicy().hasHeightForWidth())
        self.check_cash_btn.setSizePolicy(sizePolicy)
        self.check_cash_btn.setObjectName("check_cash_btn")
        self.verticalLayout.addWidget(self.check_cash_btn)
        self.check_card_btn = QtWidgets.QPushButton(self.verticalLayoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.check_card_btn.sizePolicy().hasHeightForWidth())
        self.check_card_btn.setSizePolicy(sizePolicy)
        self.check_card_btn.setObjectName("check_card_btn")
        self.check_card_btn.setStyleSheet("background-color: lightgray;")
        self.verticalLayout.addWidget(self.check_card_btn)
        self.check_pay_btn = QtWidgets.QPushButton(self.verticalLayoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.check_pay_btn.sizePolicy().hasHeightForWidth())
        self.check_pay_btn.setSizePolicy(sizePolicy)
        self.check_pay_btn.setAutoFillBackground(False)
        self.check_pay_btn.setStyleSheet("background-color: rgb(87, 227, 137);")
        self.check_pay_btn.setObjectName("check_pay_btn")
        self.verticalLayout.addWidget(self.check_pay_btn)
        self.detail = QtWidgets.QLabel(self.centralwidget)
        self.detail.setGeometry(QtCore.QRect(40, 250, 201, 201))
        self.detail.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.detail.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.detail.setText("")
        self.detail.setAlignment(QtCore.Qt.AlignCenter)
        self.detail.setObjectName("detail")
        
        MainWindow.setCentralWidget(self.centralwidget)
        
        self.tables.append(self.table1)
        self.tables.append(self.table2)
        self.tables.append(self.table3)
        self.tables.append(self.table4)
        self.tables.append(self.table5)
        self.tables.append(self.table6)
        
        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

        """ 이벤트 연결 """
        self.table1.clicked.connect(lambda checked, idx=1: self.table_click(idx))
        self.table2.clicked.connect(lambda checked, idx=2: self.table_click(idx))
        self.table3.clicked.connect(lambda checked, idx=3: self.table_click(idx))
        self.table4.clicked.connect(lambda checked, idx=4: self.table_click(idx))
        self.table5.clicked.connect(lambda checked, idx=5: self.table_click(idx))
        self.table6.clicked.connect(lambda checked, idx=6: self.table_click(idx))
        
        self.cancel_btn.clicked.connect(self.remove_selected_item)
        self.go_btn.clicked.connect(self.robot_move)
        
        self.wait_time_btn.clicked.connect(self.set_wait_time)
        self.serving_list_btn.clicked.connect(self.add_traversal)
        
        self.go_to_kitchen_btn.clicked.connect(self.go_to_kitchen)
        self.stop_btn.clicked.connect(self.stop)
        self.go_to_home_btn.clicked.connect(self.go_to_home)
        
        self.check_cash_btn.clicked.connect(self.cash)
        self.check_card_btn.clicked.connect(self.card)
        self.check_pay_btn.clicked.connect(self.pay)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.cancel_btn.setText(_translate("MainWindow", "주문취소"))
        self.orderStatus.setText(_translate("MainWindow", "N번 테이블에서 음식을 주문 하였습니다."))
        self.go_btn.setText(_translate("MainWindow", "Go"))
        self.table4.setText(_translate("MainWindow", "[ 4번 테이블 ]"))
        self.table1.setText(_translate("MainWindow", "[ 1번 테이블 ]"))
        self.table5.setText(_translate("MainWindow", "[ 5번 테이블 ]"))
        self.table6.setText(_translate("MainWindow", "[ 6번 테이블 ]"))
        self.table3.setText(_translate("MainWindow", "[ 3번 테이블 ]"))
        self.table2.setText(_translate("MainWindow", "[ 2번 테이블 ]"))
        self.go_to_kitchen_btn.setText(_translate("MainWindow", "Kitchen"))
        self.stop_btn.setText(_translate("MainWindow", "Stop"))
        self.go_to_home_btn.setText(_translate("MainWindow", "Home"))
        self.robot_location.setText(_translate("MainWindow", "로봇이 Home에 있습니다."))
        self.wait_time_btn.setText(_translate("MainWindow", "입력"))
        self.wait_time_txt.setText(_translate("MainWindow", "테이블 대기시간을 설정하세요 (기본값 10초)"))
        self.serving_list_txt.setText(_translate("MainWindow", "서빙 대기 테이블"))
        self.serving_list_btn.setText(_translate("MainWindow", "서빙 리스트 추가"))
        self.check_cash_btn.setText(_translate("MainWindow", "현금"))
        self.check_card_btn.setText(_translate("MainWindow", "카드"))
        self.check_pay_btn.setText(_translate("MainWindow", "결제"))

    def is_valid_table(self) :
        return ((self.selected_table != -1) and
                (self.node.order_list[self.selected_table] != None))

    def is_not_delivering(self) :
        return not((self.selected_table in self.node.traversal_list) or 
                   (self.node.order_status_list[self.selected_table] == DELIVERING))

    def table_click(self, idx):
        if idx == -1 :
            for btn in self.tables :
                btn.setStyleSheet("")
                
        else :
            if self.selected_table != -1 :
                # 이전에 선택된 버튼의 색상을 원래대로 돌림
                self.tables[self.selected_table-1].setStyleSheet("")
                
            # 새로운 버튼의 인덱스를 selected_table에 저장
            self.selected_table = idx

            # 선택된 버튼의 배경색 변경
            self.tables[self.selected_table-1].setStyleSheet("background-color: lightblue;")
            
            # 결제 창에 표시
            if self.node.order_list[self.selected_table] : 
                text = self.tables[self.selected_table-1].text()
                self.detail.setText(text)
                
            else :
                self.detail.setText("")
                
    def update_order_status_msg(self):
        """주문 상태 문구 업데이트"""
        table_number = self.node.last_table
        self.orderStatus.setText(f"방금, {table_number}번 테이블에서 음식을 주문 하였습니다.")        

    def update_summary(self):
        """주문 목록 업데이트"""
        
        for table, summary in self.node.order_list.items() :
            if summary == None :
                continue
    
            # 실제 QT 버튼(테이블 버튼)에 값을 넣어 주기 위해 문자열 포맷으로 변경
            summary = self.node.generate_order_summary(table, 
                                                       self.node.order_list[table]["menu_list"], 
                                                       self.node.order_list[table]["menu_quantities"], 
                                                       self.node.order_list[table]["total_price"])
            
            self.tables[table-1].setText(summary)
         
    def update_travesal_view(self) :
        # QListWidget 초기화
        self.traversal_list.clear()
        
        # 리스트의 값을 QListWidget에 추가
        for item in self.node.traversal_list :
            self.traversal_list.addItem(item)
         
    def update_robot_position(self) :
        self.robot_location.setText(self.node.where_robot)

    def remove_selected_item(self):
        """선택된 주문 항목 삭제"""
        if ((self.is_valid_table()) and (self.is_not_delivering())):
            self.tables[self.selected_table-1].setText(f"[ {self.selected_table}번 테이블 ]")
            self.node.order_list[self.selected_table] = None
        
            self.node.get_logger().info(f"Removed order: {self.selected_table}번 테이블 주문 취소")

            log_message = ""
            for table_number, order_info in self.node.order_list.items():
                if order_info is None:
                    log_message += f"{table_number}번 테이블 : 주문 없음\n"
                else:
                    log_message += f"{table_number}번 테이블 : {order_info}\n"

            self.node.get_logger().info(f"\n현재 주문 정보 : \n{log_message}")
                
            self.node.change_order_status(self.selected_table, CANCELED)
            
            QMessageBox.information(None, 'Info', f'{self.selected_table}번 테이블의 주문이 취소되었습니다.')
            
            self.detail.setText("")
            self.selected_table = -1
            self.table_click(self.selected_table)
                
    def set_wait_time(self) :
        if self.wait_time.text().isdigit() :
            wait_time = int(self.wait_time.text())

            if (5 <= wait_time <= 30) : 
                self.node.wait_time = wait_time
                self.wait_time.setText("")
                self.wait_time_txt.setStyleSheet("color: black;")
                self.wait_time_txt.setText(f"현재 설정된 테이블 대기시간은 {wait_time}초 입니다.")
            elif (wait_time < 5) :
                self.wait_time.setText("")
                self.wait_time_txt.setStyleSheet("color: red;")
                self.wait_time_txt.setText(f"최소 대기 시간은 5초 입니다.")
            else :
                self.wait_time.setText("")
                self.wait_time_txt.setStyleSheet("color: red;")
                self.wait_time_txt.setText(f"최대 대기 시간은 30초 입니다.")
        
        else :
            self.wait_time.setText("")
            self.wait_time_txt.setStyleSheet("color: red;")
            self.wait_time_txt.setText(f"정수를 입력하세요 (최소 : 5초 / 최대 : 30초)")
            
    def add_traversal(self) :
        if (self.selected_table != -1) and (self.node.order_list[self.selected_table]) :
            if self.selected_table not in self.node.traversal_list:
                self.serving_list.addItem(str(self.selected_table))  # QListWidget에 항목 추가
                self.node.traversal_list.append(self.selected_table)
        
        print(self.node.traversal_list)
        
    def go_to_kitchen(self) :
        self.node.send_goal("Kitchen", 
                            coordinates["Kitchen"][0], 
                            coordinates["Kitchen"][1], 
                            coordinates["Kitchen"][2])
    
    def go_to_home(self) :
        self.node.send_goal("Home", 
                            coordinates["Home"][0], 
                            coordinates["Home"][1], 
                            coordinates["Home"][2])

    def robot_move(self) :
        # 테이블 순회 리스트가 있을 때
        if self.node.traversal_list :
            pass

        else :
            # 그냥 테이블 하나만 보낼 때
            if self.is_valid_table() :
                    self.node.send_goal(self.selected_table, #
                                        coordinates[self.selected_table][0], 
                                        coordinates[self.selected_table][1], 
                                        coordinates[self.selected_table][2])

    def stop(self) :
        self.node.cancel_goal()
        self.traversal_list.clear()
        
    def cash(self) :
        self.How_to_pay = "현금"
        
        if self.How_to_pay == "현금" :
            self.check_cash_btn.setStyleSheet("background-color: lightgray;")
            self.check_card_btn.setStyleSheet("background-color: None;")
            
    def card(self) :
        self.How_to_pay = "카드"
        
        if self.How_to_pay == "카드" :
            self.check_card_btn.setStyleSheet("background-color: lightgray;")
            self.check_cash_btn.setStyleSheet("background-color: None;")
        
    def pay(self) :
        if ((self.is_valid_table()) and (self.is_not_delivering())):
            reply = QMessageBox.question(
                None, 
                '결제 진행', 
                f"'{self.How_to_pay}' 결제를 진행합니다.", 
                QMessageBox.Yes | QMessageBox.No, 
                QMessageBox.Yes
            )
            
            if reply == QMessageBox.Yes:
                # 실제 테이블 주문 목록 초기화
                self.tables[self.selected_table-1].setText(f"[ {self.selected_table}번 테이블 ]")
                self.node.order_list[self.selected_table] = None
                self.node.change_order_status(self.selected_table, INIT)
                
                # DB에 저장 (주말에 추가)

                # UI 초기화
                self.detail.setText("")
                self.selected_table = -1
                self.table_click(self.selected_table)
                
                # 로그
                self.node.get_logger().info(f"Removed order: {self.selected_table}번 테이블 결제 완료")

                log_message = ""
                for table_number, order_info in self.node.order_list.items():
                    if order_info is None:
                        log_message += f"{table_number}번 테이블 : 주문 없음\n"
                    else:
                        log_message += f"{table_number}번 테이블 : {order_info}\n"

                self.node.get_logger().info(f"\n현재 주문 정보 : \n{log_message}")
                
                QMessageBox.information(None, 'Info', '결제가 승인되었습니다.')
            else:
                QMessageBox.information(None, 'Info', '결제가 취소되었습니다.')
            

######################################################################
def main(args=None) :
    rclpy.init(args=args)
    kitchen_node = KitchenNone()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(kitchen_node)
    
    try :
        app = QApplication(sys.argv)
        MainWindow = QMainWindow()
        
        ui = kitchen_program(kitchen_node)
        ui.setupUi(MainWindow)
        MainWindow.show()
        
        sys.exit(app.exec_())
        
        executor.spin()

    except KeyboardInterrupt:
        kitchen_node.get_logger().info('Keyboard Interrupt (SIGINT)')
        
    finally:
        executor.shutdown()
        kitchen_node.destroy_node() # 노드 종료
        rclpy.shutdown() # rclpy 종료


######################################################################
if __name__=="__main__":
    main()
