#!/usr/bin/env python3
import sys
from PyQt5 import QtCore, QtWidgets
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtCore import QTimer
import rclpy
from rclpy.node import Node
from system_interfaces.srv import OrderDetail
from system_interfaces.msg import OrderStatus
from rclpy.callback_groups import ReentrantCallbackGroup


class KitchenUI(QMainWindow):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.setupUi()

        # QTimer를 사용하여 ROS2 콜백 실행
        self.timer = QTimer()
        self.timer.timeout.connect(self.spin_once)
        self.timer.start(100)  # 100ms마다 ROS2 콜백 처리

    def setupUi(self):
        self.setObjectName("MainWindow")
        self.resize(958, 623)
        self.centralwidget = QtWidgets.QWidget(self)
        self.centralwidget.setObjectName("centralwidget")

        # 카메라 이미지
        self.cameraImage = QtWidgets.QLabel(self.centralwidget)
        self.cameraImage.setGeometry(QtCore.QRect(30, 100, 371, 371))
        self.cameraImage.setFrameShape(QtWidgets.QFrame.Box)
        self.cameraImage.setAlignment(QtCore.Qt.AlignCenter)
        self.cameraImage.setText("Robot Cam here.")

        # 주문 목록
        self.orderList = QtWidgets.QListWidget(self.centralwidget)
        self.orderList.setGeometry(QtCore.QRect(430, 100, 491, 381))

        # 버튼들
        self.cancel_btn = QtWidgets.QPushButton("주문취소", self.centralwidget)
        self.cancel_btn.setGeometry(QtCore.QRect(840, 50, 81, 41))
        self.cancel_btn.clicked.connect(self.handle_order_cancel)

        self.delivery_btn = QtWidgets.QPushButton("Delivery", self.centralwidget)
        self.delivery_btn.setGeometry(QtCore.QRect(530, 490, 291, 101))
        self.delivery_btn.clicked.connect(self.handle_delivery)

        # 카메라 제어 버튼
        self.camera_btn = QtWidgets.QPushButton("Camera On", self.centralwidget)
        self.camera_btn.setGeometry(QtCore.QRect(30, 490, 171, 71))

        self.comeBackHome = QtWidgets.QPushButton("Return to Base", self.centralwidget)
        self.comeBackHome.setGeometry(QtCore.QRect(230, 490, 171, 71))

        # 상태 라벨
        self.status_label = QtWidgets.QLabel(self.centralwidget)
        self.status_label.setGeometry(QtCore.QRect(430, 50, 311, 41))
        self.status_label.setText("주문 대기중")

        self.setCentralWidget(self.centralwidget)
        self.setWindowTitle("Kitchen Management System")

    def spin_once(self):
        """QTimer로 호출되는 ROS2 콜백 처리"""
        rclpy.spin_once(self.ros_node, timeout_sec=0)
        self.update_order_list()  # 주문 목록 업데이트

    def handle_order_cancel(self):
        selected_items = self.orderList.selectedItems()
        if selected_items:
            item_text = selected_items[0].text()
            try:
                table_id = int(item_text.split()[1])
                for status in self.ros_node.order_status_list[:]:
                    if f"Table {table_id}" in status:
                        self.ros_node.order_status_list.remove(status)
                self.status_label.setText(f"테이블 {table_id} 주문 취소됨")
            except (IndexError, ValueError):
                self.status_label.setText("취소할 주문을 선택하세요.")
        else:
            self.status_label.setText("취소할 주문을 선택하세요.")

    def handle_delivery(self):
        selected_items = self.orderList.selectedItems()
        if selected_items:
            item_text = selected_items[0].text()
            try:
                table_id = int(item_text.split()[1])
                self.ros_node.change_order_status(table_id, "DELIVERING")
                self.status_label.setText(f"테이블 {table_id} 배달 시작")
            except (IndexError, ValueError):
                self.status_label.setText("배달할 주문을 선택하세요.")
        else:
            self.status_label.setText("배달할 주문을 선택하세요.")

    def update_order_list(self):
        """주문 목록 업데이트"""
        current_items = set(self.orderList.item(i).text() for i in range(self.orderList.count()))
        new_items = set(self.ros_node.order_status_list)

        # 새로운 항목 추가
        for item in new_items - current_items:
            self.orderList.addItem(item)

        # 제거된 항목 삭제
        for i in range(self.orderList.count() - 1, -1, -1):
            if self.orderList.item(i).text() not in new_items:
                self.orderList.takeItem(i)


class KitchenNode(Node):
    def __init__(self):
        super().__init__('kitchen_node')

        # 콜백 그룹 설정
        self.callback_group = ReentrantCallbackGroup()

        # 주문 접수 서비스
        self.order_service = self.create_service(
            OrderDetail,
            'order_detail',
            self.handle_order,
            callback_group=self.callback_group
        )

        # 주문 상태 퍼블리셔
        self.status_publisher = self.create_publisher(OrderStatus, 'order_status', 10)

        # 주문 상태 관리
        self.order_status_list = []
        self.current_index = 0  # 순차적 발행을 위한 인덱스

        # 주문 상태 발행 타이머
        self.create_timer(1.0, self.publish_status)
        self.get_logger().info("Kitchen node has been started.")

    def handle_order(self, request, response):
        """주문 요청 처리"""
        try:
            self.get_logger().info(f"Order received: table_id={request.table_id}, menu_items={request.menu_items}, total_price={request.total_price}")

            # 주문 상태 바로 추가
            self.change_order_status(request.table_id, "COOKING")

            # 응답 데이터 설정
            response.success = True
            response.message = f"{request.table_id}의 주문이 성공적으로 접수 되었습니다."
            self.get_logger().info(f"Order response sent: {response.message}")
            return response

        except Exception as e:
            self.get_logger().error(f"Error processing order: {str(e)}")
            response.success = False
            response.message = "Error processing order"
            return response

    def change_order_status(self, table_id, new_status):
        """주문 상태 변경"""
        for status in self.order_status_list[:]:
            if f"Table {table_id}" in status:
                self.order_status_list.remove(status)
        new_status_str = f"Table {table_id} {new_status}"
        self.order_status_list.append(new_status_str)

    def publish_status(self):
        """현재 테이블 상태를 순차적으로 발행"""
        try:
            # 발행할 상태가 있는지 확인
            if self.order_status_list:
                # 현재 발행할 테이블의 상태를 가져옴
                current_status = self.order_status_list[self.current_index]

                # 상태 문자열에서 테이블 번호와 상태 추출
                table_number = int(current_status.split()[1])  # "Table 1 COOKING" -> 1
                order_state = current_status

                # ROS2 메시지 생성
                msg = OrderStatus()
                msg.table_numbers = table_number
                msg.order_states = order_state

                # 메시지 발행
                self.status_publisher.publish(msg)
                self.get_logger().info(f"Published status: Table {table_number}, State: {order_state}")

                # 다음 테이블로 이동
                self.current_index = (self.current_index + 1) % len(self.order_status_list)
            else:
                self.get_logger().info("No orders to publish.")
        except Exception as e:
            self.get_logger().error(f"Error publishing status: {str(e)}")


def main(args=None):
    rclpy.init(args=args)

    # ROS2 노드 생성
    node = KitchenNode()

    # PyQt5 애플리케이션 생성
    app = QApplication(sys.argv)
    window = KitchenUI(node)
    node.ui = window  # 노드에서 UI 접근 가능하도록 연결
    window.show()

    try:
        app.exec_()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
