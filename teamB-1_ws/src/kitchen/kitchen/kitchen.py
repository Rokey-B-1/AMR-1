import sys
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QListWidgetItem
from PyQt5.QtGui import QPixmap, QBrush, QColor
from PySide2.QtCore import *

import rclpy
from rclpy.node import Node
from system_interfaces.msg import OrderStatus
from system_interfaces.srv import OrderDetail
# from std_msgs.msg import String
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

INIT = "안녕하세요"
PREPARING = "Preparing..."
CANCELED = "Canceled..."
DELIVERING = "Delivering..."

class KitchenNone(Node) :
    def __init__(self):
        Node.__init__(self, "kitchen_node") # Node의 init
        self.callback_group = ReentrantCallbackGroup()
        
        """ 변수 """
        # 주문 상태 관리 (테이블 6개)
        self.order_status_list = {1 : INIT, 2 : INIT, 3 : INIT,
                                  4 : INIT, 5 : INIT, 6 : INIT}
        # 6개 테이블의 상태 퍼블리쉬를 순회하기 위한 변수
        self.current_index = 0
        # 주방 시스템이 관리하는 주문 목록
        self.order_list = {1 : None, 2 : None, 3 : None,
                           4 : None, 6 : None, 7 : None}
        
        self.last_table = "N"
        
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
            self.create_timer(3.0, lambda: self.reset_order_status(table_id))
        
    def reset_order_status(self, table_id):
        """주문 상태를 "안녕하세요"로 복원"""
        self.order_status_list[table_id] = INIT
        
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


###############################################################################3
class kitchen_program(object) :
    def __init__(self, node) :
        self.node = node
        
        """ 변수 """
        # 6개의 테이블 (setupUi 에서 업데이트)
        self.tables = []
        # 선택한 테이블
        self.selected_table = -1
        
        # QTimer를 사용하여 ROS2 콜백 실행
        self.timer = QTimer()
        self.timer.timeout.connect(self.spin_once)
        self.timer.start(200)  # 200ms마다 ROS2 콜백 처리
        
    def spin_once(self):
        """QTimer로 호출되는 ROS2 콜백 처리"""
        rclpy.spin_once(self.node, timeout_sec=0.1)
        self.update_order_status_msg() # 주문 정보 문구 업데이트
        self.update_summary()  # 주문 목록 업데이트
    
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(958, 623)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.cameraImage = QtWidgets.QLabel(self.centralwidget)
        self.cameraImage.setGeometry(QtCore.QRect(30, 100, 371, 371))
        self.cameraImage.setFrameShape(QtWidgets.QFrame.Box)
        self.cameraImage.setAlignment(QtCore.Qt.AlignCenter)
        self.cameraImage.setObjectName("cameraImage")
        self.camera_btn = QtWidgets.QPushButton(self.centralwidget)
        self.camera_btn.setGeometry(QtCore.QRect(50, 480, 141, 61))
        self.camera_btn.setObjectName("camera_btn")
        self.cancel_btn = QtWidgets.QPushButton(self.centralwidget)
        self.cancel_btn.setGeometry(QtCore.QRect(430, 490, 241, 91))
        self.cancel_btn.setObjectName("cancel_btn")
        self.orderStatus = QtWidgets.QLabel(self.centralwidget)
        self.orderStatus.setGeometry(QtCore.QRect(430, 50, 491, 41))
        self.orderStatus.setAlignment(QtCore.Qt.AlignCenter)
        self.orderStatus.setObjectName("orderStatus")
        self.go_btn = QtWidgets.QPushButton(self.centralwidget)
        self.go_btn.setGeometry(QtCore.QRect(680, 490, 241, 91))
        self.go_btn.setObjectName("go_btn")
        self.return_btn = QtWidgets.QPushButton(self.centralwidget)
        self.return_btn.setGeometry(QtCore.QRect(240, 480, 141, 61))
        self.return_btn.setObjectName("return_btn")
        self.gridLayoutWidget = QtWidgets.QWidget(self.centralwidget)
        self.gridLayoutWidget.setGeometry(QtCore.QRect(420, 100, 511, 381))
        self.gridLayoutWidget.setObjectName("gridLayoutWidget")
        self.gridLayout = QtWidgets.QGridLayout(self.gridLayoutWidget)
        self.gridLayout.setContentsMargins(0, 0, 0, 0)
        self.gridLayout.setObjectName("gridLayout")
        
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
        self.cancel_btn.clicked.connect(self.remove_selected_item)
        self.table1.clicked.connect(lambda checked, idx=1: self.table_click(idx))
        self.table2.clicked.connect(lambda checked, idx=2: self.table_click(idx))
        self.table3.clicked.connect(lambda checked, idx=3: self.table_click(idx))
        self.table4.clicked.connect(lambda checked, idx=4: self.table_click(idx))
        self.table5.clicked.connect(lambda checked, idx=5: self.table_click(idx))
        self.table6.clicked.connect(lambda checked, idx=6: self.table_click(idx))
        
        """ 초기화"""

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.cameraImage.setText(_translate("MainWindow", "Robot Cam here."))
        self.camera_btn.setText(_translate("MainWindow", "Camera On"))
        self.cancel_btn.setText(_translate("MainWindow", "주문취소"))
        self.orderStatus.setText(_translate("MainWindow", "N번 테이블에서 음식을 주문 하였습니다."))
        self.go_btn.setText(_translate("MainWindow", "Go"))
        self.return_btn.setText(_translate("MainWindow", "Return to Home"))
        self.table6.setText(_translate("MainWindow", "[ 6번 테이블 ]"))
        self.table1.setText(_translate("MainWindow", "[ 1번 테이블 ]"))
        self.table5.setText(_translate("MainWindow", "[ 5번 테이블 ]"))
        self.table3.setText(_translate("MainWindow", "[ 3번 테이블 ]"))
        self.table4.setText(_translate("MainWindow", "[ 4번 테이블 ]"))
        self.table2.setText(_translate("MainWindow", "[ 2번 테이블 ]"))

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
            self.tables[table-1].setText(summary)
            
    def remove_selected_item(self):
        """선택된 주문 항목 삭제"""
        if (self.selected_table != -1) and (self.node.order_list[self.selected_table] != None) :
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
            
            self.selected_table = -1
            self.table_click(self.selected_table)

######################################################################
def main(args=None) :
    rclpy.init(args=args)
    kitchen_node = KitchenNone()
    executor = MultiThreadedExecutor(num_threads=3)
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
        executor.shutdown() # 스레드 종료
        kitchen_node.destroy_node() # 노드 종료
        rclpy.shutdown() # rclpy 종료


######################################################################
if __name__=="__main__":
    main()
