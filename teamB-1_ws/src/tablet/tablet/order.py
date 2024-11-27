import sys
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel
from PyQt5.QtGui import QPixmap, QFont
from PySide2.QtCore import *

import rclpy
from rclpy.node import Node
from std_msgs.msg import String # 추후에 인터페이스 임포트 변경
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

class TableNode(QThread, Node):
    message_received = Signal(str)

    def __init__(self, table_number):
        QThread.__init__(self) # QThread의 init
        Node.__init__(self, "Table_Node") # Node의 init
        self.callback_group = ReentrantCallbackGroup()
        
        self.table_number = table_number
        self.order_status = ""

        # 주문 상태를 서브스크라이브
        self.subscription = self.create_subscription(
            String, 
            'order_status', 
            self.subscription_callback, 
            10, 
            callback_group=self.callback_group
        ) # 추후 인터페이스 타입 변경
        
        # 주문을 보내는 서비스
        self.order_client = self.create_client(
            String, # 추후 인터페이스 타입 변경
            'order_detail',
            callback_group=self.callback_group
        )
        
        while not self.order_client.wait_for_service(timeout_sec=0.1) :
            self.get_logger.warning("The order service (server -> kitchen) not available.")

    def subscription_callback(self, msg):
        num = msg.table_number
        status = msg.status
        
        if num == self.table_number :
            # self.get_logger().info(f'Received message: {status}')
            self.message_received.emit(status)
            
    def send_request(self) :
        pass 

    def run(self):
        rclpy.spin(self)


class order_program(object):
    def __init__(self, table_number, node) :
        self.node = node
        # self.node.message_received.connect(self.add_message)
        self.number = table_number # 주방 시스템에 서비스로 보낼 것
        
        """ 변수 """
        self.selected_menu = [] # 주방 시스템에 서비스로 보낼 것 (주문한 메뉴)
        self.current_index = 0
        self.total_price = 0 # 주방 시스템에 서비스로 보낼 것 (주문 총 가격)
        self.seleted_dict = {}

        
    def set_menu(self) :
        """ 로직 """
        # 메뉴 리스트 설정
        self.menu_list = []
        menus_path = "src/tablet/menus/menus.txt"
        
        with open(menus_path, "r") as f:
            menus = f.readlines()
    
            for menu in menus:
                menu = menu.strip("\n").split("/")
                self.menu_list.append(tuple(menu))
                
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(950, 650)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.horizontalLayoutWidget = QtWidgets.QWidget(self.centralwidget)
        self.horizontalLayoutWidget.setGeometry(QtCore.QRect(40, 20, 861, 241))
        self.horizontalLayoutWidget.setObjectName("horizontalLayoutWidget")
        self.menu_layout = QtWidgets.QHBoxLayout(self.horizontalLayoutWidget)
        self.menu_layout.setContentsMargins(0, 0, 0, 0)
        self.menu_layout.setObjectName("menu_layout")
        self.left_image = QtWidgets.QLabel(self.horizontalLayoutWidget)
        self.left_image.setAlignment(QtCore.Qt.AlignCenter)
        self.left_image.setObjectName("left_image")
        self.menu_layout.addWidget(self.left_image)
        self.center_image = QtWidgets.QLabel(self.horizontalLayoutWidget)
        self.center_image.setAlignment(QtCore.Qt.AlignCenter)
        self.center_image.setObjectName("center_image")
        self.menu_layout.addWidget(self.center_image)
        self.right_image = QtWidgets.QLabel(self.horizontalLayoutWidget)
        self.right_image.setAlignment(QtCore.Qt.AlignCenter)
        self.right_image.setObjectName("right_image")
        self.menu_layout.addWidget(self.right_image)
        self.horizontalLayoutWidget_2 = QtWidgets.QWidget(self.centralwidget)
        self.horizontalLayoutWidget_2.setGeometry(QtCore.QRect(40, 320, 861, 91))
        self.horizontalLayoutWidget_2.setObjectName("horizontalLayoutWidget_2")
        self.btn_layout = QtWidgets.QHBoxLayout(self.horizontalLayoutWidget_2)
        self.btn_layout.setContentsMargins(0, 0, 0, 0)
        self.btn_layout.setObjectName("btn_layout")
        self.previous_btn = QtWidgets.QPushButton(self.horizontalLayoutWidget_2)
        self.previous_btn.setObjectName("previous_btn")
        self.btn_layout.addWidget(self.previous_btn)
        self.select_btn = QtWidgets.QPushButton(self.horizontalLayoutWidget_2)
        self.select_btn.setObjectName("select_btn")
        self.btn_layout.addWidget(self.select_btn)
        self.next_btn = QtWidgets.QPushButton(self.horizontalLayoutWidget_2)
        self.next_btn.setObjectName("next_btn")
        self.btn_layout.addWidget(self.next_btn)
        self.horizontalLayoutWidget_3 = QtWidgets.QWidget(self.centralwidget)
        self.horizontalLayoutWidget_3.setGeometry(QtCore.QRect(40, 430, 861, 191))
        self.horizontalLayoutWidget_3.setObjectName("horizontalLayoutWidget_3")
        self.horizontalLayout_4 = QtWidgets.QHBoxLayout(self.horizontalLayoutWidget_3)
        self.horizontalLayout_4.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        self.seleted_view = QtWidgets.QTextBrowser(self.horizontalLayoutWidget_3)
        self.seleted_view.setObjectName("seleted_view")
        self.horizontalLayout_4.addWidget(self.seleted_view)
        self.verticalLayout = QtWidgets.QVBoxLayout()
        self.verticalLayout.setObjectName("verticalLayout")
        self.table_number = QtWidgets.QLabel(self.horizontalLayoutWidget_3)
        self.table_number.setAlignment(QtCore.Qt.AlignCenter)
        self.table_number.setObjectName("table_number")
        self.verticalLayout.addWidget(self.table_number)
        self.status_msg = QtWidgets.QLabel(self.horizontalLayoutWidget_3)
        self.status_msg.setAlignment(QtCore.Qt.AlignCenter)
        self.status_msg.setObjectName("status_msg")
        self.verticalLayout.addWidget(self.status_msg)
        self.clear_btn = QtWidgets.QPushButton(self.horizontalLayoutWidget_3)
        self.clear_btn.setObjectName("clear_btn")
        self.verticalLayout.addWidget(self.clear_btn)
        self.order_btn_2 = QtWidgets.QPushButton(self.horizontalLayoutWidget_3)
        self.order_btn_2.setObjectName("order_btn_2")
        self.verticalLayout.addWidget(self.order_btn_2)
        self.horizontalLayout_4.addLayout(self.verticalLayout)
        self.horizontalLayoutWidget_4 = QtWidgets.QWidget(self.centralwidget)
        self.horizontalLayoutWidget_4.setGeometry(QtCore.QRect(310, 270, 331, 41))
        self.horizontalLayoutWidget_4.setObjectName("horizontalLayoutWidget_4")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.horizontalLayoutWidget_4)
        self.horizontalLayout.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.menu_label = QtWidgets.QLabel(self.horizontalLayoutWidget_4)
        self.menu_label.setAlignment(QtCore.Qt.AlignCenter)
        self.menu_label.setObjectName("menu_label")
        self.horizontalLayout.addWidget(self.menu_label)
        self.price_label = QtWidgets.QLabel(self.horizontalLayoutWidget_4)
        self.price_label.setAlignment(QtCore.Qt.AlignCenter)
        self.price_label.setObjectName("price_label")
        self.horizontalLayout.addWidget(self.price_label)
        MainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)
        
        """ 초기화 """
        # 메뉴 세팅
        self.set_menu()
        # 초기 메뉴 표시
        self.update_menu()
        
        """ 이벤트 연결 """
        self.previous_btn.clicked.connect(self.show_previous)
        self.next_btn.clicked.connect(self.show_next)
        self.select_btn.clicked.connect(self.select_menu)
        
    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.left_image.setText(_translate("MainWindow", "left_image"))
        self.center_image.setText(_translate("MainWindow", "center_image"))
        self.right_image.setText(_translate("MainWindow", "right_image"))
        self.previous_btn.setText(_translate("MainWindow", "이전"))
        self.select_btn.setText(_translate("MainWindow", "선택"))
        self.next_btn.setText(_translate("MainWindow", "다음"))
        self.table_number.setText(_translate("MainWindow", f"{self.number}번 테이블"))
        self.status_msg.setText(_translate("MainWindow", "메뉴를 고른 후 \'주문\' 버튼을 눌러주세요"))
        self.clear_btn.setText(_translate("MainWindow", "초기화"))
        self.order_btn_2.setText(_translate("MainWindow", "주문"))
        self.menu_label.setText(_translate("MainWindow", "메뉴명"))
        self.price_label.setText(_translate("MainWindow", "가격"))
        
    def update_menu(self):
        # 왼쪽 메뉴 설정
        if self.current_index > 0:
            pixmap_left = QPixmap(f"src/tablet/menus/menu_images/{self.menu_list[self.current_index - 1][2]}").scaled(200, 150)
            self.left_image.setPixmap(pixmap_left)
        else:
            self.left_image.clear()

        # 가운데 메뉴 설정 (크게 보이게 설정)
        pixmap_center = QPixmap(f"src/tablet/menus/menu_images/{self.menu_list[self.current_index][2]}").scaled(400, 300)
        self.center_image.setPixmap(pixmap_center)
        self.menu_label.setText(self.menu_list[self.current_index][0])
        self.price_label.setText(f"{self.menu_list[self.current_index][1]}원")
        self.menu_label.setFont(QFont('Arial', 14, QFont.Bold))

        # 오른쪽 메뉴 설정
        if self.current_index < len(self.menu_list) - 1:
            pixmap_right = QPixmap(f"src/tablet/menus/menu_images/{self.menu_list[self.current_index + 1][2]}").scaled(200, 150)
            self.right_image.setPixmap(pixmap_right)
        else:
            self.right_image.clear()

    def show_previous(self):
        if self.current_index > 0:
            self.current_index -= 1
            self.update_menu()

    def show_next(self):
        if self.current_index < len(self.menu_list) - 1:
            self.current_index += 1
            self.update_menu()
            
    def select_menu(self):
        menu_name = self.menu_label.text()
        price = int(self.price_label.text()[0:-1])
        
        # 딕셔너리에 메뉴 추가 또는 업데이트
        if menu_name in self.seleted_dict:
            current_count, current_total = self.seleted_dict[menu_name]
            self.seleted_dict[menu_name] = (current_count + 1, current_total+price)
        else : self.seleted_dict[menu_name] = (1, price)

        # 서비스로 보내기 위해 저장해둠
        self.selected_menu = list(self.seleted_dict.keys())
        self.total_price += price
        print(self.selected_menu, self.total_price)

        seleted_message = f"[ 선택된 메뉴 ] \n"
        for m, (c, p) in self.seleted_dict.items() :
            seleted_message += f"{m}      {c}개      {p}원\n"
            
        self.seleted_view.setText(seleted_message)
        self.status_msg.setText(f"                    총 {self.total_price}원                    ")
        
    def order() :
        pass
        
######################################################################
def main(args=None) :
    table_number = int(input("테이블 번호를 입력하세요(1~6) : "))
    
    rclpy.init(args=args)
    table_node = TableNode(table_number)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(table_node)
    
    try :
        app = QApplication(sys.argv)
        MainWindow = QMainWindow()
        
        ui = order_program(table_number, table_node)
        ui.setupUi(MainWindow)
        MainWindow.show()
        
        sys.exit(app.exec_())
        
        executor.spin()

    except KeyboardInterrupt:
        table_node.get_logger().info('Keyboard Interrupt (SIGINT)')
        
    finally:
        executor.shutdown() # 스레드 종료
        table_node.destroy_node() # 노드 종료
        rclpy.shutdown() # rclpy 종료


######################################################################
if __name__=="__main__":
    main()