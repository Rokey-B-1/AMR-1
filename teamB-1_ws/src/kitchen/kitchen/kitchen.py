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

class kitchen_program(object):
    def __init__(self) :
        pass
    
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
        self.camera_btn.setGeometry(QtCore.QRect(30, 490, 171, 71))
        self.camera_btn.setObjectName("camera_btn")
        self.orderList = QtWidgets.QListWidget(self.centralwidget)
        self.orderList.setGeometry(QtCore.QRect(430, 100, 491, 381))
        self.orderList.setObjectName("orderList")
        item = QtWidgets.QListWidgetItem()
        self.orderList.addItem(item)
        self.cancel_btn = QtWidgets.QPushButton(self.centralwidget)
        self.cancel_btn.setGeometry(QtCore.QRect(840, 50, 81, 41))
        self.cancel_btn.setObjectName("cancel_btn")
        self.label = QtWidgets.QLabel(self.centralwidget)
        self.label.setGeometry(QtCore.QRect(430, 50, 311, 41))
        self.label.setObjectName("label")
        self.ok_btn = QtWidgets.QPushButton(self.centralwidget)
        self.ok_btn.setGeometry(QtCore.QRect(750, 50, 81, 41))
        self.ok_btn.setObjectName("ok_btn")
        self.delivery_btn = QtWidgets.QPushButton(self.centralwidget)
        self.delivery_btn.setGeometry(QtCore.QRect(530, 490, 291, 101))
        self.delivery_btn.setObjectName("delivery_btn")
        self.comeBackHome = QtWidgets.QPushButton(self.centralwidget)
        self.comeBackHome.setGeometry(QtCore.QRect(230, 490, 171, 71))
        self.comeBackHome.setObjectName("comeBackHome")
        MainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.cameraImage.setText(_translate("MainWindow", "Robot Cam here."))
        self.camera_btn.setText(_translate("MainWindow", "Camera On"))
        __sortingEnabled = self.orderList.isSortingEnabled()
        self.orderList.setSortingEnabled(False)
        item = self.orderList.item(0)
        item.setText(_translate("MainWindow", "N번 테이블     |     메뉴,메뉴,메뉴     |     가격"))
        self.orderList.setSortingEnabled(__sortingEnabled)
        self.cancel_btn.setText(_translate("MainWindow", "주문취소"))
        self.label.setText(_translate("MainWindow", "N번 테이블에서 음식을 주문 하였습니다."))
        self.ok_btn.setText(_translate("MainWindow", "주문확정"))
        self.delivery_btn.setText(_translate("MainWindow", "Delivery"))
        self.comeBackHome.setText(_translate("MainWindow", "Camera On"))


######################################################################
def main(args=None) :
    
    rclpy.init(args=args)
    # table_node = TableNode(table_number)
    # executor = MultiThreadedExecutor(num_threads=2)
    # executor.add_node(table_node)
    
    try :
        app = QApplication(sys.argv)
        MainWindow = QMainWindow()
        
        ui = kitchen_program()
        ui.setupUi(MainWindow)
        MainWindow.show()
        
        sys.exit(app.exec_())
        
        # executor.spin()

    except KeyboardInterrupt:
        # table_node.get_logger().info('Keyboard Interrupt (SIGINT)')
        pass
        
    finally:
        # executor.shutdown() # 스레드 종료
        # table_node.destroy_node() # 노드 종료
        rclpy.shutdown() # rclpy 종료


######################################################################
if __name__=="__main__":
    main()
