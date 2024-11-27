import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QPushButton, QListWidget, QWidget

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        
        # Window 설정
        self.setWindowTitle("PyQt 목록 추가")
        self.setGeometry(200, 200, 400, 300)
        
        # 메인 위젯과 레이아웃 설정
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.layout = QVBoxLayout()
        self.central_widget.setLayout(self.layout)
        
        # 목록 위젯 생성
        self.list_widget = QListWidget()
        self.layout.addWidget(self.list_widget)
        
        # 버튼 생성
        self.add_button = QPushButton("목록 추가")
        self.layout.addWidget(self.add_button)
        
        # 버튼 클릭 시 목록 추가 이벤트 연결
        self.add_button.clicked.connect(self.add_item)
        
        # 목록 항목 번호 추적
        self.item_count = 0
    
    def add_item(self):
        # 새 항목 추가
        self.item_count += 1
        self.list_widget.addItem(f"항목 {self.item_count}")

# PyQt5 애플리케이션 실행
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
