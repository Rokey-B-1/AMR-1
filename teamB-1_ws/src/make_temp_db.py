import sqlite3
import json
from datetime import datetime, timedelta

# 한국 표준시(KST)로 변환하는 함수
def get_kst_time():
    return (datetime.utcnow() + timedelta(hours=9)).strftime('%Y-%m-%d %H:%M:%S')

# 데이터베이스 연결
conn = sqlite3.connect('kitchen_sales.db')
cursor = conn.cursor()

# orders 테이블 생성
cursor.execute('''
CREATE TABLE IF NOT EXISTS orders (
    order_id INTEGER PRIMARY KEY AUTOINCREMENT,
    table_id INTEGER NOT NULL,
    menu_items TEXT NOT NULL,
    total_price INTEGER NOT NULL,
    order_time DATETIME DEFAULT CURRENT_TIMESTAMP
)
''')

# 임의 데이터 삽입
menu_data = [
    {"table_id": 1, "menu_items": {"스테이크": 1, "굴 파스타": 2}, "total_price": 55700},
    {"table_id": 2, "menu_items": {"토마토 파스타": 1, "팬케이크 with 라즈베리": 3}, "total_price": 36600},
    {"table_id": 3, "menu_items": {"버섯 리조또": 2, "감자튀김": 1}, "total_price": 29700},
    {"table_id": 4, "menu_items": {"와인 한 잔": 4, "굴 파스타": 1}, "total_price": 24100}
]

# 데이터 삽입
for order in menu_data:
    cursor.execute(
        "INSERT INTO orders (table_id, menu_items, total_price, order_time) VALUES (?, ?, ?, ?)",
        (order["table_id"], json.dumps(order["menu_items"]), order["total_price"], get_kst_time())
    )

conn.commit()
conn.close()

print("Sample database created and data inserted!")
