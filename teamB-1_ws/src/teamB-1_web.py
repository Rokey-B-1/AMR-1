import sqlite3
import gradio as gr
import pandas as pd
import matplotlib.pyplot as plt
from datetime import datetime

# 메뉴 이름 변환 딕셔너리 추가
MENU_TRANSLATION = {
    "스테이크": "Steak",
    "굴 파스타": "Oyster Pasta",
    "토마토 파스타": "Tomato Pasta",
    "팬케이크 with 라즈베리": "Pancake with Raspberry",
    "버섯 리조또": "Mushroom Risotto",
    "감자튀김": "Fries",
    "와인 한 잔": "Glass of Wine"
}

# 데이터베이스 연결 함수
def connect_db():
    return sqlite3.connect('kitchen_sales.db')

# 메뉴 이름 변환 함수
def translate_menu_name(korean_name):
    return MENU_TRANSLATION.get(korean_name, korean_name)

# get_sales_data 함수 수정
def get_sales_data():
    conn = connect_db()
    cursor = conn.cursor()
    cursor.execute("""
        SELECT table_id, menu_items, total_price, order_time 
        FROM orders
    """)
    rows = cursor.fetchall()

    data = []
    for row in rows:
        table_id, menu_items, total_price, order_time = row
        menu_dict = eval(menu_items)
        for menu, quantity in menu_dict.items():
            # 메뉴 이름을 영어로 변환
            english_menu = translate_menu_name(menu)
            data.append({
                "Table ID": table_id,
                "Menu": english_menu,  # 변환된 메뉴 이름 사용
                "Quantity": quantity,
                "Total Price": total_price,
                "Order Time": order_time
            })
    df = pd.DataFrame(data)
    conn.close()
    return df

# get_sales_statistics 함수 수정
def get_sales_statistics(start_date, end_date):
    conn = connect_db()
    cursor = conn.cursor()

    cursor.execute("""
        SELECT menu_items, total_price, order_time 
        FROM orders 
        WHERE DATE(order_time) BETWEEN ? AND ?
    """, (start_date, end_date))
    rows = cursor.fetchall()

    total_sales = 0
    menu_sales = {}
    for row in rows:
        menu_items, total_price, _ = row
        menu_dict = eval(menu_items)
        total_sales += total_price
        for menu, quantity in menu_dict.items():
            # 메뉴 이름을 영어로 변환
            english_menu = translate_menu_name(menu)
            if english_menu not in menu_sales:
                menu_sales[english_menu] = 0
            menu_sales[english_menu] += quantity

    conn.close()
    return total_sales, menu_sales

# 시각화 생성 함수
def create_visualization(menu_sales):
    fig, ax = plt.subplots(figsize=(10, 6))
    menus = list(menu_sales.keys())
    quantities = list(menu_sales.values())

    ax.bar(menus, quantities, color='skyblue')
    ax.set_title("Menu Sales Statistics", fontsize=16)
    ax.set_xlabel("Menu Items", fontsize=12)
    ax.set_ylabel("Quantity Sold", fontsize=12)
    ax.tick_params(axis='x', rotation=45)
    plt.tight_layout()
    return fig

# 매출 데이터 페이지
def display_sales():
    df = get_sales_data()
    return df

# 매출 통계 페이지
def display_statistics(start_date, end_date):
    total_sales, menu_sales = get_sales_statistics(start_date, end_date)
    menu_sales_df = pd.DataFrame(list(menu_sales.items()), columns=["Menu", "Quantity Sold"])
    return f"Total Sales: {total_sales:,} KRW", menu_sales_df, create_visualization(menu_sales)

# Gradio 인터페이스 생성
def create_app():
    with gr.Blocks() as app:
        gr.Markdown("## Restaurant Sales Management System")

        # Sales Data 페이지
        with gr.Tab("Sales Data"):
            gr.Markdown("### View All Sales Data")
            sales_data = gr.DataFrame(headers=["Table ID", "Menu", "Quantity", "Total Price", "Order Time"], interactive=False)
            refresh_button = gr.Button("Refresh Data")
            refresh_button.click(fn=display_sales, outputs=sales_data)

        # Sales Statistics 페이지
        with gr.Tab("Sales Statistics"):
            gr.Markdown("### View Sales Statistics for a Specific Period")
            start_date = gr.Textbox(label="Start Date (YYYY-MM-DD)", value=str(datetime.now().date()))
            end_date = gr.Textbox(label="End Date (YYYY-MM-DD)", value=str(datetime.now().date()))
            calculate_button = gr.Button("Calculate Statistics")
            total_sales = gr.Textbox(label="Total Sales", interactive=False)
            menu_statistics = gr.DataFrame(headers=["Menu", "Quantity Sold"], interactive=False)
            sales_chart = gr.Plot(label="Sales Visualization", format="png")
            
            calculate_button.click(
                display_statistics, 
                inputs=[start_date, end_date], 
                outputs=[total_sales, menu_statistics, sales_chart]
            )

    return app

# 실행
if __name__ == "__main__":
    app = create_app()
    app.launch(share=True)