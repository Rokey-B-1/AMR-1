#!/usr/bin/env python3
# ROS2 service 통신 테스트용 // 추후 ui 및 로직 추가 필요!!
import rclpy
from rclpy.node import Node
from system_interfaces.srv import OrderDetail
from system_interfaces.msg import OrderStatus

class KitchenNode(Node):
    def __init__(self):
        super().__init__('kitchen_node')
        
        # 주문 접수 서비스
        self.order_service = self.create_service(
            OrderDetail,
            'order_detail',
            self.handle_order
        )
        
        # 주문 상태 퍼블리셔
        self.status_publisher = self.create_publisher(
            OrderStatus,
            'order_status',
            10
        )
        
        # 주문 상태 리스트
        self.order_status_list = []
        
        # 주문 상태 발행을 위한 타이머
        self.create_timer(1.0, self.publish_status)  # 1초마다 상태 발행
        
        self.get_logger().info('Kitchen node has been started')

    def handle_order(self, request, response):
        """주문 요청 처리"""
        try:
            # 주문 정보 로깅
            self.get_logger().info(f'Order received from table {request.table_id}')
            self.get_logger().info(f'Menu items: {request.menu_items}')
            self.get_logger().info(f'Total price: {request.total_price}')
            
            # 주문 상태 리스트에 추가
            status_str = f"Table {request.table_id} COOKING"
            if status_str not in self.order_status_list:
                self.order_status_list.append(status_str)
            
            response.success = True
            response.message = f"테이블 번호 {request.table_id} 의 주문이 접수 되었습니다."
            
        except Exception as e:
            self.get_logger().error(f'Error processing order: {str(e)}')
            response.success = False
            response.message = "Error processing order"
            
        return response

    def change_order_status(self, table_id: int, new_status: str):
        """주문 상태 변경"""
        # 기존 상태 찾아서 제거
        for status in self.order_status_list[:]:
            if f"Table {table_id}" in status:
                self.order_status_list.remove(status)
        
        # 새로운 상태 추가
        new_status_str = f"Table {table_id} {new_status}"
        self.order_status_list.append(new_status_str)
        self.get_logger().info(f'Status changed: {new_status_str}')

    def publish_status(self):
        """현재 모든 주문 상태 발행"""
        try:
            if self.order_status_list:  # 리스트가 비어있지 않을 때만 발행
                msg = OrderStatus()
                msg.table_numbers = 1  # 현재 발행하는 상태의 테이블 번호
                
                # 모든 상태를 하나의 문자열로 결합
                msg.order_states = ' | '.join(self.order_status_list)
                
                self.status_publisher.publish(msg)
                
        except Exception as e:
            self.get_logger().error(f'Error publishing status: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = KitchenNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()