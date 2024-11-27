#!/usr/bin/env python3
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
        
        # 주문 상태 관리
        self.order_states = {}  # {table_number: state}
        self.get_logger().info('Kitchen node has been started')

    def handle_order(self, request, response):
        """주문 요청 처리"""
        # 주문 접수 시 상태 업데이트
        self.update_order_state(request.table_id, "RECEIVED")
        
        # 주문 처리 로직...
        
        response.success = True
        response.message = f"Order from table {request.table_id} received"
        return response

    def update_order_state(self, table_number: int, state: str):
        """주문 상태 업데이트 및 발행"""
        self.order_states[table_number] = state
        self.publish_status()
        self.get_logger().info(f'Table {table_number} state updated to: {state}')

    def publish_status(self):
        """현재 모든 주문 상태 발행"""
        msg = OrderStatus()
        msg.table_numbers = list(self.order_states.keys())
        msg.order_states = list(self.order_states.values())
        self.status_publisher.publish(msg)

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