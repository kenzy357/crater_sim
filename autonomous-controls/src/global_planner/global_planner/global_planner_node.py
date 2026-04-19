import rclpy
from rclpy.node import Node 

class GlobalPlannerNode(Node):

    def __init__(self):
        super.__init__('global_planner')

def main():
    rclpy.init()
    node = GlobalPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == "__main__":
    main()