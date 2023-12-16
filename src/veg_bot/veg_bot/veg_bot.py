import rclpy
from std_msgs.msg import Empty
from rclpy.logging import get_logger
from rclpy.callback_groups import ReentrantCallbackGroup
from threading import Thread
from pymoveit2 import MoveIt2

pick_lemon = [-0.6660551770555824,-0.5356524884715981, 0.07971503194846172, -0.42450636828655863, 1.1828722500470634, 0.557996973053764]
home = [0., 0., 0., 0., 0., 0.]
place_lemon = [1.0098069402903564, -1.1275450375606022, 1.4109251376652807, 0.23905296742250726, 1.2046140808045513, -0.006214707278761809]

def main():
    rclpy.init()
    logger = get_logger("vegbot_node_logger")
    node = rclpy.create_node(node_name="veg_bot_node")
 
    callback_group = ReentrantCallbackGroup()

    attach_pub = node.create_publisher(Empty, "/lemon/attach", 1)
    detach_pub = node.create_publisher(Empty, "/lemon/detach", 1)

    logger.info("detaching lemon")
    detach_pub.publish(Empty())    

    moveit2 = MoveIt2(node=node, 
                      joint_names=['joint_a1', 'joint_a2','joint_a3','joint_a4','joint_a5','joint_a6'],
                      base_link_name='base_link',
                      end_effector_name='tool0',
                      group_name='arm',
                      callback_group=callback_group,
                      follow_joint_trajectory_action_name="/arm_controller/follow_joint_trajectory"
                      )

    # Spin the node in background thread(s) and wait a bit for initialization
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    node.create_rate(1.0).sleep()

    logger.info("Move To Home Position")

    moveit2.move_to_configuration(home)
    moveit2.wait_until_executed()

    logger.info("Move To Pick Lemon")
    moveit2.move_to_configuration(pick_lemon)
    moveit2.wait_until_executed()

    logger.info("Attaching Lemon")
    attach_pub.publish(Empty())

    logger.info("Move To Place Lemon")
    moveit2.move_to_configuration(place_lemon)
    moveit2.wait_until_executed()

    logger.info("Detaching Lemon")

    detach_pub.publish(Empty())
    logger.info("Moving Home")
    
    moveit2.move_to_configuration(home)
    moveit2.wait_until_executed()

    rclpy.shutdown()
    executor_thread.join()
    exit(0)

if __name__ == '__main__':
    main()
