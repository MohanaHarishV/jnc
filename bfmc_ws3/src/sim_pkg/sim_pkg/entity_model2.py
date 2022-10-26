import os
import sys
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity

def main(args=None):
    rclpy.init(args=args)
    sdf_file_path = os.path.join(
#       get_package_share_directory("sim_pkg"), "models",
#       "rcCar_assembly", "model.sdf")
         get_package_share_directory("sim_pkg"), "models",
        "track", "model.sdf")
    
    node = Node("entity_spawner")
    argv = sys.argv[1:]
    
    client = node.create_client(SpawnEntity, "/spawn_entity")
    while not client.wait_for_service(1.0):
        node.get_logger().warn("Waiting for Server /spawn_entity....")
        
    request = SpawnEntity.Request()
    request.name = "automobile"
    request.xml = open(sdf_file_path,'r').read()
    request.robot_namespace = argv[1]
    request.initial_pose.position.x = float(argv[2])
    request.initial_pose.position.y = float(argv[3])
    request.initial_pose.position.z = float(argv[4])
    request.initial_pose.orientation.x = 0.0
    request.initial_pose.orientation.y = 0.0
    request.initial_pose.orientation.z = float(argv[5])
    request.initial_pose.orientation.w = 0.0
    
    future = client.call_async(request)
    
    try:
        response = future.result()
    except Exception as e:
        node.get_logger().error("service call failed %r" %(e,))
    
    rclpy.spin_until_future_complete(node, future)
    
    rclpy.shutdown()
        
    
    
if __name__ == "__main__":
    main()