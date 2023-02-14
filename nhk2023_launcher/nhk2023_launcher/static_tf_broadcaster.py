import rclpy
from rclpy.node import Node
from tf2_ros import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

class static_tf_broadcaster(Node):
    def __init__(self):
        # Initialize the node
        super().__init__('static_odom_map_broadcaster')
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.publish_odom_map_transform()
        self.publish_map_pole_transform()

    def publish_odom_map_transform(self):
        # Create the transform
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'odom'
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0
        # Publish the transform
        self.tf_static_broadcaster.sendTransform(transform)

    def publish_map_pole_transform(self):
        # Create the transform
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'type3_pole'
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.4
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0
        # Publish the transform
        self.tf_static_broadcaster.sendTransform(transform)

def main():
    logger = rclpy.logging.get_logger('static_odom_map_broadcaster')
    rclpy.init()
    node = static_tf_broadcaster()
    logger.info('static_odom_map_broadcaster published')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()