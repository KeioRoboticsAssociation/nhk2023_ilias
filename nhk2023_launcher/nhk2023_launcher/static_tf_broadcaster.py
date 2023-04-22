import rclpy
from rclpy.node import Node
from tf2_ros import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from typing import TypedDict


class PoleType(TypedDict):
    name: str
    place: tuple[float, float]
    radius: float


pole_pos: list[PoleType]

pole_pos = [{
    'name': 'type1_1',
    'place': (3.2, 0.0),
    'radius': 0.0508
}, {
    'name': 'type1_2',
    'place': (3.2, 3.2),
    'radius': 0.0508
}, {
    'name': 'type1_3',
    'place': (-3.2, 3.2),
    'radius': 0.0508
}, {
    'name': 'type1_4',
    'place': (-3.2, 0.0),
    'radius': 0.0508
}, {
    'name': 'type1_5',
    'place': (-3.2, -3.2),
    'radius': 0.0508
}, {
    'name': 'type1_6',
    'place': (3.2, -3.2),
    'radius': 0.0508
}, {
    'name': 'type2_1',
    'place': (1.3, 1.3),
    'radius': 0.0508
}, {
    'name': 'type2_2',
    'place': (-1.3, 1.3),
    'radius': 0.0508
}, {
    'name': 'type2_3',
    'place': (-1.3, -1.3),
    'radius': 0.0508
}, {
    'name': 'type2_4',
    'place': (1.3, -1.3),
    'radius': 0.0508
}, {
    'name': 'type3',
    'place': (0.0, 0.0),
    'radius': 0.0699
}]


class static_tf_broadcaster(Node):

    def __init__(self):
        # Initialize the node
        super().__init__('static_tf_broadcaster')
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.gen_odom_map_transform()
        self.gen_map_pole_transform()
        self.gen_odom_er_base_link_transform()
        self.gen_odom_rr_base_link_transform()
        self.tf_static_broadcaster.sendTransform([
            self.er_odom_map_transform, self.rr_odom_map_transform,
            *self.map_pole_transform, self.odom_er_base_link_transform,
            self.odom_rr_base_link_transform
        ])

    def gen_odom_map_transform(self):
        # Create the transform
        self.er_odom_map_transform = TransformStamped()
        self.er_odom_map_transform.header.stamp = self.get_clock().now(
        ).to_msg()
        self.er_odom_map_transform.header.frame_id = 'map'
        self.er_odom_map_transform.child_frame_id = 'er/odom'
        self.er_odom_map_transform.transform.translation.x = 0.0
        self.er_odom_map_transform.transform.translation.y = 0.0
        self.er_odom_map_transform.transform.translation.z = 0.0
        self.er_odom_map_transform.transform.rotation.x = 0.0
        self.er_odom_map_transform.transform.rotation.y = 0.0
        self.er_odom_map_transform.transform.rotation.z = 0.0
        self.er_odom_map_transform.transform.rotation.w = 1.0

        self.rr_odom_map_transform = TransformStamped()
        self.rr_odom_map_transform.header.stamp = self.get_clock().now(
        ).to_msg()
        self.rr_odom_map_transform.header.frame_id = 'map'
        self.rr_odom_map_transform.child_frame_id = 'rr/odom'
        self.rr_odom_map_transform.transform.translation.x = 0.0
        self.rr_odom_map_transform.transform.translation.y = 0.0
        self.rr_odom_map_transform.transform.translation.z = 0.0
        self.rr_odom_map_transform.transform.rotation.x = 0.0
        self.rr_odom_map_transform.transform.rotation.y = 0.0
        self.rr_odom_map_transform.transform.rotation.z = 0.0
        self.rr_odom_map_transform.transform.rotation.w = 1.0

    def gen_map_pole_transform(self):
        # Create the transform
        self.map_pole_transform: list[TransformStamped] = []
        for e in pole_pos:
            pole = TransformStamped()
            pole.header.stamp = self.get_clock().now().to_msg()
            pole.header.frame_id = 'map'
            pole.child_frame_id = e['name']
            pole.transform.translation.x = e['place'][0]
            pole.transform.translation.y = e['place'][1]
            pole.transform.translation.z = 0.0
            pole.transform.rotation.x = 0.0
            pole.transform.rotation.y = 0.0
            pole.transform.rotation.z = 0.0
            pole.transform.rotation.w = 1.0
            self.map_pole_transform.append(pole)

    def gen_odom_er_base_link_transform(self):
        # Create the transform
        self.odom_er_base_link_transform = TransformStamped()
        self.odom_er_base_link_transform.header.stamp = self.get_clock().now(
        ).to_msg()
        self.odom_er_base_link_transform.header.frame_id = 'er/odom'
        self.odom_er_base_link_transform.child_frame_id = 'er/base_link'
        self.odom_er_base_link_transform.transform.translation.x = -5.0
        self.odom_er_base_link_transform.transform.translation.y = -0.2
        self.odom_er_base_link_transform.transform.translation.z = 0.0
        self.odom_er_base_link_transform.transform.rotation.x = 0.0
        self.odom_er_base_link_transform.transform.rotation.y = 0.0
        self.odom_er_base_link_transform.transform.rotation.z = 0.0
        self.odom_er_base_link_transform.transform.rotation.w = 1.0

    def gen_odom_rr_base_link_transform(self):
        # rr
        self.odom_rr_base_link_transform = TransformStamped()
        self.odom_rr_base_link_transform.header.stamp = self.get_clock().now(
        ).to_msg()
        self.odom_rr_base_link_transform.header.frame_id = 'rr/odom'
        self.odom_rr_base_link_transform.child_frame_id = 'rr/base_link'
        self.odom_rr_base_link_transform.transform.translation.x = -5.0
        self.odom_rr_base_link_transform.transform.translation.y = -0.2
        self.odom_rr_base_link_transform.transform.translation.z = 0.0
        self.odom_rr_base_link_transform.transform.rotation.x = 0.0
        self.odom_rr_base_link_transform.transform.rotation.y = 0.0
        self.odom_rr_base_link_transform.transform.rotation.z = 0.0
        self.odom_rr_base_link_transform.transform.rotation.w = 1.0


def main():
    logger = rclpy.logging.get_logger('static_tf_broadcaster')
    rclpy.init()
    node = static_tf_broadcaster()
    logger.info('static_tf_broadcaster published')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
