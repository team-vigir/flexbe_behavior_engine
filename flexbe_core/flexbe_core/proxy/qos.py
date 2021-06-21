from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

# see https://index.ros.org/doc/ros2/Concepts/About-Quality-of-Service-Settings for details
QOS_DEFAULT = QoSProfile(depth=10)  # default queue_size setting
""" Matches the default QoS behavior of ROS topics. """

QOS_LATCH = QoSProfile(depth=1,
                       durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
""" Ensure that late subscribers always receive the latest previous message on the topic. """

QOS_LOSSY = QoSProfile(depth=10,
                       reliability=QoSReliabilityPolicy.BEST_EFFORT)
""" Allow dropped messages in favor of receiving data as fast as possible. """
