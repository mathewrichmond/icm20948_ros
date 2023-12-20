# System Imports
import qwiic_icm20948

# ROS Imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import Imu, MagneticField, Temperature

NAME = "icm20948"
QUEUE_SIZE = 10
PERIOD = 0.25  # Seconds
TEMPERATURE_PERIOD = 10.0  # Seconds


IMU = "imu"
MAGNETOMETER = "magnetometer"
TEMPERATURE = "temperature"


class Icm20948Node(Node):
    def __init__(self) -> None:
        super().__init__(NAME)

        # Initialize the sensor.
        self._qwiic = qwiic_icm20948.QwiicIcm20948()
        assert (
            self._qwiic.connected
        ), "The Qwiic ICM20948 device isn't connected to the system. Please check your connection."
        self._qwiic.begin()

        # Initialize the publish timers.
        self.timer = self.create_timer(PERIOD, self.timer_callback)
        self.temperature_timer = self.create_timer(
            TEMPERATURE_PERIOD, self.temperature_callback
        )

        # Initialize the ROS publishers.
        self.imu_publisher = self.create_publisher(Imu, f"/{NAME}/imu", QUEUE_SIZE)
        self.magnetometer_publisher = self.create_publisher(
            MagneticField, f"/{NAME}/magnetometer", QUEUE_SIZE
        )
        self.temperature_publisher = self.create_publisher(
            Temperature, f"/{NAME}/temperature", QUEUE_SIZE
        )

    def timer_callback(self) -> None:
        stamp = self.get_clock().now().to_msg()
        if self._qwiic.dataReady():
            # Read all axis and temp from sensor. Note this also updates all instance variables.
            self._qwiic.getAgmt()

            # Publish the magnetometer data.
            self.magnetometer_publisher.publish(
                MagneticField(
                    header=Header(stamp=stamp, frame_id=NAME),
                    magnetic_field=(self.imu.mxRaw, self.imu.myRaw, self.imu.mzRaw),
                )
            )

            # Publish the IMU data.
            self.imu_publisher.publish(
                Imu(header=Header(stamp=stamp), frame_id=NAME),
                orientation_covariance=[-1] * 9,
                angular_velocity=(self.imu.gxRaw, self.imu.gyRaw, self.imu.gzRaw),
                angular_velocity_covariance=[-1] * 9,
                linear_acceleration=(
                    self.imu.axRaw,
                    self.imu.ayRaw,
                    self.imu.azRaw,
                ),
                linear_acceleration_covariance=[-1] * 9,
            )
        else:
            self.get_logger().info("Waiting for data")

    def temperature_callback(self) -> None:
        self.temperature_publisher.publish(
            Temperature(
                header=Header(stamp=self.get_clock().now().to_msg(), frame_id=NAME),
                temperature=self.imu.tmpRaw,
            )
        )


def main(args=None):
    rclpy.init(args=args)
    node = Imu()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
