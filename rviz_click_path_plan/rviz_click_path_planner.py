#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
import os
from ament_index_python.packages import get_package_share_directory

class PointLogger(Node):

    def __init__(self):
        super().__init__('click_planner')
        self.get_logger().info('Point Logger Node Started')

        # 토픽 구독
        self.subscription = self.create_subscription(
            PointStamped,
            'clicked_point',  # 구독할 토픽 이름
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        # 패키지 소스 디렉토리 경로 설정
        self.package_src_directory = '/home/macroorin3/pass_ws/src/rviz_click_path_plan'
        self.path_folder = os.path.join(self.package_src_directory, 'path')
        os.makedirs(self.path_folder, exist_ok=True)
        self.file_path = self.get_unique_file_path()
        # self.file_path = os.path.join(self.path_folder, 'global_path.txt')

        # 파일 열기
        self.file = open(self.file_path, 'a')

        # Path 메시지 퍼블리셔 생성
        self.path_pub = self.create_publisher(Path, '/global_path', 10)

        # Path 초기화
        self.path = Path()
        self.path.header.frame_id = 'camera_init'
        self.path.header.stamp = self.get_clock().now().to_msg()

        # MarkerArray 메시지 퍼블리셔 생성
        self.marker_pub = self.create_publisher(MarkerArray, 'markers', 10)

        # MarkerArray 초기화
        self.marker_array = MarkerArray()

        self.marker_id = 0

    def get_unique_file_path(self):
        base_path = os.path.join(self.path_folder, 'global_path')
        file_path = base_path + '.txt'
        counter = 2

        while os.path.exists(file_path):
            file_path = f"{base_path}{counter}.txt"
            counter += 1

        return file_path

    def listener_callback(self, msg):
        # 메시지에서 좌표 추출
        x = msg.point.x
        y = msg.point.y
        z = msg.point.z

        pose = PoseStamped()
        pose.header.frame_id = 'camera_init'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        # Path에 PoseStamped 객체 추가
        self.path.poses.append(pose)

        # Path 메시지 퍼블리시
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.path)

        # 마커 추가
        self.add_marker(x, y, z)

        # MarkerArray 메시지 퍼블리시
        self.marker_pub.publish(self.marker_array)

        # 좌표를 파일에 저장
        self.file.write(f'{x} {-y}\n')    ## Change Axis to NED
        self.file.flush()  # 버퍼를 비워 데이터 저장
        self.get_logger().info(f'Saved: x: {x}, y: {-y}')

    def __del__(self):
        # 파일 닫기
        if hasattr(self, 'file'):
            self.file.flush()  # 버퍼를 비워 데이터 저장
            self.file.close()

    def add_marker(self, x, y, z):
        # 새로운 Marker 객체 생성
        marker = Marker()
        marker.header.frame_id = 'camera_init'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = self.marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        # 마커의 위치와 크기 설정
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.scale.x = 0.7  # 로봇의 가로 크기
        marker.scale.y = 0.7  # 로봇의 세로 크기
        marker.scale.z = 0.1  # 로봇의 높이 (얇게)
        marker.color.a = 0.3
        marker.color.r = 0.0
        marker.color.g = 0.5
        marker.color.b = 0.0

        # MarkerArray에 추가
        self.marker_array.markers.append(marker)

        # Marker ID 업데이트
        self.marker_id += 1

def main(args=None):
    rclpy.init(args=args)
    node = PointLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
