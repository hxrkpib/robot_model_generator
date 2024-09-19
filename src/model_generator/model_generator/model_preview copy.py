#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
import os
from ament_index_python.packages import get_package_share_directory
import asyncio


class URDFFileHandler(FileSystemEventHandler):
    def __init__(self, node, urdf_path):
        super().__init__()
        self.node = node
        self.urdf_path = urdf_path

    def on_modified(self, event):
        if os.path.abspath(event.src_path) == self.urdf_path:
            self.node.get_logger().info(f"检测到URDF文件变化: {event.src_path}")
            self.node.update_robot_description()


class URDFMonitorNode(Node):
    def __init__(self):
        super().__init__('urdf_monitor_node')

        # 获取包的share目录
        package_share_directory = get_package_share_directory(
            'model_preview')  # 修改为您的包名
        urdf_path = '/home/lizhen/works/open_source/robot_model_generator/generator/model_generate/output/output.urdf'  # 修改为您的URDF文件名和路径

        self.urdf_path = os.path.abspath(urdf_path)

        if not os.path.exists(self.urdf_path):
            self.get_logger().error(f"URDF文件不存在: {self.urdf_path}")
            rclpy.shutdown()

        # 设置 watchdog 观察者
        event_handler = URDFFileHandler(self, self.urdf_path)
        self.observer = Observer()
        self.observer.schedule(event_handler, path=os.path.dirname(
            self.urdf_path), recursive=False)
        self.observer.start()

        self.get_logger().info(f"正在监控URDF文件: {self.urdf_path}")

        # 创建与 robot_state_publisher 的 SetParameters 服务客户端
        self.parameter_client = self.create_client(
            SetParameters, '/robot_state_publisher/set_parameters')

        while not self.parameter_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 /robot_state_publisher/set_parameters 服务...')

        self.get_logger().info('已连接到 /robot_state_publisher/set_parameters 服务')

        # 首次加载并设置 robot_description
        self.update_robot_description()

    def update_robot_description(self):
        try:
            with open(self.urdf_path, 'r') as urdf_file:
                urdf_content = urdf_file.read()

            # 创建 SetParameters 请求
            request = SetParameters.Request()
            param = Parameter('robot_description',
                              Parameter.Type.STRING, urdf_content)
            request.parameters.append(param.to_parameter_msg())

            # 发送请求
            future = self.parameter_client.call_async(request)
            future.add_done_callback(self.set_parameters_callback)

            self.get_logger().info("正在发送 robot_description 参数更新请求...")

        except Exception as e:
            self.get_logger().error(f"读取URDF文件失败: {e}")

    def set_parameters_callback(self, future):
        try:
            response = future.result()
            if response.successful:
                self.get_logger().info("已成功更新 robot_description 参数。")
            else:
                self.get_logger().error(
                    f"更新 robot_description 参数失败: {response.reason}")
        except Exception as e:
            self.get_logger().error(f"调用 SetParameters 服务失败: {e}")

    def destroy_node(self):
        self.observer.stop()
        self.observer.join()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = URDFMonitorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("正在关闭 URDF Monitor 节点。")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
