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
import pandas as pd
import numpy as np
from scipy.spatial.transform import Rotation as R

base_color = "0.8 0.4 0 1"
link_color_list = ["1 0.4 0.4 1", "1 1 0.4 1",
                   "0.4 1 0.4 1", "0 0.5 1 1", "0.8 1.0 0.8 1", "1 0.5 1 1"]


class URDFGenerator:
    def __init__(self, data_file_path, mesh_folder_path, output_folder_path, urdf_template_file_path, link_template_file_path, joint_template_file_path):
        self.urdf_output_file = output_folder_path
        self.data_excel_file_path = data_file_path
        self.mesh_folder = mesh_folder_path
        self.urdf_template_file = urdf_template_file_path
        self.link_template_file = link_template_file_path
        self.joint_template_file = joint_template_file_path

    def mirror_euler(self, roll, pitch, yaw, mirror_plane):

        # 步骤 1: 将欧拉角转换为旋转矩阵
        def euler_to_matrix(roll, pitch, yaw):
            Rz = np.array([
                [np.cos(yaw), -np.sin(yaw), 0],
                [np.sin(yaw),  np.cos(yaw), 0],
                [0,            0,           1]
            ])

            Ry = np.array([
                [np.cos(pitch), 0, np.sin(pitch)],
                [0,              1, 0],
                [-np.sin(pitch), 0, np.cos(pitch)]
            ])

            Rx = np.array([
                [1, 0,           0],
                [0, np.cos(roll), -np.sin(roll)],
                [0, np.sin(roll),  np.cos(roll)]
            ])

            return Rz @ Ry @ Rx  # ZYX 顺序

        # 步骤 2: 定义反射矩阵
        def get_reflection_matrix(mirror_plane):
            if mirror_plane.upper() == 'X':
                # 反射 X 轴
                return np.diag([1, -1, -1])
            elif mirror_plane.upper() == 'Y':
                # 反射 Y 轴
                return np.diag([-1, 1, -1])
            elif mirror_plane.upper() == 'Z':
                # 反射 Z 轴
                return np.diag([-1, -1, 1])
            else:
                raise ValueError(
                    "Invalid mirror_plane. Choose from 'YZ', 'XZ', 'XY'.")

        # 步骤 3: 将旋转矩阵转换回欧拉角
        def matrix_to_euler(R):
            if not isinstance(R, np.ndarray) or R.shape != (3, 3):
                raise ValueError("R 必须是一个 3x3 的 NumPy 数组。")

            # 检查是否存在奇异性（万向节锁）
            if abs(R[2, 0]) != 1:
                pitch = -np.arcsin(R[2, 0])
                cos_pitch = np.cos(pitch)
                yaw = np.arctan2(R[2, 1] / cos_pitch, R[2, 2] / cos_pitch)
                roll = np.arctan2(R[1, 0] / cos_pitch, R[0, 0] / cos_pitch)
            else:
                # 奇异性处理
                roll = 0  # 可以设为0
                if R[2, 0] == -1:
                    pitch = np.pi / 2
                    yaw = roll + np.arctan2(R[0, 1], R[0, 2])
                else:
                    pitch = -np.pi / 2
                    yaw = -roll + np.arctan2(-R[0, 1], -R[0, 2])

            return roll, pitch, yaw

        # 执行步骤 1
        R = euler_to_matrix(roll, pitch, yaw)

        # 执行步骤 2
        M = get_reflection_matrix(mirror_plane)

        # 应用反射
        R_mirrored = M @ R

        # 执行步骤 3
        mirrored_roll, mirrored_pitch, mirrored_yaw = matrix_to_euler(
            R_mirrored)

        return mirrored_roll, mirrored_pitch, mirrored_yaw

    def urdf_generator(self):
        # 获取所有工作表名称
        data_excel_file = pd.ExcelFile(
            self.data_excel_file_path)
        sheet_names = data_excel_file.sheet_names
        urdf_text = ''
        joint_index = 0

        # 遍历所有工作表
        for index, sheet_name in enumerate(sheet_names):
            df = pd.read_excel(data_excel_file, sheet_name=sheet_name,
                               header=None, dtype=str)

            if sheet_name == 'base':
                robot_name = str(df.iloc[0, 1])
                base_name = str(df.iloc[1, 1])
                com = str(df.iloc[3, 2])+" " + \
                    str(df.iloc[3, 3])+" "+str(df.iloc[3, 4])
                mass = str(df.iloc[4, 2])
                ixx = str(df.iloc[5, 2])
                iyy = str(df.iloc[6, 3])
                izz = str(df.iloc[7, 4])
                ixy = str(df.iloc[5, 3])
                ixz = str(df.iloc[5, 4])
                iyz = str(df.iloc[6, 4])
                link_mesh = "file://"+self.mesh_folder+str(df.iloc[8, 1])
                with open(self.urdf_template_file, 'r', encoding='utf-8') as file:
                    urdf_text = file.read()
                    urdf_text = urdf_text.replace(
                        'template_robot_name', robot_name)
                # print(urdf_text)
                with open(self.link_template_file, 'r', encoding='utf-8') as file:
                    text = file.read()
                    text = text.replace('template_link_name', base_name)
                    text = text.replace('template_link_com', com)
                    text = text.replace('template_link_mass', mass)
                    text = text.replace('template_ixx', ixx)
                    text = text.replace('template_iyy', iyy)
                    text = text.replace('template_izz', izz)
                    text = text.replace('template_ixy', ixy)
                    text = text.replace('template_ixz', ixz)
                    text = text.replace('template_iyz', iyz)
                    text = text.replace('template_mesh_file', link_mesh)
                    text = text.replace('template_mesh_color', base_color)
                    text = text.replace('template_scale', '1 1 1')
                urdf_text = urdf_text.replace(
                    '<!-- urdf auto generate tool -->', text)
                # print(urdf_text)
            else:
                print(f"正在处理工作表：{sheet_name}")
                # 获取第一列数据
                first_column = df.iloc[:, 0]

                # 找到第一列中最后一个非空单元格的行号
                last_valid_row = first_column.last_valid_index()
                joint_color_index = 0
                for i in range(last_valid_row):
                    if df.iloc[i, 0] == 'Link Name':
                        joint_index = joint_index+1
                        joint_color_index = joint_color_index+1
                        # link
                        link_name = str(df.iloc[i, 1])
                        com = str(df.iloc[i+2, 2])+" " + \
                            str(df.iloc[i+2, 3])+" "+str(df.iloc[i+2, 4])
                        mass = str(df.iloc[i+3, 2])
                        ixx = str(df.iloc[i+4, 2])
                        iyy = str(df.iloc[i+5, 3])
                        izz = str(df.iloc[i+6, 4])
                        ixy = str(df.iloc[i+4, 3])
                        ixz = str(df.iloc[i+4, 4])
                        iyz = str(df.iloc[i+5, 4])
                        # joint
                        joint_name = "idx" + \
                            f'{joint_index:02}'+"_"+str(link_name)
                        joint_trans = str(
                            df.iloc[i+7, 2])+" "+str(df.iloc[i+7, 3])+" "+str(df.iloc[i+7, 4])
                        joint_rpy = str(df.iloc[i+8, 2])+" " + \
                            str(df.iloc[i+8, 3])+" "+str(df.iloc[i+8, 4])
                        joint_axis = str(
                            df.iloc[i+9, 2])+" "+str(df.iloc[i+9, 3])+" "+str(df.iloc[i+9, 4])
                        joint_lower_limit = str(df.iloc[i+10, 2])
                        joint_upper_limit = str(df.iloc[i+11, 2])
                        joint_velocity = str(df.iloc[i+12, 2])
                        joint_effort = str(df.iloc[i+13, 2])
                        parent_link = str(df.iloc[i+14, 1])
                        child_link = link_name
                        joint_type = str(df.iloc[i+15, 1])
                        if joint_type == 'ball':
                            joint_type = 'fixed'
                        link_mesh = "file://" + \
                            self.mesh_folder+str(df.iloc[i+16, 1])

                        with open(self.link_template_file, 'r', encoding='utf-8') as file:
                            text = file.read()
                            text = text.replace(
                                'template_link_name', link_name)
                            text = text.replace('template_link_com', com)
                            text = text.replace('template_link_mass', mass)
                            text = text.replace('template_ixx', ixx)
                            text = text.replace('template_iyy', iyy)
                            text = text.replace('template_izz', izz)
                            text = text.replace('template_ixy', ixy)
                            text = text.replace('template_ixz', ixz)
                            text = text.replace('template_iyz', iyz)
                            text = text.replace(
                                'template_mesh_file', link_mesh)
                            text = text.replace(
                                'template_mesh_color', link_color_list[joint_color_index % len(link_color_list) - 1])
                            text = text.replace('template_scale', '1 1 1')
                            urdf_text = urdf_text.replace(
                                '<!-- urdf auto generate tool -->', text)
                            # print(urdf_text)
                        with open(self.joint_template_file, 'r', encoding='utf-8') as file:
                            text = file.read()
                            text = text.replace(
                                'template_joint_name', joint_name)
                            text = text.replace(
                                'template_joint_type', joint_type)
                            text = text.replace(
                                'template_joint_xyz', joint_trans)
                            text = text.replace(
                                'template_joint_rpy', joint_rpy)
                            text = text.replace(
                                'template_parent_link', parent_link)
                            text = text.replace(
                                'template_child_link', child_link)
                            text = text.replace(
                                'template_joint_axis', joint_axis)
                            text = text.replace(
                                'template_joint_lower_limit', joint_lower_limit)
                            text = text.replace(
                                'template_joint_upper_limit', joint_upper_limit)
                            text = text.replace(
                                'template_joint_effort', joint_effort)
                            text = text.replace(
                                'template_joint_velocity', joint_velocity)
                            urdf_text = urdf_text.replace(
                                '<!-- urdf auto generate tool -->', text)
                            # print(urdf_text)
                # 处理镜像
                is_mirror = str(df.iloc[0, 1])
                mirror_name = str(df.iloc[1, 1])
                if is_mirror == "Y":
                    joint_color_index = 0
                    for i in range(last_valid_row):
                        if df.iloc[i, 0] == 'Link Name':
                            joint_index = joint_index+1
                            joint_color_index = joint_color_index+1
                            # link
                            link_name = str(df.iloc[i, 1])
                            if mirror_name == "left":
                                link_name = link_name.replace('left', 'right')
                            else:
                                link_name = link_name.replace('right', 'left')
                            coe_x = float(1)
                            coe_y = float(1)
                            coe_z = float(1)
                            stl_coe_x = float(1)
                            stl_coe_y = float(1)
                            stl_coe_z = float(1)
                            mirror_axis = str(df.iloc[i+17, 1])
                            mirror_stl_axis = str(df.iloc[i+18, 1])
                            if mirror_axis == 'X':
                                coe_x = float(-1)
                            if mirror_axis == 'Y':
                                coe_y = float(-1)
                            if mirror_axis == 'Z':
                                coe_z = float(-1)
                            if mirror_stl_axis == 'X':
                                stl_coe_x = float(-1)
                            if mirror_stl_axis == 'Y':
                                stl_coe_y = float(-1)
                            if mirror_stl_axis == 'Z':
                                stl_coe_z = float(-1)
                            com = str(stl_coe_x*float(df.iloc[i+2, 2]))+" " + \
                                str(stl_coe_y*float(df.iloc[i+2, 3])
                                    )+" "+str(stl_coe_z*float(df.iloc[i+2, 4]))
                            mass = str(df.iloc[i+3, 2])
                            ixx = str(df.iloc[i+4, 2])
                            iyy = str(df.iloc[i+5, 3])
                            izz = str(df.iloc[i+6, 4])
                            ixy = str(stl_coe_x*stl_coe_y *
                                      float(df.iloc[i+4, 3]))
                            ixz = str(stl_coe_x*stl_coe_z *
                                      float(df.iloc[i+4, 4]))
                            iyz = str(stl_coe_y*stl_coe_z *
                                      float(df.iloc[i+5, 4]))
                            # joint
                            joint_name = "idx" + \
                                f'{joint_index:02}'+"_"+str(link_name)
                            joint_trans = str(
                                coe_x*float(df.iloc[i+7, 2]))+" "+str(coe_y*float(df.iloc[i+7, 3]))+" "+str(coe_z*float(df.iloc[i+7, 4]))
                            r, p, y = self.mirror_euler(float(
                                df.iloc[i+8, 2]), float(df.iloc[i+8, 3]), float(df.iloc[i+8, 4]), mirror_axis)
                            joint_rpy = str(df.iloc[i+8, 2])+" " + \
                                str(df.iloc[i+8, 3]) + \
                                " "+str(df.iloc[i+8, 4])
                            joint_axis = str(
                                df.iloc[i+9, 2])+" "+str(df.iloc[i+9, 3])+" "+str(df.iloc[i+9, 4])
                            joint_lower_limit = str(df.iloc[i+10, 2])
                            joint_upper_limit = str(df.iloc[i+11, 2])
                            if mirror_axis == 'Z':
                                joint_lower_limit = str(df.iloc[i+11, 2])
                                joint_upper_limit = str(df.iloc[i+10, 2])
                            joint_velocity = str(df.iloc[i+12, 2])
                            joint_effort = str(df.iloc[i+13, 2])
                            parent_link = str(df.iloc[i+14, 1])
                            if mirror_name == "left":
                                parent_link = parent_link.replace(
                                    'left', 'right')
                            else:
                                parent_link = parent_link.replace(
                                    'right', 'left')
                            child_link = link_name
                            joint_type = str(df.iloc[i+15, 1])
                            if joint_type == 'ball':
                                joint_type = 'fixed'
                            link_mesh = "file://" + \
                                self.mesh_folder+str(df.iloc[i+16, 1])

                            with open(self.link_template_file, 'r', encoding='utf-8') as file:
                                text = file.read()
                                text = text.replace(
                                    'template_link_name', link_name)
                                text = text.replace('template_link_com', com)
                                text = text.replace('template_link_mass', mass)
                                text = text.replace('template_ixx', ixx)
                                text = text.replace('template_iyy', iyy)
                                text = text.replace('template_izz', izz)
                                text = text.replace('template_ixy', ixy)
                                text = text.replace('template_ixz', ixz)
                                text = text.replace('template_iyz', iyz)
                                text = text.replace(
                                    'template_mesh_file', link_mesh)
                                text = text.replace(
                                    'template_mesh_color', link_color_list[joint_color_index % len(link_color_list) - 1])
                                text = text.replace('template_scale', str(
                                    stl_coe_x)+" " + str(stl_coe_y) + " "+str(stl_coe_z))
                                urdf_text = urdf_text.replace(
                                    '<!-- urdf auto generate tool -->', text)
                                # print(urdf_text)
                            with open(self.joint_template_file, 'r', encoding='utf-8') as file:
                                text = file.read()
                                text = text.replace(
                                    'template_joint_name', joint_name)
                                text = text.replace(
                                    'template_joint_type', joint_type)
                                text = text.replace(
                                    'template_joint_xyz', joint_trans)
                                text = text.replace(
                                    'template_joint_rpy', joint_rpy)
                                text = text.replace(
                                    'template_parent_link', parent_link)
                                text = text.replace(
                                    'template_child_link', child_link)
                                text = text.replace(
                                    'template_joint_axis', joint_axis)
                                text = text.replace(
                                    'template_joint_lower_limit', joint_lower_limit)
                                text = text.replace(
                                    'template_joint_upper_limit', joint_upper_limit)
                                text = text.replace(
                                    'template_joint_effort', joint_effort)
                                text = text.replace(
                                    'template_joint_velocity', joint_velocity)
                                urdf_text = urdf_text.replace(
                                    '<!-- urdf auto generate tool -->', text)
                                # print(urdf_text)

        # print(urdf_text)
        directory = os.path.dirname(self.urdf_output_file)
        if not os.path.exists(directory):
            os.makedirs(directory)
        with open(self.urdf_output_file, 'w') as file:
            file.write(urdf_text)


class DataFileHandler(FileSystemEventHandler):
    def __init__(self, node, data_path):
        super().__init__()
        self.node = node
        self.data_path = data_path

    def on_modified(self, event):
        if os.path.abspath(event.src_path) == self.data_path:
            self.node.get_logger().info(
                f"检测到robot_data.xlsx文件变化: {event.src_path}")
            self.node.urdf_generator.urdf_generator()
            self.node.update_robot_description()


class URDFGeneratorNode(Node):
    def __init__(self):
        super().__init__('urdf_monitor_node')

        # 获取包的share目录
        package_share_directory = get_package_share_directory(
            'model_generator')  # 修改为您的包名
        # 修改为您的URDF文件名和路径
        data_folder_path = self.declare_parameter(
            'data_path', '').get_parameter_value().string_value
        if data_folder_path == "" or not os.path.exists(data_folder_path):
            self.get_logger().error(f"数据路径错误!!!!!!!!")
            rclpy.shutdown()
        data_file_path = data_folder_path+"/"+"robot_data.xlsx"
        mesh_folder_path = data_folder_path+"/"+"meshes/"
        output_folder_path = data_folder_path+"/"+"output/"
        urdf_output_file_path = output_folder_path+"/output.urdf"
        urdf_template_file_path = package_share_directory + \
            "/templates/urdf/urdf.template"
        link_template_file_path = package_share_directory + \
            "/templates/urdf/link.template"
        joint_template_file_path = package_share_directory + \
            "/templates/urdf/joint.template"

        self.urdf_generator = URDFGenerator(data_file_path, mesh_folder_path, urdf_output_file_path,
                                            urdf_template_file_path, link_template_file_path, joint_template_file_path)
        self.urdf_generator.urdf_generator()

        self.urdf_path = urdf_output_file_path

        if not os.path.exists(self.urdf_path):
            self.get_logger().error(f"URDF文件不存在: {self.urdf_path}")
            rclpy.shutdown()

        # 设置 watchdog 观察者
        event_handler = DataFileHandler(self, data_file_path)
        self.observer = Observer()
        self.observer.schedule(event_handler, path=os.path.dirname(
            data_file_path), recursive=False)
        self.observer.start()

        self.get_logger().info(f"正在监控robot_data.xlsx文件: {data_file_path}")

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
            all_successful = True
            for idx, result in enumerate(response.results):
                if not result.successful:
                    all_successful = False
                    self.get_logger().error(
                        f"参数设置失败 (参数 {idx+1}): {result.reason}")
            if all_successful:
                self.get_logger().info("已成功更新 robot_description 参数。")
            else:
                self.get_logger().error("部分参数设置失败。")
        except Exception as e:
            self.get_logger().error(f"调用 SetParameters 服务失败: {e}")

    def destroy_node(self):
        self.observer.stop()
        self.observer.join()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = URDFGeneratorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("正在关闭 URDF Monitor 节点。")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
