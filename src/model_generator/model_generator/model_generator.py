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


class XMLGenerator:
    def __init__(self, output_file_path, data_file_path, mesh_folder_path, xml_template_file_path, body_template_file_path, joint_template_file_path, asset_template_file_path, visual_geom_template_file_path, collision_geom_template_file_path, actuator_template_file_path, sensor_template_file_path):
        self.xml_output_file = output_file_path
        self.data_excel_file_path = data_file_path
        self.mesh_folder = mesh_folder_path
        self.xml_template_file = xml_template_file_path
        self.body_template_file = body_template_file_path
        self.joint_template_file = joint_template_file_path
        self.asset_template_file = asset_template_file_path
        self.visual_geom_template_file = visual_geom_template_file_path
        self.collision_geom_template_file = collision_geom_template_file_path
        self.actuator_template_file = actuator_template_file_path
        self.sensor_template_file = sensor_template_file_path

    def add_spaces_to_string(self, text, num_spaces):
        # 生成指定数量的空格
        spaces = ' ' * num_spaces

        # 将输入字符串按行分割，然后为每行添加空格
        lines = text.splitlines()
        modified_lines = [spaces + line for line in lines]

        # 将处理后的行重新组合成一个字符串，并保留原有的换行符
        return '\n'.join(modified_lines)

    def xml_generator(self):
        # 获取所有工作表名称
        data_excel_file = pd.ExcelFile(
            self.data_excel_file_path)
        sheet_names = data_excel_file.sheet_names
        xml_text = ''
        joint_index = 0
        space_dict = {}
        # 遍历所有工作表
        for index, sheet_name in enumerate(sheet_names):
            df = pd.read_excel(data_excel_file, sheet_name=sheet_name,
                               header=None, dtype=str)

            if sheet_name == 'base':

                robot_name = str(df.iloc[0, 1])
                base_name = str(df.iloc[1, 1])
                space_dict[base_name] = 8
                com = str(df.iloc[3, 2])+" " + \
                    str(df.iloc[3, 3])+" "+str(df.iloc[3, 4])
                mass = str(df.iloc[4, 2])
                inertia = str(df.iloc[5, 2])+' '+str(df.iloc[6, 3])+' '+str(df.iloc[7, 4]) + ' ' + \
                    str(df.iloc[5, 3])+' '+str(df.iloc[5, 4]) + \
                    ' '+str(df.iloc[6, 4])
                link_mesh = str(df.iloc[8, 1])
                collision_mesh = str(df.iloc[9, 1])
                link_geom = ''
                collision_geom = ''

                if ':' in link_mesh:
                    parts = link_mesh.split(':', 1)
                    link_geom = f"type = '{parts[0]}' size = '{parts[1]}'"
                elif '.' in link_mesh:
                    link_geom = f"type = 'mesh' mesh = '{base_name}_mesh_visual'"
                if ':' in collision_mesh:
                    parts = collision_mesh.split(':', 1)
                    collision_geom = f"type = '{parts[0]}' size = '{parts[1]}'"
                elif '.' in collision_mesh:
                    collision_geom = f"type = 'mesh' mesh = '{base_name}_mesh_collision'"
                if str(df.iloc[9, 1]) == "" or str(df.iloc[9, 1]) == "nan" or str(df.iloc[9, 1]) == "-":
                    collision_mesh = ""
                with open(self.xml_template_file, 'r', encoding='utf-8') as file:
                    xml_text = file.read()
                    xml_text = xml_text.replace(
                        'template_robot_name', robot_name)
                    xml_text = xml_text.replace(
                        'template_base_link_name', base_name)
                    xml_text = xml_text.replace(
                        'template_mesh_path', self.mesh_folder)
                if not ':' in link_mesh:
                    with open(self.asset_template_file, 'r', encoding='utf-8') as file:
                        asset_text = file.read()
                        asset_text = asset_text.replace(
                            'template_mesh_name', base_name+'_mesh_visual')
                        asset_text = asset_text.replace(
                            'template_mesh_file_path', link_mesh)
                        asset_text = asset_text.replace(
                            'template_mesh_scale', '1 1 1')
                        xml_text = xml_text.replace(
                            '<!-- asset auto generate -->', asset_text)
                if not ':' in collision_mesh:
                    with open(self.asset_template_file, 'r', encoding='utf-8') as file:
                        asset_text = file.read()
                        asset_text = asset_text.replace(
                            'template_mesh_name', base_name+'_mesh_collision')
                        asset_text = asset_text.replace(
                            'template_mesh_file_path', collision_mesh)
                        asset_text = asset_text.replace(
                            'template_mesh_scale', '1 1 1')
                        xml_text = xml_text.replace(
                            '<!-- asset auto generate -->', asset_text)
                with open(self.body_template_file, 'r', encoding='utf-8') as file:
                    body_text = file.read()
                    body_text = body_text.replace(
                        'template_body_name', base_name)
                    body_text = body_text.replace(
                        'template_body_pos', '0 0 0')
                    body_text = body_text.replace(
                        'template_body_rpy', '0 0 0')
                    body_text = body_text.replace(
                        'template_body_com', com)
                    body_text = body_text.replace(
                        'template_body_mass', mass)
                    body_text = body_text.replace(
                        'template_body_inertia', inertia)
                    body_text = body_text.replace(
                        f'<!-- {base_name} joint -->', '<freejoint />'+'\n'+'    <site name = "imu" size = "0.01" pos = "0 0 0" />')
                    body_text = body_text.replace(
                        '<!-- template_parent_body_name child body -->', '<!-- world child body -->')
                    xml_text = xml_text.replace(
                        '<!-- world child body -->', self.add_spaces_to_string(body_text, 8))
                with open(self.visual_geom_template_file, 'r', encoding='utf-8') as file:
                    mesh_text = file.read()
                    mesh_text = mesh_text.replace(
                        'template_geom_mesh', link_geom)
                    mesh_text = mesh_text.replace(
                        'template_geom_color', base_color)
                    xml_text = xml_text.replace(
                        f'<!-- {base_name} visual geom -->', mesh_text)
                with open(self.collision_geom_template_file, 'r', encoding='utf-8') as file:
                    mesh_text = file.read()
                    mesh_text = mesh_text.replace(
                        'template_geom_mesh', collision_geom)
                    xml_text = xml_text.replace(
                        f'<!-- {base_name} collision geom -->', mesh_text)
            else:
                # print(f"正在处理工作表：{sheet_name}")
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
                        inertia = str(df.iloc[i+4, 2])+' '+str(df.iloc[i+5, 3])+' '+str(df.iloc[i+6, 4])+' '+str(
                            df.iloc[i+4, 3])+' '+str(df.iloc[i+4, 4])+' '+str(df.iloc[i+5, 4])
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
                        space_dict[link_name] = space_dict[parent_link]+4
                        joint_type = str(df.iloc[i+15, 1])
                        limit = 'true'
                        if joint_type == 'revolute':
                            joint_type = 'hinge'
                            limit = 'true'
                        elif joint_type == 'continuous':
                            joint_type = 'hinge'
                            limit = 'false'
                        elif joint_type == 'prismatic':
                            joint_type = 'slide'
                            limit = 'true'
                        elif joint_type == 'ball':
                            limit = 'false'
                        link_mesh = str(df.iloc[i+16, 1])
                        collision_mesh = str(df.iloc[i+17, 1])
                        link_geom = ''
                        collision_geom = ''

                        if ':' in link_mesh:
                            parts = link_mesh.split(':', 1)
                            link_geom = f"type = '{parts[0]}' size = '{parts[1]}'"
                        elif '.' in link_mesh:
                            link_geom = f"type = 'mesh' mesh = '{link_name}_mesh_visual'"
                        if ':' in collision_mesh:
                            parts = collision_mesh.split(':', 1)
                            collision_geom = f"type = '{parts[0]}' size = '{parts[1]}'"
                        elif '.' in collision_mesh:
                            collision_geom = f"type = 'mesh' mesh = '{link_name}_mesh_collision'"

                        if str(df.iloc[i+17, 1]) == "" or str(df.iloc[i+17, 1]) == "nan" or str(df.iloc[i+17, 1]) == "-":
                            collision_mesh = ""
                        with open(self.body_template_file, 'r', encoding='utf-8') as file:
                            body_text = file.read()
                            body_text = body_text.replace(
                                'template_body_name', link_name)
                            body_text = body_text.replace(
                                'template_body_pos', joint_trans)
                            body_text = body_text.replace(
                                'template_body_rpy', joint_rpy)
                            body_text = body_text.replace(
                                'template_body_com', com)
                            body_text = body_text.replace(
                                'template_body_mass', mass)
                            body_text = body_text.replace(
                                'template_body_inertia', inertia)
                            body_text = body_text.replace(
                                '<!-- template_body_name child body -->', f'<!-- {link_name} child body -->')
                            body_text = body_text.replace(
                                '<!-- template_parent_body_name child body -->', f'<!-- {parent_link} child body -->')
                            xml_text = xml_text.replace(
                                f'<!-- {parent_link} child body -->', self.add_spaces_to_string(body_text, space_dict[link_name]))
                        if not ':' in link_mesh:
                            with open(self.asset_template_file, 'r', encoding='utf-8') as file:
                                asset_text = file.read()
                                asset_text = asset_text.replace(
                                    'template_mesh_name', link_name+'_mesh_visual')
                                asset_text = asset_text.replace(
                                    'template_mesh_file_path', link_mesh)
                                asset_text = asset_text.replace(
                                    'template_mesh_scale', '1 1 1')
                                xml_text = xml_text.replace(
                                    '<!-- asset auto generate -->', asset_text)
                        if not ':' in collision_mesh:
                            with open(self.asset_template_file, 'r', encoding='utf-8') as file:
                                if not collision_mesh == '':
                                    asset_text = file.read()
                                    asset_text = asset_text.replace(
                                        'template_mesh_name', link_name+'_mesh_collision')
                                    asset_text = asset_text.replace(
                                        'template_mesh_file_path', collision_mesh)
                                    asset_text = asset_text.replace(
                                        'template_mesh_scale', '1 1 1')
                                    xml_text = xml_text.replace(
                                        '<!-- asset auto generate -->', asset_text)
                        if not joint_type == 'fixed':
                            with open(self.joint_template_file, 'r', encoding='utf-8') as file:
                                joint_text = file.read()
                                joint_text = joint_text.replace(
                                    'template_joint_name', joint_name)
                                joint_text = joint_text.replace(
                                    'template_joint_type', joint_type)
                                joint_text = joint_text.replace(
                                    'template_joint_axis', joint_axis)
                                joint_text = joint_text.replace(
                                    'template_joint_limit', limit)
                                joint_text = joint_text.replace(
                                    'template_joint_lower_limit', joint_lower_limit)
                                joint_text = joint_text.replace(
                                    'template_joint_upper_limit', joint_upper_limit)
                                xml_text = xml_text.replace(
                                    f'<!-- {link_name} joint -->', joint_text)
                        with open(self.visual_geom_template_file, 'r', encoding='utf-8') as file:
                            geom_text = file.read()
                            geom_text = geom_text.replace(
                                'template_geom_mesh', link_geom)
                            geom_text = geom_text.replace(
                                'template_geom_color', link_color_list[joint_color_index % len(link_color_list) - 1])
                            xml_text = xml_text.replace(
                                f'<!-- {link_name} visual geom -->', geom_text)
                        if not collision_mesh == '':
                            with open(self.collision_geom_template_file, 'r', encoding='utf-8') as file:
                                geom_text = file.read()
                                geom_text = geom_text.replace(
                                    'template_geom_mesh', collision_geom)
                                xml_text = xml_text.replace(
                                    f'<!-- {link_name} collision geom -->', geom_text)
                        if joint_type == "hinge" or joint_type == "slide":
                            with open(self.actuator_template_file, 'r', encoding='utf-8') as file:
                                actuator_text = file.read()
                                actuator_text = actuator_text.replace(
                                    'template_motor_name', 'motor_'+joint_name)
                                actuator_text = actuator_text.replace(
                                    'template_joint_name', joint_name)
                                actuator_text = actuator_text.replace(
                                    'template_effort_limit', joint_effort)
                                xml_text = xml_text.replace(
                                    '<!-- actuator auto generate -->', actuator_text)

                            with open(self.sensor_template_file, 'r', encoding='utf-8') as file:
                                sensor_text = file.read()
                                sensor_text = sensor_text.replace(
                                    'template_pos_sensor_name', 'jointpos_'+joint_name)
                                sensor_text = sensor_text.replace(
                                    'template_vel_sensor_name', 'jointvel_'+joint_name)
                                sensor_text = sensor_text.replace(
                                    'template_joint_name', joint_name)
                                xml_text = xml_text.replace(
                                    '<!-- sensor auto generate -->', sensor_text)

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
                            mirror_axis = str(df.iloc[i+18, 1])
                            mirror_stl_axis = str(df.iloc[i+19, 1])
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
                            inertia = str(df.iloc[i+4, 2])+' '+str(df.iloc[i+5, 3])+' '+str(df.iloc[i+6, 4])+' '+str(
                                stl_coe_x*stl_coe_y *
                                float(df.iloc[i+4, 3]))+' '+str(stl_coe_x*stl_coe_z *
                                                                float(df.iloc[i+4, 4]))+' '+str(stl_coe_y*stl_coe_z *
                                                                                                float(df.iloc[i+5, 4]))
                            # joint
                            joint_name = "idx" + \
                                f'{joint_index:02}'+"_"+str(link_name)
                            joint_trans = str(
                                coe_x*float(df.iloc[i+7, 2]))+" "+str(coe_y*float(df.iloc[i+7, 3]))+" "+str(coe_z*float(df.iloc[i+7, 4]))
                            joint_rpy = str(df.iloc[i+8, 2])+" " + \
                                str(df.iloc[i+8, 3]) + \
                                " "+str(df.iloc[i+8, 4])
                            joint_axis = str(
                                df.iloc[i+9, 2])+" "+str(df.iloc[i+9, 3])+" "+str(df.iloc[i+9, 4])
                            joint_lower_limit = str(df.iloc[i+10, 2])
                            joint_upper_limit = str(df.iloc[i+11, 2])
                            if mirror_axis == 'Z':
                                joint_lower_limit = -str(df.iloc[i+11, 2])
                                joint_upper_limit = -str(df.iloc[i+10, 2])
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
                            space_dict[link_name] = space_dict[parent_link]+4
                            joint_type = str(df.iloc[i+15, 1])
                            limit = 'true'
                            if joint_type == 'revolute':
                                joint_type = 'hinge'
                                limit = 'true'
                            elif joint_type == 'continuous':
                                joint_type = 'hinge'
                                limit = 'false'
                            elif joint_type == 'prismatic':
                                joint_type = 'slide'
                                limit = 'true'
                            elif joint_type == 'ball':
                                joint_type = 'slide'
                                limit = 'false'
                            link_mesh = str(df.iloc[i+16, 1])
                            collision_mesh = str(df.iloc[i+17, 1])
                            if ':' in link_mesh:
                                parts = link_mesh.split(':', 1)
                                link_geom = f"type = '{parts[0]}' size = '{parts[1]}'"
                            elif '.' in link_mesh:
                                link_geom = f"type = 'mesh' mesh = '{link_name}_mesh_visual'"
                            if ':' in collision_mesh:
                                parts = collision_mesh.split(':', 1)
                                collision_geom = f"type = '{parts[0]}' size = '{parts[1]}'"
                            elif '.' in collision_mesh:
                                collision_geom = f"type = 'mesh' mesh = '{link_name}_mesh_collision'"
                            if str(df.iloc[i+17, 1]) == "" or str(df.iloc[i+17, 1]) == "nan" or str(df.iloc[i+17, 1]) == "-":
                                collision_mesh = ""
                            with open(self.body_template_file, 'r', encoding='utf-8') as file:
                                body_text = file.read()
                                body_text = body_text.replace(
                                    'template_body_name', link_name)
                                body_text = body_text.replace(
                                    'template_body_pos', joint_trans)
                                body_text = body_text.replace(
                                    'template_body_rpy', joint_rpy)
                                body_text = body_text.replace(
                                    'template_body_com', com)
                                body_text = body_text.replace(
                                    'template_body_mass', mass)
                                body_text = body_text.replace(
                                    'template_body_inertia', inertia)
                                body_text = body_text.replace(
                                    '<!-- template_body_name child body -->', f'<!-- {link_name} child body -->')
                                body_text = body_text.replace(
                                    '<!-- template_parent_body_name child body -->', f'<!-- {parent_link} child body -->')
                                xml_text = xml_text.replace(
                                    f'<!-- {parent_link} child body -->', self.add_spaces_to_string(body_text, space_dict[link_name]))
                            if not ':' in link_mesh:
                                with open(self.asset_template_file, 'r', encoding='utf-8') as file:
                                    asset_text = file.read()
                                    asset_text = asset_text.replace(
                                        'template_mesh_name', link_name+'_mesh_visual')
                                    asset_text = asset_text.replace(
                                        'template_mesh_file_path', link_mesh)
                                    asset_text = asset_text.replace(
                                        'template_mesh_scale', str(stl_coe_x)+" " + str(stl_coe_y) + " "+str(stl_coe_z))
                                    xml_text = xml_text.replace(
                                        '<!-- asset auto generate -->', asset_text)
                            if not ':' in collision_mesh:
                                with open(self.asset_template_file, 'r', encoding='utf-8') as file:
                                    if not collision_mesh == '':
                                        asset_text = file.read()
                                        asset_text = asset_text.replace(
                                            'template_mesh_name', link_name+'_mesh_collision')
                                        asset_text = asset_text.replace(
                                            'template_mesh_file_path', collision_mesh)
                                        asset_text = asset_text.replace(
                                            'template_mesh_scale', str(stl_coe_x)+" " + str(stl_coe_y) + " "+str(stl_coe_z))
                                        xml_text = xml_text.replace(
                                            '<!-- asset auto generate -->', asset_text)
                            if not joint_type == 'fixed':
                                with open(self.joint_template_file, 'r', encoding='utf-8') as file:
                                    joint_text = file.read()
                                    joint_text = joint_text.replace(
                                        'template_joint_name', joint_name)
                                    joint_text = joint_text.replace(
                                        'template_joint_type', joint_type)
                                    joint_text = joint_text.replace(
                                        'template_joint_axis', joint_axis)
                                    joint_text = joint_text.replace(
                                        'template_joint_limit', limit)
                                    joint_text = joint_text.replace(
                                        'template_joint_lower_limit', joint_lower_limit)
                                    joint_text = joint_text.replace(
                                        'template_joint_upper_limit', joint_upper_limit)
                                    xml_text = xml_text.replace(
                                        f'<!-- {link_name} joint -->', joint_text)
                            with open(self.visual_geom_template_file, 'r', encoding='utf-8') as file:
                                geom_text = file.read()
                                geom_text = geom_text.replace(
                                    'template_geom_mesh', link_geom)
                                geom_text = geom_text.replace(
                                    'template_geom_color', link_color_list[joint_color_index % len(link_color_list) - 1])
                                xml_text = xml_text.replace(
                                    f'<!-- {link_name} visual geom -->', geom_text)
                            if not collision_mesh == '':
                                with open(self.collision_geom_template_file, 'r', encoding='utf-8') as file:
                                    geom_text = file.read()
                                    geom_text = geom_text.replace(
                                        'template_geom_mesh', collision_geom)
                                    xml_text = xml_text.replace(
                                        f'<!-- {link_name} collision geom -->', geom_text)
                            if joint_type == "hinge" or joint_type == "slide":
                                with open(self.actuator_template_file, 'r', encoding='utf-8') as file:
                                    actuator_text = file.read()
                                    actuator_text = actuator_text.replace(
                                        'template_motor_name', 'motor_'+joint_name)
                                    actuator_text = actuator_text.replace(
                                        'template_joint_name', joint_name)
                                    actuator_text = actuator_text.replace(
                                        'template_effort_limit', joint_effort)
                                    xml_text = xml_text.replace(
                                        '<!-- actuator auto generate -->', actuator_text)

                                with open(self.sensor_template_file, 'r', encoding='utf-8') as file:
                                    sensor_text = file.read()
                                    sensor_text = sensor_text.replace(
                                        'template_pos_sensor_name', 'jointpos_'+joint_name)
                                    sensor_text = sensor_text.replace(
                                        'template_vel_sensor_name', 'jointvel_'+joint_name)
                                    sensor_text = sensor_text.replace(
                                        'template_joint_name', joint_name)
                                    xml_text = xml_text.replace(
                                        '<!-- sensor auto generate -->', sensor_text)

        directory = os.path.dirname(self.xml_output_file)
        if not os.path.exists(directory):
            os.makedirs(directory)
        with open(self.xml_output_file, 'w') as file:
            file.write(xml_text)


class URDFGenerator:
    def __init__(self, data_file_path, mesh_folder_path, output_folder_path, urdf_template_file_path, link_template_file_path, joint_template_file_path):
        self.urdf_output_file = output_folder_path
        self.data_excel_file_path = data_file_path
        self.mesh_folder = mesh_folder_path
        self.urdf_template_file = urdf_template_file_path
        self.link_template_file = link_template_file_path
        self.joint_template_file = joint_template_file_path

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
                link_mesh = ""
                collision_mesh = ""
                if ':' in str(df.iloc[8, 1]):
                    parts = str(df.iloc[8, 1]).split(':', 1)
                    if parts[0] == 'sphere':
                        link_mesh = f'<{parts[0]} radius="{parts[1]}" />'
                    else:
                        link_mesh = f'<{parts[0]} size="{parts[1]}" />'
                elif '.' in str(df.iloc[8, 1]):
                    link_mesh = f'<mesh filename="file://{self.mesh_folder+str(df.iloc[8, 1])}" scale="1 1 1"/>'
                if ':' in str(df.iloc[9, 1]):
                    parts = str(df.iloc[9, 1]).split(':', 1)
                    if parts[0] == 'sphere':
                        collision_mesh = f'<{parts[0]} radius="{parts[1]}" />'
                    else:
                        collision_mesh = f'<{parts[0]} size="{parts[1]}" />'
                elif '.' in str(df.iloc[9, 1]):
                    collision_mesh = f'<mesh filename="file://{self.mesh_folder+str(df.iloc[9, 1])}" scale="1 1 1"/>'

                if str(df.iloc[9, 1]) == "" or str(df.iloc[9, 1]) == "nan" or str(df.iloc[9, 1]) == "-":
                    collision_mesh = '<mesh filename="" scale="1 1 1"/>'
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
                    text = text.replace('template_visual_geometry', link_mesh)
                    text = text.replace(
                        'template_collision_geometry', collision_mesh)
                    text = text.replace('template_mesh_color', base_color)
                urdf_text = urdf_text.replace(
                    '<!-- urdf auto generate tool -->', text)
                # print(urdf_text)
            else:
                # print(f"正在处理工作表：{sheet_name}")
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

                        link_mesh = ""
                        collision_mesh = ""
                        if ':' in str(df.iloc[i+16, 1]):
                            parts = str(df.iloc[i+16, 1]).split(':', 1)
                            if parts[0] == 'sphere':
                                link_mesh = f'<{parts[0]} radius="{parts[1]}" />'
                            else:
                                link_mesh = f'<{parts[0]} size="{parts[1]}" />'
                        elif '.' in str(df.iloc[i+16, 1]):
                            link_mesh = f'<mesh filename="file://{self.mesh_folder+str(df.iloc[i+16, 1])}" scale="1 1 1"/>'
                        if ':' in str(df.iloc[i+17, 1]):
                            parts = str(df.iloc[i+17, 1]).split(':', 1)
                            if parts[0] == 'sphere':
                                collision_mesh = f'<{parts[0]} radius="{parts[1]}" />'
                            else:
                                collision_mesh = f'<{parts[0]} size="{parts[1]}" />'
                        elif '.' in str(df.iloc[i+17, 1]):
                            collision_mesh = f'<mesh filename="file://{self.mesh_folder+str(df.iloc[i+17, 1])}" scale="1 1 1"/>'

                        if str(df.iloc[i+17, 1]) == "" or str(df.iloc[i+17, 1]) == "nan" or str(df.iloc[i+17, 1]) == "-":
                            collision_mesh = '<mesh filename="" scale="1 1 1"/>'
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
                                'template_visual_geometry', link_mesh)
                            text = text.replace(
                                'template_collision_geometry', collision_mesh)
                            text = text.replace(
                                'template_mesh_color', link_color_list[joint_color_index % len(link_color_list) - 1])
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
                            mirror_axis = str(df.iloc[i+18, 1])
                            mirror_stl_axis = str(df.iloc[i+19, 1])
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
                            joint_rpy = str(df.iloc[i+8, 2])+" " + \
                                str(df.iloc[i+8, 3]) + \
                                " "+str(df.iloc[i+8, 4])
                            joint_axis = str(
                                df.iloc[i+9, 2])+" "+str(df.iloc[i+9, 3])+" "+str(df.iloc[i+9, 4])
                            joint_lower_limit = str(df.iloc[i+10, 2])
                            joint_upper_limit = str(df.iloc[i+11, 2])
                            if mirror_axis == 'Z':
                                joint_lower_limit = -str(df.iloc[i+11, 2])
                                joint_upper_limit = -str(df.iloc[i+10, 2])
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
                            link_mesh = ""
                            collision_mesh = ""
                            if ':' in str(df.iloc[i+16, 1]):
                                parts = str(df.iloc[i+16, 1]).split(':', 1)
                                if parts[0] == 'sphere':
                                    link_mesh = f'<{parts[0]} radius="{parts[1]}" />'
                                else:
                                    link_mesh = f'<{parts[0]} size="{parts[1]}" />'
                            elif '.' in str(df.iloc[i+16, 1]):
                                link_mesh = f'<mesh filename="file://{self.mesh_folder+str(df.iloc[i+16, 1])}" scale="{str(stl_coe_x)+" " + str(stl_coe_y) + " "+str(stl_coe_z)}"/>'
                            if ':' in str(df.iloc[i+17, 1]):
                                parts = str(df.iloc[i+17, 1]).split(':', 1)
                                if parts[0] == 'sphere':
                                    collision_mesh = f'<{parts[0]} radius="{parts[1]}" />'
                                else:
                                    collision_mesh = f'<{parts[0]} size="{parts[1]}" />'
                            elif '.' in str(df.iloc[i+17, 1]):
                                collision_mesh = f'<mesh filename="file://{self.mesh_folder+str(df.iloc[i+17, 1])}" scale="{str(stl_coe_x)+" " + str(stl_coe_y) + " "+str(stl_coe_z)}"/>'

                            if str(df.iloc[i+17, 1]) == "" or str(df.iloc[i+17, 1]) == "nan" or str(df.iloc[i+17, 1]) == "-":
                                collision_mesh = '<mesh filename="" scale="1 1 1"/>'
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
                                    'template_visual_geometry', link_mesh)
                                text = text.replace(
                                    'template_collision_geometry', collision_mesh)
                                text = text.replace(
                                    'template_mesh_color', link_color_list[joint_color_index % len(link_color_list) - 1])
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
            self.node.xml_generator.xml_generator()


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

        xml_output_file_path = output_folder_path+"/output.xml"
        xml_template_file_path = package_share_directory + \
            "/templates/xml/xml.template"
        body_template_file_path = package_share_directory + \
            "/templates/xml/body.template"
        xml_joint_template_file_path = package_share_directory + \
            "/templates/xml/joint.template"
        asset_template_file_path = package_share_directory + \
            "/templates/xml/asset.template"
        visual_geom_template_file_path = package_share_directory + \
            "/templates/xml/visual_geom.template"
        collision_geom_template_file_path = package_share_directory + \
            "/templates/xml/collision_geom.template"
        actuator_template_file_path = package_share_directory + \
            "/templates/xml/actuator.template"
        sensor_template_file_path = package_share_directory + \
            "/templates/xml/sensor.template"

        self.urdf_generator = URDFGenerator(data_file_path, mesh_folder_path, urdf_output_file_path,
                                            urdf_template_file_path, link_template_file_path, joint_template_file_path)
        self.urdf_generator.urdf_generator()
        self.xml_generator = XMLGenerator(
            xml_output_file_path, data_file_path, mesh_folder_path, xml_template_file_path, body_template_file_path, xml_joint_template_file_path, asset_template_file_path, visual_geom_template_file_path, collision_geom_template_file_path, actuator_template_file_path, sensor_template_file_path)
        self.xml_generator.xml_generator()

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
