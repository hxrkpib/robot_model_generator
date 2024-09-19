import pandas as pd

# 读取Excel文件
urdf_output_file = "/home/lizhen/works/code/mc_code_dev/model/0918/robot_model/resource/generator/model_generate/output/output.urdf"
data_excel_file = pd.ExcelFile(
    '/home/lizhen/works/urdf_template/robot_data.xlsx')
mesh_folder = "/home/lizhen/works/code/mc_code_dev/model/0918/robot_model/resource/meshes/raise_a2_t2d0/"
urdf_template_file = './templates/urdf/urdf.template'
link_template_file = './templates/urdf/link.template'
joint_template_file = './templates/urdf/joint.template'

base_color = "0.8 0.4 0 1"
link_color_list = ["1 0.4 0.4 1", "1 1 0.4 1",
                   "0.4 1 0.4 1", "0 0.5 1 1", "0.8 1.0 0.8 1", "1 0.5 1 1"]


# 获取所有工作表名称
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
        com = str(df.iloc[3, 2])+" "+str(df.iloc[3, 3])+" "+str(df.iloc[3, 4])
        mass = str(df.iloc[4, 2])
        ixx = str(df.iloc[5, 2])
        iyy = str(df.iloc[6, 3])
        izz = str(df.iloc[7, 4])
        ixy = str(df.iloc[5, 3])
        ixz = str(df.iloc[5, 4])
        iyz = str(df.iloc[6, 4])
        link_mesh = mesh_folder+str(df.iloc[8, 1])
        with open(urdf_template_file, 'r', encoding='utf-8') as file:
            urdf_text = file.read()
            urdf_text = urdf_text.replace('template_robot_name', robot_name)
        # print(urdf_text)
        with open(link_template_file, 'r', encoding='utf-8') as file:
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
        urdf_text = urdf_text.replace(
            '<!-- urdf auto generate tool -->', text)
        # print(urdf_text)
    else:
        print(f"正在处理工作表：{sheet_name}")
        # 获取第一列数据
        first_column = df.iloc[:, 0]

        # 找到第一列中最后一个非空单元格的行号
        last_valid_row = first_column.last_valid_index()

        print(f"第一列最后一个有内容的单元格行号是：{last_valid_row}")
        for i in range(last_valid_row):
            if df.iloc[i, 0] == 'Link Name':
                joint_index = joint_index+1
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
                joint_name = "idx"+f'{joint_index:02}'+"_"+str(link_name)
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
                link_mesh = mesh_folder+str(df.iloc[i+16, 1])

                with open(link_template_file, 'r', encoding='utf-8') as file:
                    text = file.read()
                    text = text.replace('template_link_name', link_name)
                    text = text.replace('template_link_com', com)
                    text = text.replace('template_link_mass', mass)
                    text = text.replace('template_ixx', ixx)
                    text = text.replace('template_iyy', iyy)
                    text = text.replace('template_izz', izz)
                    text = text.replace('template_ixy', ixy)
                    text = text.replace('template_ixz', ixz)
                    text = text.replace('template_iyz', iyz)
                    text = text.replace('template_mesh_file', link_mesh)
                    text = text.replace(
                        'template_mesh_color', link_color_list[joint_index % len(link_color_list) - 1])
                    urdf_text = urdf_text.replace(
                        '<!-- urdf auto generate tool -->', text)
                    # print(urdf_text)
                with open(joint_template_file, 'r', encoding='utf-8') as file:
                    text = file.read()
                    text = text.replace('template_joint_name', joint_name)
                    text = text.replace('template_joint_type', joint_type)
                    text = text.replace('template_joint_xyz', joint_trans)
                    text = text.replace('template_joint_rpy', joint_rpy)
                    text = text.replace('template_parent_link', parent_link)
                    text = text.replace('template_child_link', child_link)
                    text = text.replace('template_joint_axis', joint_axis)
                    text = text.replace(
                        'template_joint_lower_limit', joint_lower_limit)
                    text = text.replace(
                        'template_joint_upper_limit', joint_upper_limit)
                    text = text.replace('template_joint_effort', joint_effort)
                    text = text.replace(
                        'template_joint_velocity', joint_velocity)
                    urdf_text = urdf_text.replace(
                        '<!-- urdf auto generate tool -->', text)
                    # print(urdf_text)
print(urdf_text)
with open(urdf_output_file, 'w') as file:
    file.write(urdf_text)
