import os
import argparse
import numpy as np
import pinocchio as pin
import json  # 添加JSON模块用于TXT文件保存

def quat_apply(quat, vec):
    """
    使用四元数旋转向量
    quat: 四元数 [qx, qy, qz, qw] 
    vec: 3D向量 [x, y, z]
    返回: 旋转后的3D向量
    """
    quat_pin = pin.Quaternion(quat)  # qx, qy, qz, qw
    rotated_vec = quat_pin.matrix() @ vec
    return rotated_vec

def quat_conjugate(quat):
    """
    计算四元数的共轭
    quat: 四元数 [qx, qy, qz, qw]
    返回: 共轭四元数 [-qx, -qy, -qz, qw]
    """
    quat_conj = np.array([-quat[0], -quat[1], -quat[2], quat[3]])
    return quat_conj

class MotionProcessor:
    def __init__(self, args):
        self.args = args
        self.start_frame = args.start_frame
        self.end_frame = args.end_frame
        self.fps = 30  # 保留fps信息
        self.joint_names = []  # 关节名称列表（需为29个）
        self.link_names = []   # 连杆名称列表
        
        # 加载机器人模型
        current_dir = os.path.dirname(os.path.abspath(__file__))  # 获取当前脚本目录
        # 使用相对路径（相对于当前motions目录）指向 ymbot_e 的 URDF 与 mesh 目录
        # 当前文件位于: legged_lab/assets/ymbot_e/motions
        urdf_path = os.path.normpath(os.path.join(current_dir, '..', 'urdf', 'ymboy-23dof.urdf'))
        mesh_dir = os.path.normpath(os.path.join(current_dir, '..', 'urdf'))
        model, collision_model, visual_model = pin.buildModelsFromUrdf(
            urdf_path,  # 使用相对路径
            mesh_dir,
            pin.JointModelFreeFlyer()
        )
        self.robot = pin.RobotWrapper(model, collision_model, visual_model)

        # 显式定义G1的29个关节名称（与CSV中29个关节弧度顺序对应）
        self.joint_names = [
            "left_hip_pitch_joint",               # 1
            "left_hip_roll_joint",                # 2
            "left_hip_yaw_joint",                 # 3
            "left_knee_joint",                    # 4
            "left_ankle_pitch_joint",             # 5
            "left_ankle_roll_joint",              # 6
            "right_hip_pitch_joint",              # 7
            "right_hip_roll_joint",               # 8  
            "right_hip_yaw_joint",                # 9
            "right_knee_joint",                   # 10
            "right_ankle_pitch_joint",            # 11
            "right_ankle_roll_joint",             # 12
            "waist_yaw_joint",                    # 13
            "left_shoulder_pitch_joint",          # 14
            "left_shoulder_roll_joint",           # 15
            "left_shoulder_yaw_joint",            # 16
            "left_elbow_joint",                   # 17
            "left_wrist_yaw_joint",               # 18
            "right_shoulder_pitch_joint",         # 19
            "right_shoulder_roll_joint",          # 20
            "right_shoulder_yaw_joint",           # 21
            "right_elbow_joint",                  # 22
            "right_wrist_yaw_joint",              # 23
        ]

        self.target_link_names = [  # 用户手动设置需要处理的连杆名称（示例）
            "pelvis",                  
            "left_wrist_yaw_link",
            "right_wrist_yaw_link", 
            "left_ankle_roll_link", 
            "right_ankle_roll_link"
        ]
        
        # 手动过滤连杆（仅保留target_link_names中的连杆）
        self.link_names = []
        valid_frame_ids = []
        all_frame_names = [self.robot.model.frames[i].name for i in range(self.robot.model.nframes)]
        
        # 检查目标连杆是否存在
        for name in self.target_link_names:
            if name not in all_frame_names:
                print(f"警告：目标连杆 [{name}] 不存在于机器人模型中（可用连杆: {all_frame_names}）")
        
        # 遍历所有框架，仅保留target_link_names中的BODY类型框架
        for i in range(self.robot.model.nframes):
            frame = self.robot.model.frames[i]
            if frame.type == pin.FrameType.BODY and frame.name in self.target_link_names:
                valid_frame_ids.append(i)
                self.link_names.append(frame.name)
                print(f"  保留手动指定的连杆: {frame.name}")
        
        self.valid_frame_ids = valid_frame_ids
        print(f"最终有效连杆列表（手动过滤后）: {self.link_names}")


    def load_data(self):
        """加载CSV运动数据"""
        # 提取纯文件名（去掉路径和扩展名）
        file_basename = os.path.splitext(os.path.basename(self.args.file_path))[0]
        # 构建CSV路径（支持绝对路径和相对路径输入）
        if os.path.isabs(self.args.file_path):
            csv_path = self.args.file_path
        else:
            # 现在脚本在motions目录下，CSV文件也在同一目录
            csv_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), self.args.file_path)
        data = np.genfromtxt(csv_path, delimiter=',')
        return data

    def process_and_save_data(self):
        """处理运动数据并保存为TXT文件"""
        motion_data = self.load_data()
        if motion_data.size == 0:
            raise ValueError("未加载到有效数据, 请检查CSV文件路径或格式")
        
        # 处理帧范围截取
        total_frames = motion_data.shape[0]
        start = self.start_frame
        end = self.end_frame if self.end_frame != -1 else total_frames
        
        # 验证帧范围有效性
        if start < 0 or end > total_frames or start >= end:
            raise ValueError(f"无效的帧范围: start_frame={start}, end_frame={end}, 总帧数={total_frames}")
        
        # 截取指定范围的帧数据
        motion_data = motion_data[start:end]
        print(f"已截取帧数据: 从{start}到{end}（共{len(motion_data)}帧）")
        
        # 可选：对根关节位置进行归零处理（建议开启以提高AMP训练稳定性）
        normalize_root_position = self.args.normalize_position  # 使用命令行参数控制
        if normalize_root_position and len(motion_data) > 0:
            # 以第一帧的根关节x,y位置为基准点进行归零（保持z坐标和旋转不变）
            first_frame_root_xy = motion_data[0, :2]  # 取第一帧的x,y位置
            motion_data[:, 0] -= first_frame_root_xy[0]  # x坐标归零
            motion_data[:, 1] -= first_frame_root_xy[1]  # y坐标归零
            print(f"已将根关节x,y位置归零: 第一帧偏移=({first_frame_root_xy[0]:.3f}, {first_frame_root_xy[1]:.3f})")
        
        joint_positions = []
        root_positions = []  # 新增：存储根关节位置（x,y,z,qx,qy,qz,qw）
        num_joints = len(self.joint_names)  # 应为29
        
        # 遍历每一帧数据，提取根关节和关节弧度
        for frame in motion_data:
            # 验证帧长度：根关节(7) + 关节弧度(29) = 36个数据
            if len(frame) < 7 + num_joints:
                raise ValueError(f"CSV帧数据长度不足, 期望36个数据(7个根关节+29个关节弧度), 实际{len(frame)}个")
            
            # 提取根关节位置（前7个数据）和关节弧度（后29个数据）
            root_pos = frame[:7]  # 新增
            joint_pos = frame[7 : 7 + num_joints]
            root_positions.append(root_pos)  # 新增
            joint_positions.append(joint_pos)
        
        # 计算关节速度 (dof_velocities) 和根关节速度 (root_velocities)
        dt = 1.0 / self.fps
        joint_velocities = []
        root_velocities = []  # 新增：存储根关节速度（线速度3 + 角速度3）
        
        for i in range(len(joint_positions)):
            # 计算关节速度
            if i == 0:
                joint_vel = np.zeros_like(joint_positions[i])
            else:
                joint_vel = (joint_positions[i] - joint_positions[i-1]) / dt
            joint_velocities.append(joint_vel)
            
            # 计算根关节速度（新增）
            if i == 0:
                root_vel = np.zeros(6)  # 线速度3 + 角速度3
            else:
                # 线速度：位置差分（x,y,z）
                pos_current = root_positions[i][:3]
                pos_prev = root_positions[i-1][:3]
                lin_vel = (pos_current - pos_prev) / dt
                
                # 角速度：通过AngleAxis获取旋转向量（关键修改）
                quat_current = pin.Quaternion(root_positions[i][3:7])  # 四元数顺序：qx,qy,qz,qw
                quat_prev = pin.Quaternion(root_positions[i-1][3:7])
                quat_diff = quat_current * quat_prev.inverse()  # 计算四元数差（当前相对于前一帧的旋转）
                
                # 使用AngleAxis获取旋转向量（角度*轴）
                angle_axis = pin.AngleAxis(quat_diff)  # 构造AngleAxis对象
                ang_vel = angle_axis.angle * angle_axis.axis / dt  # 关键修改：去掉属性后的括号
                
                root_vel = np.concatenate([lin_vel, ang_vel])
            root_velocities.append(root_vel)  # 注意：此行需要正确缩进（原代码缩进错误）
        
        # 初始化存储连杆数据的列表
        body_positions = []        # (N, B, 3)
        body_rotations = []        # (N, B, 4) - wxyz四元数
        body_linear_velocities = [] # (N, B, 3)
        body_angular_velocities = [] # (N, B, 3)
        
        # 为每个帧计算连杆信息
        for i in range(len(joint_positions)):
            # 设置机器人配置（根关节+关节位置）
            q = np.concatenate([root_positions[i], joint_positions[i]])  # 使用存储的root_positions
            v = np.concatenate([root_velocities[i], joint_velocities[i]])  # 完整速度向量（关键修改）
            
            # 前向运动学计算（显式更新关节和框架位姿）
            self.robot.forwardKinematics(q, v)  # 新增：传入速度v以计算导数（关键修复）
            self.robot.framesForwardKinematics(q)
            pin.updateFramePlacements(self.robot.model, self.robot.data)

            # 计算所有连杆的位置和姿态（保持原有逻辑）
            current_body_pos = []
            current_body_rot = []
            current_body_lin_vel = []
            current_body_ang_vel = []
            
            for frame_id in self.valid_frame_ids:
                # 位置和旋转保持原有计算逻辑
                pos = self.robot.data.oMf[frame_id].translation.copy()
                current_body_pos.append(pos)
                
                rot = self.robot.data.oMf[frame_id].rotation.copy()
                quat = pin.Quaternion(rot).coeffs()
                quat = quat[[3, 0, 1, 2]]  # 转换为wxyz格式
                current_body_rot.append(quat)
                
                # 速度计算修改（使用pin.getFrameVelocity获取框架速度）
                # 获取连杆在世界坐标系下的速度（关键修复）
                frame_vel = pin.getFrameVelocity(
                    self.robot.model, 
                    self.robot.data, 
                    frame_id, 
                    pin.ReferenceFrame.WORLD  # 指定参考坐标系为世界坐标系
                )  # 替换原data.vf访问方式
                lin_vel = frame_vel.linear.copy()
                ang_vel = frame_vel.angular.copy()
                
                current_body_lin_vel.append(lin_vel)
                current_body_ang_vel.append(ang_vel)
            
            body_positions.append(current_body_pos)
            body_rotations.append(current_body_rot)
            body_linear_velocities.append(current_body_lin_vel)
            body_angular_velocities.append(current_body_ang_vel)
        
        # 指定需要导出位置的连杆名称（按照AMP顺序：left_hand, right_hand, left_foot, right_foot）
        target_links = ["left_wrist_yaw_link", "right_wrist_yaw_link", "left_ankle_roll_link", "right_ankle_roll_link"]

        # 获取目标连杆的索引
        target_link_indices = []
        for link_name in target_links:
            try:
                link_index = self.link_names.index(link_name)
                target_link_indices.append(link_index)
                print(f"找到目标连杆: {link_name} (索引: {link_index})")
            except ValueError:
                raise ValueError(f"目标连杆 {link_name} 不存在于模型中。可用连杆: {self.link_names}")
        
        # 构建每一帧的数据：29个关节位置 + 29个关节速度 + 12维连杆位置
        frames_data = []
        
        # 创建从CSV索引到AMP索引的映射（用于最终输出时的转换）
        csv_joint_names = [
            "left_hip_pitch_joint",               # 1
            "left_hip_roll_joint",                # 2
            "left_hip_yaw_joint",                 # 3
            "left_knee_joint",                    # 4
            "left_ankle_pitch_joint",             # 5
            "left_ankle_roll_joint",              # 6
            "right_hip_pitch_joint",              # 7
            "right_hip_roll_joint",               # 8  
            "right_hip_yaw_joint",                # 9
            "right_knee_joint",                   # 10
            "right_ankle_pitch_joint",            # 11
            "right_ankle_roll_joint",             # 12
            "waist_yaw_joint",                    # 13
            "left_shoulder_pitch_joint",          # 14
            "left_shoulder_roll_joint",           # 15
            "left_shoulder_yaw_joint",            # 16
            "left_elbow_joint",                   # 17
            "left_wrist_yaw_joint",               # 18
            "right_shoulder_pitch_joint",         # 19
            "right_shoulder_roll_joint",          # 20
            "right_shoulder_yaw_joint",           # 21
            "right_elbow_joint",                  # 22
            "right_wrist_yaw_joint",              # 23
        ]
        
        # AMP顺序的关节名称
        amp_joint_names = [
            # 腰部关节 (1个)
            "waist_yaw_joint",
            # 右臂关节 (5个)
            "right_shoulder_pitch_joint",
            "right_shoulder_roll_joint",
            "right_shoulder_yaw_joint",
            "right_elbow_joint",
            "right_wrist_yaw_joint",
            # 左臂关节 (5个)
            "left_shoulder_pitch_joint", 
            "left_shoulder_roll_joint", 
            "left_shoulder_yaw_joint",
            "left_elbow_joint", 
            "left_wrist_yaw_joint",
            # 右腿关节 (6个)
            "right_hip_pitch_joint",
            "right_hip_roll_joint",
            "right_hip_yaw_joint",
            "right_knee_joint",
            "right_ankle_pitch_joint",
            "right_ankle_roll_joint",
            # 左腿关节 (6个)
            "left_hip_pitch_joint",
            "left_hip_roll_joint",
            "left_hip_yaw_joint",
            "left_knee_joint",
            "left_ankle_pitch_joint",
            "left_ankle_roll_joint",
        ]
        
        # 创建从CSV索引到AMP索引的映射
        csv_to_amp_mapping = []
        for amp_joint in amp_joint_names:
            csv_idx = csv_joint_names.index(amp_joint)
            csv_to_amp_mapping.append(csv_idx)
        
        for i in range(len(joint_positions)):
            frame_data = []
            
            # 将CSV顺序的关节位置转换为AMP顺序
            amp_joint_pos = np.zeros_like(joint_positions[i])
            for amp_idx, csv_idx in enumerate(csv_to_amp_mapping):
                amp_joint_pos[amp_idx] = joint_positions[i][csv_idx]
            
            # 将CSV顺序的关节速度转换为AMP顺序
            amp_joint_vel = np.zeros_like(joint_velocities[i])
            for amp_idx, csv_idx in enumerate(csv_to_amp_mapping):
                amp_joint_vel[amp_idx] = joint_velocities[i][csv_idx]
            
            # 添加29个关节位置（AMP顺序）
            frame_data.extend(amp_joint_pos)
            
            # 添加29个关节速度（AMP顺序）
            frame_data.extend(amp_joint_vel)
            
            # 添加4个目标连杆的相对于根坐标系的位置（每个连杆3维，共12维）
            # 按照AMP顺序：left_hand, right_hand, left_foot, right_foot
            
            # 获取根连杆的位置和旋转（与CSV数据中的根关节位置保持一致）
            root_world_pos = root_positions[i][:3]  # 根关节世界位置 (x,y,z)
            root_world_quat = root_positions[i][3:7]  # 根关节世界四元数 (qx,qy,qz,qw)
            
            for link_idx in target_link_indices:
                # 获取目标连杆在世界坐标系中的位置
                link_world_pos = body_positions[i][link_idx]
                
                # 计算世界坐标系下的相对位置 (world_offset)
                world_offset = link_world_pos - root_world_pos
                
                # 关键修改：将世界坐标系下的偏移转换为机器人局部坐标系
                # 使用根关节四元数的共轭来将世界坐标转换为局部坐标
                root_quat_conj = quat_conjugate(root_world_quat)  # 计算根关节四元数的共轭
                local_offset = quat_apply(root_quat_conj, world_offset)  # 将世界偏移转换为局部偏移
                
                # 添加机器人局部坐标系下的相对位置 (x, y, z)
                frame_data.extend(local_offset)
            
            # 确保每一帧数据是一个列表，而不是嵌套的单元素列表
            frames_data.append(frame_data)

        
        # 验证数据维度
        expected_dim = 23 + 23 + 12  
        for i, frame in enumerate(frames_data):
            if len(frame) != expected_dim:
                raise ValueError(f"第{i}帧数据维度错误: 期望{expected_dim}维，实际{len(frame)}维")
        
        print(f"数据维度验证通过: 每帧{expected_dim}维 (29关节位置[AMP顺序] + 29关节速度[AMP顺序] + 12连杆相对位置)")
        
        # 准备TXT格式的数据结构（匹配原始walk.txt格式）
        txt_data = {
            "LoopMode": "Wrap",
            "FrameDuration": 1.0 / self.fps,  # 帧持续时间
            "EnableCycleOffsetPosition": True,
            "EnableCycleOffsetRotation": True,
            "MotionWeight": 0.5,
            "Frames": frames_data  # 每一帧是一个包含70个浮点数的数组
        }

        # 保存为TXT文件
        current_dir = os.path.dirname(os.path.abspath(__file__))
        output_dir_txt = os.path.join(current_dir, 'txt')
        os.makedirs(output_dir_txt, exist_ok=True)
        
        # 提取纯文件名（去掉路径和扩展名）
        file_basename = os.path.splitext(os.path.basename(self.args.file_path))[0]

        # 构建输出文件名
        output_filename = f"{file_basename}_clipped_{start}_{end}.txt"
        output_path = os.path.join(output_dir_txt, output_filename)

        with open(output_path, 'w', encoding='utf-8') as f:
            # 手动构建JSON格式，确保每帧数据在同一行
            f.write('{\n')
            f.write('"LoopMode": "Wrap",\n')
            f.write(f'"FrameDuration": {1.0 / self.fps},\n')
            f.write('"EnableCycleOffsetPosition": true,\n')
            f.write('"EnableCycleOffsetRotation": true,\n')
            f.write('"MotionWeight": 0.5,\n')
            f.write('\n"Frames":\n[\n')
            
            # 写入每一帧数据
            for i, frame_data in enumerate(frames_data):
                # 将每帧数据格式化为一行
                frame_str = '  [' + ', '.join(f'{val:.6f}' for val in frame_data) + ']'
                if i < len(frames_data) - 1:
                    frame_str += ','
                f.write(frame_str + '\n')
            
            f.write(']\n}')
            
        print(f"TXT数据保存成功: {output_path}")
        print(f"包含 {len(frames_data)} 帧数据，每帧 {expected_dim} 维")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--file_path', type=str, help="CVS file name", default='default.csv')
    # 添加帧范围控制参数
    parser.add_argument('--start_frame', type=int, default=0, help="起始帧索引 (包含) 0默认0")
    parser.add_argument('--end_frame', type=int, default=-1, help="结束帧索引（不包含），-1表示最后一帧")
    # 添加位置归零控制参数
    parser.add_argument('--normalize_position', action='store_true', default=False, 
                       help="对根关节x,y位置进行归零处理（推荐用于AMP训练）")
    args = parser.parse_args()
    
    processor = MotionProcessor(args)
    processor.process_and_save_data()