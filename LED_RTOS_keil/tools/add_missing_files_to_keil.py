#!/usr/bin/env python3
"""
自动添加缺失的源文件到Keil项目
修改FSP_Project.uvprojx文件，添加必需的.c文件
"""

import xml.etree.ElementTree as ET
import os
import shutil
from datetime import datetime

# 需要添加的源文件列表
REQUIRED_FILES = [
    "src\\motor_ctrl_step.c",
    "src\\degradation.c",
    "src\\log.c",
    "src\\rtmon.c",
    "src\\taskmon.c",
    "src\\medicine_db.c",
]

OPTIONAL_FILES = [
    "src\\cabinet_demo.c",
    "src\\cabinet_executor.c",
    "src\\cabinet_main.c",
    "src\\pid_tuner.c",
    "src\\realtime_monitor.c",
    "src\\task_monitor.c",
    "src\\visual_servo.c",
]

def backup_project_file(project_path):
    """备份项目文件"""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    backup_path = f"{project_path}.backup_{timestamp}"
    shutil.copy2(project_path, backup_path)
    print(f"✓ 备份项目文件: {backup_path}")
    return backup_path

def parse_keil_project(project_path):
    """解析Keil项目文件"""
    tree = ET.parse(project_path)
    root = tree.getroot()
    return tree, root

def get_existing_files(root):
    """获取项目中已存在的源文件"""
    existing_files = set()
    
    # 查找所有FilePath元素
    for file_elem in root.findall(".//FilePath"):
        if file_elem.text:
            existing_files.add(file_elem.text)
    
    return existing_files

def create_file_element(file_path):
    """创建文件元素（XML节点）"""
    file_elem = ET.Element("File")
    
    # FileName
    file_name = ET.SubElement(file_elem, "FileName")
    file_name.text = os.path.basename(file_path)
    
    # FileType (1 = C文件)
    file_type = ET.SubElement(file_elem, "FileType")
    file_type.text = "1"
    
    # FilePath
    file_path_elem = ET.SubElement(file_elem, "FilePath")
    file_path_elem.text = file_path
    
    return file_elem

def add_files_to_group(root, files_to_add):
    """添加文件到Source Group 1"""
    # 查找Source Group 1
    groups = root.findall(".//Groups/Group")
    source_group = None
    
    for group in groups:
        group_name = group.find("GroupName")
        if group_name is not None and "Source" in group_name.text:
            source_group = group
            break
    
    if source_group is None:
        print("✗ 错误: 找不到Source Group")
        return 0
    
    # 查找Files容器
    files_container = source_group.find("Files")
    if files_container is None:
        files_container = ET.SubElement(source_group, "Files")
    
    # 添加文件
    added_count = 0
    for file_path in files_to_add:
        file_elem = create_file_element(file_path)
        files_container.append(file_elem)
        added_count += 1
        print(f"  + {file_path}")
    
    return added_count

def main():
    project_dir = os.path.dirname(os.path.abspath(__file__))
    project_path = os.path.join(project_dir, "FSP_Project.uvprojx")
    
    if not os.path.exists(project_path):
        print(f"✗ 错误: 找不到项目文件 {project_path}")
        return 1
    
    print("=" * 60)
    print("Keil项目文件自动修改工具")
    print("=" * 60)
    
    # 1. 备份项目文件
    backup_path = backup_project_file(project_path)
    
    # 2. 解析项目文件
    print("\n正在解析项目文件...")
    tree, root = parse_keil_project(project_path)
    
    # 3. 获取已存在的文件
    existing_files = get_existing_files(root)
    print(f"✓ 项目中已有 {len(existing_files)} 个源文件")
    
    # 4. 确定需要添加的文件
    files_to_add = []
    for file_path in REQUIRED_FILES:
        if file_path not in existing_files:
            # 检查文件是否存在
            full_path = os.path.join(project_dir, file_path)
            if os.path.exists(full_path):
                files_to_add.append(file_path)
            else:
                print(f"⚠ 警告: 文件不存在 {file_path}")
    
    if not files_to_add:
        print("\n✓ 所有必需文件已在项目中，无需添加")
        return 0
    
    # 5. 添加文件到项目
    print(f"\n正在添加 {len(files_to_add)} 个文件到项目...")
    added_count = add_files_to_group(root, files_to_add)
    
    # 6. 保存修改后的项目文件
    if added_count > 0:
        tree.write(project_path, encoding="utf-8", xml_declaration=True)
        print(f"\n✓ 成功添加 {added_count} 个文件到项目")
        print(f"✓ 项目文件已更新: {project_path}")
        print(f"\n如果出现问题，可以恢复备份: {backup_path}")
    else:
        print("\n✗ 没有文件被添加")
        return 1
    
    print("\n" + "=" * 60)
    print("完成！请在Keil中重新打开项目并编译")
    print("=" * 60)
    
    return 0

if __name__ == "__main__":
    exit(main())
