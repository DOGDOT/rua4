#!/usr/bin/env python3
"""
Tiện ích lưu bản đồ từ SLAM Toolbox cho TurtleBot 4 (Phần 2)
Lưu thành file map.yaml và map.pgm trong thư mục maps/
"""

import os
import sys
import argparse
import subprocess
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError


def main():
    parser = argparse.ArgumentParser(description='Lưu bản đồ SLAM Toolbox thành map.yaml và map.pgm')
    parser.add_argument('-n', '--name', default='map', help='Tên file bản đồ (mặc định: map)')
    parser.add_argument('-o', '--output-dir', default='', help='Thư mục lưu bản đồ (mặc định: thư mục maps của workspace hoặc package)')
    parser.add_argument('-t', '--topic', default='map', help='Topic bản đồ (mặc định: map)')
    args = parser.parse_args()

    # Tìm đường dẫn thư mục maps trong workspace source nếu có, ngược lại dùng share directory
    workspace_src_maps = '/home/gdon/ur_ws/src/rua4/maps'
    if args.output_dir:
        target_dir = os.path.abspath(args.output_dir)
    elif os.path.exists(workspace_src_maps):
        target_dir = workspace_src_maps
    else:
        try:
            target_dir = os.path.join(get_package_share_directory('rua4'), 'maps')
        except PackageNotFoundError:
            target_dir = os.path.abspath('maps')

    os.makedirs(target_dir, exist_ok=True)
    map_prefix = os.path.join(target_dir, args.name)

    print("=" * 65)
    print("   [rua4] TIỆN ÍCH LƯU BẢN ĐỒ SLAM TOOLBOX")
    print("=" * 65)
    print(f"Topic bản đồ: /{args.topic}")
    print(f"Đường dẫn lưu: {map_prefix}.yaml & {map_prefix}.pgm")
    print("Đang gọi map_saver_cli...")

    cmd = [
        'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
        '-t', args.topic,
        '-f', map_prefix,
        '--ros-args', '-p', 'use_sim_time:=true'
    ]

    try:
        ret = subprocess.run(cmd)
        if ret.returncode == 0:
            yaml_file = f"{map_prefix}.yaml"
            pgm_file = f"{map_prefix}.pgm"
            print("-" * 65)
            if os.path.exists(yaml_file) and os.path.exists(pgm_file):
                print("[THÀNH CÔNG] Bản đồ đã được lưu:")
                print(f"  + YAML: {yaml_file} ({os.path.getsize(yaml_file)} bytes)")
                print(f"  + PGM:  {pgm_file} ({os.path.getsize(pgm_file)} bytes)")
                print("Bản đồ này đã sẵn sàng cho Phần 3 (Localization) và Phần 4 (Nav2)!")
            else:
                print("[CHÚ Ý] Lệnh thực hiện xong nhưng không tìm thấy file sinh ra.")
        else:
            print(f"[THẤT BẠI] Lỗi khi lưu bản đồ (Mã lỗi: {ret.returncode}).")
            print("Đảm bảo rằng SLAM Toolbox hoặc topic /map đang hoạt động trước khi lưu.")
    except Exception as e:
        print(f"[LỖI] {e}")

    print("=" * 65)


if __name__ == '__main__':
    main()
