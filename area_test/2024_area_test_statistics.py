#!/usr/bin/python3
# coding=utf8
"""
面积统计数据脚本
读取所有时间戳命名的JSON文件，计算最小值和最大值
"""
import json
import os
import glob
from pathlib import Path

def load_all_area_data(data_dir='area_test'):
    """
    加载指定目录下所有面积数据JSON文件
    """
    json_files = glob.glob(os.path.join(data_dir, 'area_data_*.json'))
    
    all_data = []
    for json_file in json_files:
        try:
            with open(json_file, 'r', encoding='utf-8') as f:
                data = json.load(f)
                data['filename'] = os.path.basename(json_file)
                all_data.append(data)
        except Exception as e:
            print(f"Error loading {json_file}: {e}")
    
    return all_data

def calculate_statistics(all_data):
    """
    计算所有面积数据的最小值和最大值
    """
    if not all_data:
        return None
    
    # 收集所有ROI区域的最大面积
    roi_max_areas = {}  # {roi_index: [areas]}
    global_max_areas = []
    
    for data in all_data:
        # 收集全局最大面积
        if 'global_max_area' in data:
            global_max_areas.append(data['global_max_area'])
        
        # 收集每个ROI区域的最大面积
        if 'roi_areas' in data:
            for roi_area in data['roi_areas']:
                roi_index = roi_area['roi_index']
                if roi_index not in roi_max_areas:
                    roi_max_areas[roi_index] = []
                roi_max_areas[roi_index].append(roi_area['max_area'])
    
    # 计算统计信息
    statistics = {
        'total_files': len(all_data),
        'global_max_area': {
            'min': min(global_max_areas) if global_max_areas else 0,
            'max': max(global_max_areas) if global_max_areas else 0,
            'values': global_max_areas
        },
        'roi_statistics': {}
    }
    
    # 计算每个ROI区域的统计信息
    for roi_index, areas in roi_max_areas.items():
        statistics['roi_statistics'][f'roi_{roi_index}'] = {
            'min': min(areas) if areas else 0,
            'max': max(areas) if areas else 0,
            'count': len(areas),
            'values': areas
        }
    
    return statistics

def print_statistics(statistics):
    """
    打印统计信息
    """
    if not statistics:
        print("No data found!")
        return
    
    print("=" * 60)
    print("面积检测统计结果")
    print("=" * 60)
    print(f"总文件数: {statistics['total_files']}")
    print()
    
    print("全局最大面积统计:")
    print(f"  最小值: {statistics['global_max_area']['min']:.2f}")
    print(f"  最大值: {statistics['global_max_area']['max']:.2f}")
    print(f"  数据点数量: {len(statistics['global_max_area']['values'])}")
    print()
    
    print("各ROI区域统计:")
    for roi_key, roi_stat in sorted(statistics['roi_statistics'].items()):
        print(f"  {roi_key.upper()}:")
        print(f"    最小值: {roi_stat['min']:.2f}")
        print(f"    最大值: {roi_stat['max']:.2f}")
        print(f"    数据点数量: {roi_stat['count']}")
    print("=" * 60)

def save_statistics(statistics, output_file='area_test/area_statistics.json'):
    """
    保存统计结果到JSON文件
    """
    # 确保输出目录存在
    output_dir = os.path.dirname(output_file)
    if output_dir and not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    # 移除values列表以减小文件大小（可选）
    statistics_copy = json.loads(json.dumps(statistics))
    for roi_key in statistics_copy.get('roi_statistics', {}):
        if 'values' in statistics_copy['roi_statistics'][roi_key]:
            del statistics_copy['roi_statistics'][roi_key]['values']
    if 'values' in statistics_copy.get('global_max_area', {}):
        del statistics_copy['global_max_area']['values']
    
    with open(output_file, 'w', encoding='utf-8') as f:
        json.dump(statistics_copy, f, indent=2, ensure_ascii=False)
    
    print(f"\n统计结果已保存到: {output_file}")

if __name__ == '__main__':
    # 加载所有面积数据
    print("正在加载面积数据文件...")
    all_data = load_all_area_data('area_test')
    
    if not all_data:
        print("未找到任何面积数据文件！")
        print("请先运行 area_test_detection.py 生成数据文件。")
        exit(1)
    
    print(f"成功加载 {len(all_data)} 个数据文件")
    print()
    
    # 计算统计信息
    statistics = calculate_statistics(all_data)
    
    # 打印统计结果
    print_statistics(statistics)
    
    # 保存统计结果
    save_statistics(statistics)

