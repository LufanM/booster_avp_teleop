import numpy as np
import matplotlib.pyplot as plt
import os

def visualize_joint_data(original_file, filtered_file):
    # 加载数据
    original_data = np.loadtxt(original_file)
    filtered_data = np.loadtxt(filtered_file)
    
    # 确保两个文件的数据长度一致
    min_len = min(len(original_data), len(filtered_data))
    original_data = original_data[:min_len]
    filtered_data = filtered_data[:min_len]
    
    # 提取时间信息（第一列）
    timestamps = original_data[:, 0]
    # 转换为相对时间（从0开始）
    relative_time = timestamps - timestamps[0]
    
    # 创建14个子图（每个关节一个）
    fig, axs = plt.subplots(7, 2, figsize=(20, 28), sharex=True)
    fig.suptitle('Joint Position Comparison: Raw vs Filtered', fontsize=20)
    
    # 遍历每个关节
    for joint_idx in range(14):
        # 计算数据列索引（跳过时间戳列）
        col_idx = joint_idx + 1
        
        # 获取当前关节的原始位置和滤波后位置
        raw_pos = original_data[:, col_idx]
        filtered_pos = filtered_data[:, col_idx]
        
        # 确定子图位置
        row = joint_idx // 2
        col = joint_idx % 2
        ax = axs[row, col]
        
        # 绘制数据
        ax.plot(relative_time, raw_pos, 'b-', alpha=0.7, label='Raw Position')
        ax.plot(relative_time, filtered_pos, 'r-', label='Filtered Position')
        
        # 设置图表标题和标签
        ax.set_title(f'Joint {joint_idx+2}', fontsize=14)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Position (rad)')
        ax.grid(True)
        ax.legend()
        
        # 添加数据统计信息
        # stats_text = (f"Raw: Δ={np.ptp(raw_pos):.4f} rad\n"
        #              f"Filtered: Δ={np.ptp(filtered_pos):.4f} rad")
        # ax.text(0.98, 0.02, stats_text, transform=ax.transAxes, 
        #         ha='right', va='bottom', fontsize=10,
        #         bbox=dict(facecolor='white', alpha=0.7))
    
    # 调整布局
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    
    # 保存图像
    output_file = os.path.splitext(original_file)[0] + '_comparison.png'
    plt.savefig(output_file, dpi=150)
    print(f"Visualization saved to: {output_file}")
    
    # 显示图像
    plt.show()

if __name__ == "__main__":

    data_file = "joint_pos_20250722_145213.txt"
    data_filtered_file = "joint_pos_filtered_20250722_145213.txt"
    visualize_joint_data(data_file, data_filtered_file)