import os
import sys
import glob

try:
    import numpy as np
    import matplotlib.pyplot as plt
except ImportError:
    print("错误: 缺少依赖库 numpy 或 matplotlib。")
    print("请使用您的包管理器安装依赖，例如: uv pip install numpy matplotlib")
    sys.exit(1)

def find_csv_files():
    """搜索当前目录和 scripts 目录下的所有 CSV 文件"""
    files = glob.glob("*.csv") + glob.glob("scripts/*.csv")
    return sorted(list(set(files)))

def select_file(files):
    if not files:
        print("当前目录和 scripts 目录下未找到任何 CSV 文件！")
        return None
        
    print("\n=== 找到以下 CSV 文件 ===")
    for i, f in enumerate(files):
        print(f"[{i}] {f}")
        
    while True:
        try:
            choice = input(f"请输入文件序号 (0-{len(files)-1}): ").strip()
            if not choice:
                continue
            index = int(choice)
            if 0 <= index < len(files):
                return files[index]
            else:
                print("序号超出范围，请重新输入。")
        except ValueError:
            print("请输入有效的数字序号。")

def get_row_range(total_rows):
    print(f"\n文件共有 {total_rows} 行数据。")
    
    start_row = 0
    while True:
        choice = input("请输入起始行号 (直接回车默认 0): ").strip()
        if not choice:
            break
        try:
            val = int(choice)
            if 0 <= val < total_rows:
                start_row = val
                break
            else:
                print(f"起始行必须在 0 到 {total_rows-1} 之间。")
        except ValueError:
            print("请输入有效数字。")
            
    end_row = total_rows
    while True:
        choice = input(f"请输入结束行号 (直接回车默认 {total_rows}): ").strip()
        if not choice:
            break
        try:
            val = int(choice)
            if start_row < val <= total_rows:
                end_row = val
                break
            else:
                print(f"结束行必须在 {start_row+1} 到 {total_rows} 之间。")
        except ValueError:
            print("请输入有效数字。")
            
    return start_row, end_row

def get_columns(total_cols):
    print(f"\n文件共有 {total_cols} 列数据 (列号从 0 到 {total_cols-1})。")
    print("例如，电机1原始值为 0，滤波值为 1；电机2为 2, 3，依此类推。")
    
    while True:
        choice = input("请输入需要绘制的列号，用逗号分隔 (例如: 0,1): ").strip()
        if not choice:
            continue
        try:
            cols = [int(c.strip()) for c in choice.split(',')]
            if all(0 <= c < total_cols for c in cols):
                return cols
            else:
                print("有列号超出范围，请重新输入。")
        except ValueError:
            print("格式错误，请使用逗号分隔数字。")

def main():
    print("=== 电机转速数据分析与可视化工具 ===")
    files = find_csv_files()
    filename = select_file(files)
    
    if not filename:
        return
        
    print(f"\n正在加载文件: {filename} ...")
    try:
        # 使用 numpy 加载数据，忽略可能存在的非数字表头行
        # 如果文件有 header，genfromtxt 配合 skip_header 或直接解析通常更稳健
        # 考虑到可能全是数字，或者第一行包含字母
        try:
            data = np.loadtxt(filename, delimiter=',')
        except ValueError:
            # 如果加载失败，可能是第一行有表头，跳过第一行重试
            data = np.loadtxt(filename, delimiter=',', skiprows=1)
            
        if data.ndim == 1:
            data = data.reshape(-1, 1)
            
        total_rows, total_cols = data.shape
        
        start_row, end_row = get_row_range(total_rows)
        selected_cols = get_columns(total_cols)
        
        # 截取数据
        sliced_data = data[start_row:end_row, :]
        
        print("\n=== 量化指标分析结果 ===")
        print(f"{'列号':<8} | {'均值 (Mean)':<12} | {'标准差 (Std Dev)':<15} | {'差分标准差 (Std Diff)':<15}")
        print("-" * 65)
        
        plt.figure(figsize=(12, 6))
        
        for col in selected_cols:
            y = sliced_data[:, col]
            
            # 计算指标
            mean_val = np.mean(y)
            std_val = np.std(y)
            
            # 计算差分标准差 (反映平滑度)
            diff_y = np.diff(y)
            std_diff = np.std(diff_y) if len(diff_y) > 0 else 0
            
            print(f"Col {col:<4} | {mean_val:<12.3f} | {std_val:<15.3f} | {std_diff:<15.3f}")
            
            label_name = f"Col {col} (Std: {std_val:.2f}, StdDiff: {std_diff:.2f})"
            plt.plot(range(start_row, end_row), y, label=label_name, alpha=0.8, linewidth=1.5)
            
        print("-" * 65)
        print("提示: 差分标准差 (Std Diff) 越小，说明曲线越平滑 (滤波效果越好)。\n")
        
        plt.title(f"Motor RPM Data Analysis\nFile: {os.path.basename(filename)} (Rows: {start_row}-{end_row})")
        plt.xlabel("Sample Index (Time)")
        plt.ylabel("RPM")
        plt.legend()
        plt.grid(True, linestyle='--', alpha=0.6)
        plt.tight_layout()
        
        print("正在打开图表窗口，请在弹出的窗口中查看。")
        plt.show()
        
    except Exception as e:
        print(f"处理文件时发生错误: {e}")

if __name__ == "__main__":
    main()
