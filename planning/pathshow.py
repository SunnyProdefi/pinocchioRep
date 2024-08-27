import pandas as pd
import matplotlib.pyplot as plt

# 读取第一个CSV文件
data1 = pd.read_csv('planning/pathPoints.csv')

# 创建第一个图形
plt.figure(1)
for column in data1.columns:
    plt.plot(data1[column], label=column)
plt.legend()
plt.title('Path Points')  # 添加图表标题

# 读取第二个CSV文件
data2 = pd.read_csv('planning/interpolatedPoints.csv')

# 创建第二个图形
plt.figure(2)
for column in data2.columns:
    plt.plot(data2[column], label=column)
plt.legend()
plt.title('Interpolated Points')  # 添加图表标题

# 显示所有图表
plt.show()
