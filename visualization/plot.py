import numpy as np
import matplotlib.pyplot as plt

def plot_data(filename='euler_data.txt'):
    # 读取数据
    data = np.loadtxt(filename, delimiter=',', skiprows=1)
    time = data[:, 0]

    # 定义列名
    columns = ['x_l', 'y_l', 'z_l', 'euler_r_l', 'euler_p_l', 'euler_y_l',
               'x_r', 'y_r', 'z_r', 'euler_r_r', 'euler_p_r', 'euler_y_r']

    # 循环绘制对比图
    for i, col in enumerate(columns):
        original_col = i + 1
        original_data = data[:, original_col]

        plt.figure()
        plt.plot(time, original_data, label='Original')
        plt.xlabel('Time')
        plt.ylabel(col)
        plt.title(f'{col} Plot')
        plt.grid(True)
        plt.legend()

    plt.show()

def plot_comparison(filename='euler_data.txt'):
    # 读取数据
    data = np.loadtxt(filename, delimiter=',', skiprows=1)
    time = data[:, 0]

    # 定义列名
    columns = ['x_l', 'y_l', 'z_l', 'euler_r_l', 'euler_p_l', 'euler_y_l',
               'x_r', 'y_r', 'z_r', 'euler_r_r', 'euler_p_r', 'euler_y_r']

    # 循环绘制对比图
    for i, col in enumerate(columns):
        original_col = i + 1
        filtered_col = i + 13

        original_data = data[:, original_col]
        filtered_data = data[:, filtered_col]

        plt.figure()
        plt.plot(time, original_data, label='Original')
        plt.plot(time, filtered_data, label='Filtered')
        plt.xlabel('Time')
        plt.ylabel(col)
        plt.title(f'{col} Comparison')
        plt.grid(True)
        plt.legend()

    plt.show()

def plot_Hz_comparison(file_16Hz, file_32Hz, file_1000Hz):
    # 读取三个文件的数据
    data_16Hz = np.loadtxt(file_16Hz, delimiter=',', skiprows=1)
    data_32Hz = np.loadtxt(file_32Hz, delimiter=',', skiprows=1)
    data_1000Hz = np.loadtxt(file_1000Hz, delimiter=',', skiprows=1)

    # 获取时间列
    time_16Hz = data_16Hz[:, 0]
    time_32Hz = data_32Hz[:, 0]
    time_1000Hz = data_1000Hz[:, 0]

    # 定义列名
    columns = ['time', 'x_l', 'y_l', 'z_l', 'euler_r_l', 'euler_p_l', 'euler_y_l',
               'x_r', 'y_r', 'z_r', 'euler_r_r', 'euler_p_r', 'euler_y_r']

    # 循环绘制对比图，跳过第0列（时间列）
    for i in range(1, 12):
        # 获取当前列的数据
        col_16Hz = data_16Hz[:, i]
        col_32Hz = data_32Hz[:, i]
        col_1000Hz = data_1000Hz[:, i]

        # 创建新的图形
        plt.figure()
        plt.plot(time_16Hz, col_16Hz, label='16Hz')
        plt.plot(time_32Hz, col_32Hz, label='32Hz')
        plt.plot(time_1000Hz, col_1000Hz, label='1000Hz')

        plt.xlabel('Time')
        plt.ylabel(columns[i])
        plt.title(f'{columns[i]} Comparison')
        plt.grid(True)
        plt.legend()

    plt.show()


def save_data(data, filename):
    np.savetxt(filename, data, header='time, x_l, y_l, z_l, euler_r_l, euler_p_l, euler_y_l, x_r, y_r, z_r, euler_r_r, euler_p_r, euler_y_r,', delimiter=',')

def save_data_filted(data, filename='euler_data.txt'):
    np.savetxt(filename, data, header='time, x_l, y_l, z_l, euler_r_l, euler_p_l, euler_y_l, x_r, y_r, z_r, euler_r_r, euler_p_r, euler_y_r,'
    'x_l_filtered, y_l_filtered, z_l_filtered, euler_r_l_filtered, euler_p_l_filtered, euler_y_l_filtered, x_r_filtered, y_r_filtered, z_r_filtered, euler_r_r_filtered, euler_p_r_filtered, euler_y_r_filtered', delimiter=',')

class WeightedMovingFilter:
    '''
    当数据发生突变时，加权移动平均滤波需要一定的时间来适应新的数据，导致滤波结果不能及时反映数据的变化
    '''
    def __init__(self, weights, data_size = 14):
        self._window_size = len(weights)
        self._weights = np.array(weights)
        assert np.isclose(np.sum(self._weights), 1.0), "[WeightedMovingFilter] the sum of weights list must be 1.0!"
        self._data_size = data_size
        self._filtered_data = np.zeros(self._data_size)
        self._data_queue = []

    def _apply_filter(self):
        if len(self._data_queue) < self._window_size:
            return self._data_queue[-1]

        data_array = np.array(self._data_queue)
        temp_filtered_data = np.zeros(self._data_size)
        for i in range(self._data_size):
            temp_filtered_data[i] = np.convolve(data_array[:, i], self._weights, mode='valid')[-1]
        
        return temp_filtered_data

    def add_data(self, new_data):
        assert len(new_data) == self._data_size

        if len(self._data_queue) > 0 and np.array_equal(new_data, self._data_queue[-1]):
            return  # skip duplicate data
        
        if len(self._data_queue) >= self._window_size:
            self._data_queue.pop(0)

        self._data_queue.append(new_data)
        self._filtered_data = self._apply_filter()

    @property
    def filtered_data(self):
        return self._filtered_data   
    
def test1():
    filename='euler_data.txt'
    data = np.loadtxt(filename, delimiter=',', skiprows=1)
    # smooth_filter_ori = WeightedMovingFilter(np.array([0.7, 0.2, 0.07, 0.03]), 3)
    smooth_filter_ori = WeightedMovingFilter(np.array([0.2, 0.25, 0.3, 0.25]), 3)
    filtered_data = np.zeros_like(data)
    filtered_data[:, 0] = data[:, 0]      

    for i in range(data.shape[0]):
        smooth_filter_ori.add_data(data[i, 1:])
        filtered_data[i, 1:] = smooth_filter_ori.filtered_data

    save_data(filtered_data, 'euler_data_filtered.txt')
    
    # plot_data('euler_data_filtered.txt')
    plot_comparison('euler_data.txt', 'euler_data_filtered.txt')

def test2():
    filename = "hand_tracking_32Hz.txt"
    print("plot: ", filename)
    plot_data(filename)    

def test3():
    file_16Hz = 'hand_tracking_16Hz.txt'
    file_32Hz = 'hand_tracking_32Hz.txt'
    file_xHz = 'hand_tracking_xHz.txt'
    plot_Hz_comparison(file_16Hz, file_32Hz, file_xHz)


if __name__ == "__main__":
    # test1()
    # test2()
    test3()