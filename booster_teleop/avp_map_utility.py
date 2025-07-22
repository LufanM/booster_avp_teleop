import numpy as np

T_robot_avp = np.array([[0, 0, -1, 0], # rot x 90° and rot z -90° non-fixed axis: Rotz(-90d)*Rotx(90d)
                        [-1, 0, 0, 0],
                        [0, 1, 0, 0],
                        [0, 0, 0, 1]])

T_head_booster = np.array([[0, -1, 0, 0], # rot z 90°
                            [1, 0, 0, 0],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]]) 

T_wrist_left_booster = np.array([[0, 1, 0, 0], # rot x 90° and roty 90° fixed axis of Wrist (Robot Convention): Roty(90°)*Rotx(90°)
                                    [0, 0, -1, 0],
                                    [-1, 0, 0, 0],
                                    [0, 0, 0, 1]]) 

T_wrist_right_booster = np.array([[0, 1, 0, 0], # rot x -90° and rotz -90° fixed axis of Wrist(Robot Convention): Roty(-90°)*Rotx(-90°)
                                    [0, 0, 1, 0],
                                    [1, 0, 0, 0],
                                    [0, 0, 0, 1]]) 

T_map_grapper_left = np.array([[1, 0, 0, 0],    # rot x 90°
                                [0, 0, -1, 0],
                                [0, 1, 0, 0],
                                [0, 0, 0, 1]]) 

T_map_grapper_right = np.array([[1, 0, 0, 0],    # rot x 90°
                                [0, 0, -1, 0],
                                [0, 1, 0, 0],
                                [0, 0, 0, 1]]) 


def fast_mat_inv(mat):
    ret = np.eye(4)
    ret[:3, :3] = mat[:3, :3].T
    ret[:3, 3] = -mat[:3, :3].T @ mat[:3, 3]
    return ret

def avp2booster_transform(rw_mat, lw_mat, rfinger_mat, lfinger_mat):
    '''
    wrist transform: 
    step1: avp raw data is T_{avp}_{wrist}. Need transform to robot convetion world: T_{robot}_{wrist} = T_{robot}_{avp} * T_{avp}_{wrist}.
    step2: the wrist coordinate in robot is diff from booster, Need right multiply to transform(T_{wrist}_{booster_w} )
           that is T_{robot}_{booster_w} = T_{robot}_{wrist} * T_{wrist}_{booster_w}
    finger transfrom
    a litter diff from wrist transform:beacuse avp raw finger data is based on wrist, not in ther avp world coordinate. so the transform is:
            T_{robot}_{finger}(booster convetion) = T_{robot}_{booster_w} * T_{booster_w}_{wrist} * T_{wrist}_{finger}
    the reason for left multiply T_{booster_w}_{wrist}: T_{wrist}_{finger} is a relative description calculated without wrist transformation
    '''
    T_robot_wrist_right = T_robot_avp @ rw_mat
    T_robot_wrist_left = T_robot_avp @ lw_mat
    T_booster_wrist_right = T_robot_wrist_right @ T_wrist_right_booster
    T_booster_wrist_left = T_robot_wrist_left @ T_wrist_left_booster

    T_booster_finger_right = T_robot_wrist_right @ rfinger_mat 
    T_booster_finger_left = T_robot_wrist_left @ lfinger_mat 

    return T_booster_wrist_right, T_booster_wrist_left, T_booster_finger_right, T_booster_finger_left


class WeightedMovingFilter:
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
    
def save_data(data, filename):
    np.savetxt(filename, data, header='time, x_l, y_l, z_l, euler_r_l, euler_p_l, euler_y_l, x_r, y_r, z_r, euler_r_r, euler_p_r, euler_y_r,', delimiter=',')

def save_data_filted(data, filename='euler_data.txt'):
    np.savetxt(filename, data, header='time, x_l, y_l, z_l, euler_r_l, euler_p_l, euler_y_l, x_r, y_r, z_r, euler_r_r, euler_p_r, euler_y_r,'
    'x_l_filtered, y_l_filtered, z_l_filtered, euler_r_l_filtered, euler_p_l_filtered, euler_y_l_filtered, x_r_filtered, y_r_filtered, z_r_filtered, euler_r_r_filtered, euler_p_r_filtered, euler_y_r_filtered', delimiter=',')
