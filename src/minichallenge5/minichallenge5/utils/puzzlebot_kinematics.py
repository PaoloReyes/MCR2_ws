import numpy as np

def get_puzzlebot_kinematic_model(r, l):
    return np.array([[r/2, r/2],
                   [r/l, -r/l]])

def get_inverse_puzzlebot_kinematic_model(r, l):
    return np.linalg.inv(get_puzzlebot_kinematic_model(r, l))

def get_linearized_puzzlebot_model_matrix(v, theta, delta):
    return np.array([[1, 0, -v*np.sin(theta)*delta],
                     [0, 1,  v*np.cos(theta)*delta],
                     [0, 0,           1           ]])

def get_linearized_puzzlebot_input_model_matrix(r, l, theta, delta):
    return (1/2*r*delta) * np.array([[np.cos(theta), np.cos(theta)],
                                    [np.sin(theta), np.sin(theta)],
                                    [     2/l,          -2/l     ]])

speeds_decomposer = lambda v, w, theta: np.array([v * np.cos(theta), v * np.sin(theta), w])