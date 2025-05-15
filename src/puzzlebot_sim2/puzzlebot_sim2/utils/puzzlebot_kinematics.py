import numpy as np

def get_puzzlebot_kinematic_model(r, l):
    return np.array([[r/2, r/2],
                   [r/l, -r/l]])

def get_inverse_puzzlebot_kinematic_model(r, l):
    return np.linalg.inv(get_puzzlebot_kinematic_model(r, l))

speeds_decomposer = lambda v, w, theta: np.array([v * np.cos(theta), v * np.sin(theta), w])