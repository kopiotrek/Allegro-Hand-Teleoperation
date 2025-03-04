import numpy as np
import cv2

def normalize_vector(vector):
    return vector / np.linalg.norm(vector)

def moving_average(vector, moving_average_queue, limit):
    moving_average_queue.append(vector)

    if len(moving_average_queue) > limit:
        moving_average_queue.pop(0)

    mean_vector = np.mean(moving_average_queue, axis = 0)
    return mean_vector

def get_distance(start_vector, end_vector):
    return np.linalg.norm(end_vector - start_vector)

def linear_transform(curr_val, source_bound, target_bound):
    multiplier = (target_bound[1] - target_bound[0]) / (source_bound[1] - source_bound[0])
    target_val = ((curr_val - source_bound[0]) * multiplier) + target_bound[0]
    return target_val

def perspective_transform(input_coordinates, given_bound, target_bound):
    transformation_matrix = cv2.getPerspectiveTransform(np.float32(given_bound), np.float32(target_bound))
    transformed_coordinate = np.matmul(np.array(transformation_matrix), np.array([input_coordinates[0], input_coordinates[1], 1]))
    transformed_coordinate = transformed_coordinate / transformed_coordinate[-1]

    return transformed_coordinate[0], transformed_coordinate[1]

def calculate_angle(coord_1, coord_2, coord_3):
    vector_1 = coord_2 - coord_1
    vector_2 = coord_3 - coord_2

    inner_product = np.inner(vector_1, vector_2)
    norm = np.linalg.norm(vector_1) * np.linalg.norm(vector_2)
    angle = np.arccos(inner_product / norm)
    return angle

def calculate_angle_x(coord_1, coord_2, coord_3):
    # Project coordinates onto the YZ plane by ignoring the X component
    vector_1 = np.array([0, coord_2[1] - coord_1[1], coord_2[2] - coord_1[2]])
    vector_2 = np.array([0, coord_3[1] - coord_2[1], coord_3[2] - coord_2[2]])

    inner_product = np.inner(vector_1, vector_2)
    norm = np.linalg.norm(vector_1) * np.linalg.norm(vector_2)
    angle = np.arccos(inner_product / norm)
    return angle

def calculate_angle_y(coord_1, coord_2, coord_3):
    # Project coordinates onto the XZ plane by ignoring the Y component
    vector_1 = np.array([coord_2[0] - coord_1[0], 0, coord_2[2] - coord_1[2]])
    vector_2 = np.array([coord_3[0] - coord_2[0], 0, coord_3[2] - coord_2[2]])

    inner_product = np.inner(vector_1, vector_2)
    norm = np.linalg.norm(vector_1) * np.linalg.norm(vector_2)
    angle = np.arccos(inner_product / norm)
    return angle

def calculate_angle_z(coord_1, coord_2, coord_3):
    # Project coordinates onto the XY plane by ignoring the Z component
    vector_1 = np.array([coord_2[0] - coord_1[0], coord_2[1] - coord_1[1], 0])
    vector_2 = np.array([coord_3[0] - coord_2[0], coord_3[1] - coord_2[1], 0])

    inner_product = np.inner(vector_1, vector_2)
    norm = np.linalg.norm(vector_1) * np.linalg.norm(vector_2)
    angle = np.arccos(inner_product / norm)
    return angle

def coord_in_bound(bound, coord):
    bound_points = np.array(bound, dtype=np.float32).reshape(-1, 2)
    return cv2.pointPolygonTest(bound_points, tuple(coord), False)    

def rotate_point(point, angles):
    """
    Rotates a 3D point by given Euler angles (in radians).
    :param point: List or array of [x, y, z].
    :param angles: Tuple of angles (rx, ry, rz) in radians.
    :return: Rotated point as a numpy array.
    """
    rx, ry, rz = angles
    
    # Rotation matrix for X-axis
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(rx), -np.sin(rx)],
        [0, np.sin(rx), np.cos(rx)]
    ])
    
    # Rotation matrix for Y-axis
    Ry = np.array([
        [np.cos(ry), 0, np.sin(ry)],
        [0, 1, 0],
        [-np.sin(ry), 0, np.cos(ry)]
    ])
    
    # Rotation matrix for Z-axis
    Rz = np.array([
        [np.cos(rz), -np.sin(rz), 0],
        [np.sin(rz), np.cos(rz), 0],
        [0, 0, 1]
    ])
    
    # Combined rotation matrix
    R = Rz @ Ry @ Rx
    
    # Apply rotation
    return R @ point