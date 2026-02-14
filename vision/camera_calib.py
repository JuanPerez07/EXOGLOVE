""" 
This script performs camera calibration using images of a chessboard pattern.
It calculates and saves the camera's intrinsic and extrinsic parameters and evaluates 
the reprojection error to measure calibration accuracy.
"""
import cv2 # requiered installation
import numpy as np # requiered installation
import glob
import os

# dataset directory and image format
DATASET_DIR = 'dataset'
IMG_FORMAT = "/*.png"
DATASET_SIZE = 15 # maximum size of the dataset
# Camera parameter directory
PARAM_DIR = "params/"

# Chessboard pattern size (internal corners per row and column)
CHESSBOARD_SIZE = (6, 9)  # Adjust according to your pattern
SQUARE_SIZE = 29  # Square size in mm (adjust accordingly)

# Termination criteria for corner refinement
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Prepare 3D object points in the real world (in mm)
objp = np.zeros((CHESSBOARD_SIZE[0] * CHESSBOARD_SIZE[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHESSBOARD_SIZE[0], 0:CHESSBOARD_SIZE[1]].T.reshape(-1, 2) * SQUARE_SIZE

# Storage for 3D and 2D points
objpoints = []  # 3D world points
imgpoints = []  # 2D image points

def calculate_params(images):
    """ Calculates and saves the intrinsic and extrinsic parameters of the camera. """
    
    # Read the first image to get its size
    img = cv2.imread(images[0])
    height, width = img.shape[:2]
    image_shape = (width, height)  # OpenCV expects (width, height)
    idx = 0 # counter of images included in the dataset
    for fname in images: # per each image in the folder
       if idx < DATASET_SIZE: # limits the dataset to check different Mean Projection Errors
            img = cv2.imread(fname)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)  # Convert to grayscale

            # Detect chessboard corners
            ret, corners = cv2.findChessboardCorners(gray, CHESSBOARD_SIZE, None)

            if ret:
                objpoints.append(objp)  # Store 3D points
                refined_corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)  
                imgpoints.append(refined_corners)  # Store 2D points
       else:
           break
       idx = idx + 1 # update idx counter  
          
    # Camera calibration
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, image_shape, None, None
    )

    # Compute extrinsic parameters for each image
    extrinsics = []
    for i in range(min(len(rvecs), len(tvecs))):
        R, _ = cv2.Rodrigues(rvecs[i])
        T = np.hstack((R, tvecs[i]))  # [R | t] transformation matrix
        extrinsics.append(T)

    # Save parameters to files
    os.makedirs(PARAM_DIR, exist_ok=True)
    np.save(PARAM_DIR + "camera_matrix.npy", camera_matrix)
    np.save(PARAM_DIR + "dist_coeffs.npy", dist_coeffs)
    np.save(PARAM_DIR + "extrinsics.npy", extrinsics)

    return objpoints, imgpoints

def compute_error(objpoints, imgpoints):
    """ Computes the reprojection error using stored calibration parameters. """

    # Load calibration parameters
    camera_matrix = np.load(PARAM_DIR + "camera_matrix.npy")
    dist_coeffs = np.load(PARAM_DIR + "dist_coeffs.npy")
    extrinsics = np.load(PARAM_DIR + "extrinsics.npy", allow_pickle=True)

    total_error = 0
    total_points = 0
    for i in range(len(objpoints)):
        R = extrinsics[i][:, :3]  # Extract rotation matrix
        t = extrinsics[i][:, 3]   # Extract translation vector
        rvec, _ = cv2.Rodrigues(R)

        # Project 3D points back onto the 2D image plane
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvec, t, camera_matrix, dist_coeffs)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2)
        total_error += error**2
        total_points += len(imgpoints2)

    mean_error = np.sqrt(total_error / total_points)  # Compute RMS reprojection error
    return mean_error

if __name__ == '__main__':
    # Get all image file paths
    images = glob.glob(DATASET_DIR + IMG_FORMAT)

    if not images:
        print(" No images found in dataset directory!")
        exit()

    # Perform calibration
    img_points, obj_points = calculate_params(images)

    # Compute the Mean Reproyection Error
    error = compute_error(img_points, obj_points)
    print(f" RMS Reprojection Error: {error}")