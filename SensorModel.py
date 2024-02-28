from PathSolution import *
from math import exp, pi

def get_lidar_pdf(r, z, sigma=0.5):
    # r: Perceived distance from the lidar sensor
    # z: Actual distance before an obstacle
    return exp((-(r - z) ** 2) / (2 * sigma ** 2)) / sqrt(2 * pi * sigma)
    # Pdf of a sensor corrupted by Gaussian noise of zero mean and variance sigma^2

# def get_camera_pdf(image):

def update_occ_grid(sol, image, r): # Update occupancy grid based on current sensor reading

    # image: image captured by the camera
    # r: Perceived distance from the lidar sensor
    # TODO update occupancy grid of a single node based on "image" and "r"

    r=25 # Sample
    z=25 # Sample

    lidar_pdf = get_lidar_pdf(r,z)
    print(lidar_pdf)