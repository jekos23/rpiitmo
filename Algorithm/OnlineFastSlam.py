import numpy as np
import cv2
import time
import os
from Algorithm.FastSlam import ParticleFilter

class OnlineFastSlam:
    def __init__(self, show_map=True):
        self.show_map = show_map
        
        # Parameters adapted for 360-degree LiDAR and real-time use
        initMapXLength, initMapYLength = 30, 30 # Smaller map for performance (Meters)
        unitGridSize = 0.05 # 5cm resolution (coarser for performance)
        lidarFOV = 2 * np.pi # 360 degrees
        lidarMaxRange = 8.0 # LD06 max is 8m-12m
        numSamplesPerRev = 360 # 1 sample per degree
        wallThickness = 5 * unitGridSize
        
        initXY = {'x': 0, 'y': 0, 'theta': 0, 'range': [0.0]*360}
        
        scanMatchSearchRadius, scanMatchSearchHalfRad, scanSigmaInNumGrid = 1.0, 0.2, 2
        moveRSigma, maxMoveDeviation, turnSigma = 0.1, 0.2, 0.3
        missMatchProbAtCoarse, coarseFactor = 0.15, 3
        
        numParticles = 5 # Reduced from 10/15 for Raspberry Pi performance
        
        ogParameters = [initMapXLength, initMapYLength, initXY, unitGridSize, lidarFOV, lidarMaxRange, numSamplesPerRev, wallThickness]
        smParameters = [scanMatchSearchRadius, scanMatchSearchHalfRad, scanSigmaInNumGrid, moveRSigma, maxMoveDeviation, turnSigma, missMatchProbAtCoarse, coarseFactor]
        
        self.pf = ParticleFilter(numParticles, ogParameters, smParameters)
        self.count = 0

    def process_scan(self, scan_ranges, estimated_dx, estimated_dy, estimated_dtheta, lidar_offset_x=0.0, lidar_offset_y=0.0):
        """
        scan_ranges: array of 360 floats (distances in meters)
        estimated_d*: pseudo-odometry from motor commands (in meters / radians)
        lidar_offset_x: LiDAR position offset X from robot center
        lidar_offset_y: LiDAR position offset Y from robot center
        """
        self.count += 1
        
        # Provide odometry estimate based on previous state + pseudo odometry movement
        if not hasattr(self, 'robot_odom_x'):
            self.robot_odom_x = 0.0
            self.robot_odom_y = 0.0
            self.robot_odom_theta = 0.0
            
        # Обновляем координаты центра робота
        self.robot_odom_x += estimated_dx * np.cos(self.robot_odom_theta) - estimated_dy * np.sin(self.robot_odom_theta)
        self.robot_odom_y += estimated_dx * np.sin(self.robot_odom_theta) + estimated_dy * np.cos(self.robot_odom_theta)
        self.robot_odom_theta += estimated_dtheta
        
        # Вычисляем координаты лидара в пространстве с учетом его положения (по X и Y)
        lidar_x = self.robot_odom_x + lidar_offset_x * np.cos(self.robot_odom_theta) - lidar_offset_y * np.sin(self.robot_odom_theta)
        lidar_y = self.robot_odom_y + lidar_offset_x * np.sin(self.robot_odom_theta) + lidar_offset_y * np.cos(self.robot_odom_theta)
        lidar_theta = self.robot_odom_theta
        
        reading = {
            'x': lidar_x,
            'y': lidar_y,
            'theta': lidar_theta,
            'range': scan_ranges
        }
        
        # FastSlam.py expects lists instead of array for range sometimes, just pass list
        self.pf.updateParticles(reading, self.count)
        
        if self.pf.weightUnbalanced():
            self.pf.resample()

        # Автоматическое сохранение карты каждые 20 кадров (~2 секунды) даже в Headless режиме
        if self.count % 20 == 0:
            self.save_map_to_file()
            
        if self.show_map and self.count % 2 == 0: # render every 2nd frame
            self.display_map()

    def display_map(self):
        bestParticle = self.pf.particles[0]
        maxWeight = -1
        for particle in self.pf.particles:
            if maxWeight < particle.weight:
                maxWeight = particle.weight
                bestParticle = particle

        xRange, yRange = [-10, 10], [-10, 10] # Display a 20x20m area around origin
        # Avoid divide by zero
        ogMapTotal = np.where(bestParticle.og.occupancyGridTotal == 0, 1, bestParticle.og.occupancyGridTotal)
        ogMap = bestParticle.og.occupancyGridVisited / ogMapTotal
        
        xIdx, yIdx = bestParticle.og.convertRealXYToMapIdx(xRange, yRange)
        ogMap = ogMap[yIdx[0]: yIdx[1], xIdx[0]: xIdx[1]]
        
        # Convert to image format
        ogMap_img = (np.flipud(1 - ogMap) * 255).astype(np.uint8)
        
        # Resize for better visibility
        ogMap_img_large = cv2.resize(ogMap_img, (600, 600), interpolation=cv2.INTER_NEAREST)
        
        try:
            cv2.imshow("FastSLAM Real-Time Map", ogMap_img_large)
            cv2.waitKey(1)
        except Exception as e:
            print(f"\n[ВНИМАНИЕ] Ошибка вывода графики (возможно, нет доступа к X-серверу): {e}")
            print("[ВНИМАНИЕ] Автоматически переключаюсь в режим 'Не показывать карту' для предотвращения сбоя.")
            self.show_map = False

    def save_map_to_file(self):
        os.makedirs("results", exist_ok=True)
        bestParticle = self.pf.particles[0]
        maxWeight = -1
        for particle in self.pf.particles:
            if maxWeight < particle.weight:
                maxWeight = particle.weight
                bestParticle = particle

        xRange, yRange = [-10, 10], [-10, 10]
        ogMapTotal = np.where(bestParticle.og.occupancyGridTotal == 0, 1, bestParticle.og.occupancyGridTotal)
        ogMap = bestParticle.og.occupancyGridVisited / ogMapTotal
        
        xIdx, yIdx = bestParticle.og.convertRealXYToMapIdx(xRange, yRange)
        ogMap = ogMap[yIdx[0]: yIdx[1], xIdx[0]: xIdx[1]]
        
        ogMap_img = (np.flipud(1 - ogMap) * 255).astype(np.uint8)
        ogMap_img_large = cv2.resize(ogMap_img, (600, 600), interpolation=cv2.INTER_NEAREST)
        
        cv2.imwrite("results/map_latest.png", ogMap_img_large)
