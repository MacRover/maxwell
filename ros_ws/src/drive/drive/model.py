from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from typing import List
import numpy as np
import math

from .drive_module import DriveModule

from custom_interfaces.msg import SwerveModule


class SteeringModel:

	## Solving V_i = A_i * V_b
	## Angles are from the x axis

	def __init__(self, driveModules: List[DriveModule]):
		self.driveModules = driveModules
		self.ActuatorMatrix = np.zeros((2 * len(driveModules), 3))
		
		for i, module in enumerate(driveModules):
			self.ActuatorMatrix[2 * i, 0] = 1.0
			self.ActuatorMatrix[2 * i + 1, 1] = 1.0
			self.ActuatorMatrix[2 * i, 2] = -module.pos_xy_from_centre[1]
			self.ActuatorMatrix[2 * i + 1, 2] = module.pos_xy_from_centre[0]
		
		self.ActuatorMatrixInv = np.linalg.pinv(self.ActuatorMatrix)
		self.body_state = None

	def updateOdom(self, driveModules: List[SwerveModule]):
		module_states = np.zeros((len(driveModules) * 2))
		
		for i, module in enumerate(driveModules):
			module_states[2 * i] = module.speed * np.cos(module.angle)
			module_states[2 * i + 1] = module.speed * np.sin(module.angle)

		self.body_state = np.matmul(self.ActuatorMatrixInv, module_states)
		return self.body_state

	def getDriveModuleVelocities(self) -> List[SwerveModule]:

		drive_module_states = np.matmul(self.ActuatorMatrix, self.body_state)
		print("Twist: " + str(self.body_state))
		print("Drives: ", drive_module_states)
		out = []
		for i in range(len(drive_module_states) // 2):
			module = SwerveModule()
			module.speed = np.sqrt(drive_module_states[2 * i] ** 2 + drive_module_states[2 * i + 1] ** 2)
			angle = np.arctan2(drive_module_states[2 * i + 1], drive_module_states[2 * i])
			# if angle is negative, flip speed and make angle positive
			
			if abs(math.pi - angle) < 0.0001:
				angle = 0.0
				module.speed *= -1
			if angle < 0:
				module.speed *= -1
				angle += np.pi
			
			module.angle = angle
			out.append(module)
		
		return out
    
    