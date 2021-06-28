import pybullet as p
import pybullet_data
import time

#Se connecter au serveur 
physicsClient = p.connect(p.GUI)

#Permet d'aller récupérer les datas intégrées à PyBullet
p.setAdditionalSearchPath(pybullet_data.getDataPath())

#Charger différents objets
plane = p.loadURDF("plane.urdf")
robot = p.loadURDF("kuka_experimental/kuka_kr210_support/urdf/kr210l150.urdf", [0,0,0], useFixedBase = 1)

##Récupérer la position/orientation d'un objet
position, orientation = p.getBasePositionAndOrientation(robot)

numJointsRobot = p.getNumJoints(robot)
##print(numJointsRobot)

joint_index = 2
joint_info = p.getJointInfo(robot, joint_index)
#print(joint_info)

#Initialiser la gravité dans la simulation
p.setGravity(0, 0, -9.81)

#Initialiser le temps à chaque pas de la simulation
p.setTimeStep(0.0001)

#permet de gérer la simulation en temps réel (0 : desactivée, 1 : activée)
p.setRealTimeSimulation(0)

#Exemple : controle du robot
p.setJointMotorControlArray(robot, range(6), p.POSITION_CONTROL, targetPositions=[0.1]*6) 
#setJointMotorControlArray permet de controler toutes les articulations en même temps
for i in range(10000):
    p.stepSimulation()
    time.sleep(1./240.)

""" p.setGravity(0,0,-10)
startPos = [0,0,1]
startOrientation = p.getQuaternionFromEuler([0,0,0])
boxId = p.loadURDF("r2d2.urdf", startPos, startOrientation)
for i in range(10000):
    p.stepSimulation()
    time.sleep(1./240.)
cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
print(cubePos, cubeOrn)
"""

p.disconnect
