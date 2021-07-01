import pybullet as p
import time
import pybullet_data
import os 

path = "C:/Users/Faga/Desktop/Nathan/ENSTA/Cours/2A/PRe/PRe/burg-toolkit/data/tmp/table.urdf"


physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version

p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.loadURDF("plane.urdf")
p.setGravity(0,0,-9.81)

startPosBox = [0,0,2]
startOrientationBox = p.getQuaternionFromEuler([0,0,0])
boxId = p.loadURDF("r2d2.urdf",startPosBox, startOrientationBox)

startPosTable = [5,0,0]
startOrientationTable = p.getQuaternionFromEuler([0,0,0])
TableId = p.loadURDF(path, startPosTable, startOrientationTable)

#numJoints = p.getNumJoints(boxId)
#print(numJoints)

#print(p.getJointInfo(boxId, 2))
#print(p.getJointInfo(boxId, 3))
#print(p.getJointInfo(boxId, 6))
#print(p.getJointInfo(boxId, 7))

maxForce = 500
targetVel = -0.01
#p.setJointMotorControl2(bodyUniqueId = boxId, jointIndex = 2, controlMode = p.VELOCITY_CONTROL, targetVelocity = targetVel, force = maxForce)
#p.setJointMotorControl2(bodyUniqueId = boxId, jointIndex = 3, controlMode = p.VELOCITY_CONTROL, targetVelocity = targetVel, force = maxForce)
#p.setJointMotorControl2(bodyUniqueId = boxId, jointIndex = 6, controlMode = p.VELOCITY_CONTROL, targetVelocity = targetVel, force = maxForce)
#p.setJointMotorControl2(bodyUniqueId = boxId, jointIndex = 7, controlMode = p.VELOCITY_CONTROL, targetVelocity = targetVel, force = maxForce)

#p.setJointMotorControlArray(bodyIndex = boxId, jointIndices = [2,3,6,7], controlMode = p.VELOCITY_CONTROL, targetVelocities = [targetVel, targetVel, targetVel, targetVel], forces = [maxForce,maxForce,maxForce,maxForce])
#p.applyExternalForce(objectUniqueId = boxId, linkIndex = -1, forceObj = [-10,0,0], posObj = [1,0,0], flags = p.WORLD_FRAME)

#set the center of mass frame (loadURDF sets base link frame) startPos/Ornp.resetBasePositionAndOrientation(boxId, startPos, startOrientation)
for i in range (10000):
    #print(p.getJointState(bodyUniqueId = boxId, jointIndex = 2))
    p.setJointMotorControlArray(bodyIndex = boxId, jointIndices = [2,3,6,7], controlMode = p.VELOCITY_CONTROL, targetVelocities = [i*targetVel, i*targetVel, i*targetVel, i*targetVel], forces = [maxForce,maxForce,maxForce,maxForce])
    p.stepSimulation()
    time.sleep(1./240.)

cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
print(cubePos,cubeOrn)

p.disconnect()
