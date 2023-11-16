import pybullet as p
import pybullet_data as pd 
import time

p.connect(p.GUI)
p.resetDebugVisualizerCamera( cameraDistance=.25, cameraYaw=115, cameraPitch=-25, cameraTargetPosition=[0,0,0.05])
robot = p.loadURDF("models/talos_gripper/urdf/talos_gripper_half.urdf",[0,0,0],[0,0,0,1], useFixedBase=True)

jointIds=[]
paramIds=[]

p.setPhysicsEngineParameter(numSolverIterations=100)
p.changeDynamics(robot,-1,linearDamping=0, angularDamping=0)
activeJoint=0
for j in range (p.getNumJoints(robot)):
	p.changeDynamics(robot,j,linearDamping=0, angularDamping=0)
	info = p.getJointInfo(robot,j)
	#print(info)
	jointName = info[1]
	jointType = info[2]
	if (jointType==p.JOINT_PRISMATIC or jointType==p.JOINT_REVOLUTE):
		jointIds.append(j)
		paramIds.append(p.addUserDebugParameter(jointName.decode("utf-8"),0,1,0))
		p.resetJointState(robot, j, 0)
		activeJoint+=1

p.setRealTimeSimulation(1)
while(1):
	for i in range(len(paramIds)):
		c = paramIds[i]
		targetPos = p.readUserDebugParameter(c)
		p.setJointMotorControl2(robot,jointIds[i],p.POSITION_CONTROL,targetPos, force=140.)	
	time.sleep(0.01)
