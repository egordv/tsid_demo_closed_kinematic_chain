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

passiveJointIds=[]  

jointAnglesRefByName={}   

activeJoint=0
_joint_name_to_index = {}
for j in range (p.getNumJoints(robot)):
	p.changeDynamics(robot,j,linearDamping=0.1, angularDamping=0.1)
	info = p.getJointInfo(robot,j)
	jointName = info[1]
	jointType = info[2]
	if (jointType==p.JOINT_PRISMATIC or jointType==p.JOINT_REVOLUTE):
		name = jointName.decode("utf-8")
		jointIds.append(j)
		_joint_name_to_index[name] = j
		refAngle = 0
		if name in jointAnglesRefByName.keys():
			refAngle = jointAnglesRefByName[name]
		p.resetJointState(robot, j, refAngle)
		activeJoint+=1
		
		# Do not include joints marked as passive by it's name in URDF to pybullet GUI
		if name.find("_PASSIVE") != -1:
			passiveJointIds.append(j)
			print("Passive joint found: ", j, " - ", name)	
		else:
			paramIds.append(p.addUserDebugParameter(jointName.decode("utf-8"),0,1,refAngle))

_link_name_to_index = {p.getBodyInfo(robot)[0].decode('UTF-8'):-1,}
for _id in range(p.getNumJoints(robot)):
	_name = p.getJointInfo(robot, _id)[12].decode('UTF-8')
	_link_name_to_index[_name] = _id

for j in passiveJointIds:
	p.setJointMotorControl2(robot, j, p.VELOCITY_CONTROL, force=0.)	


c1 = p.createConstraint( robot, 
					_link_name_to_index["gripper_left_motor_single_link_ckc_axis"], 
					robot, 
					_link_name_to_index["gripper_left_fingertip_3_link_ckc_axis"], 
					p.JOINT_POINT2POINT,
					jointAxis=[0, 0, 0], 
					parentFramePosition=[0, 0, 0], 
					childFramePosition=[0, 0, 0])
p.changeConstraint(c1, maxForce=10000,erp=1.0)  


p.setRealTimeSimulation(1)
while(1):
	#p.getCameraImage(320,200)
	for i in range(len(paramIds)):
		c = paramIds[i]
		targetPos = p.readUserDebugParameter(c)

		if jointIds[i] in passiveJointIds:
			# Passive joints - set zero torque
			p.setJointMotorControl2(robot,jointIds[i],p.TORQUE_CONTROL,targetPos, force=0.)	
		else:
			# Active joint - control by positiont
			p.setJointMotorControl2(robot,jointIds[i],p.POSITION_CONTROL,targetPos, force=140.)	 
	time.sleep(0.01)
