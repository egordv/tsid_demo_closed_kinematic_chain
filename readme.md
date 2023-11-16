This repo contains a small demo how to create a contact between arbitrary robot model frames for TSID framework (https://github.com/stack-of-tasks/tsid)

This code is related to the issue "TSID for "pantograph/biarticular" robot leg design" https://github.com/stack-of-tasks/tsid/issues/165

Usage:
1. Install TSID, meshcat, pybullet
2. Run demo_pybullet_talos_gripper_open_chains.py to play with gripper model **without** closed kinematic chains
3. Run demo_pybullet_talos_gripper_closed_chains.py to play with gripper model **with** closed kinematic chains
4. Run demo_tsid_talos_gripper_closed_kinematic_chain.py to check how the closed kinematic chain works in TSID with Meshcat visualisation, emlulating the same behaivour as in pybullet  

Sample how this gripper works **with** additional contact via ContactTwoFramePositions thus creating a closed kinematic chain:  
![TalosGripperWithClosedKinematicChain](https://github.com/stack-of-tasks/tsid/assets/40291783/900cc0ae-c727-44c2-8dd6-d64202d2333e)

Sample how this gripper works **without** additional contact (no closed kinematic chains used):  
![TalosGripperNoClosedKinematicChain](https://github.com/stack-of-tasks/tsid/assets/40291783/fb0ae0cb-cba8-4e65-a72c-e51b8ec0daeb)


