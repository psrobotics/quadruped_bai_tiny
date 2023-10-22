clear;
clc;

l1=29.1;
l2=48;
l3=38;
l6=34.5;
l5=33;
l4=60;

x=17.5;
y=90;

startup_rvc();

disp('Program started');

sim=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
sim.simxFinish(-1); % just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);

if (clientID>-1)
    disp('Connected to remote API server');
    
    joint_handle = [0 0 0 0 0 0 0 0 0 0 0 0];
    r_code = [0 0 0 0 0 0 0 0 0 0 0 0];
    [r_code(1),joint_handle(1)]=sim.simxGetObjectHandle(clientID,'j_hip_fl',sim.simx_opmode_blocking);
    [r_code(2),joint_handle(2)]=sim.simxGetObjectHandle(clientID,'j_l1_fl',sim.simx_opmode_blocking);
    [r_code(3),joint_handle(3)]=sim.simxGetObjectHandle(clientID,'j_l2_fl',sim.simx_opmode_blocking);
    
    [r_code(4),joint_handle(4)]=sim.simxGetObjectHandle(clientID,'j_hip_fr',sim.simx_opmode_blocking);
    [r_code(5),joint_handle(5)]=sim.simxGetObjectHandle(clientID,'j_l1_fr',sim.simx_opmode_blocking);
    [r_code(6),joint_handle(6)]=sim.simxGetObjectHandle(clientID,'j_l2_fr',sim.simx_opmode_blocking);
    
    [r_code(7),joint_handle(7)]=sim.simxGetObjectHandle(clientID,'j_hip_bl',sim.simx_opmode_blocking);
    [r_code(8),joint_handle(8)]=sim.simxGetObjectHandle(clientID,'j_l1_bl',sim.simx_opmode_blocking);
    [r_code(9),joint_handle(9)]=sim.simxGetObjectHandle(clientID,'j_l2_bl',sim.simx_opmode_blocking);
    
    [r_code(10),joint_handle(10)]=sim.simxGetObjectHandle(clientID,'j_hip_br',sim.simx_opmode_blocking);
    [r_code(11),joint_handle(11)]=sim.simxGetObjectHandle(clientID,'j_l1_br',sim.simx_opmode_blocking);
    [r_code(12),joint_handle(12)]=sim.simxGetObjectHandle(clientID,'j_l2_br',sim.simx_opmode_blocking);
    r_code
    
    disp('Got all joint handle!');
    
    rad_init = [0 0.35 0.5 0 -0.35 -0.5 0 -0.35 -0.5 0 0.35 0.5];
    
%     for step = 0:0.001:1.5
%         for i = 1:12
%             sim.simxSetJointTargetPosition(clientID,joint_handle(i),rad_init(i)*step,sim.simx_opmode_streaming);
%         end
%     end
    
    for x_step = 50:0.001:80
        [j_1,j_2]=leg_2d_ik(l1,l2,l3,l4,l5,l6,0,x_step)
        sim.simxSetJointTargetPosition(clientID,joint_handle(2),j_2-pi/2,sim.simx_opmode_streaming);
        sim.simxSetJointTargetPosition(clientID,joint_handle(3),j_1-pi/2,sim.simx_opmode_streaming);
    end
    
end