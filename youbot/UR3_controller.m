function joint_controller()
%###############################################################固定程序创建VREP和matlab的连接并远程启动###################################################################
	disp('Program started');
	%加载remoteApiProto.m文件
	vrep=remApi('remoteApi'); 
	%关闭所有VREP和matlab的连接
	vrep.simxFinish(-1); 
	%创建新的VREP和matlab的连接，返回状态为id，如果id<0代表创建连接失败
	id=vrep.simxStart('127.0.0.1',19997,true,true,5000,5);
	if id < 0,
		disp('Failed connecting to remote API server. Exiting.');
		vrep.delete();
		return;
	end
	fprintf('Connection %d to remote API server open.\n', id);
	% Make sure we close the connexion whenever the script is interrupted.
	cleanupObj = onCleanup(@() cleanup_vrep(vrep, id));
	%由matlab远程启动VREP的仿真
	res = vrep.simxStartSimulation(id, vrep.simx_opmode_oneshot_wait);
%#################################################初始化程序变量###############################################################################
	h = UR3_init(vrep, id);	
    N_step=100;
	%利用五次规划方法计算reach状态的关节角度序列
	[joint1,joint1d,joint1dd]=tpoly(0*pi/180,30*pi/180,N_step);
	[joint2,joint2d,joint2dd]=tpoly(10*pi/180,0*pi/180,N_step);
	[joint3,joint3d,joint3dd]=tpoly(-60*pi/180,60*pi/180,N_step);
	[joint4,joint4d,joint4dd]=tpoly(0*pi/180,90*pi/180,N_step);
	[joint5,joint5d,joint5dd]=tpoly(0*pi/180,-180*pi/180,N_step);
	[joint6,joint6d,joint6dd]=tpoly(0*pi/180,90*pi/180,N_step);
	i=1;
	fsm='reach';
	%启动VREP和matlab同步处理机制
	res = vrep.simxSynchronous(h.id, true); vrchk(vrep, res);
%计算目标物体相对于机械臂基坐标系的位置和姿濠
   	[res targetPos] = vrep.simxGetObjectPosition(h.id,h.cylinder,h.UR3_base,vrep.simx_opmode_buffer)
	[res targetEuler] = vrep.simxGetObjectOrientation(h.id, h.cylinder, h.UR3_base,vrep.simx_opmode_buffer)
%####################################################机器人有限状态机部分######################################################################
%开始有限状态机
	while true,
	if strcmp(fsm,'reach')
       res = vrep.simxSetJointTargetPosition(id, h.UR_Joints(1), joint1(i),vrep.simx_opmode_oneshot_wait);
        vrchk(vrep, res, true);
        res = vrep.simxSetJointTargetPosition(id, h.UR_Joints(2), joint2(i),vrep.simx_opmode_oneshot_wait);
        vrchk(vrep, res, true);
        res = vrep.simxSetJointTargetPosition(id, h.UR_Joints(3), joint3(i),vrep.simx_opmode_oneshot_wait);
        vrchk(vrep, res, true);
        res = vrep.simxSetJointTargetPosition(id, h.UR_Joints(4), joint4(i),vrep.simx_opmode_oneshot_wait);
        vrchk(vrep, res, true);
        res = vrep.simxSetJointTargetPosition(id, h.UR_Joints(5), joint5(i),vrep.simx_opmode_oneshot_wait);
        vrchk(vrep, res, true);
        res = vrep.simxSetJointTargetPosition(id, h.UR_Joints(6), joint6(i),vrep.simx_opmode_oneshot_wait);
        vrchk(vrep, res, true);

        i=i+1;
        if i<N_step
            fsm='reach';
        else
            fsm='back';
            %计算目标物体相对于机械臂基坐标系的位置和姿濠
   	[res robotPos] = vrep.simxGetObjectPosition(h.id,h.cylinder,h.UR3_base,vrep.simx_opmode_buffer)
	[res robotEuler] = vrep.simxGetObjectOrientation(h.id, h.cylinder, h.UR3_base,vrep.simx_opmode_buffer)
   
    robotPos(2)=robotPos(2)-0.1
            pause(2);
            i=1;
        end
    elseif strcmp(fsm,'back')
	%启动笛卡尔空间控制模式
    % %1:joint control   2:decart control
	res = vrep.simxSetIntegerSignal(id, 'km_mode', 2, vrep.simx_opmode_oneshot_wait);vrchk(vrep, res, true);


	res = vrep.simxSetObjectPosition(id, h.UR3_target, h.UR3_base,robotPos , vrep.simx_opmode_oneshot);
    res = vrep.simxSetObjectOrientation(id, h.UR3_target, h.UR3_base, robotEuler, vrep.simx_opmode_oneshot);

            %fsm='finished';
            pause(20);
    elseif strcmp(fsm,'finished')

            break;
    end
  vrep.simxSynchronousTrigger(id);

%end while
end
%endfunction
disp('Program ended');
end



