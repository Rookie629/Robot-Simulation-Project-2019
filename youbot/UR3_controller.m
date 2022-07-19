function joint_controller()
%###############################################################�̶����򴴽�VREP��matlab�����Ӳ�Զ������###################################################################
	disp('Program started');
	%����remoteApiProto.m�ļ�
	vrep=remApi('remoteApi'); 
	%�ر�����VREP��matlab������
	vrep.simxFinish(-1); 
	%�����µ�VREP��matlab�����ӣ�����״̬Ϊid�����id<0����������ʧ��
	id=vrep.simxStart('127.0.0.1',19997,true,true,5000,5);
	if id < 0,
		disp('Failed connecting to remote API server. Exiting.');
		vrep.delete();
		return;
	end
	fprintf('Connection %d to remote API server open.\n', id);
	% Make sure we close the connexion whenever the script is interrupted.
	cleanupObj = onCleanup(@() cleanup_vrep(vrep, id));
	%��matlabԶ������VREP�ķ���
	res = vrep.simxStartSimulation(id, vrep.simx_opmode_oneshot_wait);
%#################################################��ʼ���������###############################################################################
	h = UR3_init(vrep, id);	
    N_step=100;
	%������ι滮��������reach״̬�ĹؽڽǶ�����
	[joint1,joint1d,joint1dd]=tpoly(0*pi/180,30*pi/180,N_step);
	[joint2,joint2d,joint2dd]=tpoly(10*pi/180,0*pi/180,N_step);
	[joint3,joint3d,joint3dd]=tpoly(-60*pi/180,60*pi/180,N_step);
	[joint4,joint4d,joint4dd]=tpoly(0*pi/180,90*pi/180,N_step);
	[joint5,joint5d,joint5dd]=tpoly(0*pi/180,-180*pi/180,N_step);
	[joint6,joint6d,joint6dd]=tpoly(0*pi/180,90*pi/180,N_step);
	i=1;
	fsm='reach';
	%����VREP��matlabͬ���������
	res = vrep.simxSynchronous(h.id, true); vrchk(vrep, res);
%����Ŀ����������ڻ�е�ۻ�����ϵ��λ�ú����
   	[res targetPos] = vrep.simxGetObjectPosition(h.id,h.cylinder,h.UR3_base,vrep.simx_opmode_buffer)
	[res targetEuler] = vrep.simxGetObjectOrientation(h.id, h.cylinder, h.UR3_base,vrep.simx_opmode_buffer)
%####################################################����������״̬������######################################################################
%��ʼ����״̬��
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
            %����Ŀ����������ڻ�е�ۻ�����ϵ��λ�ú����
   	[res robotPos] = vrep.simxGetObjectPosition(h.id,h.cylinder,h.UR3_base,vrep.simx_opmode_buffer)
	[res robotEuler] = vrep.simxGetObjectOrientation(h.id, h.cylinder, h.UR3_base,vrep.simx_opmode_buffer)
   
    robotPos(2)=robotPos(2)-0.1
            pause(2);
            i=1;
        end
    elseif strcmp(fsm,'back')
	%�����ѿ����ռ����ģʽ
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



