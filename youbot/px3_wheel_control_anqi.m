function px3_wheel_control()
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
	%��ʼ��������Ʊ���
	h = PX3_init(vrep, id);
	res = vrep.simxSynchronous(h.id, true); vrchk(vrep, res);

%####################################################����������״̬������######################################################################
	%��ʼ����״̬��
	%fsm=XXXX
	while true,
	%���ڴ˴�����״̬����ش���
    %��ȡ�ƶ������˱����������ϵ�������������ϵ��λ�ú���̬
    [res youbotPos] = vrep.simxGetObjectPosition(h.id, h.Pioneer_p3dx, -1,vrep.simx_opmode_buffer); vrchk(vrep, res);
    [res youbotEuler] = vrep.simxGetObjectOrientation(h.id, h.Pioneer_p3dx, -1,vrep.simx_opmode_buffer); vrchk(vrep, res); 
    youbotPos
    youbotEuler
	%�����ƶ�����������ģ�ͽ�ģ����
    r=0.0975;
    b=0.1655;
    forwBackVel=0.1;
    rotVel=0.1;
	vLeft=(forwBackVel-b*rotVel)/r;
    vRight=(forwBackVel+b*rotVel)/r;
	%��vrep����˫��ת��
    vrep.simxSetJointTargetVelocity(h.id, h.wheelJoints(1),vLeft,vrep.simx_opmode_oneshot); vrchk(vrep, res);
    vrep.simxSetJointTargetVelocity(h.id, h.wheelJoints(2),vRight,vrep.simx_opmode_oneshot); vrchk(vrep, res);
    if 1,
    cellsize=0.25
    [X,Y] = meshgrid((-7.5+cellsize/2):cellsize:(7.5-cellsize/2),(-7.5+cellsize/2):cellsize:(7.5-cellsize/2));
    plot( youbotPos(1), youbotPos(2), 'ob',7.5, 0, 'or', 0, 7.5, 'og');
    axis equal;
    axis([-7.8 7.8 -7.8 7.8]);
  end
  drawnow;
	vrep.simxSynchronousTrigger(id);
	%end while
	end
%endfunction
disp('Program ended');
end



