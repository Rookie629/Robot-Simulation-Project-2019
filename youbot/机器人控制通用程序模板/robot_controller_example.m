function robot_controller_example()
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
	%h=robot_init(vrep, id);
	%fsm=XXXX
	%����VREP��matlab��ͬ������
	res = vrep.simxSynchronous(h.id, true); vrchk(vrep, res);
%#############################################����������״̬������######################################################################
	%��ʼ����״̬��
	while true,
	%���ڴ˴�����״̬����ش���
	

	
	
	 %����ͬ��
	 vrep.simxSynchronousTrigger(id);
	%end while
	end
%����������
disp('Program ended');
%endfunction
end



