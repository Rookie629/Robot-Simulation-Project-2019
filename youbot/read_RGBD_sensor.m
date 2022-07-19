function read_RGBD_sensor()
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
	h = RGBD_sensor_init(vrep, id)
	%����VREP��matlab��ͬ������
	res = vrep.simxSynchronous(h.id, true); vrchk(vrep, res);


%#################################################��ʼ����״̬��#########################################
	while true,
		%��ȡ��ɫͼ����Ϣ
		[res, resolution, image] = vrep.simxGetVisionSensorImage2(id, h.rgbSensor, 0, vrep.simx_opmode_oneshot_wait);
		%��ȡXYZ�״�ͼ����Ϣ
		pts = youbot_xyz_sensor(vrep, h, vrep.simx_opmode_oneshot_wait);
		%pts
		%�����״�ͼ��X-Y-Z��Ϣ���ߣ�������Ҫ�ſ�ע�ͣ�
		%plot(pts(1,:))
		% hold on
		% plot(pts(2,:))
		% hold on
        subplot(1,2,1)
		plot(pts(3,:))
		% hold on
		% plot(pts(4,:))
		%plot3(pts(1,:),pts(3,:),pts(2,:))
		%��ʾ��ɫͼ��
        subplot(1,2,2)
		imshow(image);
        RP_R=image(:,:,1); 
        RP_G=image(:,:,2); 
        RP_B=image(:,:,3);
		%�����Դ�������
		vrep.simxSynchronousTrigger(id);
		%��ͣ1s
		pause(1)
		%end while
	end
	%endfunction
	disp('Program ended');
end



