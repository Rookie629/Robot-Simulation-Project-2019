function youbot_moving_arm2()

%###############################################################�̶����򴴽�VREP��matlab�����Ӳ�Զ������###################################################################
    disp('Program started');
    %����remoteApiProto.m�ļ� 
    vrep = remApi('remoteApi');
    %�ر�����VREP��matlab������
    vrep.simxFinish(-1);
    %�����µ�VREP��matlab�����ӣ�����״̬Ϊid�����id<0����������ʧ��
    id = vrep.simxStart('127.0.0.1', 19997, true, true, 2000, 5);
    if id < 0
        disp('Failed connecting to remote API server. Exiting.');
        vrep.delete();
        return;
    end
    fprintf('Connection %d to remote API server open.\n', id);
    % ȷ�����������жϺ�ϵͳ�Զ����VREP��matlab����������
    cleanupObj = onCleanup(@() cleanup_vrep(vrep, id));
	%��matlabԶ������VREP�ķ���
    vrep.simxStartSimulation(id, vrep.simx_opmode_oneshot_wait);
%###############################################################################��ʼ���������###############################################################################
    % ��ʼ��youbot�����˻�ȡ������ؾ�� .
    h = youbot_init(vrep, id);
   % ��ʼ��Hokuyo�����״��ȡ������ؾ�� 
    h = youbot_hokuyo_init(vrep, h);
    % �ݶ�2s��֤��ʼ��������� 
    pause(2);
      % ����RGBD����Ĺ�Ƕ���
      
          
        vrep.simxSetObjectOrientation(id,h.rgbdCasing,h.ref,[0 0 125*pi/180], vrep.simx_opmode_oneshot);    
    res = vrep.simxSetFloatSignal(id, 'rgbd_sensor_scan_angle', pi / 8, vrep.simx_opmode_oneshot_wait);
    vrchk(vrep, res);
    
    
    % ͨ���ź�������XYZ������� 
    res = vrep.simxSetIntegerSignal(id, 'handle_xyz_sensor', 1, vrep.simx_opmode_oneshot_wait);
    vrchk(vrep, res);
    
    % ͨ���ź������Ʋ�ɫ�������
    res = vrep.simxSetIntegerSignal(id, 'handle_rgb_sensor', 1, vrep.simx_opmode_oneshot_wait);
    vrchk(vrep, res);
    
    % Make sure everything is settled before we start. 
    pause(2);

    % ��ʼ��״̬��

    fsm='identify';

    
        % ��ȡ�ƶ��������������������ϵ��λ�ú���̬ 
        [res, youbotPos] = vrep.simxGetObjectPosition(id, h.ref, -1, vrep.simx_opmode_buffer);
        vrchk(vrep, res, true);
        [res, youbotEuler] = vrep.simxGetObjectOrientation(id, h.ref, -1, vrep.simx_opmode_buffer);
        vrchk(vrep, res, true);
          %��ת״̬��   

             
            % ��ȡ��ɫ���������Ϣ

            [res, resolution, image] = vrep.simxGetVisionSensorImage2(id, h.rgbSensor, 0, vrep.simx_opmode_oneshot_wait);
            vrchk(vrep, res);

             % ��ȡXYZ���3Dͼ�����ݱ��浽pts����. 
             fprintf('Capturing point cloud...\n');
             pts = youbot_xyz_sensor(vrep, h, vrep.simx_opmode_oneshot_wait);
             plot3(pts(1,:),pts(3,:),pts(2,:))
 




end % main function
   
