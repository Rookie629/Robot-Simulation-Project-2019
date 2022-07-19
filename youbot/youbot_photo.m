function youbot_photo()
%***********************************************************************************************************************
%|||||||||||||||||||||||||||||||||||||||������ʾ�����ʹ��youbot�����˵�RGB��ɫ�����ȡ��Ϣ|||||||||||||||||||||||||||||||||||
%***********************************************************************************************************************

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
%#################################################��ʼ���������###############################################################################
    % ��ʼ��youbot�����˻�ȡ������ؾ�� .
    h = youbot_init(vrep, id);
   % ��ʼ��Hokuyo�����״��ȡ������ؾ�� 
    h = youbot_hokuyo_init(vrep, h);
    % �ݶ�2s��֤��ʼ��������� 
    pause(2);

   
    % ͨ���ź������Ʋ�ɫ�������
     res = vrep.simxSetIntegerSignal(id, 'handle_rgb_sensor', 1, vrep.simx_opmode_oneshot_wait);
     vrchk(vrep, res);
    
    % ��ȡ��ɫ���������Ϣ
    fprintf('Capturing image...\n');
    [res, resolution, image] = vrep.simxGetVisionSensorImage2(id, h.rgbSensor, 0, vrep.simx_opmode_oneshot_wait);
    vrchk(vrep, res);
    fprintf('Captured %i pixels (%i x %i).\n', resolution(1) * resolution(2), resolution(1), resolution(2));
    %��ʾ��ɫͼ��
    figure; 
    imshow(image);
    drawnow;
end % main function
   
