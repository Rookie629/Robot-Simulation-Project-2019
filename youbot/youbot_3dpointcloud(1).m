function youbot_3dpointcloud()
%***********************************************************************************************************************
%|||||||||||||||||||||||||||||||||||||||������ʾ�����ʹ��youbot�����˵�XYZ�����ȡ��Ϣ|||||||||||||||||||||||||||||||||||
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
        
    vrep.simxSetObjectOrientation(id,h.rgbdCasing,h.ref,[0 0 125*pi/180], vrep.simx_opmode_oneshot);
    
    % ����RGBD����Ĺ�Ƕ���
    res = vrep.simxSetFloatSignal(id, 'rgbd_sensor_scan_angle', pi / 8, vrep.simx_opmode_oneshot_wait);
    vrchk(vrep, res);
    
    
    % ͨ���ź�������XYZ������� 
    res = vrep.simxSetIntegerSignal(id, 'handle_xyz_sensor', 1, vrep.simx_opmode_oneshot_wait);
    vrchk(vrep, res);
    
    % ͨ���ź������Ʋ�ɫ�������
    res = vrep.simxSetIntegerSignal(id, 'handle_rgb_sensor', 1, vrep.simx_opmode_oneshot_wait);
    vrchk(vrep, res);
    
    % ��ȡ��ɫ���������Ϣ

    [res, resolution, image] = vrep.simxGetVisionSensorImage2(id, h.rgbSensor, 0, vrep.simx_opmode_oneshot_wait);
    vrchk(vrep, res);

    %��ʾ��ɫͼ��
    figure(1); 
    imshow(image);
    
    

    % ��ȡXYZ���3Dͼ�����ݱ��浽pts����. 
    fprintf('Capturing point cloud...\n');
    pts = youbot_xyz_sensor(vrep, h, vrep.simx_opmode_oneshot_wait);
plot3(pts(1,:),pts(3,:),pts(2,:))
    % Plot all the points. 
%     figure(2);
%     plot3(pts(1, :), pts(2, :), pts(3, :), '*');
    

		%��ɫ����ʶ��
		diff_R = 100;
		Image=image;
        
        figure(2)
        imshow(Image)
        
		RP_R=Image(:,:,1); 
		RP_G=Image(:,:,2); 
		RP_B=Image(:,:,3);
		XYR=255*((RP_G-RP_B)>diff_R&(RP_R-RP_B)>diff_R); 
		%Ѱ�Һ�ɫ�������Ӿ���
		bw=im2bw(XYR);
		[r c]=find(bw==1);  
		[rectx,recty,area,perimeter] = minboundrect(c,r,'a'); 
		%line(rectx(:),recty(:),'color','r');
		%Ѱ�Һ�ɫ����ļ������ĵ�����
		rgb_target_centor(1)=(rectx(1)+rectx(2))/2;
		rgb_target_centor(2)=(recty(1)+recty(3))/2;

		%���������ĵ�������XYZ����ӳ�䣬���㼸�����ĵ���������λ��
		xyz_target_centor=32-ceil(rgb_target_centor/(512/32))+1;
		xyz_target_num=xyz_target_centor(1)+(xyz_target_centor(2)-1)*32;
		pts(:,xyz_target_num-2:xyz_target_num+2)
        
        arm_targetpos_camera=[pts(1,xyz_target_num) pts(2,xyz_target_num) pts(3,xyz_target_num)];
        
        [res, rgbdp]=vrep.simxGetObjectPosition(h.id, h.xyzSensor, h.armRef, vrep.simx_opmode_buffer)  ;
        [res, rgbdpo]=vrep.simxGetObjectOrientation(h.id, h.xyzSensor, h.armRef, vrep.simx_opmode_buffer) ;

        T =transl(rgbdp)*trotx(rgbdpo(1))*troty(rgbdpo(2))*trotz(rgbdpo(3));
        
        [arm_targetpos_camera(1);arm_targetpos_camera(2);arm_targetpos_camera(3);1]
        arm_targetpos_armref =T* [arm_targetpos_camera(1);arm_targetpos_camera(2);arm_targetpos_camera(3);1]
        
        
   

end % main function
   
