function RGBD_sensor_process()
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
  % ��ʼ��youbot�����˻�ȡ������ؾ�� .
    h = youbot_init(vrep, id);
   % ��ʼ��Hokuyo�����״��ȡ������ؾ�� 
    h = youbot_hokuyo_init(vrep, h);
    % �ݶ�2s��֤��ʼ��������� 
    pause(2);
    
   fsm_sight='identify_R';
   diff_R=100;
 
    
%#################################################��ʼ����״̬��#########################################
	while true,
 % ����RGBD����Ĺ�Ƕ���
    res = vrep.simxSetFloatSignal(id, 'rgbd_sensor_scan_angle', pi / 8, vrep.simx_opmode_oneshot_wait);
    vrchk(vrep, res);
    
    
    % ͨ���ź�������XYZ������� 
    res = vrep.simxSetIntegerSignal(id, 'handle_xyz_sensor', 1, vrep.simx_opmode_oneshot_wait);
    vrchk(vrep, res);
    
    % ͨ���ź������Ʋ�ɫ�������
    res = vrep.simxSetIntegerSignal(id, 'handle_rgb_sensor', 1, vrep.simx_opmode_oneshot_wait);
    vrchk(vrep, res);
 
        if strcmp(fsm_sight,'identify_R')
            theta=30*pi/180;
    vrep.simxSetObjectOrientation(id,h.rgbdCasing,h.ref,[0 0 theta], vrep.simx_opmode_oneshot);    
		%��ȡXYZ�״�ͼ����Ϣ
       fprintf('Capturing point cloud...\n');
       pts = youbot_xyz_sensor(vrep, h, vrep.simx_opmode_oneshot_wait);

		%pts = youbot_xyz_sensor(vrep, h, vrep.simx_opmode_oneshot_wait);
		%pts
		%�����״�ͼ��X-Y-Z��Ϣ���ߣ�������Ҫ�ſ�ע�ͣ�
		%plot(pts(1,:))
		% hold on
		% plot(pts(2,:))
		% hold on
		%plot(pts(3,:))
		% hold on
		% plot(pts(4,:))
		%plot3(pts(1,:),pts(3,:),pts(2,:))

		%��ȡ��ɫͼ����Ϣ
     [res, resolution, image] = vrep.simxGetVisionSensorImage2(id, h.rgbSensor, 0, vrep.simx_opmode_oneshot_wait);
    vrchk(vrep, res);
 %��ʾ��ɫͼ��
    figure(1); 
    imshow(image);
    
    

    % ��ȡXYZ���3Dͼ�����ݱ��浽pts����. 
    fprintf('Capturing point cloud...\n');
    pts = youbot_xyz_sensor(vrep, h, vrep.simx_opmode_oneshot_wait);
 %�ر����д���
        close all;
        %��ȡ����ʾͼ��
        figure(1)
        %I = imread('sim.png');
        imshow(image)
        %�趨��ɫ��ֵ
        diff_R = 100;
        Image=image;
        %��ȡ��ɫ����
        RP_R=Image(:,:,1); 
        RP_G=Image(:,:,2); 
        RP_B=Image(:,:,3);
        %��ɫ�������
        XYR=255*((RP_R-RP_G)>diff_R&(RP_R-RP_B)>diff_R); 
        %���ƺڰ�ͼ
        figure(2)
        imshow(uint8(XYR));
        %תΪ��ֵͼ
        bw=im2bw(XYR);
        %Ѱ�Һ�ɫ����
        [r c]=find(bw==1)
        %Ѱ�Һ�ɫ�������С��Ӿ���   
        [rectx,recty,area,perimeter] = minboundrect(c,r,'a');
        %������Ӿ��ζ������ֱ�߿� 
        line(rectx(:),recty(:),'color','r');
        %��ʾ��Ӿ��ζ�������
        %rectx
        %recty
        % end
		%Imagecenterx=(rectx(1)+rectx(3))/2
        %Imagecentery=(recty(1)+recty(3))/2
        %xyzcenterx=32*Imagecenterx/512
        %xyzcentery=32*Imagecentery/512
		rectx
		recty
		rgb_target_centor(1)=(rectx(1)+rectx(2))/2
		rgb_target_centor(2)=(recty(1)+recty(3))/2

		%���������ĵ�������XYZ����ӳ�䣬���㼸�����ĵ���������λ��
		xyz_target_centor=32-ceil(rgb_target_centor/(512/32))+1
		xyz_target_num=xyz_target_centor(1)+(xyz_target_centor(2)-1)*32
		pts(:,xyz_target_num-2:xyz_target_num+2);
        arm_targetpos_camera=[pts(1,xyz_target_num) pts(2,xyz_target_num) pts(3,xyz_target_num)];
        [res,rgbdp]=vrep.simxGetObjectPosition(h.id,h.xyzSensor,h.armRef,vrep.simx_opmode_buffer)  
        [res,rgbdpo]=vrep.simxGetObjectOrientation(h.id,h.xyzSensor,h.armRef,vrep.simx_opmode_buffer)
        T=transl(rgbdp)*trotx(rgbdpo(1))*trotx(rgbdpo(2)) *trotx(rgbdpo(3));
        arm_targetpos_armref=T*[arm_targetpos_camera(1);arm_targetpos_camera(2);arm_targetpos_camera(3);1]
        
        
    % Plot all the points. 
    figure(2);
    plot3(pts(1, :), pts(2, :), pts(3, :), '*');
		
	
        fsm_sight='capture'
        end
    %end
	%endfunction
	disp('Program ended');
end



