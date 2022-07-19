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
	h =RGBD_sensor_init(vrep, id);
	%����VREP��matlab��ͬ������
	res = vrep.simxSynchronous(h.id, true); vrchk(vrep, res);
    diff_R = 100;

%#################################################��ʼ����״̬��#########################################
	while true,
		%��ȡXYZ�״�ͼ����Ϣ
		pts = youbot_xyz_sensor(vrep, h, vrep.simx_opmode_oneshot_wait);
		%pts
		%�����״�ͼ��X-Y-Z��Ϣ���ߣ�������Ҫ�ſ�ע�ͣ�
		%plot(pts(1,:))
		% hold on
		% plot(pts(2,:))
		% hold on
		plot(pts(3,:))
		% hold on
		% plot(pts(4,:))
		%plot3(pts(1,:),pts(3,:),pts(2,:))

		%��ȡ��ɫͼ����Ϣ
		[res, resolution, I] = vrep.simxGetVisionSensorImage2(id, h.rgbSensor, 0, vrep.simx_opmode_oneshot_wait);
		    %��ʾ��ɫͼ��
        figure(1); 
        imshow(image);
		%����Ŀ�����弸����������������ϵ��λ��  IΪ��ɫͼƬ��Ϣ��ptsΪ�����Ϣ��3�о���
		
		RP_R=I(:,:,1); 
        RP_G=I(:,:,2); 
        RP_B=I(:,:,3);
        %��ɫ�������
        XYR=255*((RP_R-RP_G)>diff_R&(RP_R-RP_B)>diff_R); 
        %���ƺڰ�ͼ
        figure(2)
        imshow(uint8(XYR));
        %תΪ��ֵͼ
        bw=im2bw(XYR);
        %Ѱ�Һ�ɫ����
        [r c]=find(bw==1);
        %Ѱ�Һ�ɫ�������С��Ӿ���  
        [rectx,recty,area,perimeter] = minboundrect(c,r,'a');
        %������Ӿ��ζ������ֱ�߿� 
        line(rectx(:),recty(:),'color','r');
        %��ʾ��Ӿ��ζ�������
		center(1)=(rectx(1)+rectx(2))/2;
        center(2)=(recty(1)+recty(3))/2;
        target_center=32-ceil(center/(512/32))+1;
        target_center_numb=target_center(1)+(target_center(2)-1)*32;
        imgcenter=pts(:,target_center_numb-2:target_center_numb+2)
        %x=pts(1,center)
        %y=pts(2,center)
		%z=pts(3,center)
		
		%�����Դ�������
		vrep.simxSynchronousTrigger(id);
		%��ͣ1s
		pause(1)
		%end while
	end
	%endfunction
	disp('Program ended');
end



