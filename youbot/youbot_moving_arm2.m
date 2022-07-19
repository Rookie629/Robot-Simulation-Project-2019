function youbot_moving_arm2()
%***********************************************************************************************************************
%|||||||||||||||||||||||||||||||||||||||������ʾ�����ʹ��youbot�����˵ĵ������ɶȼ�״̬������ܹ���ʹ�÷���|||||||||||||||||||||||||||||||||||
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
    %����״̬��
    diff_R = 100;
    diff_Y = 75;

    fsm_sight='red';
    % Youbot constants
    timestep = .05;

    forwBackVel = 0;
    leftRightVel = 0;
    rotVel = 0;
    prevOri = 0; 
    prevLoc = 0;
    %�����е�۹ؽڿռ��˶���Χ
    armJointRanges = [-2.9496064186096, 2.9496064186096;
                      -1.5707963705063, 1.308996796608;
                      -2.2863812446594, 2.2863812446594;
                      -1.7802357673645, 1.7802357673645;
                      -1.5707963705063, 1.5707963705063 ];

    % Make sure everything is settled before we start. 
    pause(2);
    
    % ��ʼ��״̬��
    fsm = 'Rot1';

    %% Start the demo. 
    while true
        tic % See end of loop to see why it's useful. 
        
        if vrep.simxGetConnectionId(id) == -1
          error('Lost connection to remote API.');
        end
    
        % ��ȡ�ƶ��������������������ϵ��λ�ú���̬ 
        [res, youbotPos] = vrep.simxGetObjectPosition(id, h.ref, -1, vrep.simx_opmode_buffer);
        vrchk(vrep, res, true);
        [res, youbotEuler] = vrep.simxGetObjectOrientation(id, h.ref, -1, vrep.simx_opmode_buffer);
        vrchk(vrep, res, true);
        % ��ȡ��е��ĩ��tip����ϵ��λ��
    [res, homeGripperPosition] = vrep.simxGetObjectPosition(id, h.ptip, h.armRef, vrep.simx_opmode_buffer);
    vrchk(vrep, res, true);
                %��ת״̬��   
        if strcmp(fsm, 'Rot1');
            % Rotate. 
            rotVel = angdiff( pi / 2, youbotEuler(3));
          h = youbot_drive(vrep, h, forwBackVel, leftRightVel, rotVel);   
            % Stop when the robot is at an angle close to -pi/2. 
            if abs(angdiff( pi / 2, youbotEuler(3))) < .1 / 180 * pi
                rotVel = 0;
                fsm = 'lin_forwBack1';
            end

        


        %������ǰ���ƶ�״̬��. 
        %elseif strcmp(fsm, 'cst_forwBack');
            
            %ǰ���ٶ�
            %forwBackVel = -1;
            
            % ʹ�û����˱ƽ�Y=-6.5��λ��. 
            %if youbotPos(2) + 5.3 < .001
            %    forwBackVel = 0;
             %   fsm = 'lin_forwBack';
            %end
        %    
        elseif strcmp(fsm, 'lin_forwBack1');
            %���ƻ�����ǰ���˶��ٶ���Y=-3λ�����
            forwBackVel = 2 * (youbotPos(1) + 3);
            %forwBackVel = -2 
         
            
            % ʹ�û����˱ƽ�Y=-3��λ��. 
            if abs(youbotPos(1) + 2.8) <0.01
                forwBackVel = 0;
                fsm = 'LeftRight1';
            end
            
        elseif strcmp(fsm, 'LeftRight1');
            % ���ƻ�����ǰ���˶��ٶ���X=-5.3λ�����
            leftRightVel = - 2 * (youbotPos(2) +5.3);
           
            
            % ʹ�û����˱ƽ�X=-5.6��λ��.
            if abs(youbotPos(2) + 5.3) < .001
                leftRightVel = 0;
                          fsm = 'identify';
                 pause(1);
            end
        elseif strcmp(fsm,'identify')
             
            % ��ȡ��ɫ���������Ϣ
             pause(2);
            [res, resolution, image] = vrep.simxGetVisionSensorImage2(id, h.rgbSensor, 0, vrep.simx_opmode_oneshot_wait);
            vrchk(vrep, res);

             %��ʾ��ɫͼ��
             figure(1); 
             imshow(image);
    
    

             % ��ȡXYZ���3Dͼ�����ݱ��浽pts����. 
             fprintf('Capturing point cloud...\n');
             pts = youbot_xyz_sensor(vrep, h, vrep.simx_opmode_oneshot_wait);
              plot3(pts(1,:),pts(3,:),pts(2,:))
             %�ر����д���
             close all;
             %��ȡ����ʾͼ��
              figure(1)
              %I = imread('sim.png');
              imshow(image)
              %�趨��ɫ��ֵ
              Image=image;
              %��ȡ��ɫ����
              RP_R=Image(:,:,1); 
              RP_G=Image(:,:,2); 
              RP_B=Image(:,:,3);
              %��ɫ�������
              %XYR=255*((RP_R-RP_G)>diff_R&(RP_R-RP_B)>diff_R); 
              XYY=255*((RP_R-RP_B)>diff_Y&(RP_G-RP_B)>diff_Y); 
              %���ƺڰ�ͼ
              figure(2)
              imshow(uint8(XYY));
              %תΪ��ֵͼ
              bw=im2bw(XYY);
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
		      rectx;
		      recty;
		      rgb_target_centor(1)=(rectx(1)+rectx(2))/2;
		      rgb_target_centor(2)=(recty(1)+recty(3))/2;

		      %���������ĵ�������XYZ����ӳ�䣬���㼸�����ĵ���������λ��
		      xyz_target_centor=32-ceil(rgb_target_centor/(512/32))+1;
		      xyz_target_num=xyz_target_centor(1)+(xyz_target_centor(2)-1)*32;
		      pts(:,xyz_target_num-2:xyz_target_num+2);
              arm_targetpos_camera=[pts(1,xyz_target_num) pts(2,xyz_target_num) pts(3,xyz_target_num)];
              [res,rgbdp]=vrep.simxGetObjectPosition(h.id,h.xyzSensor,h.armRef,vrep.simx_opmode_buffer);
              [res,rgbdpo]=vrep.simxGetObjectOrientation(h.id,h.xyzSensor,h.armRef,vrep.simx_opmode_buffer); 
               rgbdpo
              T=transl(rgbdp)*trotx(rgbdpo(1))*troty(rgbdpo(2)) *trotz(rgbdpo(3))
              arm_targetpos_armref=T*[arm_targetpos_camera(1);arm_targetpos_camera(2);arm_targetpos_camera(3);1]
              cylinder_center=[arm_targetpos_armref(1),arm_targetpos_armref(2),arm_targetpos_armref(3)]
              
          % Plot all the points. 
          %figure(3);
          %plot3(pts(1, :), pts(2, :), pts(3, :), '*');
          fsm='capture';
             %��е�۵��ؽڿռ����״̬��
        elseif strcmp(fsm, 'capture')
            % Define each angle of the robot arm.0��  30��  52.4��  72.68,  90
            
          
            chooseAngle = [-90*pi/180, 15*pi/180, 130*pi/180, -55*pi/180, 0*pi/180]; 
            % Apply the value to each articulation.
            for i = 1:5
                res = vrep.simxSetJointTargetPosition(id, h.armJoints(i), chooseAngle(i), vrep.simx_opmode_oneshot);
                vrchk(vrep, res, true);
            end
            % Wait until the robot arm is in position.
            pause(5);
            fsm = 'useIK';
            
        % ��е�۵ѿ����ռ����״̬��
        elseif strcmp(fsm, 'useIK')
            % Get the arm position. 
            [res, tpos] = vrep.simxGetObjectPosition(id, h.ptip, h.armRef, vrep.simx_opmode_buffer);
            vrchk(vrep, res, true);
            
            % ������е�ۿ���ģʽΪ2���ѿ����ռ�ģʽ����Ĭ��0���ؽڿռ����ģʽ��
            res = vrep.simxSetIntegerSignal(id, 'km_mode', 2, vrep.simx_opmode_oneshot_wait);
            vrchk(vrep, res, true);
            
            % Set the new position wwanted for the gripper.  tpos(1) , tpos(2) , tpos(3)    0,-3253,0.0151
            %tpos =[-0.360,0,0.015];
            tpos =cylinder_center
            res = vrep.simxSetObjectPosition(id, h.ptarget, h.armRef, tpos, vrep.simx_opmode_oneshot);
            vrchk(vrep, res, true);
            
            % Wait long enough so that the tip is at the right position and go on to the next state. 
            pause(1);
            fsm = 'rotGrip';

        % ���ؽ�ģʽ���ƻ�е��ĩ����ץ��ת״̬��
        elseif strcmp(fsm, 'rotGrip')
            % Remove the inverse kinematics (IK) mode.
            res = vrep.simxSetIntegerSignal(id, 'km_mode', 0, vrep.simx_opmode_oneshot_wait);
            % Set the new gripper angle to "0".
            res = vrep.simxSetJointTargetPosition(id, h.armJoints(5), 0, vrep.simx_opmode_oneshot);
            vrchk(vrep, res, true);
            
            % Wait long enough so that the tip is at the right position and go on to the next state. 
            %pause(1);
            fsm = 'grasp';
            
        % ��ץ�պϿ���״̬��
        elseif strcmp(fsm, 'grasp')
            res = vrep.simxSetIntegerSignal(id, 'gripper_open', 0, vrep.simx_opmode_oneshot_wait);
            vrchk(vrep, res);
            
            pause(1);
            fsm = '1';
            %%%%%%%%%%%%%%%%%��е�۹�λ22222222222222
        elseif strcmp(fsm, '1')
            % Define each angle of the robot arm.0��  30��  52.4��  72.68,  90
             % ������е�ۿ���ģʽΪ0���ѿ����ռ�ģʽ����Ĭ��0���ؽڿռ����ģʽ��
            
            res = vrep.simxSetIntegerSignal(id, 'km_mode', 0, vrep.simx_opmode_oneshot_wait);
            vrchk(vrep, res, true);
            chooseAngle = [0*pi/180, 30*pi/180, 52.4*pi/180, 72.68*pi/180, 90*pi/180];
            
            % Apply the value to each articulation.
            for i = 1:5
                res = vrep.simxSetJointTargetPosition(id, h.armJoints(i), chooseAngle(i), vrep.simx_opmode_oneshot);
                vrchk(vrep, res, true);
            end
              %pause(1);
            fsm = 'LeftRight2';

        elseif strcmp(fsm, 'LeftRight2');
            % ���ƻ�����ǰ���˶��ٶ���X=-2.85λ�����
            leftRightVel = - 0.5 * (youbotPos(2) + 3);
            
            % ʹ�û����˱ƽ�X=-2.85��λ��.
            if abs(youbotPos(2) + 3) < .001
                leftRightVel = 0;
                fsm = 'Rot2';
            end
        elseif strcmp(fsm, 'Rot2');
            % Rotate. 
            rotVel = angdiff( 91*pi/180, youbotEuler(3));
            
            % Stop when the robot is at an angle close to -pi/2. 
            if abs(angdiff( 91*pi/180, youbotEuler(3))) < .1 / 180 * pi
                rotVel = 0;
                fsm = 'lin_forwBack2';
            end
        elseif strcmp(fsm, 'lin_forwBack2');
            %���ƻ�����ǰ���˶��ٶ���Y=-6.4λ�����
            forwBackVel = 1 * (youbotPos(1) + 6.2);
         
            
            % ʹ�û����˱ƽ�Y=-6.4��λ��. 
            if abs(youbotPos(1) + 6.2) < .001
                forwBackVel = 0;
                fsm = 'noIK2';
            end
        elseif strcmp(fsm, 'noIK2')
            % Define each angle of the robot arm.0��  30��  52.4��  72.68,  90
           chooseAngle = [180*pi/180, 30*pi/180, 52.4*pi/180, 72.68*pi/180, 0*pi/180]; 
            % Apply the value to each articulation.
            for i = 1:5
                res = vrep.simxSetJointTargetPosition(id, h.armJoints(i), chooseAngle(i), vrep.simx_opmode_oneshot);
                vrchk(vrep, res, true);
            end
            % Wait until the robot arm is in position.
            pause(5);
            fsm = 'release';
             % ��ץ�ſ�����״̬��
       elseif strcmp(fsm, 'release')
            res = vrep.simxSetIntegerSignal(id, 'gripper_open', 1, vrep.simx_opmode_oneshot_wait);
            vrchk(vrep, res);
            pause(5);
            fsm = 'finished';
        elseif strcmp(fsm, 'finished'),
            pause(3);
            break
            
        else
            error(sprintf('Unknown state %s.', fsm));
        end

        % Update wheel velocities for each loop
        h = youbot_drive(vrep, h, forwBackVel, leftRightVel, rotVel);

        % Make sure that we do not go faster that the simulator. 
        elapsed = toc;
        timeleft = timestep - elapsed;
        if (timeleft > 0),
          pause(min(timeleft, .01));
        end
    end

end % main function
   
