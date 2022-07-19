function youbot_moving_arm2()
%***********************************************************************************************************************
%|||||||||||||||||||||||||||||||||||||||本程序示例如何使用youbot机器人的底盘自由度及状态机软件架构的使用方法|||||||||||||||||||||||||||||||||||
%***********************************************************************************************************************

%###############################################################固定程序创建VREP和matlab的连接并远程启动###################################################################
    disp('Program started');
    %加载remoteApiProto.m文件 
    vrep = remApi('remoteApi');
    %关闭所有VREP和matlab的连接
    vrep.simxFinish(-1);
    %创建新的VREP和matlab的连接，返回状态为id，如果id<0代表创建连接失败
    id = vrep.simxStart('127.0.0.1', 19997, true, true, 2000, 5);
    if id < 0
        disp('Failed connecting to remote API server. Exiting.');
        vrep.delete();
        return;
    end
    fprintf('Connection %d to remote API server open.\n', id);
    % 确保程序意外中断后系统自动清除VREP和matlab的连接数据
    cleanupObj = onCleanup(@() cleanup_vrep(vrep, id));
	%由matlab远程启动VREP的仿真
    vrep.simxStartSimulation(id, vrep.simx_opmode_oneshot_wait);
%###############################################################################初始化程序变量###############################################################################
    % 初始化youbot机器人获取所有相关句柄 .
    h = youbot_init(vrep, id);
   % 初始化Hokuyo激光雷达获取所有相关句柄 
    h = youbot_hokuyo_init(vrep, h);
    % 暂定2s保证初始化过程完成 
    pause(2);
      % 调整RGBD相机的广角度数
       vrep.simxSetObjectOrientation(id,h.rgbdCasing,h.ref,[0 0 125*pi/180], vrep.simx_opmode_oneshot); 
    res = vrep.simxSetFloatSignal(id, 'rgbd_sensor_scan_angle', pi / 8, vrep.simx_opmode_oneshot_wait);
    vrchk(vrep, res);
    
    
    % 通过信号量控制XYZ相机开机 
    res = vrep.simxSetIntegerSignal(id, 'handle_xyz_sensor', 1, vrep.simx_opmode_oneshot_wait);
    vrchk(vrep, res);
    
    % 通过信号量控制彩色相机开机
    res = vrep.simxSetIntegerSignal(id, 'handle_rgb_sensor', 1, vrep.simx_opmode_oneshot_wait);
    vrchk(vrep, res);
    %进入状态机
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
    %定义机械臂关节空间运动范围
    armJointRanges = [-2.9496064186096, 2.9496064186096;
                      -1.5707963705063, 1.308996796608;
                      -2.2863812446594, 2.2863812446594;
                      -1.7802357673645, 1.7802357673645;
                      -1.5707963705063, 1.5707963705063 ];

    % Make sure everything is settled before we start. 
    pause(2);
    
    % 初始化状态机
    fsm = 'Rot1';

    %% Start the demo. 
    while true
        tic % See end of loop to see why it's useful. 
        
        if vrep.simxGetConnectionId(id) == -1
          error('Lost connection to remote API.');
        end
    
        % 获取移动机器人相对于世界坐标系的位置和姿态 
        [res, youbotPos] = vrep.simxGetObjectPosition(id, h.ref, -1, vrep.simx_opmode_buffer);
        vrchk(vrep, res, true);
        [res, youbotEuler] = vrep.simxGetObjectOrientation(id, h.ref, -1, vrep.simx_opmode_buffer);
        vrchk(vrep, res, true);
        % 获取机械臂末端tip坐标系的位置
    [res, homeGripperPosition] = vrep.simxGetObjectPosition(id, h.ptip, h.armRef, vrep.simx_opmode_buffer);
    vrchk(vrep, res, true);
                %旋转状态机   
        if strcmp(fsm, 'Rot1');
            % Rotate. 
            rotVel = angdiff( pi / 2, youbotEuler(3));
          h = youbot_drive(vrep, h, forwBackVel, leftRightVel, rotVel);   
            % Stop when the robot is at an angle close to -pi/2. 
            if abs(angdiff( pi / 2, youbotEuler(3))) < .1 / 180 * pi
                rotVel = 0;
                fsm = 'lin_forwBack1';
            end

        


        %机器人前后移动状态机. 
        %elseif strcmp(fsm, 'cst_forwBack');
            
            %前后速度
            %forwBackVel = -1;
            
            % 使得机器人逼近Y=-6.5的位置. 
            %if youbotPos(2) + 5.3 < .001
            %    forwBackVel = 0;
             %   fsm = 'lin_forwBack';
            %end
        %    
        elseif strcmp(fsm, 'lin_forwBack1');
            %控制机器人前进运动速度与Y=-3位置相关
            forwBackVel = 2 * (youbotPos(1) + 3);
            %forwBackVel = -2 
         
            
            % 使得机器人逼近Y=-3的位置. 
            if abs(youbotPos(1) + 2.8) <0.01
                forwBackVel = 0;
                fsm = 'LeftRight1';
            end
            
        elseif strcmp(fsm, 'LeftRight1');
            % 控制机器人前进运动速度与X=-5.3位置相关
            leftRightVel = - 2 * (youbotPos(2) +5.3);
           
            
            % 使得机器人逼近X=-5.6的位置.
            if abs(youbotPos(2) + 5.3) < .001
                leftRightVel = 0;
                          fsm = 'identify';
                 pause(1);
            end
        elseif strcmp(fsm,'identify')
             
            % 获取彩色相机像素信息
             pause(2);
            [res, resolution, image] = vrep.simxGetVisionSensorImage2(id, h.rgbSensor, 0, vrep.simx_opmode_oneshot_wait);
            vrchk(vrep, res);

             %显示彩色图像
             figure(1); 
             imshow(image);
    
    

             % 获取XYZ相机3D图像数据保存到pts变量. 
             fprintf('Capturing point cloud...\n');
             pts = youbot_xyz_sensor(vrep, h, vrep.simx_opmode_oneshot_wait);
              plot3(pts(1,:),pts(3,:),pts(2,:))
             %关闭所有窗口
             close all;
             %获取并显示图像
              figure(1)
              %I = imread('sim.png');
              imshow(image)
              %设定颜色阈值
              Image=image;
              %获取颜色矩阵
              RP_R=Image(:,:,1); 
              RP_G=Image(:,:,2); 
              RP_B=Image(:,:,3);
              %红色区域分析
              %XYR=255*((RP_R-RP_G)>diff_R&(RP_R-RP_B)>diff_R); 
              XYY=255*((RP_R-RP_B)>diff_Y&(RP_G-RP_B)>diff_Y); 
              %绘制黑白图
              figure(2)
              imshow(uint8(XYY));
              %转为二值图
              bw=im2bw(XYY);
              %寻找红色区域
              [r c]=find(bw==1)
             %寻找红色区域的最小外接矩形   
              [rectx,recty,area,perimeter] = minboundrect(c,r,'a');
              %根据外接矩形顶点绘制直线框 
              line(rectx(:),recty(:),'color','r');
              %显示外接矩形顶点坐标
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

		      %将几何中心点坐标向XYZ坐标映射，计算几何中心点距离相机的位置
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
             %机械臂单关节空间控制状态机
        elseif strcmp(fsm, 'capture')
            % Define each angle of the robot arm.0，  30，  52.4，  72.68,  90
            
          
            chooseAngle = [-90*pi/180, 15*pi/180, 130*pi/180, -55*pi/180, 0*pi/180]; 
            % Apply the value to each articulation.
            for i = 1:5
                res = vrep.simxSetJointTargetPosition(id, h.armJoints(i), chooseAngle(i), vrep.simx_opmode_oneshot);
                vrchk(vrep, res, true);
            end
            % Wait until the robot arm is in position.
            pause(5);
            fsm = 'useIK';
            
        % 机械臂笛卡尔空间控制状态机
        elseif strcmp(fsm, 'useIK')
            % Get the arm position. 
            [res, tpos] = vrep.simxGetObjectPosition(id, h.ptip, h.armRef, vrep.simx_opmode_buffer);
            vrchk(vrep, res, true);
            
            % 调整机械臂控制模式为2（笛卡尔空间模式）（默认0：关节空间控制模式）
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

        % 单关节模式控制机械臂末端手抓旋转状态机
        elseif strcmp(fsm, 'rotGrip')
            % Remove the inverse kinematics (IK) mode.
            res = vrep.simxSetIntegerSignal(id, 'km_mode', 0, vrep.simx_opmode_oneshot_wait);
            % Set the new gripper angle to "0".
            res = vrep.simxSetJointTargetPosition(id, h.armJoints(5), 0, vrep.simx_opmode_oneshot);
            vrchk(vrep, res, true);
            
            % Wait long enough so that the tip is at the right position and go on to the next state. 
            %pause(1);
            fsm = 'grasp';
            
        % 手抓闭合控制状态机
        elseif strcmp(fsm, 'grasp')
            res = vrep.simxSetIntegerSignal(id, 'gripper_open', 0, vrep.simx_opmode_oneshot_wait);
            vrchk(vrep, res);
            
            pause(1);
            fsm = '1';
            %%%%%%%%%%%%%%%%%机械臂归位22222222222222
        elseif strcmp(fsm, '1')
            % Define each angle of the robot arm.0，  30，  52.4，  72.68,  90
             % 调整机械臂控制模式为0（笛卡尔空间模式）（默认0：关节空间控制模式）
            
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
            % 控制机器人前进运动速度与X=-2.85位置相关
            leftRightVel = - 0.5 * (youbotPos(2) + 3);
            
            % 使得机器人逼近X=-2.85的位置.
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
            %控制机器人前进运动速度与Y=-6.4位置相关
            forwBackVel = 1 * (youbotPos(1) + 6.2);
         
            
            % 使得机器人逼近Y=-6.4的位置. 
            if abs(youbotPos(1) + 6.2) < .001
                forwBackVel = 0;
                fsm = 'noIK2';
            end
        elseif strcmp(fsm, 'noIK2')
            % Define each angle of the robot arm.0，  30，  52.4，  72.68,  90
           chooseAngle = [180*pi/180, 30*pi/180, 52.4*pi/180, 72.68*pi/180, 0*pi/180]; 
            % Apply the value to each articulation.
            for i = 1:5
                res = vrep.simxSetJointTargetPosition(id, h.armJoints(i), chooseAngle(i), vrep.simx_opmode_oneshot);
                vrchk(vrep, res, true);
            end
            % Wait until the robot arm is in position.
            pause(5);
            fsm = 'release';
             % 手抓张开控制状态机
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
   
