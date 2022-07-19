function youbot_3dpointcloud()
%***********************************************************************************************************************
%|||||||||||||||||||||||||||||||||||||||本程序示例如何使用youbot机器人的XYZ相机获取信息|||||||||||||||||||||||||||||||||||
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
%#################################################初始化程序变量###############################################################################
    % 初始化youbot机器人获取所有相关句柄 .
    h = youbot_init(vrep, id);
   % 初始化Hokuyo激光雷达获取所有相关句柄 
    h = youbot_hokuyo_init(vrep, h);
    % 暂定2s保证初始化过程完成 
    pause(2);
        
    vrep.simxSetObjectOrientation(id,h.rgbdCasing,h.ref,[0 0 125*pi/180], vrep.simx_opmode_oneshot);
    
    % 调整RGBD相机的广角度数
    res = vrep.simxSetFloatSignal(id, 'rgbd_sensor_scan_angle', pi / 8, vrep.simx_opmode_oneshot_wait);
    vrchk(vrep, res);
    
    
    % 通过信号量控制XYZ相机开机 
    res = vrep.simxSetIntegerSignal(id, 'handle_xyz_sensor', 1, vrep.simx_opmode_oneshot_wait);
    vrchk(vrep, res);
    
    % 通过信号量控制彩色相机开机
    res = vrep.simxSetIntegerSignal(id, 'handle_rgb_sensor', 1, vrep.simx_opmode_oneshot_wait);
    vrchk(vrep, res);
    
    % 获取彩色相机像素信息

    [res, resolution, image] = vrep.simxGetVisionSensorImage2(id, h.rgbSensor, 0, vrep.simx_opmode_oneshot_wait);
    vrchk(vrep, res);

    %显示彩色图像
    figure(1); 
    imshow(image);
    
    

    % 获取XYZ相机3D图像数据保存到pts变量. 
    fprintf('Capturing point cloud...\n');
    pts = youbot_xyz_sensor(vrep, h, vrep.simx_opmode_oneshot_wait);
plot3(pts(1,:),pts(3,:),pts(2,:))
    % Plot all the points. 
%     figure(2);
%     plot3(pts(1, :), pts(2, :), pts(3, :), '*');
    

		%红色区域识别
		diff_R = 100;
		Image=image;
        
        figure(2)
        imshow(Image)
        
		RP_R=Image(:,:,1); 
		RP_G=Image(:,:,2); 
		RP_B=Image(:,:,3);
		XYR=255*((RP_G-RP_B)>diff_R&(RP_R-RP_B)>diff_R); 
		%寻找红色区域的外接矩形
		bw=im2bw(XYR);
		[r c]=find(bw==1);  
		[rectx,recty,area,perimeter] = minboundrect(c,r,'a'); 
		%line(rectx(:),recty(:),'color','r');
		%寻找红色区域的几何中心点坐标
		rgb_target_centor(1)=(rectx(1)+rectx(2))/2;
		rgb_target_centor(2)=(recty(1)+recty(3))/2;

		%将几何中心点坐标向XYZ坐标映射，计算几何中心点距离相机的位置
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
   
