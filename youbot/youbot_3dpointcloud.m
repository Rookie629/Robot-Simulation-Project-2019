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
        % 调整RGBD相机的广角度数
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
    fsm='identify';
    if strcmp(fsm,'identify')
    vrep.simxSetObjectOrientation(id,h.rgbdCasing,h.ref,[0 0 30*pi/180], vrep.simx_opmode_oneshot);    
     vrchk(vrep, res);
    % 获取彩色相机像素信息
    % 通过信号量控制彩色相机开机
    res = vrep.simxSetIntegerSignal(id, 'handle_rgb_sensor', 1, vrep.simx_opmode_oneshot_wait);
    vrchk(vrep, res);
    
    [res, resolution, image] = vrep.simxGetVisionSensorImage2(id, h.rgbSensor, 0, vrep.simx_opmode_oneshot_wait);
    vrchk(vrep, res);

    %显示彩色图像
    figure(1); 
    imshow(image);
    
    

    % 获取XYZ相机3D图像数据保存到pts变量. 
    fprintf('Capturing point cloud...\n');
    pts = youbot_xyz_sensor(vrep, h, vrep.simx_opmode_oneshot_wait);
 %关闭所有窗口
        close all;
        %获取并显示图像
        figure(1)
        %I = imread('sim.png');
        imshow(image)
        %设定颜色阈值
        diff_R = 100;
        Image=image;
        %获取颜色矩阵
        RP_R=Image(:,:,1);
        RP_G=Image(:,:,2); 
        RP_B=Image(:,:,3);
        %红色区域分析
        XYR=255*((RP_R-RP_G)>diff_R&(RP_R-RP_B)>diff_R); 
        %绘制黑白图
        figure(2)
        imshow(uint8(XYR));
        %转为二值图
        bw=im2bw(XYR);
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
		rectx
		recty
		rgb_target_centor(1)=(rectx(1)+rectx(2))/2
		rgb_target_centor(2)=(recty(1)+recty(3))/2

		%将几何中心点坐标向XYZ坐标映射，计算几何中心点距离相机的位置
		xyz_target_centor=32-ceil(rgb_target_centor/(512/32))+1
		xyz_target_num=xyz_target_centor(1)+(xyz_target_centor(2)-1)*32
		pts(:,xyz_target_num-2:xyz_target_num+2);
        arm_targetpos_camera=[pts(1,xyz_target_num) pts(2,xyz_target_num) pts(3,xyz_target_num)];
        [res,rgbdp]=vrep.simxGetObjectPosition(h.id,h.xyzSensor,h.armRef,vrep.simx_opmode_buffer)  
        [res,rgbdpo]=vrep.simxGetObjectOrientation(h.id,h.xyzSensor,h.armRef,vrep.simx_opmode_buffer)
        T=transl(rgbdp)*trotx(rgbdpo(1))*trotx(rgbdpo(2)) *trotx(rgbdpo(3));
        arm_targetpos_armref=T*[arm_targetpos_camera(1);arm_targetpos_camera(2);arm_targetpos_camera(3);1]
        cylinder_center=[arm_targetpos_armref(1);arm_targetpos_armref(2);arm_targetpos_armref(3)]
        
    % Plot all the points. 
    figure(2);
    plot3(pts(1, :), pts(2, :), pts(3, :), '*');
    end
    % Plot the points of the wall (further away than 1.87 m) in a different colour. 
%     hold on; 
%     ptsWall = pts(1:3, pts(4, :) >= 1.87);
%     plot3(ptsWall(1, :), ptsWall(3, :), ptsWall(2, :), '.r');
    
    % Save the pointcloud to pc.xyz. (This file can be displayed with http://www.meshlab.net/.)
%     fileID = fopen('pc.xyz','w');
%     fprintf(fileID,'%f %f %f\n',pts);
%     fclose(fileID);
%     fprintf('Read %i 3D points, saved to pc.xyz.\n', max(size(pts)));

end % main function
   
