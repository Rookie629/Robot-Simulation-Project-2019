function youbot_photo()
%***********************************************************************************************************************
%|||||||||||||||||||||||||||||||||||||||本程序示例如何使用youbot机器人的RGB彩色相机获取信息|||||||||||||||||||||||||||||||||||
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

   
    % 通过信号量控制彩色相机开机
     res = vrep.simxSetIntegerSignal(id, 'handle_rgb_sensor', 1, vrep.simx_opmode_oneshot_wait);
     vrchk(vrep, res);
    
    % 获取彩色相机像素信息
    fprintf('Capturing image...\n');
    [res, resolution, image] = vrep.simxGetVisionSensorImage2(id, h.rgbSensor, 0, vrep.simx_opmode_oneshot_wait);
    vrchk(vrep, res);
    fprintf('Captured %i pixels (%i x %i).\n', resolution(1) * resolution(2), resolution(1), resolution(2));
    %显示彩色图像
    figure; 
    imshow(image);
    drawnow;
end % main function
   
