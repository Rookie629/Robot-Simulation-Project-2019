function read_RGBD_sensor()
%###############################################################固定程序创建VREP和matlab的连接并远程启动###################################################################
	disp('Program started');
	%加载remoteApiProto.m文件
	vrep=remApi('remoteApi'); 
	%关闭所有VREP和matlab的连接
	vrep.simxFinish(-1); 
	%创建新的VREP和matlab的连接，返回状态为id，如果id<0代表创建连接失败
	id=vrep.simxStart('127.0.0.1',19997,true,true,5000,5);
	if id < 0,
		disp('Failed connecting to remote API server. Exiting.');
		vrep.delete();
		return;
	end
	fprintf('Connection %d to remote API server open.\n', id);
	% Make sure we close the connexion whenever the script is interrupted.
	cleanupObj = onCleanup(@() cleanup_vrep(vrep, id));
	%由matlab远程启动VREP的仿真
	res = vrep.simxStartSimulation(id, vrep.simx_opmode_oneshot_wait);
%#################################################初始化程序变量###############################################################################
	h = RGBD_sensor_init(vrep, id)
	%启动VREP和matlab的同步机制
	res = vrep.simxSynchronous(h.id, true); vrchk(vrep, res);


%#################################################开始有限状态机#########################################
	while true,
		%获取彩色图像信息
		[res, resolution, image] = vrep.simxGetVisionSensorImage2(id, h.rgbSensor, 0, vrep.simx_opmode_oneshot_wait);
		%获取XYZ雷达图像信息
		pts = youbot_xyz_sensor(vrep, h, vrep.simx_opmode_oneshot_wait);
		%pts
		%绘制雷达图像X-Y-Z信息曲线（根据需要放开注释）
		%plot(pts(1,:))
		% hold on
		% plot(pts(2,:))
		% hold on
        subplot(1,2,1)
		plot(pts(3,:))
		% hold on
		% plot(pts(4,:))
		%plot3(pts(1,:),pts(3,:),pts(2,:))
		%显示彩色图像
        subplot(1,2,2)
		imshow(image);
        RP_R=image(:,:,1); 
        RP_G=image(:,:,2); 
        RP_B=image(:,:,3);
		%周期性触发函数
		vrep.simxSynchronousTrigger(id);
		%暂停1s
		pause(1)
		%end while
	end
	%endfunction
	disp('Program ended');
end



