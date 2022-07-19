function RGBD_sensor_process()
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
	h =RGBD_sensor_init(vrep, id);
	%启动VREP和matlab的同步机制
	res = vrep.simxSynchronous(h.id, true); vrchk(vrep, res);
    diff_R = 100;

%#################################################开始有限状态机#########################################
	while true,
		%获取XYZ雷达图像信息
		pts = youbot_xyz_sensor(vrep, h, vrep.simx_opmode_oneshot_wait);
		%pts
		%绘制雷达图像X-Y-Z信息曲线（根据需要放开注释）
		%plot(pts(1,:))
		% hold on
		% plot(pts(2,:))
		% hold on
		plot(pts(3,:))
		% hold on
		% plot(pts(4,:))
		%plot3(pts(1,:),pts(3,:),pts(2,:))

		%获取彩色图像信息
		[res, resolution, I] = vrep.simxGetVisionSensorImage2(id, h.rgbSensor, 0, vrep.simx_opmode_oneshot_wait);
		    %显示彩色图像
        figure(1); 
        imshow(image);
		%分析目标物体几何中心相对相机坐标系的位置  I为彩色图片信息，pts为深度信息（3行矩阵）
		
		RP_R=I(:,:,1); 
        RP_G=I(:,:,2); 
        RP_B=I(:,:,3);
        %红色区域分析
        XYR=255*((RP_R-RP_G)>diff_R&(RP_R-RP_B)>diff_R); 
        %绘制黑白图
        figure(2)
        imshow(uint8(XYR));
        %转为二值图
        bw=im2bw(XYR);
        %寻找红色区域
        [r c]=find(bw==1);
        %寻找红色区域的最小外接矩形  
        [rectx,recty,area,perimeter] = minboundrect(c,r,'a');
        %根据外接矩形顶点绘制直线框 
        line(rectx(:),recty(:),'color','r');
        %显示外接矩形顶点坐标
		center(1)=(rectx(1)+rectx(2))/2;
        center(2)=(recty(1)+recty(3))/2;
        target_center=32-ceil(center/(512/32))+1;
        target_center_numb=target_center(1)+(target_center(2)-1)*32;
        imgcenter=pts(:,target_center_numb-2:target_center_numb+2)
        %x=pts(1,center)
        %y=pts(2,center)
		%z=pts(3,center)
		
		%周期性触发函数
		vrep.simxSynchronousTrigger(id);
		%暂停1s
		pause(1)
		%end while
	end
	%endfunction
	disp('Program ended');
end



