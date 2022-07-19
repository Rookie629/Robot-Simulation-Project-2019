function handles=PX3_init(vrep,id)
%构建一个结构体handles
handles = struct('id', id);
%获取移动机器人左右旋转轴的句柄并保存到数组中
[res wheelJoints(1)] = vrep.simxGetObjectHandle(id, 'Revolute_joint_left', vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);
[res wheelJoints(2)] = vrep.simxGetObjectHandle(id, 'Revolute_joint_right', vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);
%获取移动机器人的本体（固连坐标系）的句柄
[res Pioneer_p3dx] = vrep.simxGetObjectHandle(id, 'body', vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);
%将移动机器人相关句柄保存到结构体中
handles.wheelJoints = wheelJoints;
handles.Pioneer_p3dx=Pioneer_p3dx;
%初始化获取移动机器人本体的位置和姿态（相对于世界坐标系）
res = vrep.simxGetObjectPosition(id, Pioneer_p3dx, -1, vrep.simx_opmode_streaming); vrchk(vrep, res, true);
res = vrep.simxGetObjectOrientation(id, Pioneer_p3dx, -1, vrep.simx_opmode_streaming); vrchk(vrep, res, true);

end