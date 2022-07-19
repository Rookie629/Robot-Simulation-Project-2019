function handles=PX3_init(vrep,id)
%����һ���ṹ��handles
handles = struct('id', id);
%��ȡ�ƶ�������������ת��ľ�������浽������
[res wheelJoints(1)] = vrep.simxGetObjectHandle(id, 'Revolute_joint_left', vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);
[res wheelJoints(2)] = vrep.simxGetObjectHandle(id, 'Revolute_joint_right', vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);
%��ȡ�ƶ������˵ı��壨��������ϵ���ľ��
[res Pioneer_p3dx] = vrep.simxGetObjectHandle(id, 'body', vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);
%���ƶ���������ؾ�����浽�ṹ����
handles.wheelJoints = wheelJoints;
handles.Pioneer_p3dx=Pioneer_p3dx;
%��ʼ����ȡ�ƶ������˱����λ�ú���̬���������������ϵ��
res = vrep.simxGetObjectPosition(id, Pioneer_p3dx, -1, vrep.simx_opmode_streaming); vrchk(vrep, res, true);
res = vrep.simxGetObjectOrientation(id, Pioneer_p3dx, -1, vrep.simx_opmode_streaming); vrchk(vrep, res, true);

end