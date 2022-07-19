function P3dx_control_point()
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
%��ʼ����������ر������ظ�h�ṹ��
h = PX3_init(vrep, id);
  %С�����ӵİ뾶
  r = 0.0975;
  %С���־�뾶
  b = 0.1655;
  %�趨Ŀ������ϵ
  target(1)=2;%xy(goal(2))
  target(2)=-3;%xy(goal(1))
  %��ʼ������״̬������
  fsm='rotate';
  res = vrep.simxSynchronous(h.id, true); vrchk(vrep, res);
  global cellsize;
  cellsize=0.25;
  %���õ�����
    load('fmap.mat');
  goal = [ ij(1.125), ij(-3.35) ];
  dx = Dstar(double(fmap), 'quiet');
  dx.plan([goal(1); goal(2)]);
  %���㵼���켣TRAJ
  %��ȡ�ƶ������˱����������ϵ�������������ϵ��λ�ú���̬
  [res youbotPos] = vrep.simxGetObjectPosition(h.id, h.Pioneer_p3dx, -1,vrep.simx_opmode_buffer); vrchk(vrep, res);
  [res youbotEuler] = vrep.simxGetObjectOrientation(h.id, h.Pioneer_p3dx, -1,vrep.simx_opmode_buffer); vrchk(vrep, res);  
  traj = dx.path([ij(youbotPos(1)) ; ij(youbotPos(2))]);
  traj = [ xy(traj(:,1)) xy(traj(:,2)) ];
  traj = [traj; [xy(goal(1)) xy(goal(2))]];
  traj = traj (2:end,:);
%####################################################����������״̬������######################################################################
%��ʼ����״̬��
   a=size(traj);
for i=1:a(1)
    target(1)=traj(i,1);
    target(2)=traj(i,2);
while true,%��ȡ�ƶ������˱����������ϵ�������������ϵ��λ�ú���̬
  [res, youbotPos] = vrep.simxGetObjectPosition(h.id, h.Pioneer_p3dx, -1,vrep.simx_opmode_buffer); vrchk(vrep, res);
  [res youbotEuler] = vrep.simxGetObjectOrientation(h.id, h.Pioneer_p3dx, -1,vrep.simx_opmode_buffer); vrchk(vrep, res);  
%�벹��������Ŀ����ڻ����˱��������µ�����
%code1***************************************************************

 
P_ow=[target(1);target(2);1];
  T_cw=se2(youbotPos(1),youbotPos(2),youbotEuler(3));
  P_oc=inv(T_cw)*P_ow;   
%���ڴ˴�����ת��״̬��ش��룬�����ƶ�����������ת���ٶ�
%code2***************************************************************
     %kr=10
     angl=atan2(P_oc(2),P_oc(1));
     rotVel=10*angl;
%���ڴ˴�����ת��״̬��ش��룬�����ƶ�����������ǰ���ٶ�
%code3***************************************************************
     %kv=1;
     forwBackVel=1*sqrt((youbotPos(1)-target(1))^2+(youbotPos(2)-target(2))^2);
     
     if sqrt((youbotPos(1)-target(1))^2+(youbotPos(2)-target(2))^2)<0.1
         break
     end
  % �����ƶ������˿���״̬
  res = vrep.simxPauseCommunication(h.id, true); vrchk(vrep, res);
    %˫�ֲ���ƶ��������������̣�����˫��ת��
    vLeft = (forwBackVel - b * rotVel) / r;
    vRight = (forwBackVel + b * rotVel) / r;
    %��vrep����˫��ת��
    vrep.simxSetJointTargetVelocity(h.id, h.wheelJoints(1),vLeft,vrep.simx_opmode_oneshot); vrchk(vrep, res);
    vrep.simxSetJointTargetVelocity(h.id, h.wheelJoints(2),vRight,vrep.simx_opmode_oneshot); vrchk(vrep, res);
  res = vrep.simxPauseCommunication(h.id, false); vrchk(vrep, res);
  %��������ͼ
  if 1,
    cellsize=0.25
    [X,Y] = meshgrid((-7.5+cellsize/2):cellsize:(7.5-cellsize/2),(-7.5+cellsize/2):cellsize:(7.5-cellsize/2));
    plot( youbotPos(1), youbotPos(2), 'ob',7.5, 0, 'or', 0, 7.5, 'og',target(1), target(2), '*b');
    axis equal;
    axis([-7.8 7.8 -7.8 7.8]);
  end
  drawnow;
  vrep.simxSynchronousTrigger(id);
%end while
end
%endfunction
disp('Program ended');
end


