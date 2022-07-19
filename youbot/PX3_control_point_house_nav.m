function PX3_control_point_house_nav()
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
	%С��ǰ���ٶȺ�ת���ٶ�
	forwBackVel = 1;
	rotVel = 0;
    global cellsize;
    cellsize=0.25
%###################################���ڵ�ͼ�ĵ�������####################################
  %�趨����Ŀ��
  load('fmap.mat');
  
  goal = [ ij(5), ij(-2) ];
  dx = Dstar(double(fmap), 'quiet');
  dx.plan([goal(1); goal(2)]);
  %���㵼���켣TRAJ
  %��ȡ�ƶ������˱����������ϵ�������������ϵ��λ�ú���̬
  [res youbotPos] = vrep.simxGetObjectPosition(h.id, h.Pioneer_p3dx, -1,vrep.simx_opmode_buffer); vrchk(vrep, res);
  [res youbotEuler] = vrep.simxGetObjectOrientation(h.id, h.Pioneer_p3dx, -1,vrep.simx_opmode_buffer); vrchk(vrep, res);  
  traj = dx.path([ij(youbotPos(1)) ; ij(youbotPos(2))]);
  traj = [ xy(traj(:,1)) xy(traj(:,2)) ];
  traj = [traj; [xy(goal(1)) xy(goal(2))]];
  traj = traj (2:end,:)
  %������صı�����ʼ������
  num=1;
  %��ʼ������״̬������  
  fsm='rotate';
  res = vrep.simxSynchronous(h.id, true); vrchk(vrep, res);

%####################################################����������״̬������######################################################################
%��ʼ����״̬��
	while true,
	%��ȡ�ƶ������˱����������ϵ�������������ϵ��λ�ú���̬
	[res youbotPos] = vrep.simxGetObjectPosition(h.id, h.Pioneer_p3dx, -1,vrep.simx_opmode_buffer); vrchk(vrep, res);
	[res youbotEuler] = vrep.simxGetObjectOrientation(h.id, h.Pioneer_p3dx, -1,vrep.simx_opmode_buffer); vrchk(vrep, res);  
	
	%��ȡ�µ�Ŀ���
	target(1)=traj(num,1);
	target(2)=traj(num,2);
  
	%����Ŀ����ڻ����˱��������µ�����
        %Ŀ�������������ϵ������
        P_ow=[target(1);target(2);1];
        %�ƶ������˹�������ϵ�����������µľ����ʾ
		T_cw=se2(youbotPos(1),youbotPos(2),youbotEuler(3));
        %����Ŀ������ƶ������˹�������ϵ������
		P_oc=inv(T_cw)*P_ow; 
        
	%ת��״̬��
     if strcmp(fsm, 'rotate'),
	%ת��״̬��ش��룬�����ƶ�����������ת���ٶ�
        angl=atan2(P_oc(2),P_oc(1));
	    rotVel =5*angl;
        fsm = 'drive';
	%ǰ��״̬��
     elseif strcmp(fsm, 'drive'),
	%ǰ��״̬��ش��룬�����ƶ�����������ǰ���ٶ�
        forwBackVel =0.5*sqrt((youbotPos(1)-target(1))^2+(youbotPos(2)-target(2))^2);
        fsm = 'rotate';
        if sqrt((youbotPos(1)-target(1))^2+(youbotPos(2)-target(2))^2)<0.1
             fsm='finished';
        end
        
	%����״̬��
     elseif strcmp(fsm, 'finished'),
	%����״̬����ش���
        num=num+1;
        fsm='rotate';
        if target==traj(end,:)
            break;
        end
     end

  % �����ƶ������˿���״̬
	res = vrep.simxPauseCommunication(h.id, true); vrchk(vrep, res);
		%˫�ֲ���ƶ��������������̣�����˫��ת��
		vLeft = 1*(forwBackVel - b * rotVel) / r;
		vRight = 1*(forwBackVel + b * rotVel) / r;
		%��vrep����˫��ת��
		vrep.simxSetJointTargetVelocity(h.id, h.wheelJoints(1),vLeft,vrep.simx_opmode_oneshot); vrchk(vrep, res);
		vrep.simxSetJointTargetVelocity(h.id, h.wheelJoints(2),vRight,vrep.simx_opmode_oneshot); vrchk(vrep, res);
	res = vrep.simxPauseCommunication(h.id, false); vrchk(vrep, res);
	%��������ͼ
	if 1,
		
		[X,Y] = meshgrid((-7.5+cellsize/2):cellsize:(7.5-cellsize/2),(-7.5+cellsize/2):cellsize:(7.5-cellsize/2));
		plot( youbotPos(1), youbotPos(2), 'ob',7.5, 0, 'or', 0, 7.5, 'og',target(1), target(2), '*b');
		axis equal;
		axis([-7.8 7.8 -7.8 7.8]);
	end
	drawnow;
	vrep.simxSynchronousTrigger(id);
	%end while
	end

disp('Program ended');
%endfunction
end



