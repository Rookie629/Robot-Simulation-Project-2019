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
    % ��ʼ��youbot�����˻�ȡ������ؾ�� .
    h = youbot_init(vrep, id);
   % ��ʼ��Hokuyo�����״��ȡ������ؾ�� 
    h = youbot_hokuyo_init(vrep, h);
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
  
  goal = [ ij(-2.725), ij(-2.675) ];
  dx = Dstar(double(fmap), 'quiet');
  dx.plan([goal(1); goal(2)]);
  %���㵼���켣TRAJ
  %��ȡ�ƶ������˱����������ϵ�������������ϵ��λ�ú���̬
  %[res youbotPos]=vrep.simxGetObjectPosition(h.id, h.ref,-1,vrep.simx_opmode_buffer);

  %[res youbotEuler]=vrep.simxGetObjectOrientation(h.id, h.ref,-1,vrep.simx_opmode_buffer);
  %traj = dx.path([ij(youbotPos(1)) ; ij(youbotPos(2))]);
  %traj = [ xy(traj(:,1)) xy(traj(:,2)) ];
  %traj = [traj; [xy(goal(1)) xy(goal(2))]];
  %traj = traj (2:end,:)
  %������صı�����ʼ������
  num=1;
  %��ʼ������״̬������  
  fsm='rotate';
  res = vrep.simxSynchronous(h.id, true); vrchk(vrep, res);
  leftRightVel=0;
%####################################################����������״̬������######################################################################
%��ʼ����״̬��
	while true,
	%��ȡ�ƶ������˱����������ϵ�������������ϵ��λ�ú���̬
[res youbotPos]=vrep.simxGetObjectPosition(h.id, h.ref,-1,vrep.simx_opmode_buffer);

[res youbotEuler]=vrep.simxGetObjectOrientation(h.id, h.ref,-1,vrep.simx_opmode_buffer);

	
	%��ȡ�µ�Ŀ���
%	target(1)=traj(num,1);
%	target(2)=traj(num,2);
  target(1)=-3;
	%����Ŀ����ڻ����˱��������µ�����
        %Ŀ�������������ϵ������
       % P_ow=[target(1);target(2);1];
        %�ƶ������˹�������ϵ�����������µľ����ʾ
		%T_cw=se2(youbotPos(1),youbotPos(2),youbotEuler(3));
        %����Ŀ������ƶ������˹�������ϵ������
		%P_oc=inv(T_cw)*P_ow; 
        
	%ת��״̬��
     if strcmp(fsm, 'rotate')
	%ת��״̬��ش��룬�����ƶ�����������ת���ٶ�
  %if P_oc(1)>0
  %angl=-pi/2+atan2(P_oc(2),P_oc(1));
  %elseif P_oc(1)<0
  %angl=pi/2+atan2(P_oc(2),P_oc(1));  
  %elseif P_oc(1)==0
            %angl=0;
  %end
     angl=-90*pi/180;
       % angl=-atan2(P_oc(1),P_oc(2));
	    rotVel =5*angl;
        if ((youbotEuler(3))<(-90*pi/180))
        fsm = 'drivex';
        rotVel=0;
        end
	%ǰ��״̬��
     elseif strcmp(fsm, 'drivex')
	%ǰ��״̬��ش��룬�����ƶ�����������ǰ���ٶ�
    
        forwBackVel =-1;
        %fsm = 'rotate';
        if (youbotPos(1)<-3)
             fsm='drivey';
             forwBackVel=0;
        end
     elseif  strcmp(fsm, 'drivey') 
        leftRightVel =1;
        %fsm = 'rotate';
        if (youbotPos(2)<-5.325)
             fsm='finished';
             leftRightVel=0;  
        end
	%����״̬��
     elseif strcmp(fsm, 'finished')
	%����״̬����ش���
        %num=num+1;
        fsm='rotate';
        %if target==traj(end,:)
            break;
        %end
     end

  % �����ƶ������˿���״̬   
        h = youbot_drive(vrep, h, forwBackVel, leftRightVel, rotVel);

	%��������ͼ
	%if 1,
		
		%[X,Y] = meshgrid((-7.5+cellsize/2):cellsize:(7.5-cellsize/2),(-7.5+cellsize/2):cellsize:(7.5-cellsize/2));
		%plot( youbotPos(1), youbotPos(2), 'ob',7.5, 0, 'or', 0, 7.5, 'og',target(1), target(2), '*b');
		%axis equal;
		%axis([-7.8 7.8 -7.8 7.8]);
	%end
	drawnow;
	vrep.simxSynchronousTrigger(id);
	%end while
	end

disp('Program ended');
%endfunction
end



