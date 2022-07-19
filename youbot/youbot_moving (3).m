function youbot_moving()
%***********************************************************************************************************************
%|||||||||||||||||||||||||||||||||||||||������ʾ�����ʹ��youbot�����˵ĵ������ɶȼ�״̬������ܹ���ʹ�÷���|||||||||||||||||||||||||||||||||||
%***********************************************************************************************************************

%###############################################################�̶����򴴽�VREP��matlab�����Ӳ�Զ������###################################################################
    disp('Program started');
    %����remoteApiProto.m�ļ� 
    vrep = remApi('remoteApi');
    %�ر�����VREP��matlab������
    vrep.simxFinish(-1);
    %�����µ�VREP��matlab�����ӣ�����״̬Ϊid�����id<0����������ʧ��
    id = vrep.simxStart('127.0.0.1', 19997, true, true, 2000, 5);
    if id < 0
        disp('Failed connecting to remote API server. Exiting.');
        vrep.delete();
        return;
    end
    fprintf('Connection %d to remote API server open.\n', id);
    % ȷ�����������жϺ�ϵͳ�Զ����VREP��matlab����������
    cleanupObj = onCleanup(@() cleanup_vrep(vrep, id));
	%��matlabԶ������VREP�ķ���
    vrep.simxStartSimulation(id, vrep.simx_opmode_oneshot_wait);
%###############################################################################��ʼ���������###############################################################################
    % ��ʼ��youbot�����˻�ȡ������ؾ�� .
    h = youbot_init(vrep, id);
   % ��ʼ��Hokuyo�����״��ȡ������ؾ�� 
    h = youbot_hokuyo_init(vrep, h);
    % �ݶ�2s��֤��ʼ��������� 
    pause(2);

    % Youbot constants
    timestep = .05;

    forwBackVel = 0;
    leftRightVel = 0;
    rotVel = 0;
    prevOri = 0; 
    prevLoc = 0;


    % Make sure everything is settled before we start. 
    pause(2);
    
    % ��ʼ��״̬��
    fsm = 'Rot1';

    %% Start the demo. 
    while true
        tic % See end of loop to see why it's useful. 
        
        if vrep.simxGetConnectionId(id) == -1
          error('Lost connection to remote API.');
        end
    
        % ��ȡ�ƶ��������������������ϵ��λ�ú���̬ 
        [res, youbotPos] = vrep.simxGetObjectPosition(id, h.ref, -1, vrep.simx_opmode_buffer);
        vrchk(vrep, res, true);
        [res, youbotEuler] = vrep.simxGetObjectOrientation(id, h.ref, -1, vrep.simx_opmode_buffer);
        vrchk(vrep, res, true);
                %��ת״̬��   
        if strcmp(fsm, 'Rot1');
            % Rotate. 
            rotVel = angdiff( pi / 2, youbotEuler(3));
            
            % Stop when the robot is at an angle close to -pi/2. 
            if abs(angdiff( pi / 2, youbotEuler(3))) < .1 / 180 * pi
                rotVel = 0;
                fsm = 'lin_forwBack1';
            end

        


        %������ǰ���ƶ�״̬��. 
        %elseif strcmp(fsm, 'cst_forwBack');
            
            %ǰ���ٶ�
            %forwBackVel = -1;
            
            % ʹ�û����˱ƽ�Y=-6.5��λ��. 
            %if youbotPos(2) + 5.3 < .001
            %    forwBackVel = 0;
             %   fsm = 'lin_forwBack';
            %end
        %    
        elseif strcmp(fsm, 'lin_forwBack1');
            %���ƻ�����ǰ���˶��ٶ���Y=-3λ�����
            forwBackVel = 2 * (youbotPos(1) + 3);
            %forwBackVel = -2 
         
            
            % ʹ�û����˱ƽ�Y=-3��λ��. 
            if abs(youbotPos(1) + 3) < .001
                forwBackVel = 0;
                fsm = 'LeftRight1';
            end
            
        elseif strcmp(fsm, 'LeftRight1');
            % ���ƻ�����ǰ���˶��ٶ���X=-5.3λ�����
            leftRightVel = - 2 * (youbotPos(2) + 5.3);
            
            % ʹ�û����˱ƽ�X=-5.3��λ��.
            if abs(youbotPos(2) + 5.3) < .001
                leftRightVel = 0;
                fsm = 'LeftRight2';
            end

        elseif strcmp(fsm, 'LeftRight2');
            % ���ƻ�����ǰ���˶��ٶ���X=-2.85λ�����
            leftRightVel = - 0.5 * (youbotPos(2) + 3);
            
            % ʹ�û����˱ƽ�X=-2.85��λ��.
            if abs(youbotPos(2) + 3) < .001
                leftRightVel = 0;
                fsm = 'Rot2';
            end
        elseif strcmp(fsm, 'Rot2');
            % Rotate. 
            rotVel = angdiff( 91*pi/180, youbotEuler(3));
            
            % Stop when the robot is at an angle close to -pi/2. 
            if abs(angdiff( 91*pi/180, youbotEuler(3))) < .1 / 180 * pi
                rotVel = 0;
                fsm = 'lin_forwBack2';
            end
        elseif strcmp(fsm, 'lin_forwBack2');
            %���ƻ�����ǰ���˶��ٶ���Y=-6.4λ�����
            forwBackVel = 1 * (youbotPos(1) + 6.2);
         
            
            % ʹ�û����˱ƽ�Y=-6.4��λ��. 
            if abs(youbotPos(1) + 6.2) < .001
                forwBackVel = 0;
                fsm = 'nav';
            end
        elseif strcmp(fsm, 'nav');
    global cellsize;
    cellsize=0.25

%###################################���ڵ�ͼ�ĵ�������####################################
  %�趨����Ŀ��
  load('fmap.mat');
  goal = [ ij(5.3), ij(-2.5) ];
  dx = Dstar(double(fmap), 'quiet');
  dx.plan([goal(1); goal(2)]);
  %���㵼���켣TRAJ
  %��ȡ�ƶ������˱����������ϵ�������������ϵ��λ�ú���̬
   % ��ȡ�ƶ��������������������ϵ��λ�ú���̬ 
        [res, youbotPos] = vrep.simxGetObjectPosition(id, h.ref, -1, vrep.simx_opmode_buffer);
        vrchk(vrep, res, true);
        [res, youbotEuler] = vrep.simxGetObjectOrientation(id, h.ref, -1, vrep.simx_opmode_buffer);
        vrchk(vrep, res, true);  
  traj = dx.path([ij(youbotPos(1)) ; ij(youbotPos(2))]);
  traj = [ xy(traj(:,1)) xy(traj(:,2)) ];
  traj = [traj; [xy(goal(1)) xy(goal(2))]];
  traj = traj (2:end,:)
  %������صı�����ʼ������
  num=1;
  %��ʼ������״̬������  
  i='rotate';
  res = vrep.simxSynchronous(h.id, true); vrchk(vrep, res);
%####################################################����������״̬������######################################################################
%��ʼ����״̬��
	while true,
	%��ȡ�ƶ������˱����������ϵ�������������ϵ��λ�ú���̬
	 % ��ȡ�ƶ��������������������ϵ��λ�ú���̬ 
        [res, youbotPos] = vrep.simxGetObjectPosition(id, h.ref, -1, vrep.simx_opmode_buffer);
        vrchk(vrep, res, true);
        [res, youbotEuler] = vrep.simxGetObjectOrientation(id, h.ref, -1, vrep.simx_opmode_buffer);
        vrchk(vrep, res, true);  
	
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
     if strcmp(i, 'rotate'),
	%ת��״̬��ش��룬�����ƶ�����������ת���ٶ�
        %angl=atan2(P_oc(2),P_oc(1));
        if P_oc(1)>0;
             angl=atan(P_oc(2)/P_oc(1))-pi/2;
        end
        
        if  P_oc(1)<0;
            angl=atan(P_oc(2)/P_oc(1))+pi/2;
        end
        
        if P_oc(1)==0
            angl=0;
        end
	    rotVel =5*angl;
        i = 'drive';
	%ǰ��״̬��
     elseif strcmp(i, 'drive'),
	%ǰ��״̬��ش��룬�����ƶ�����������ǰ���ٶ�
        forwBackVel =0.5*sqrt((youbotPos(1)-target(1))^2+(youbotPos(2)-target(2))^2);
        i = 'rotate';
        if sqrt((youbotPos(1)-target(1))^2+(youbotPos(2)-target(2))^2)<0.1
             i='finished';
        end
        
	%����״̬��
     elseif strcmp(i, 'finished'),
	%����״̬����ش���
        num=num+1;
        i='rotate';
        if target==traj(end,:)
            break;
        end
     end
     end
  % �����ƶ������˿���״̬
   h = youbot_drive(vrep, h, forwBackVel, leftRightVel, rotVel);
        %elseif strcmp(fsm, 'finished'),
            %pause(3);
            %break
            
        else
            error(sprintf('Unknown state %s.', fsm));
        end

        % Update wheel velocities for each loop
        h = youbot_drive(vrep, h, forwBackVel, leftRightVel, rotVel);

        % Make sure that we do not go faster that the simulator. 
        elapsed = toc;
        timeleft = timestep - elapsed;
        if (timeleft > 0),
          pause(min(timeleft, .01));
        end
    end

end % main function
   
