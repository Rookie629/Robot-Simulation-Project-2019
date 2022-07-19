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
    fsm = 'Rot';

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
        if strcmp(fsm, 'Rot');
            % Rotate. 
            rotVel = angdiff( pi / 2, youbotEuler(3));
            
            % Stop when the robot is at an angle close to -pi/2. 
            if abs(angdiff( pi / 2, youbotEuler(3))) < .1 / 180 * pi
                rotVel = 0;
                fsm = 'lin_forwBack';
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
        elseif strcmp(fsm, 'lin_forwBack');
            %���ƻ�����ǰ���˶��ٶ���Y=-3λ�����
            forwBackVel = 2 * (youbotPos(1) + 3);
            %forwBackVel = -2 
         
            
            % ʹ�û����˱ƽ�Y=-3��λ��. 
            if abs(youbotPos(1) + 3) < .001
                forwBackVel = 0;
                fsm = 'LeftRight';
            end
            
        elseif strcmp(fsm, 'LeftRight');
            % ���ƻ�����ǰ���˶��ٶ���X=-5.3λ�����
            leftRightVel = - 2 * (youbotPos(2) + 5.3);
            
            % ʹ�û����˱ƽ�X=-5.3��λ��.
            if abs(youbotPos(2) + 5.3) < .001
                leftRightVel = 0;
                fsm = 'LeftRight2';
            end
        elseif strcmp(fsm, 'LeftRight2');
            % ���ƻ�����ǰ���˶��ٶ���X=-2.85λ�����
            leftRightVel = - 1 * (youbotPos(2) + 3);
            
            % ʹ�û����˱ƽ�X=-2.85��λ��.
            if abs(youbotPos(2) + 3) < .001
                leftRightVel = 0;
                fsm = 'lin_forwBack2';
            end
                    elseif strcmp(fsm, 'lin_forwBack2');
            %���ƻ�����ǰ���˶��ٶ���Y=-6.4λ�����
            forwBackVel = 2 * (youbotPos(1) + 6.4);
            %forwBackVel = -2 
         
            
            % ʹ�û����˱ƽ�Y=-6.4��λ��. 
            if abs(youbotPos(1) + 6.4) < .001
                forwBackVel = 0;
                fsm = 'finished';
            end
        elseif strcmp(fsm, 'finished'),
            pause(3);
            break
            
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
   
