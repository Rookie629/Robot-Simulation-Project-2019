function handles = RGBD_sensor_init(vrep, id)
% Initialize youBot


handles = struct('id', id);



[res xyzSensor] = vrep.simxGetObjectHandle(id, 'xyz_Sensor', vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);
[res rgbSensor] = vrep.simxGetObjectHandle(id, 'rgb_Sensor', vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);


handles.xyzSensor = xyzSensor;
handles.rgbSensor = rgbSensor;

vrep.simxGetPingTime(id); % make sure that all streaming data has reached the client at least once

end
