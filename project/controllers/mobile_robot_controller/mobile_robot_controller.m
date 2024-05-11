% MATLAB controller for Webots
% File:          controller.m
% Date:
% Description:
% Author:
% Modifications:

% global variable initialization
STAGE = 0
SUBSTAGE = 0
TIME_STEP = 64;
MAX_OMEGA = 1.5625;
REV_SPD = MAX_OMEGA*0.0025; %wheel speed in revolution
vel_1 = 0;
vel_2 = 0;
vel_3 = 0;
vel_4 = 0;
TARGET = [0.1, -0.35];
TARGET_ORIENTATON = 0;
DESTINATION = [2.5, 0.03];
DESTINATION_ORIENTATON = 90;
OBSTACLE_THR = 500;
target_bearing = 0;
last_bearing = 0;
ccw_turn = 1 % 1--> CCW, -1-->ACCW
GRADIENT = 0
INTERCEPT = 0

% initializing motors
wheel_1 = wb_robot_get_device('wheel_1');
wheel_2 = wb_robot_get_device('wheel_2');
wheel_3 = wb_robot_get_device('wheel_3');
wheel_4 = wb_robot_get_device('wheel_4');

wb_motor_set_position(wheel_1, inf)
wb_motor_set_position(wheel_2, inf)
wb_motor_set_position(wheel_3, inf)
wb_motor_set_position(wheel_4, inf)

wb_motor_set_velocity(wheel_1, 0);
wb_motor_set_velocity(wheel_2, 0);
wb_motor_set_velocity(wheel_3, 0);
wb_motor_set_velocity(wheel_4, 0);

% initializing distance sensors
ds_centre = wb_robot_get_device('ds_centre');
ds_left   = wb_robot_get_device('ds_left');
ds_right  = wb_robot_get_device('ds_right');
ds_dFL  = wb_robot_get_device('dFL');
ds_dRL  = wb_robot_get_device('dRL');

wb_distance_sensor_enable(ds_centre, TIME_STEP);
wb_distance_sensor_enable(ds_left, TIME_STEP);
wb_distance_sensor_enable(ds_right, TIME_STEP);
wb_distance_sensor_enable(ds_dFL, TIME_STEP);
wb_distance_sensor_enable(ds_dRL, TIME_STEP);

box_1 = wb_robot_get_device('load_sensor_1');
wb_distance_sensor_enable(box_1, TIME_STEP);
box_2 = wb_robot_get_device('load_sensor_2');
wb_distance_sensor_enable(box_2, TIME_STEP);

ds_dFL = wb_robot_get_device('ds_dFL');
wb_distance_sensor_enable(ds_dFL, TIME_STEP);
ds_dRL = wb_robot_get_device('ds_dRL');
wb_distance_sensor_enable(ds_dRL, TIME_STEP);

% initializing GPS module
gps = wb_robot_get_device('gps');
wb_gps_enable(gps, TIME_STEP);

% initializing compass module
compass = wb_robot_get_device('compass');
wb_compass_enable(compass, TIME_STEP);

% loop
while wb_robot_step(TIME_STEP) ~= -1
  
  % reading distance sensors
  value_centre = wb_distance_sensor_get_value(ds_centre)
  value_left = wb_distance_sensor_get_value(ds_left)
  value_right = wb_distance_sensor_get_value(ds_right)
  
  % reading gps module
  current_pos = wb_gps_get_values(gps)
  
  % reading compass module
  current_ori = wb_compass_get_values(compass);
  
  bearing = get_bearing(current_ori,last_bearing)
  last_bearing = bearing + 0
  % stage machine
  if STAGE == 0 % initializations for target
    dy = TARGET(2)-current_pos(2)
    dx = TARGET(1)-current_pos(1)
    GRADIENT = dy/dx;
    INTERCEPT = TARGET(2)-(GRADIENT*TARGET(1));
    
    [vel_1, vel_2, vel_3, vel_4] = stop();
    STAGE = 1;
    wb_console_print(strcat('STAGE: ', num2str(STAGE), '  y = ', num2str(GRADIENT), '  x + ', num2str(INTERCEPT)), WB_STDOUT);
  elseif STAGE == 1; % moving towards target
    if round(TARGET(1)*10)==round(current_pos(1)*10) & round(TARGET(2)*10)==round(current_pos(2)*10)
      %checking for arrival of destination
      [vel_1, vel_2, vel_3, vel_4] = stop();
      STAGE = 5; %go to arrival orientation correction
    elseif value_centre<OBSTACLE_THR | value_left<OBSTACLE_THR | value_right<OBSTACLE_THR 
      %checking for obstacle -> if true right revolve
      wb_console_print(strcat('STAGE: ', num2str(STAGE), '  ds_centre: ', num2str(value_centre), '  ds_left: ', num2str(value_left), '  ds_right: ', num2str(value_right)), WB_STDOUT);
      [vel_1, vel_2, vel_3, vel_4] = stop();
      STAGE = 2; % revolve stage
    else
      % calculating the bearing of target wrt current pos
      %wb_console_print(strcat('x: ', num2str(current_pos(1)), ' y :', num2str(current_pos(2))), WB_STDOUT);
      angle_rad = atan2(TARGET(1)-current_pos(1),TARGET(2)-current_pos(2));
      %wb_console_print(strcat('dx: ', num2str(TARGET(1)-current_pos(1)), ' dy :', num2str(TARGET(2)-current_pos(2))), WB_STDOUT);
      target_bearing = (angle_rad) / pi * 180.0;
      if target_bearing < 0.0
        target_bearing = target_bearing + 360.0;
      end
      target_bearing = round(target_bearing*100);
      target_bearing = target_bearing/100;
      if target_bearing == 360
        target_bearing=0;
      end
      wb_console_print(strcat('STAGE: ', num2str(STAGE), '  bearing: ', num2str(bearing), '  target bearing: ', num2str(target_bearing)), WB_STDOUT);
      %if -1<target_bearing-bearing <1
      % if round(target_bearing)==round(bearing)
      if imabsdiff(bearing,target_bearing) > 2
        REV_SPD = MAX_OMEGA*0.1;
      else
        REV_SPD = MAX_OMEGA*0.0025;
      end
      
      if imabsdiff(bearing,target_bearing) < 0.02
        vel_1 = MAX_OMEGA;
        vel_2 = MAX_OMEGA;
        vel_3 = MAX_OMEGA;
        vel_4 = MAX_OMEGA;
      elseif target_bearing>bearing
        %revolve right
        vel_1 = REV_SPD;
        vel_2 = -REV_SPD;
        vel_3 = REV_SPD;
        vel_4 = -REV_SPD;
      else
        %revolve left
        vel_1 = -REV_SPD;
        vel_2 = REV_SPD;
        vel_3 = -REV_SPD;
        vel_4 = REV_SPD;
      end 
    end
  elseif STAGE == 2 % turn right and going around obstacle - bug algorithm
    if SUBSTAGE == 0
      target_bearing = mod(bearing + 90,360)
      [vel_1, vel_2, vel_3, vel_4] = stop();
      SUBSTAGE = 1;
    elseif SUBSTAGE == 1
      wb_console_print(strcat('STAGE: ', num2str(STAGE), '  bearing: ', num2str(bearing), '  target bearing: ', num2str(target_bearing)), WB_STDOUT);
      
      if imabsdiff(bearing,target_bearing) > 2
        REV_SPD = MAX_OMEGA*0.1;
      else
        REV_SPD = MAX_OMEGA*0.0025;
      end
      
      if bearing<target_bearing
        vel_1 = REV_SPD;
        vel_2 = -REV_SPD;
        vel_3 = REV_SPD;
        vel_4 = -REV_SPD;
      else
        [vel_1, vel_2, vel_3, vel_4] = stop();
        SUBSTAGE = 2;
        %STAGE = 5;
      end
    elseif SUBSTAGE == 2
      y = (GRADIENT*current_pos(1))+INTERCEPT
      abs_diff = imabsdiff( y,current_pos(2))
      if abs_diff < 0.1
        wb_console_print(strcat('STAGE: ', num2str(STAGE), '  current_x: ', num2str(current_pos(1)), '  current_y: ', num2str(current_pos(2)), '  calculated y: ', num2str(y)), WB_STDOUT);
        vel_1 = MAX_OMEGA;
        vel_2 = MAX_OMEGA;
        vel_3 = MAX_OMEGA;
        vel_4 = MAX_OMEGA;
      else
        [vel_1, vel_2, vel_3, vel_4] = stop();
        SUBSTAGE = 0;
        STAGE = 3;
      end
    end
  elseif STAGE == 3 %wall following
    y = (GRADIENT*current_pos(1))+INTERCEPT
    abs_diff = imabsdiff( y,current_pos(2))
    if abs_diff < 0.09
      wb_console_print(strcat('STAGE*: ', num2str(STAGE), '  current_x: ', num2str(current_pos(1)), '  current_y: ', num2str(current_pos(2)), '  calculated y: ', num2str(y)), WB_STDOUT);
      [vel_1, vel_2, vel_3, vel_4] = stop();
      STAGE = 4;
    
    else
      dFL = wb_distance_sensor_get_value(ds_dFL);
      dRL = wb_distance_sensor_get_value(ds_dRL);
      
      dWallSide = 150 %300 
      kWall = 0.002 
      b = 100 %in mm
      e = 200 %in mm
      wheel_radius = 85 %in mm
      a = 200 %in mm
      phi = atan((dRL-dFL)/a)
      d = (dWallSide-0.5*(dFL+dRL))
      
      gamma = kWall*d
      alpha = phi+gamma
      wL = MAX_OMEGA*(cos(alpha)+(b/e)*sin(alpha))
      wR = MAX_OMEGA*(cos(alpha)-(b/e)*sin(alpha))
      
      %if wL<1.3
      %  wL = 1.3
      %elseif wL>1.7
      %  wL = 1.7
      %end
      
      %if wR<1.3
      %  wR = 1.3
      %elseif wR>1.7
      %  wR = 1.7
      %end
      
      wb_console_print(strcat('STAGE: ', num2str(STAGE), '  dFL: ', num2str(dFL), '  dRL: ', num2str(dRL), '  wL: ', num2str(wL), '  wR: ', num2str(wR)), WB_STDOUT);
      
      vel_1 = wL;
      vel_2 = wR;
      vel_3 = wL;
      vel_4 = wR;
    end
  elseif STAGE == 4  
    if SUBSTAGE == 0
      % calculating the bearing of target wrt current pos
      %wb_console_print(strcat('x: ', num2str(current_pos(1)), ' y :', num2str(current_pos(2))), WB_STDOUT);
      angle_rad = atan2(TARGET(1)-current_pos(1),TARGET(2)-current_pos(2));
      %wb_console_print(strcat('dx: ', num2str(TARGET(1)-current_pos(1)), ' dy :', num2str(TARGET(2)-current_pos(2))), WB_STDOUT);
      target_bearing = (angle_rad) / pi * 180.0;
      if target_bearing < 0.0
        target_bearing = target_bearing + 360.0;
      end
      target_bearing = round(target_bearing*100);
      target_bearing = target_bearing/100;
      if target_bearing == 360
        target_bearing=0;
      end
      
      [vel_1, vel_2, vel_3, vel_4] = stop();
      SUBSTAGE = 1;
      accw_cost = 0
      ccw_cost = 0
      if bearing>target_bearing
        accw_cost = bearing - target_bearing
        ccw_cost = 360 - bearing + target_bearing
      else
        ccw_cost = target_bearing - bearing
        accw_cost = 360 - target_bearing + bearing
      end
      
      if accw_cost > ccw_cost
        ccw_turn = 1
      else
        ccw_turn = -1
      end
    elseif SUBSTAGE == 1
      wb_console_print(strcat('STAGE: ', num2str(STAGE), '  bearing: ', num2str(bearing), '  target bearing: ', num2str(target_bearing)), WB_STDOUT);
      abs_diff = imabsdiff( bearing, target_bearing)
      
      if imabsdiff(bearing,target_bearing) > 2
        REV_SPD = MAX_OMEGA*0.1;
      else
        REV_SPD = MAX_OMEGA*0.0025;
      end
      
      if abs_diff>0.1
        vel_1 = REV_SPD*ccw_turn;
        vel_2 = -REV_SPD*ccw_turn;
        vel_3 = REV_SPD*ccw_turn;
        vel_4 = -REV_SPD*ccw_turn;
      else
        [vel_1, vel_2, vel_3, vel_4] = stop();
        SUBSTAGE = 0;
        STAGE = 1;
      end
    end
  elseif STAGE == 5 %correcting orientaton after arrival
    if SUBSTAGE == 0
      target_bearing = TARGET_ORIENTATON
      [vel_1, vel_2, vel_3, vel_4] = stop();
      SUBSTAGE = 1;
      accw_cost = 0
      ccw_cost = 0
      if bearing>target_bearing
        accw_cost = bearing - target_bearing
        ccw_cost = 360 - bearing + target_bearing
      else
        ccw_cost = target_bearing - bearing
        accw_cost = 360 - target_bearing + bearing
      end
      
      if accw_cost > ccw_cost
        ccw_turn = 1
      else
        ccw_turn = -1
      end
      
    elseif SUBSTAGE == 1
      wb_console_print(strcat('STAGE: ', num2str(STAGE), '  bearing: ', num2str(bearing), '  target bearing: ', num2str(target_bearing)), WB_STDOUT);
      abs_diff = imabsdiff( bearing, target_bearing)
      
      if imabsdiff(bearing,target_bearing) > 2
        REV_SPD = MAX_OMEGA*0.1;
      else
        REV_SPD = MAX_OMEGA*0.0025;
      end
      
      if abs_diff>0.2
        vel_1 = REV_SPD*ccw_turn;
        vel_2 = -REV_SPD*ccw_turn;
        vel_3 = REV_SPD*ccw_turn;
        vel_4 = -REV_SPD*ccw_turn;
      else
        [vel_1, vel_2, vel_3, vel_4] = stop();
        SUBSTAGE = 2
        % SUBSTAGE = 0;
        % STAGE = 6;
        end
    elseif SUBSTAGE == 2
      if value_left < 230
        [vel_1, vel_2, vel_3, vel_4] = stop();
        SUBSTAGE = 0;
        STAGE = 6;
      else
        if TARGET(1) == DESTINATION(1) & TARGET(2) == DESTINATION(2) 
          STAGE = 100
          SUBSTAGE = 0
          [vel_1, vel_2, vel_3, vel_4] = stop();
        else
          vel_1 = MAX_OMEGA;
          vel_2 = MAX_OMEGA;
          vel_3 = MAX_OMEGA;
          vel_4 = MAX_OMEGA;
        end
      end
      % wb_console_print(strcat('STAGE: ', num2str(STAGE), '  SUBSTAGE: ', num2str(SUBSTAGE), '  center_ds: ', num2str(value_centre)), WB_STDOUT);
    end
  elseif STAGE == 6 %waiting for box
    box_presence_1 = wb_distance_sensor_get_value(box_1);
    box_presence_2 = wb_distance_sensor_get_value(box_2);
    wb_console_print(strcat('STAGE: ', num2str(STAGE), '   box_presence_1: ',num2str(box_presence_1), '   box_presence_2: ',num2str(box_presence_2)), WB_STDOUT);
    [vel_1, vel_2, vel_3, vel_4] = stop();
    if box_presence_1 < 1000 | box_presence_2 < 1000
      STAGE = 7
      % if TARGET(1) == DESTINATION(1) & TARGET(2) == DESTINATION(2) 
        % STAGE = 100
      % else
        % STAGE = 0
        % TARGET(1) = DESTINATION(1)
        % TARGET(2) = DESTINATION(2)
        % TARGET_ORIENTATON = DESTINATION_ORIENTATON
      % end
    end
  elseif STAGE == 7
    if value_left > 990
        [vel_1, vel_2, vel_3, vel_4] = stop();
        if TARGET(1) == DESTINATION(1) & TARGET(2) == DESTINATION(2) 
          STAGE = 100
        else
          STAGE = 0
          TARGET(1) = DESTINATION(1)
          TARGET(2) = DESTINATION(2)
          TARGET_ORIENTATON = DESTINATION_ORIENTATON
        end
      else
        vel_1 = -MAX_OMEGA;
        vel_2 = -MAX_OMEGA;
        vel_3 = -MAX_OMEGA;
        vel_4 = -MAX_OMEGA;
      end
  elseif STAGE == 100
    wb_console_print(strcat('STAGE: ', num2str(STAGE), ' Arrived destination '), WB_STDOUT);
    [vel_1, vel_2, vel_3, vel_4] = stop();
  else
    [vel_1, vel_2, vel_3, vel_4] = stop();
  end
  wb_motor_set_velocity(wheel_1, vel_1);
  wb_motor_set_velocity(wheel_2, vel_2);
  wb_motor_set_velocity(wheel_3, vel_3);
  wb_motor_set_velocity(wheel_4, vel_4);
  
  %wb_console_print(num2str(bearing), WB_STDOUT);
end

% cleanup code goes here: write data to files, etc.
