% MATLAB controller for Webots
% File:          controller.m
% Date:
% Description:
% Author:
% Modifications:

STAGE = 0
SUBSTAGE = 0
TIME_STEP = 64;
MAX_ANGULAR_VEL = 1.5625;
REV_SPD = MAX_ANGULAR_VEL*0.0025; 
GRADIENT = 0
INTERCEPT = 0
CURRENT_TARGET = [0.1, -0.35];
CURRENT_TARGET_ORIENTATON = 0;
DELIVERY_TARGET = [2.5, 0.03];
DELIVERY_TARGET_ORIENTATON = 90;
OBSTACLE_THR = 500;
target_bearing = 0;
last_bearing = 0;
revolve_direction = 1 % 1 is clockwise
vel_1 = 0;
vel_2 = 0;
vel_3 = 0;
vel_4 = 0;


% initializing wheel_1
wheel_1 = wb_robot_get_device('wheel_1');
wb_motor_set_position(wheel_1, inf)
wb_motor_set_velocity(wheel_1, 0);

% initializing wheel_2
wheel_2 = wb_robot_get_device('wheel_2');
wb_motor_set_position(wheel_2, inf)
wb_motor_set_velocity(wheel_2, 0);

% initializing wheel_3
wheel_3 = wb_robot_get_device('wheel_3');
wb_motor_set_position(wheel_3, inf)
wb_motor_set_velocity(wheel_3, 0);

% initializing wheel_4
wheel_4 = wb_robot_get_device('wheel_4');
wb_motor_set_position(wheel_4, inf)
wb_motor_set_velocity(wheel_4, 0);

% initializing distance sensors
ds_centre = wb_robot_get_device('ds_centre');
wb_distance_sensor_enable(ds_centre, TIME_STEP);

ds_left   = wb_robot_get_device('ds_left');
wb_distance_sensor_enable(ds_left, TIME_STEP);

ds_right  = wb_robot_get_device('ds_right');
wb_distance_sensor_enable(ds_right, TIME_STEP);

ds_dFL  = wb_robot_get_device('dFL');
wb_distance_sensor_enable(ds_dFL, TIME_STEP);

ds_dRL  = wb_robot_get_device('dRL');
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


while wb_robot_step(TIME_STEP) ~= -1
  
  % reading distance sensors
  ds_val_centre = wb_distance_sensor_get_value(ds_centre);
  ds_val_left = wb_distance_sensor_get_value(ds_left);
  ds_val_right = wb_distance_sensor_get_value(ds_right);
  
  % reading gps module
  current_gps = wb_gps_get_values(gps);
  
  % reading compass module
  current_orientation = wb_compass_get_values(compass);
  
  bearing = get_bearing(current_orientation,last_bearing);
  last_bearing = bearing + 0;
  
  if STAGE == 0 
    [vel_1, vel_2, vel_3, vel_4] = stop();  
    GRADIENT = (CURRENT_TARGET(2)-current_gps(2))/(CURRENT_TARGET(1)-current_gps(1));
    INTERCEPT = CURRENT_TARGET(2)-(GRADIENT*CURRENT_TARGET(1));
    STAGE = 1
  elseif STAGE == 1; 
    if round(CURRENT_TARGET(1)*10)==round(current_gps(1)*10) & round(CURRENT_TARGET(2)*10)==round(current_gps(2)*10)
      [vel_1, vel_2, vel_3, vel_4] = stop();
      STAGE = 5 
    elseif ds_val_centre<OBSTACLE_THR | ds_val_left<OBSTACLE_THR | ds_val_right<OBSTACLE_THR 
      wb_console_print(strcat('STAGE: ', num2str(STAGE), '  ds_centre: ', num2str(ds_val_centre), '  ds_left: ', num2str(ds_val_left), '  ds_right: ', num2str(ds_val_right)), WB_STDOUT);
      [vel_1, vel_2, vel_3, vel_4] = stop();
      STAGE = 2 
    else
      angle_rad = atan2(CURRENT_TARGET(1)-current_gps(1),CURRENT_TARGET(2)-current_gps(2));
      target_bearing = (angle_rad) / pi * 180.0;
      if target_bearing < 0.0
        target_bearing = target_bearing + 360.0;
      end
      target_bearing = round(target_bearing*100);
      target_bearing = target_bearing/100;
      if target_bearing == 360
        target_bearing=0;
      end
      
      REV_SPD = get_rev_speed(bearing,target_bearing,MAX_ANGULAR_VEL);
      
      if imabsdiff(bearing,target_bearing) < 0.02
        [vel_1, vel_2, vel_3, vel_4] = traverse(MAX_ANGULAR_VEL,1);
        
      elseif target_bearing>bearing
        [vel_1, vel_2, vel_3, vel_4] = spin_right(REV_SPD,1);
      else
        [vel_1, vel_2, vel_3, vel_4] = spin_right(REV_SPD,-1);
      end 
    end
  elseif STAGE == 2 
    if SUBSTAGE == 0
      target_bearing = mod(bearing + 90,360)
      [vel_1, vel_2, vel_3, vel_4] = stop();
      SUBSTAGE = 1
    elseif SUBSTAGE == 1
      REV_SPD = get_rev_speed(bearing,target_bearing,MAX_ANGULAR_VEL);
      
      if bearing<target_bearing
        [vel_1, vel_2, vel_3, vel_4] = spin_right(REV_SPD,1); 
      else
        [vel_1, vel_2, vel_3, vel_4] = stop();
        SUBSTAGE = 2
      end
      
    elseif SUBSTAGE == 2
      y = (GRADIENT*current_gps(1))+INTERCEPT
      abs_diff = imabsdiff( y,current_gps(2))
      if abs_diff < 0.1
        wb_console_print(strcat('STAGE: ', num2str(STAGE), '  current_x: ', num2str(current_gps(1)), '  current_y: ', num2str(current_gps(2)), '  calculated y: ', num2str(y)), WB_STDOUT);
        [vel_1, vel_2, vel_3, vel_4] = traverse(MAX_ANGULAR_VEL,1);
      else
        [vel_1, vel_2, vel_3, vel_4] = stop();
        SUBSTAGE = 0
        STAGE = 3
      end
    end
  elseif STAGE == 3 
    y = (GRADIENT*current_gps(1))+INTERCEPT
    abs_diff = imabsdiff( y,current_gps(2))
    if abs_diff < 0.09
      [vel_1, vel_2, vel_3, vel_4] = stop();
      STAGE = 4
    
    else
      dFL = wb_distance_sensor_get_value(ds_dFL);
      dRL = wb_distance_sensor_get_value(ds_dRL);
      
      dWallSide = 150 
      kWall = 0.002 
      b = 100 
      e = 200 
      wheel_radius = 85 
      a = 200 
      phi = atan((dRL-dFL)/a)
      d = (dWallSide-0.5*(dFL+dRL))
      
      gamma = kWall*d
      alpha = phi+gamma
      wL = MAX_ANGULAR_VEL*(cos(alpha)+(b/e)*sin(alpha))
      wR = MAX_ANGULAR_VEL*(cos(alpha)-(b/e)*sin(alpha))
      
     
      vel_1 = wL;
      vel_2 = wR;
      vel_3 = wL;
      vel_4 = wR;
    end
  elseif STAGE == 4  
    if SUBSTAGE == 0
      angle_rad = atan2(CURRENT_TARGET(1)-current_gps(1),CURRENT_TARGET(2)-current_gps(2));
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
      SUBSTAGE = 1
      revolve_direction = get_optimal_turn_direction(bearing,target_bearing);
      
    elseif SUBSTAGE == 1
      wb_console_print(strcat('STAGE: ', num2str(STAGE), '  bearing: ', num2str(bearing), '  target bearing: ', num2str(target_bearing)), WB_STDOUT);
      abs_diff = imabsdiff( bearing, target_bearing)
      
      REV_SPD = get_rev_speed(bearing,target_bearing,MAX_ANGULAR_VEL);
      
      if abs_diff>0.1
        [vel_1, vel_2, vel_3, vel_4] = spin_right(REV_SPD,revolve_direction);
        
      else
        [vel_1, vel_2, vel_3, vel_4] = stop();
        SUBSTAGE = 0
        STAGE = 1
      end
    end
    
  elseif STAGE == 5 
    if SUBSTAGE == 0
      target_bearing = CURRENT_TARGET_ORIENTATON;
      [vel_1, vel_2, vel_3, vel_4] = stop();
      SUBSTAGE = 1
      revolve_direction = get_optimal_turn_direction(bearing,target_bearing);
    elseif SUBSTAGE == 1
      abs_diff = imabsdiff( bearing, target_bearing);
      REV_SPD = get_rev_speed(bearing,target_bearing,MAX_ANGULAR_VEL);
      
      if abs_diff>0.2
        [vel_1, vel_2, vel_3, vel_4] = spin_right(REV_SPD,revolve_direction);
      else
        [vel_1, vel_2, vel_3, vel_4] = stop();
        SUBSTAGE = 2
      end
    elseif SUBSTAGE == 2
      if ds_val_left < 230
        [vel_1, vel_2, vel_3, vel_4] = stop();
        SUBSTAGE = 0
        STAGE = 6
      else
        if CURRENT_TARGET(1) == DELIVERY_TARGET(1) & CURRENT_TARGET(2) == DELIVERY_TARGET(2) 
          STAGE = 8
          SUBSTAGE = 0
          [vel_1, vel_2, vel_3, vel_4] = stop();
        else
          [vel_1, vel_2, vel_3, vel_4] = traverse(MAX_ANGULAR_VEL,1);
        end
      end
    end
  elseif STAGE == 6 
    box_presence_1 = wb_distance_sensor_get_value(box_1);
    box_presence_2 = wb_distance_sensor_get_value(box_2);
    [vel_1, vel_2, vel_3, vel_4] = stop();
    if box_presence_1 < 1000 | box_presence_2 < 1000
      STAGE = 7
      
    end
  elseif STAGE == 7
    if ds_val_left > 990
        [vel_1, vel_2, vel_3, vel_4] = stop();
        if CURRENT_TARGET(1) == DELIVERY_TARGET(1) & CURRENT_TARGET(2) == DELIVERY_TARGET(2) 
          STAGE = 8
        else
          STAGE = 0
          CURRENT_TARGET(1) = DELIVERY_TARGET(1);
          CURRENT_TARGET(2) = DELIVERY_TARGET(2);
          CURRENT_TARGET_ORIENTATON = DELIVERY_TARGET_ORIENTATON;
        end
      else
        [vel_1, vel_2, vel_3, vel_4] = traverse(MAX_ANGULAR_VEL,-1);
      end
  elseif STAGE == 8
    [vel_1, vel_2, vel_3, vel_4] = stop();
  else
    [vel_1, vel_2, vel_3, vel_4] = stop();
  end
  wb_motor_set_velocity(wheel_1, vel_1);
  wb_motor_set_velocity(wheel_2, vel_2);
  wb_motor_set_velocity(wheel_3, vel_3);
  wb_motor_set_velocity(wheel_4, vel_4);
  
end

