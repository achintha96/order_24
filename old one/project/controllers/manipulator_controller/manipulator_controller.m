% MATLAB controller for Webots
% File:          manipulator_controller.m
% Date:
% Description:
% Author:
% Modifications:

% uncomment the next two lines if you want to use
% MATLAB's desktop to interact with the controller:
%desktop;
%keyboard;

TIME_STEP = 64;

STAGE = 0
SUBSTAGE = 0
theta_1 = 0
theta_2 = 0
theta_3 = 0
vel_1 = 0
vel_2 = 0
vel_3 = 0
target_x = -0.2
target_y = 0.09

% initializing connector
gripper = wb_robot_get_device('gripper');
wb_connector_enable_presence(gripper, TIME_STEP)

% initializing motors
motor_1 = wb_robot_get_device('motor_1');
motor_2 = wb_robot_get_device('motor_2');
motor_3 = wb_robot_get_device('motor_3');

wb_motor_set_position(motor_1, inf)
wb_motor_set_position(motor_2, inf)
wb_motor_set_position(motor_3, inf)

wb_motor_set_velocity(motor_1, vel_1);
wb_motor_set_velocity(motor_2, vel_2);
wb_motor_set_velocity(motor_3, vel_3);

% initializing encoders
encoder_1 = wb_robot_get_device('encoder_1')
encoder_2 = wb_robot_get_device('encoder_2')
encoder_3 = wb_robot_get_device('encoder_3')

wb_position_sensor_enable(encoder_1, TIME_STEP)
wb_position_sensor_enable(encoder_2, TIME_STEP)
wb_position_sensor_enable(encoder_3, TIME_STEP)

% initializing distance sensors
robot_sensor = wb_robot_get_device('robot_sensor');
wb_distance_sensor_enable(robot_sensor, TIME_STEP);

while wb_robot_step(TIME_STEP) ~= -1
  % reading motor encoders
  pos_1 = wb_position_sensor_get_value(encoder_1)
  pos_2 = wb_position_sensor_get_value(encoder_2)
  pos_3 = wb_position_sensor_get_value(encoder_3)
  
  % reading distance sensors
  robot_presence = wb_distance_sensor_get_value(robot_sensor);
  
  if STAGE == 0
    vel_1 = 0
    vel_2 = 0
    vel_3 = 0
    %wb_console_print(strcat('STAGE: ', num2str(STAGE), '   Robot has not arrived. robot_presence:  ', num2str(robot_presence)), WB_STDOUT); 
    if robot_presence < 1000
      STAGE = 1
    else
      wb_console_print(strcat('STAGE: ', num2str(STAGE), '   Robot has not arrived. robot_presence:  ', num2str(robot_presence)), WB_STDOUT); 
    end  
  elseif STAGE == 1  
    wb_console_print(strcat('STAGE: ', num2str(STAGE), '   target_x : ', num2str(target_x), '  target_y : ', num2str(target_y)), WB_STDOUT); 
    if SUBSTAGE == 0  % going to top most safe pose
      STAGE = 40
      SUBSTAGE = 1
    elseif SUBSTAGE == 1  % safely lowering
      if target_y == 0.04
        SUBSTAGE = 2
      else
        target_y = target_y - 0.01
        STAGE = 40
      end
    elseif SUBSTAGE == 2  % gripping
      presence = wb_connector_get_presence(gripper)
      wb_console_print(strcat('GRIPPER: ',num2str(presence)), WB_STDOUT);  
      wb_connector_lock(gripper)
      SUBSTAGE = 3
    elseif SUBSTAGE == 3  % safely lifting
      if target_y == 0.08
        SUBSTAGE = 4
      else
        target_y = target_y + 0.01
        STAGE = 40
      end
    elseif SUBSTAGE == 4 % going back to dropping position
      target_x = 0.34
      target_y = 0.0
      STAGE = 40
      SUBSTAGE = 5
    elseif SUBSTAGE == 5 %dropping
      wb_connector_unlock(gripper)
      STAGE = 50
    end
    vel_1 = 0
    vel_2 = 0
    vel_3 = 0
      
  elseif STAGE == 40
    [theta_1,theta_2,theta_3] = find_IK(target_x,target_y,0.2,0.15)
    wb_console_print(strcat('STAGE: ', num2str(STAGE), '   theta_1 : ', num2str(theta_1), '  theta_2 : ', num2str(theta_2), '  theta_3 : ', num2str(theta_3)), WB_STDOUT);  
    
    vel_1 = 0
    vel_2 = 0
    vel_3 = 0
    STAGE = 41
  elseif STAGE == 41
    L1_con = 0
    L2_con = 0
    L3_con = 0
    
    wb_console_print(strcat('STAGE: ', num2str(STAGE), '   pos_1 : ', num2str(pos_1), '  pos_2 : ', num2str(pos_2), '  pos_3 : ', num2str(pos_3)), WB_STDOUT);
    
    % link 1
    if imabsdiff( theta_1, pos_1) < 0.01
      %wb_motor_set_velocity(motor_1, 0);
      vel_1 = 0
      L1_con = 1
    else
      if theta_1 > pos_1
        vel_1 = 0.1
        %wb_motor_set_velocity(motor_1, 0.1);
      else
        vel_1 = -0.1
        %wb_motor_set_velocity(motor_1, -0.1);
      end
    end
    
    % link 2
    if imabsdiff( theta_2, pos_2) < 0.01
      %wb_motor_set_velocity(motor_2, 0);
      vel_2 = 0
      L2_con = 1
    else
      if theta_2 > pos_2
        vel_2 = 0.1
        %wb_motor_set_velocity(motor_2, 0.1);
      else
        vel_2 = -0.1
        %wb_motor_set_velocity(motor_2, -0.1);
      end
    end
    
    % link 3
    if imabsdiff( theta_3, pos_3) < 0.01
      %wb_motor_set_velocity(motor_3, 0);
      vel_3 = 0
      L3_con = 1
    else
      if theta_3 > pos_3
        vel_3 = 0.1
        %wb_motor_set_velocity(motor_3, 0.1);
      else
        vel_3 = -0.1
        %wb_motor_set_velocity(motor_3, -0.1);
      end
    end 
    
    wb_console_print(strcat('STAGE: ', num2str(STAGE), '   L1 : ', num2str(L1_con), '  L2 : ', num2str(L2_con), '  L3 : ', num2str(L3_con), '  L1*L2*L3 : ', num2str(L1_con*L2_con*L3_con)), WB_STDOUT);
    
    if L1_con * L2_con * L3_con == 1
      STAGE = 1
    end
  
  
  elseif STAGE == 50
    wb_console_print(strcat('STAGE: ', num2str(STAGE) ), WB_STDOUT);
    wb_motor_set_velocity(motor_1, 0);
    wb_motor_set_velocity(motor_2, 0);
    wb_motor_set_velocity(motor_3, 0);
  end

  wb_motor_set_velocity(motor_1, vel_1);
  wb_motor_set_velocity(motor_2, vel_2);
  wb_motor_set_velocity(motor_3, vel_3);  

end

% cleanup code goes here: write data to files, etc.
