
TIME_STEP = 64;

STAGE = 0
SUBSTAGE = 0
theta_1 = 0
theta_2 = 0
theta_3 = 0
vel_1 = 0
vel_2 = 0
vel_3 = 0
target_x = 0.25
target_y = 0.1  %0.15

% initializing connector
jaw = wb_robot_get_device('connector');
wb_connector_enable_presence(jaw, TIME_STEP)

% initializing motors
motor_1 = wb_robot_get_device('link_1_motor');
wb_motor_set_position(motor_1, inf)
wb_motor_set_velocity(motor_1, vel_1);
encoder_1 = wb_robot_get_device('link_1_encoder')
wb_position_sensor_enable(encoder_1, TIME_STEP)

motor_2 = wb_robot_get_device('link_2_motor');
wb_motor_set_position(motor_2, inf)
wb_motor_set_velocity(motor_2, vel_2);
encoder_2 = wb_robot_get_device('link_2_encoder')
wb_position_sensor_enable(encoder_2, TIME_STEP)

motor_3 = wb_robot_get_device('link_3_motor');
wb_motor_set_position(motor_3, inf)
wb_motor_set_velocity(motor_3, vel_3);
encoder_3 = wb_robot_get_device('link_3_encoder')
wb_position_sensor_enable(encoder_3, TIME_STEP)


while wb_robot_step(TIME_STEP) ~= -1
  % reading motor encoders
  pos_1 = wb_position_sensor_get_value(encoder_1);
  pos_2 = wb_position_sensor_get_value(encoder_2);
  pos_3 = wb_position_sensor_get_value(encoder_3);
  
  % reading distance sensors
  % robot_presence = wb_distance_sensor_get_value(robot_sensor);
  
  if STAGE == 0
    vel_1 = 0
    vel_2 = 0
    vel_3 = 0
    %wb_console_print(strcat('STAGE: ', num2str(STAGE), '   Robot has not arrived. robot_presence:  ', num2str(robot_presence)), WB_STDOUT); 
    % if robot_presence < 1000
    if 1 < 1000
      [theta_1,theta_2,theta_3] = find_IK(target_x,target_y,0.2,0.15)
      vel_1 = 0
      vel_2 = 0
      vel_3 = 0
      STAGE = 1
    else
      wb_console_print(strcat('STAGE: ', num2str(STAGE), '   Robot has not arrived. robot_presence:  ', num2str(robot_presence)), WB_STDOUT); 
    end  
  elseif STAGE == 1  
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
      STAGE = 2
    end
  elseif STAGE == 2
    presence = wb_connector_get_presence(jaw)
    wb_console_print(strcat('GRIPPER: ',num2str(presence)), WB_STDOUT);  
    wb_connector_lock(jaw)
    theta_1 = -theta_1;
    theta_2 = -theta_2;
    theta_3 = -theta_3;
    STAGE = 3
  elseif STAGE == 3
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
      STAGE = 4
    end
  elseif STAGE == 4
    wb_connector_unlock(jaw)
    STAGE = 5
  end

  wb_motor_set_velocity(motor_1, vel_1);
  wb_motor_set_velocity(motor_2, vel_2);
  wb_motor_set_velocity(motor_3, vel_3);  

end

% cleanup code goes here: write data to files, etc.
