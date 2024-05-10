
TIME_STEP = 64;

STAGE = 0;
SUBSTAGE = 0;
theta_1 = 0;
theta_2 = 0;
theta_3 = 0;
vel_1 = 0;
vel_2 = 0;
vel_3 = 0;
target_x = 0.25;
target_y = 0.1;  %0.15

% initializing connector
jaw = wb_robot_get_device('connector');
wb_connector_enable_presence(jaw, TIME_STEP)

% initializing link 1
motor_1 = wb_robot_get_device('link_1_motor');
wb_motor_set_position(motor_1, inf)
wb_motor_set_velocity(motor_1, vel_1);
encoder_1 = wb_robot_get_device('link_1_encoder')
wb_position_sensor_enable(encoder_1, TIME_STEP)

% initializing link 2
motor_2 = wb_robot_get_device('link_2_motor');
wb_motor_set_position(motor_2, inf)
wb_motor_set_velocity(motor_2, vel_2);
encoder_2 = wb_robot_get_device('link_2_encoder')
wb_position_sensor_enable(encoder_2, TIME_STEP)

% initializing link 3
motor_3 = wb_robot_get_device('link_3_motor');
wb_motor_set_position(motor_3, inf)
wb_motor_set_velocity(motor_3, vel_3);
encoder_3 = wb_robot_get_device('link_3_encoder')
wb_position_sensor_enable(encoder_3, TIME_STEP)


while wb_robot_step(TIME_STEP) ~= -1
  
  pos_1 = wb_position_sensor_get_value(encoder_1);
  pos_2 = wb_position_sensor_get_value(encoder_2);
  pos_3 = wb_position_sensor_get_value(encoder_3);
  
  if STAGE == 0
    % if robot_presence < 1000
    if 1 < 1000
      [theta_1,theta_2,theta_3] = get_IK(target_x,target_y,0.2,0.15);
      vel_1 = 0;
      vel_2 = 0;
      vel_3 = 0;
      STAGE = 1;
    end  
  elseif STAGE == 1  
    [vel_1,L1_con] = is_rotated(theta_1, pos_1);
    [vel_2,L2_con] = is_rotated(theta_2, pos_2);
    [vel_3,L3_con] = is_rotated(theta_3, pos_3);
    
    if L1_con * L2_con * L3_con == 1
      STAGE = 2;
    end
  elseif STAGE == 2
    presence = wb_connector_get_presence(jaw);
    wb_connector_lock(jaw)
    theta_1 = -theta_1;
    theta_2 = -theta_2;
    theta_3 = -theta_3;
    STAGE = 3;
  elseif STAGE == 3
    [vel_1,L1_con] = is_rotated(theta_1, pos_1);
    [vel_2,L2_con] = is_rotated(theta_2, pos_2);
    [vel_3,L3_con] = is_rotated(theta_3, pos_3);
    
    if L1_con * L2_con * L3_con == 1
      STAGE = 4;
    end
  elseif STAGE == 4
    wb_connector_unlock(jaw)
    STAGE = 5;
  end

  wb_motor_set_velocity(motor_1, vel_1);
  wb_motor_set_velocity(motor_2, vel_2);
  wb_motor_set_velocity(motor_3, vel_3);  

end

