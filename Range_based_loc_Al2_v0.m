%% clear everything
clear all
close all
clc

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PARAMETERS

number_of_robots = 3;

% The number of timesteps for the simulation
timesteps = 100;

% The maximum distance from which our sensor can sense a landmark or other
% robot
max_read_distance = 2;

% round to take odometry and plot
inc = 10;
count_round = 10;
count_round_plot = count_round;

% Take odometry every 3 rounds 
flag_round = 0;

% Rounds to apply algorithm 
j = 1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ROBOT U
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% The initial starting position of the robot
real_position = [0;      % x
                0;     % y
                 0];  % rotation
                                     
% The movement command given to he robot at each timestep                 
movement_command = [0;     % Distance
                    0];    % Rotation      
                
% The Gaussian variance of the movement commands
movement_variance = [0;   % Distance
                     0]; % Rotation
Q = [movement_variance(1), 0.0;
     0.0, movement_variance(2)];
 
% The Gaussian variance of our sensor readings
measurement_variance = [0.2;   % Distance_x
                        0.2;   % Distance_y
                        0.15]; % Rotation
                    
R = [measurement_variance(1), 0.0, 0.0;
     0.0, measurement_variance(2), 0.0;
     0.0, 0.0, measurement_variance(3)];
                
             
  robot_u.position = real_position;
  robot_u.pos_hist = [];  
  robot_u.odometry = [];
  robot_u.odometry_hist = [];
  robot_u.pos_est = [];
  
% Create the robots and initialize them all to be in the same initial
% position. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ROBOTS W
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% x_est = [];
robots = [];
% The initial starting position of the robot
real_position_w1 = [2;      % x
                 -2;     % y
                 pi/3];  % rotation

% The initial starting position of the robot
real_position_w2 = [0;      % x
                -2;     % y
                 -pi/4];  % rotation
             
% The initial starting position of the robot
real_position_w3 = [-5;      % x
                 5;     % y
                 -pi/2];  % rotation

% The movement command given to he robot at each timestep                 
movement_command2 = [0.1;     % Distance
                     -0.04];    % Rotation    
% lineal                 
movement_command2 = [0.1;     % Distance
                    0];    % Rotation                  
                
robots(1).position = real_position_w1;
robots(2).position = real_position_w2;
robots(3).position = real_position_w3;
robots(1).pos_hist = real_position_w1;
robots(2).pos_hist = real_position_w2;
robots(3).pos_hist = real_position_w3;

% Initialize position of each robot W
  for wIdx=1:number_of_robots
%     robots(wIdx).position = [0;0;0];  
    robots(wIdx).pos_hist = []; 
    robots(wIdx).read_distance = [];
    robots(wIdx).read_angle  = [];
    robots(wIdx).vector_x = [];
    robots(wIdx).vector_y = [];
    robots(wIdx).odometry  = [];
    robots(wIdx).odometry_hist  = [];
    robots(wIdx).pose_est = [];
    robots(wIdx).dist_j = [];
    robots(wIdx).dist_k = [];
    robots(wIdx).x_est = [];
    robots(wIdx).gamma = [];
    robots(wIdx).beta = [];
  end
  
  theta_uk_wk_1 = [0,0];

  
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % SIMULATION
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  
  for round = 1:timesteps
%             if round <= timesteps/4
%                 movement_command2 = [0.1;   % Distance
%                     0.02];    % Rotation
% %             % Different control actions over time
% %             if round >= timesteps/4 && round < timesteps/2
% %                 movement_command2 = [0.02;     % Distance
% %                    -0.02];    % Rotation
%       %       elseif round >= timesteps/2 && round < (timesteps/4)*3
%       %           movement_command2 = [0.06;     % Distance
%       %               0];    % Rotation
%             end
%       
      %     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      %     % Move the actual robot
      robot_u.position = moveParticle(robot_u.position, movement_command, movement_variance);
      robot_u.pos_hist = [robot_u.pos_hist, robot_u.position];

%       pause
      
      % Move the W robots
      for wIdx = 1:number_of_robots
          % Posicion x,y, rotacion de los robots W
          robots(wIdx).position = moveParticle( ...
              robots(wIdx).position, movement_command2, movement_variance);
          % Rotacion de las particulas
          %         robots(wIdx).position(3) = robot_u.position(3);
          robots(wIdx).pos_hist = [robots(wIdx).pos_hist ,robots(wIdx).position];
      end
      
      
      
      %   For every neighboring robot w or robots
      % para cada robot w en el alcance de u Get Distance
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      %       Get Distance
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      for wIdx = 1: number_of_robots
          real_landmark = robots(wIdx).position;
          
          % Take a real (noisy) measurement from the robot to the landmark
          [z_real, G] = get_Distance(robot_u.position, real_landmark, [0;0]);
          robots(wIdx).read_distance = [robots(wIdx).read_distance,z_real(1)];
          robots(wIdx).read_angle    = [robots(wIdx).read_angle,z_real(2)];
          robots(wIdx).vector_x = [robots(wIdx).vector_x,z_real(3)];
          robots(wIdx).vector_y = [robots(wIdx).vector_y,z_real(4)];
          %
          %     k = 2;
          %     if j <= k
          
      end %for number_robots
      
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      %       Get Odometry WHEN round = count_round
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
      %     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      %         % Get Odometry real position and past position
      %         %     Robots are capable of using odometry to estimate their pose change between
      %         % rounds in their own local coordinate system.
      %         %     It is assumed
      %         % that odometry estimates are reliable over intervals of two or three rounds
      %         % (i.e. i >= j ?3), but suffer from drift over longer time intervals.
      %       %% Odometry of u
%       robot_u.pos_hist
%       robots(wIdx).pos_hist
%       robots(wIdx).read_distance
% theta_uk_wk_1
%       round
%       pause
      if round == 1
%           robot_u.odometry = 0;
%           robot_u.odometry_hist = [0;0;0];
%           %               robot_u.odometry2 = 0;
% 
          for wIdx = 1:number_of_robots
%               robots(wIdx).odometry = [0;0;0];
%               robots(wIdx).odometry_hist = [0;0;0];
%               robots(wIdx).dist_k = 0;
%               robots(wIdx).dist_j = 0;
          end
      elseif round == count_round
          %       else
          %       else
          
          robot_u.odometry = get_Odometry(robot_u.position, robot_u.pos_hist(:,round-(inc-1)));
          robot_u.odometry_hist = [robot_u.odometry_hist,robot_u.odometry];
          
          robot_u.pos_est = [robot_u.pos_est,robot_u.position];
          
          %           pause
          for wIdx = 1:number_of_robots
              %               robots(wIdx).odometry = get_Odometry( ...
              %                   robots(wIdx).position, robots(wIdx).pos_hist(:,round-(inc-1)));
              
              robots(wIdx).odometry =    get_Odometry( ...
                  (robots(wIdx).position - robots(wIdx).pos_hist(:,1)), (robots(wIdx).pos_hist(:,round-(inc-1))-robots(wIdx).pos_hist(:,1)));
             
               robots(wIdx).odometry_hist  = [ robots(wIdx).odometry_hist,  robots(wIdx).odometry];
              %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
              % RELATIVE POSITION AND ORIENTATION ESTIMATION
              %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
              dk = robots(wIdx).read_distance(round);
              dk_1 = robots(wIdx).read_distance(:,round-(inc-1));
              luk = sqrt(robots(wIdx).odometry(1)^2 + robots(wIdx).odometry(2)^2);
              robots(wIdx).gamma = acos((dk^2 + dk_1^2 - luk^2)/(2*dk* dk_1));
              
              robots(wIdx).beta = acos((luk^2 + dk^2 - dk_1^2)/(2*luk * dk));
              
%               angk = robots(wIdx).odometry(3);
              last = size(robots(wIdx).odometry_hist);
%               angk_1 = robots(wIdx).odometry_hist(3,last(2)-1);

              % angle(Puk_1|uk)
              alfa_uk = robots(wIdx).pos_hist(3,round-(inc-1));
              % 2 posible solutions
              theta_wk_uk = [alfa_uk + robots(wIdx).beta, alfa_uk - robots(wIdx).beta];
              % 2 posible solutions
%               theta_uk_wk = [theta_uk_wk_1 + robots(wIdx).gamma; theta_uk_wk_1 - robots(wIdx).gamma];
              theta_uk_wk = [theta_uk_wk_1(1) + robots(wIdx).gamma; theta_uk_wk_1(1) - robots(wIdx).gamma];
              % 8 posible solutions
%               phi_uk_wk = [theta_wk_uk(1) - theta_uk_wk(1,:) + pi; theta_wk_uk(2) - theta_uk_wk(1,:) + pi;...
%                   theta_wk_uk(1) - theta_uk_wk(2,:) + pi; theta_wk_uk(2) - theta_uk_wk(2,:) + pi];
              phi_uk_wk = theta_wk_uk(1) - theta_uk_wk(1) + pi;
              
              theta_uk_wk_1 = theta_uk_wk;
              pos = dk * [cos(theta_wk_uk(2)) ; sin(theta_wk_uk(2))];
              pose = [pos ; phi_uk_wk];
              
              robots(wIdx).pose_est = [robots(wIdx).pose_est , pose];
%               

%               theta = robots(wIdx).gamma - 
              
               
          end
          % MOSTRAR VALORES PROGRAMA
%           u_pos = robot_u.position
%           w_pos = robots(wIdx).position
%           u_od = robot_u.odometry
%           w_od = robots_odometry
%           d_j = c
%           d_k = d
%           x_est =[x_est;x]
%           posss = robots(wIdx).pose_est
%           angle = robots(wIdx).read_angle(round)
% 
%             luk
%             gamma = robots(wIdx).gamma
%             beta = robots(wIdx).beta
%             theta_wk_uk
%             theta_uk_wk
%             theta_uk_wk_1
%             phi_uk_wk
%             alfa_uk*180/pi
%             phi_uk_wk
%             robots(wIdx).pose_est
%             alfa_uk
            
%             pause

          % aumentar count_round para comparar 
          count_round = count_round + inc;
      end                      
      
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % PLOTTING
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      figure(10)
      clf;
      hold on;
      % Plot the particles in green
      %     particles_pos = [particles.position];
      
      %     plot(particles_pos(1,:), particles_pos(2,:), 'g.');
      
      
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % PLOT REAL ROBOT
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
      plot(robot_u.pos_hist(1,:), robot_u.pos_hist(2,:), 'b');
      plot(robot_u.position(1), robot_u.position(2), 'bo');
      
      % Plot the data of measurements
      %     plot(z_history(1,:), z_history(2,:), '.m');
      
      % Plot the state estimated
      %     plot(x_est_out(1,:), x_est_out(2,:), '-.r','LineWidth',1.5);
      
      % Plot the real robot
      
      %     plot(real_position(1), real_position(2), 'o',real_position_w1(1), real_position_w1(2), 'o');
      
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % PLOT ROBOTS & DISTANCE
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
      % Show the sensor measurement as an arrow
      last = size(robot_u.pos_est);
      
      for wIdx=1: number_of_robots
          %           plot(pos_history(1,:), pos_history(2,:), 'b',robots(wIdx).pos_hist(1,:), robots(wIdx).pos_hist(2,:),'r');
          plot(robots(wIdx).pos_hist(1,:), robots(wIdx).pos_hist(2,:),'r');
          %           plot(real_position(1), real_position(2), 'bo',robots(wIdx).position(1), robots(wIdx).position(2), 'ro');
          plot(robots(wIdx).position(1), robots(wIdx).position(2), 'ro');
          
          %     real_landmark = robots(:, wIdx);
          %     if(read_distance(wIdx) < max_read_distance)
          line([robot_u.position(1), robot_u.position(1)+cos(robots(wIdx).read_angle(round))*robots(wIdx).read_distance(round)], ...
              [robot_u.position(2), robot_u.position(2)+sin(robots(wIdx).read_angle(round))*robots(wIdx).read_distance(round)],'Color','g');
          %     end
          
          %           line([robot_u.position(1),robot_u.position(1)-robots(wIdx).pose_est(1,round)], ...
          %               [robot_u.position(2),robot_u.position(2)-robots(wIdx).pose_est(2,round)],'Color','g');
          %
          %           line([robot_u.position(1),robot_u.position(1)-sin(robots(wIdx).pose_est(3,round))*robots(wIdx).read_distance(round)], ...
          %               [robot_u.position(2),robot_u.position(2)-cos(robots(wIdx).pose_est(3,round))*robots(wIdx).read_distance(round)],'Color','y');
          
          if wIdx == 1
              color = 'y';
          elseif wIdx == 2
              color = 'm';
          elseif wIdx == 3
              color = 'c';
          end
          
          if isempty(robot_u.pos_est) ~= 1
              
              hold on
              % Linea con la posici�n relativa estimada
              line([robot_u.pos_est(1,last(2)), (robot_u.pos_est(1,last(2)) + robots(wIdx).pose_est(1,last(2)))], ...
                  [robot_u.pos_est(2,last(2)),(robot_u.pos_est(2,last(2)) + robots(wIdx).pose_est(2,last(2)))],'Color',color);
              
              % Linea con el �nglo relativo estimado
%               line([robot_u.pos_est(1,last(2)), (robot_u.pos_est(1,last(2)) + cos(robots(wIdx).x_est(last(2),2))* robots(wIdx).dist_k(last(2)))], ...
%                   [robot_u.pos_est(2,last(2)),(robot_u.pos_est(1,last(2)) + sin(robots(wIdx).x_est(last(2),2))* robots(wIdx).dist_k(last(2)))],'Color','y');
              
%               line([robot_u.pos_est(1,last(2)), (robot_u.pos_est(1,last(2)) + cos(robots(wIdx).x_est(last(2),1)+robots(wIdx).x_est(last(2),2))* robots(wIdx).dist_k(last(2)))], ...
%                   [robot_u.pos_est(2,last(2)),(robot_u.pos_est(1,last(2)) + sin(robots(wIdx).x_est(last(2),1)+robots(wIdx).x_est(last(2),2))* robots(wIdx).dist_k(last(2)))],'Color','c');              
    
          end
          
      end


      
      %       if round == count_round_plot
      
      %           last = size(robots(wIdx).pose_est);
      %           line([real_position(1),real_position(1)-robots(1).pose_est(1,round)], ...
      %               [real_position(2),real_position(2)-robots(1).pose_est(2,round)],'Color','g');
      
      %           line([real_position(1),real_position(1)-robots(1).pose_est(1,last(2))], ...
      %               [real_position(2),real_position(2)-robots(1).pose_est(2,last(2))],'Color','g');
      %           line([real_position(1),real_position(1)-robots(2).pose_est(1,last(2))], ...
      %               [real_position(2),real_position(2)-robots(2).pose_est(2,last(2))],'Color','m');
      %           line([real_position(1),real_position(1)-robots(3).pose_est(1,last(2))], ...
      %               [real_position(2),real_position(2)-robots(3).pose_est(2,last(2))],'Color','k');
      
      
      %           count_round_plot = count_round_plot + inc
      %           pause
      %       end
      
      
%       legend('Trayectoria u','Robot u','Trayectoria w','w','Dist Medida Sensor','Dist Estimada (y=w1)(m=w2)(c=w3)');  
      axis([-10, 10, -10, 10]);
      pause(0.01);
%       pause;
      grid on
      
      
  end
  
  s = size(robots(wIdx).pose_est);
% for wIdx=1: number_of_robots
%   for j = 1: s(2)
%       plot (robots(1).pose_est(1,:), robots(1).pose_est(2,:));
%       plot (robots(2).pose_est(1,:), robots(2).pose_est(2,:));
%       plot (robots(3).pose_est(1,:), robots(3).pose_est(2,:));
%       
%       plot(robot_u.pos_est(1,:) - robots(1).pose_est(1,:), robot_u.pos_est(2,:) + robots(1).pose_est(2,:));
%       plot(robot_u.pos_est(1,:) - robots(2).pose_est(1,:), robot_u.pos_est(2,:) + robots(2).pose_est(2,:));
%       plot(robot_u.pos_est(1,:) - robots(3).pose_est(1,:), robot_u.pos_est(2,:) + robots(3).pose_est(2,:));
%   end
% end
%   
%% PLOTS

% Robot w1
figure(2)
title('Estimaci�n robot w1')
hold on
plot(robot_u.pos_hist(1,:), robot_u.pos_hist(2,:), 'b');
plot(robot_u.position(1), robot_u.position(2), 'bo');

plot(robots(1).pos_hist(1,:), robots(1).pos_hist(2,:),'r');
plot(robots(1).position(1), robots(1).position(2), 'ro');

for i =1 : 10
    %                 line([robot_u.pos_est(1,i), (robot_u.pos_est(1,i) + robots(1).pose_est(1,i))], ...
    %                     [robot_u.pos_est(2,i),(robot_u.pos_est(2,i) - robots(1).pose_est(2,i))],'Color','y');
      
    line([robot_u.pos_est(1,i), (robot_u.pos_est(1,i) + robots(1).pose_est(1,i))], ...
        [robot_u.pos_est(2,i),(robot_u.pos_est(2,i) + robots(1).pose_est(2,i))],'Color','y');
    
%     line([robot_u.pos_est(1,i), (robot_u.pos_est(1,i) + cos(robots(1).x_est(i,2))* robots(1).dist_k(i))], ...
%         [robot_u.pos_est(2,i),(robot_u.pos_est(1,i) + sin(robots(1).x_est(i,2))* robots(1).dist_k(i))],'Color','k');
    
%     line([robot_u.pos_est(1,i), (robot_u.pos_est(1,i) + cos(robots(1).x_est(i,1)+robots(1).x_est(i,2))* robots(1).dist_k(i))], ...
%         [robot_u.pos_est(2,i),(robot_u.pos_est(1,i) + sin(robots(1).x_est(i,1)+robots(1).x_est(i,2))* robots(1).dist_k(i))],'Color','c');
    
end
legend('Trayectoria u','Robot u','Trayectoria w','w','Dist Estimada con pos relativa');

%%
% Robot w2
figure(3)
title('Estimaci�n robot w2')
hold on
plot(robot_u.pos_hist(1,:), robot_u.pos_hist(2,:), 'b');
plot(robot_u.position(1), robot_u.position(2), 'bo');

plot(robots(2).pos_hist(1,:), robots(2).pos_hist(2,:),'r');
plot(robots(2).position(1), robots(2).position(2), 'ro');

for i =1 : 10
    line([robot_u.pos_est(1,i), (robot_u.pos_est(1,i) + robots(2).pose_est(1,i))], ...
        [robot_u.pos_est(2,i),(robot_u.pos_est(2,i) + robots(2).pose_est(2,i))],'Color','m');
    
%     line([robot_u.pos_est(1,i), (robot_u.pos_est(1,i) + cos(robots(2).x_est(i,2))* robots(2).dist_k(i))], ...
%         [robot_u.pos_est(2,i),(robot_u.pos_est(1,i) + sin(robots(2).x_est(i,2))* robots(2).dist_k(i))],'Color','k');
    
%     line([robot_u.pos_est(1,i), (robot_u.pos_est(1,i) + cos(robots(2).x_est(i,1)+robots(2).x_est(i,2))* robots(2).dist_k(i))], ...
%         [robot_u.pos_est(2,i),(robot_u.pos_est(1,i) + sin(robots(2).x_est(i,1)+robots(2).x_est(i,2))* robots(2).dist_k(i))],'Color','c');
end
  
legend('Trayectoria u','Robot u','Trayectoria w','w','Dist Estimada con pos relativa');

%%
% Robot w3
figure(4)
title('Estimaci�n robot w3')
hold on
plot(robot_u.pos_hist(1,:), robot_u.pos_hist(2,:), 'b');
plot(robot_u.position(1), robot_u.position(2), 'bo');

plot(robots(3).pos_hist(1,:), robots(3).pos_hist(2,:),'r');
plot(robots(3).position(1), robots(3).position(2), 'ro');
 
for i =1 : 10
    line([robot_u.pos_est(1,i), (robot_u.pos_est(1,i) + robots(3).pose_est(1,i))], ...
        [robot_u.pos_est(2,i),(robot_u.pos_est(2,i) + robots(3).pose_est(2,i))],'Color','c');
%     
%     line([robot_u.pos_est(1,i), (robot_u.pos_est(1,i) + cos(robots(3).x_est(i,2))* robots(3).dist_k(i))], ...
%         [robot_u.pos_est(2,i),(robot_u.pos_est(1,i) + sin(robots(3).x_est(i,2))* robots(3).dist_k(i))],'Color','y');
    
%     line([robot_u.pos_est(1,i), (robot_u.pos_est(1,i) + cos(robots(3).x_est(i,1)+robots(2).x_est(i,2))* robots(3).dist_k(i))], ...
%         [robot_u.pos_est(2,i),(robot_u.pos_est(1,i) + sin(robots(3).x_est(i,1)+robots(2).x_est(i,2))* robots(3).dist_k(i))],'Color','c');
end
  
legend('Trayectoria u','Robot u','Trayectoria w','w','Dist Estimada con pos relativa');



