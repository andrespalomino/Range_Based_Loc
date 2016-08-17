%% clear everything
clear all
close all
clc

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PARAMETERS

number_of_robots = 4;

% The number of timesteps for the simulation
timesteps = 200;

% The maximum distance from which our sensor can sense a landmark or other
% robot
max_read_distance = 100;

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
  robot_u.pos_hist = [real_position];  
  robot_u.odometry = [];
  robot_u.odometry_hist = [];
  robot_u.pos_est = [];
  
% Create the robots and initialize them all to be in the same initial
% position. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ROBOTS W
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
a = -10;
b = 10;
for i = 1:number_of_robots
    rx = a + (b-a)*rand;
    ry = a + (b-a)*rand;
    rt = a + (b-a)*rand;
    robots(i).position = [rx; ry; rt];
    robots(i).pos_hist = robots(i).position;
end



% x_est = [];
% robots = [];
% % The initial starting position of the robot
% real_position_w1 = [3;      % x
%                  2;     % y
%                  pi/4];  % rotation
% 
% % The initial starting position of the robot
% real_position_w2 = [0;      % x
%                 -2;     % y
%                  -pi/4];  % rotation
%              
% % The initial starting position of the robot
% real_position_w3 = [-5;      % x
%                  5;     % y
%                  -pi/2];  % rotation
%              
%              
% % The initial starting position of the robot
% real_position_w4 = [0;      % x
%                 0;     % y
%                  0];  % rotation

% The movement command given to he robot at each timestep                 
movement_command2 = [0.1;     % Distance
                     -0.04];    % Rotation    
% lineal                 
movement_command2 = [0.1;     % Distance
                    0];    % Rotation                  
                
% robots(1).position = real_position_w1;
% robots(2).position = real_position_w2;
% robots(3).position = real_position_w3;
% robots(4).position = real_position_w4;
% robots(1).pos_hist = real_position_w1;
% robots(2).pos_hist = real_position_w2;
% robots(3).pos_hist = real_position_w3;
% robots(4).pos_hist = real_position_w4;

% Initialize position of each robot W
  for wIdx=1:number_of_robots
%     robots(wIdx).position = [0;0;0];  
%     robots(wIdx).pos_hist = []; 
    robots(wIdx).read_distance = [];
    robots(wIdx).read_angle  = [];
%     robots(wIdx).vector_x = [];
%     robots(wIdx).vector_y = [];
    robots(wIdx).odometry  = [];
    robots(wIdx).odometry_hist  = [];
    robots(wIdx).pose_est = [];
%     robots(wIdx).x_est = [];
    robots(wIdx).gamma = [];
    robots(wIdx).beta = [];
    robots(wIdx).dk = [];
    robots(wIdx).k_1 = [];    
    robots(wIdx).distk_1 = [];
    robots(wIdx).anglek_1 = [];
    robots(wIdx).distk = [];
    robots(wIdx).theta_wk_uk = [];
    robots(wIdx).epsi = [];
    robots(wIdx).theta_uk_1_wk =[];
    robots(wIdx).pk_1 = [];
    if wIdx == 1
        robots(wIdx).state = 1;
    else 
        robots(wIdx).state = 0;
    end
  end
  
%   Adjacency Matrix
  AM = ones(number_of_robots);
  AM(logical(eye(size(AM)))) = 0;
  
  
  next = 1;
  
  
%   INITIAL FOR THETA UK_1_WK and PK_1

figure(1)
clf;
hold on;
  for wIdx=1:number_of_robots
        
      real_landmark = robots(wIdx).position;
      for uIdx = 1: number_of_robots
              
              real_landmark2 = robots(uIdx).position;
              % Take a real (noisy) measurement from the robot to the landmark
              [z_real, G] = get_Distance(real_landmark, real_landmark2, [0;0]);
%               robots(wIdx).theta_uk_1_wk(uIdx) = z_real(2);
              
              aux_pk_1(:,uIdx) = robots(uIdx).position;
              % If the robot is close enough, then we can spot it
              if(z_real(1) < max_read_distance)                  
                  aux(uIdx) = z_real(1);
                  aux_ang(uIdx) = z_real(2);
              else
                  aux(uIdx) = 0;
                  aux_ang(uIdx) = 0;
              end
              
              % Real Distance GREEN
              line([real_landmark(1), real_landmark(1) + cos(z_real(2))*z_real(1)], ...
                   [real_landmark(2), real_landmark(2) + sin(z_real(2))*z_real(1)],'Color','c');

%               pause
      end
      
      robots(wIdx).theta_uk_1_wk = [robots(wIdx).theta_uk_1_wk,aux_ang'];     
      robots(wIdx).pk_1 = aux_pk_1;      
%       pk_1 =  robots(wIdx).pk_1
%       
      plot(robots(wIdx).pos_hist(1,:), robots(wIdx).pos_hist(2,:),'r');
      plot(robots(wIdx).position(1), robots(wIdx).position(2), 'ro');

  end
  
  title('Initial positions and distances');
  axis([-10, 10, -10, 10]);
  grid on
  
  
% Previous position  
k_1 = robots(wIdx).pk_1;

pause
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % SIMULATION
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  for round = 1:timesteps
      if round <= timesteps/4
          movement_command2 = [0.1;   % Distance
              -0.04];    % Rotation
          %             % Different control actions over time
      elseif round >= timesteps/4 && round < timesteps/2
          movement_command2 = [0.05;     % Distance
              0];    % Rotation
      elseif round >= timesteps/2 && round < (timesteps/4)*3
          movement_command2 = [0.06;     % Distance
              0.02];    % Rotation
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
          
          for uIdx = 1: number_of_robots
              
              real_landmark2 = robots(uIdx).position;
              % Take a real (noisy) measurement from the robot to the landmark
              [z_real, G] = get_Distance(real_landmark, real_landmark2, [0;0]);
              
              % If the robot is close enough, then we can spot it
              if(z_real(1) < max_read_distance)                  
                  aux(uIdx) = z_real(1);
                  aux_ang(uIdx) = z_real(2);
              else
                  aux(uIdx) = 0;
                  aux_ang(uIdx) = 0;
              end
              
          end

              robots(wIdx).read_distance = [robots(wIdx).read_distance,aux'];
              robots(wIdx).read_angle = [robots(wIdx).read_angle,aux_ang'];

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
 
      elseif round == count_round

           % Get Odometry real position and past position ROBOT U
           
          
          robot_u.odometry = get_Odometry(robot_u.position, robot_u.pos_hist(:,round-(inc-1)));
          robot_u.odometry_hist = [robot_u.odometry_hist,robot_u.odometry];
          
          robot_u.pos_est = [robot_u.pos_est,robot_u.position];
            
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% FIGURE TO PLOT
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%          


          % Get Odometry real position and past position ROBOT W
          for wIdx = 1:number_of_robots
              
              %----------------------------------------
              % 3: broadcast puk?1|uk, ?uk?1|uk, ?uk?1
              %----------------------------------------
              robots(wIdx).odometry =    get_Odometry( ...
                  (robots(wIdx).position - robots(wIdx).pos_hist(:,1)), (k_1(:,wIdx) - robots(wIdx).pos_hist(:,1)));
              
              % vector and Angle of distance traveled
              %               epsi_1 = robots(wIdx).epsi;
              luk = sqrt(robots(wIdx).odometry(1)^2 + robots(wIdx).odometry(2)^2);
              robots(wIdx).epsi = atan2(robots(wIdx).odometry(2),robots(wIdx).odometry(1));
              
              robots(wIdx).odometry_hist  = [robots(wIdx).odometry_hist, [robots(wIdx).odometry;robots(wIdx).epsi]];
                                
              for uIdx = 1: number_of_robots
         
                  %               Take a real (noisy) measurement (DISTANCE)
                  %               from the robot to the landmark
                  %----------------------------------------
                  % 4: receive pwk?1|wk, ?wk?1|wk, ?uk?1 for w ? Nuk
                  %----------------------------------------
                  % if state = mobile then
                  if (robots(wIdx).state == 1)
                      % Measurement Distance and angle K_1
                      real_landmark = k_1(:,uIdx)
                      [z_real, G] = get_Distance(k_1(:,wIdx), real_landmark, [0;0]);
                      robots(wIdx).distk_1  = z_real(1);
                      robots(wIdx).anglek_1 = z_real(2);
                      
                      real_landmark2 = robots(uIdx).position
                      % Measurement Distance and angle K
                      [z_real, G] = get_Distance(robots(wIdx).position, real_landmark2, [0;0]);
                      robots(wIdx).distk    = z_real(1);
                      robots(wIdx).anglek   = z_real(2);

                      % 6: ?uk ? ? ˆwk|uk through Eq. (4-5)
                      % 7: ? ˆwk|uk ? use Eq. (6-7) ?w ? Nuk
                      % 8: use previous state resolve flip in ?uk
                      
                      %
                      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                      % % RELATIVE POSITION AND ORIENTATION ESTIMATION TRILAT
                      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                      dk = robots(wIdx).distk
%                       robots(wIdx).dk = dk;
                      dk_1 = robots(wIdx).distk_1
                      %               dk_1 = robots(wIdx).distk_1;
                      %
                      %
                      robots(wIdx).beta = acos((luk^2 + dk^2 - dk_1^2)/(2*luk * dk));
                      
                      robots(wIdx).gamma = acos((dk^2 + dk_1^2 - luk^2)/(2*dk* dk_1));
                      %
                      % angle(Puk_1|uk)
                      %               alfa_uk = robots(wIdx).pos_hist(3,round-(inc-1));
                      %               alfa_uk = -robots(wIdx).odometry(3)+pi;
                      %               alfa_uk = robots(wIdx).epsi+pi;
                      %
                      alfa_uk = 0+pi;
                      % 2 posible solutions
                      theta_wk_uk = [alfa_uk + robots(wIdx).beta, alfa_uk - robots(wIdx).beta]
                      robots(wIdx).theta_wk_uk = theta_wk_uk;
                      
                      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                      % RELATIVE POSITION
                      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                      pos = dk * [cos(robots(wIdx).theta_wk_uk(1)) , sin(robots(wIdx).theta_wk_uk(1)); ...
                          cos(robots(wIdx).theta_wk_uk(2)) , sin(robots(wIdx).theta_wk_uk(2))]
                      robots(wIdx).pose_est_k = pos;
                      
                      
                      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                      %
                      %
                      %               % 2 posible solutions
                      %               %               theta_uk_wk = [theta_uk_wk_1 + robots(wIdx).gamma; theta_uk_wk_1 - robots(wIdx).gamma];
                      %
                      %               theta_uk_wk = [robots(wIdx).theta_uk_1_wk + robots(wIdx).gamma, robots(wIdx).theta_uk_1_wk - robots(wIdx).gamma];
                      %               robots(wIdx).theta_uk_wk = theta_uk_wk;
                      %               %               robots(wIdx).theta_uk_wk = theta_uk_wk;
                      %               %               t_uk_1_wk = robots(wIdx).theta_uk_1_wk*180/pi
                      %               %               robots(wIdx).anglek_1*180/pi
                      %               %               pause

                      %               % 8 posible solutions
                      %               phi_wk_uk = [theta_wk_uk(1) - theta_uk_wk(1) + pi; theta_wk_uk(1) - theta_uk_wk(2) + pi];
                      %               robots(wIdx).phi_wk_uk = phi_wk_uk;
                      %               p_uk_wk = phi_wk_uk*180/pi;
                      %
                      %               %               theta_uk_wk_1 = theta_uk_wk;
                      %               %               pos = dk * [cos(theta_wk_uk(2)) ; sin(theta_wk_uk(2))];
                      %               %               pose = [pos ; phi_uk_wk];
                      %               %
                      %               %               robots(wIdx).pose_est = [robots(wIdx).pose_est , pose];

                      %               %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                      % %               t_uk_wk = theta_uk_wk*180/pi
                      %
                      
                      %                   else
                      %                       theta_wk_uk =
                      
                      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                      % GRAFICAS DISTANCIA Y VECTORES ESTIMACION
                      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                      graficas1;
                      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

                     
%                   else
%                       
%                       for wwIdx = 1:number_of_robots
%                           for uIdx = 1:number_of_robots
%                               robots(wwIdx).pk_1(:,uIdx) = robots(uIdx).position;
%                               pk_1 = robots(wwIdx).pk_1(:,uIdx)
%                           end
%                       end
                      
                  end 
              end

              % Actualizar Pk_1 y theta_uk_1_wk    %% ESTIMATION 
              k_1(:,wIdx) = robots(wIdx).position;
%               robots(1).pk_1(:,1) = robots(wIdx).position;
%               robots(2).pk_1(:,1) = robots(wIdx).position;
%               robots(3).pk_1(:,1) = robots(wIdx).position;
              % robots(wIdx).theta_uk_1_wk = robots(wIdx).anglek;
                            
          end

          % MOSTRAR VALORES PROGRAMA
          %------------------------------------------------------
          %           u_pos = robot_u.position
          %           w_pos = robots(wIdx).position
          %           u_od = robot_u.odometry
          %           w_od = robots_odometry
          %           d_j = c
          %           d_k = d
          %           x_est =[x_est;x]
          %           posss = robots(wIdx).pose_est
          %           angle = robots(wIdx).read_angle(round)
          %                       dk
          %                       dk_1
          %             luk
          %           gamma = robots(wIdx).gamma*180/pi
          %           beta = robots(wIdx).beta*180/pi
          
          %
          %             theta_uk_wk_1
          %             phi_uk_wk
          %           alfa = alfa_uk*180/pi
          %           theta_wk_uk*180/pi
          %           epsi = robots(wIdx).epsi*180/pi
          %           t_uk_wk = theta_uk_wk*180/pi
          %             theta_uk_wk*180/pi
          %             phi_uk_wk
          %             robots(wIdx).pose_est
          %             alfa_uk3
          %------------------------------------------------------
          %------------------------------------------------------
          % state ? motion-scheduler
          %------------------------------------------------------
          for wIdx = 1:number_of_robots
              robots(wIdx).state = 0;
          end
          
          next = next +1
          if next == number_of_robots + 1;
              next = 1;
          end
          robots(next).state = 1;
          
          pause
          
          
          % aumentar count_round para comparar          
          count_round = count_round + inc;

      end
      
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % PLOTTING
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      figure(10)
      clf;
      hold on;
     
      
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % PLOT REAL ROBOT
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % History and circle U
%       plot(robot_u.pos_hist(1,:), robot_u.pos_hist(2,:), 'b');
%       plot(robot_u.position(1), robot_u.position(2), 'bo');
      
      
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % PLOT ROBOTS & DISTANCE
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
      % Show the sensor measurement as an arrow
      %       last = size(robot_u.pos_est);
      %       last_ph =size(robot_u.pos_hist(1,:));
      for wIdx=1: number_of_robots
          
          for uIdx = 1: number_of_robots
          % History and circle W
          plot(robots(wIdx).pos_hist(1,:), robots(wIdx).pos_hist(2,:),'r');
          plot(robots(wIdx).position(1), robots(wIdx).position(2), 'ro');
          
          % Real Distance GREEN
          line([robots(wIdx).position(1), robots(wIdx).position(1) + cos(robots(wIdx).read_angle(uIdx,round))*robots(wIdx).read_distance(uIdx,round)], ...
              [robots(wIdx).position(2), robots(wIdx).position(2) + sin(robots(wIdx).read_angle(uIdx,round))*robots(wIdx).read_distance(uIdx,round)],'Color','g');

%           line([robot_u.position(1)+robots(wIdx).read_distance(round)*cos(robots(wIdx).read_angle(round)), robot_u.position(1)+robots(wIdx).read_distance(round)*cos(robots(wIdx).read_angle(round))+2*cos(robots(wIdx).position(3))], ...
%                [robot_u.position(2)+robots(wIdx).read_distance(round)*sin(robots(wIdx).read_angle(round)), robot_u.position(2)+robots(wIdx).read_distance(round)*sin(robots(wIdx).read_angle(round))+2*sin(robots(wIdx).position(3))],'Color','b');
%           
          title('Movement according to motion-controller');
          axis([-10, 10, -10, 10]);
          
%                 pause;
          grid on
          end
                   
      end
      
      
      
      
      %     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      %     % Move the actual robot and other robots
      %     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      robot_u.position = moveParticle(robot_u.position, movement_command, movement_variance);
      robot_u.pos_hist = [robot_u.pos_hist, robot_u.position];
      
      % Move the W robots
      for wIdx = 1:number_of_robots    
          % if state = mobile
          if robots(wIdx).state == 1
              % Posicion x,y, rotacion de los robots W
              robots(wIdx).position = moveParticle( ...
                  robots(wIdx).position, movement_command2, movement_variance);
              % Rotacion de las particulas
              %         robots(wIdx).position(3) = robot_u.position(3);
              robots(wIdx).pos_hist = [robots(wIdx).pos_hist ,robots(wIdx).position];                        
          end
      end
      
      
      pause(0.01);
  end
  
  

%% PLOTS

% Robot w1
figure(2)
title('Estimación robot w1')
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
title('Estimación robot w2')
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
title('Estimación robot w3')
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



