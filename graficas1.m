% GRAFICAS

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PLOT ROBOTS & DISTANCE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(100+uIdx)
clf
%     subplot(2,number_of_robots,uIdx)
hold on;

%       % Show the sensor measurement as an arrow
% History and circle W
plot(robots(wIdx).pos_hist(1,:), robots(wIdx).pos_hist(2,:),'r');
plot(robots(wIdx).position(1), robots(wIdx).position(2), 'ro');

plot(robots(uIdx).pos_hist(1,:), robots(uIdx).pos_hist(2,:),'r');

% Real Distance K
line([robots(wIdx).position(1), robots(wIdx).position(1) + cos(robots(wIdx).anglek)*robots(wIdx).distk], ...
    [robots(wIdx).position(2), robots(wIdx).position(2) + sin(robots(wIdx).anglek)*robots(wIdx).distk],'Color','g');

% Each robot circle MAGENTA K_1
%                       plot(robots(wIdx).pk_1(1,uIdx),robots(wIdx).pk_1(2,uIdx),'mo')
plot(k_1(1,uIdx),k_1(2,uIdx),'mo')

% Real Distance K_1
line([k_1(1,wIdx), k_1(1,wIdx) + robots(wIdx).distk_1*cos(robots(wIdx).anglek_1)], ...
    [k_1(2,wIdx), k_1(2,wIdx) + robots(wIdx).distk_1*sin(robots(wIdx).anglek_1)],'Color','m');

% uk_wk
%               line([robot_u.position(1), robot_u.position(1) + 8*cos(theta_uk_wk(1))], ...
%                   [robot_u.position(2), robot_u.position(2) + 8*sin(theta_uk_wk(1))],'Color','c');
%               line([robot_u.position(1), robot_u.position(1) + 8*cos(theta_uk_wk(2))], ...
%                   [robot_u.position(2), robot_u.position(2) + 8*sin(theta_uk_wk(2))],'Color','y');

%               % wk_uk
%               line([robots(wIdx).position(1), robots(wIdx).position(1) + robots(wIdx).distk*cos(robots(wIdx).anglek) + 3*cos(phi_wk_uk(1))], ...
%                   [robots(wIdx).position(2), robots(wIdx).position(2) + robots(wIdx).distk*sin(robots(wIdx).anglek)+ 3*cos(phi_wk_uk(1))],'Color','k');
%               line([robots(wIdx).position(1), robots(wIdx).position(1) + robots(wIdx).distk*cos(robots(wIdx).anglek) + 3*cos(phi_wk_uk(2))], ...
%                   [robots(wIdx).position(2), robots(wIdx).position(2) + robots(wIdx).distk*sin(robots(wIdx).anglek)+ 3*cos(phi_wk_uk(2))],'Color','g');

axis([-10, 10, -10, 10]);
title(['Robot ',num2str(wIdx),' Distance and Travel to Robot ', num2str(uIdx)])
grid on

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PLOT ESTIMATED POSITION W NEIGHBOUR ROBOTS EACH REFERENCE FRAME
%%%%%%%%%%%%%%%%%%%%

figure(200+uIdx)
clf
%     subplot(2,number_of_robots,uIdx)
hold on;
plot(0, 0, 'ro');
% plot(robots(wIdx).pose_est_k(1,1),robots(wIdx).pose_est_k(1,2),'b*');
% plot(robots(wIdx).pose_est_k(2,1),robots(wIdx).pose_est_k(2,2),'b*');
r = 2;
% Show Vector of Real Initial Orientation of Robots
M = [r*cos(robots(wIdx).pos_hist(3,1)); r*sin(robots(wIdx).pos_hist(3,1))];
plotv(M,'-')
%                       if isempty(robots(wIdx).theta_wk_uk) ~= 1
M = [r*cos(robots(wIdx).position(3)), robots(wIdx).distk*cos(robots(wIdx).theta_wk_uk(1)+robots(wIdx).epsi), robots(wIdx).distk*cos(robots(wIdx).theta_wk_uk(2)+robots(wIdx).epsi);...
    r*sin(robots(wIdx).position(3)), robots(wIdx).distk*sin(robots(wIdx).theta_wk_uk(1)+robots(wIdx).epsi), robots(wIdx).distk*sin(robots(wIdx).theta_wk_uk(2)+robots(wIdx).epsi)];
plotv(M,'-')

axis([-10, 10, -10, 10]);
title(['Robot ',num2str(wIdx),' Vectors to Robot ', num2str(uIdx)])
grid on;


%%
%% SUBPLOT
% 
% figure(60+wIdx)
% subplot(2,number_of_robots,uIdx)
% hold on;
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % PLOT ROBOTS & DISTANCE
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %
% %       % Show the sensor measurement as an arrow
% % History and circle W
% plot(robots(wIdx).pos_hist(1,:), robots(wIdx).pos_hist(2,:),'r');
% plot(robots(wIdx).position(1), robots(wIdx).position(2), 'ro');
% 
% % Real Distance K
% line([robots(wIdx).position(1), robots(wIdx).position(1) + cos(robots(wIdx).anglek)*robots(wIdx).distk], ...
%     [robots(wIdx).position(2), robots(wIdx).position(2) + sin(robots(wIdx).anglek)*robots(wIdx).distk],'Color','g');
% 
% % Each robot circle MAGENTA K_1
% %                       plot(robots(wIdx).pk_1(1,uIdx),robots(wIdx).pk_1(2,uIdx),'mo')
% plot(k_1(1,uIdx),k_1(2,uIdx),'mo')
% 
% % Real Distance K_1
% line([k_1(1,wIdx), k_1(1,wIdx) + robots(wIdx).distk_1*cos(robots(wIdx).anglek_1)], ...
%     [k_1(2,wIdx), k_1(2,wIdx) + robots(wIdx).distk_1*sin(robots(wIdx).anglek_1)],'Color','m');
% 
% % uk_wk
% %               line([robot_u.position(1), robot_u.position(1) + 8*cos(theta_uk_wk(1))], ...
% %                   [robot_u.position(2), robot_u.position(2) + 8*sin(theta_uk_wk(1))],'Color','c');
% %               line([robot_u.position(1), robot_u.position(1) + 8*cos(theta_uk_wk(2))], ...
% %                   [robot_u.position(2), robot_u.position(2) + 8*sin(theta_uk_wk(2))],'Color','y');
% 
% %               % wk_uk
% %               line([robots(wIdx).position(1), robots(wIdx).position(1) + robots(wIdx).distk*cos(robots(wIdx).anglek) + 3*cos(phi_wk_uk(1))], ...
% %                   [robots(wIdx).position(2), robots(wIdx).position(2) + robots(wIdx).distk*sin(robots(wIdx).anglek)+ 3*cos(phi_wk_uk(1))],'Color','k');
% %               line([robots(wIdx).position(1), robots(wIdx).position(1) + robots(wIdx).distk*cos(robots(wIdx).anglek) + 3*cos(phi_wk_uk(2))], ...
% %                   [robots(wIdx).position(2), robots(wIdx).position(2) + robots(wIdx).distk*sin(robots(wIdx).anglek)+ 3*cos(phi_wk_uk(2))],'Color','g');
% 
% axis([-10, 10, -10, 10]);
% grid on
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % PLOT ESTIMATED POSITION W NEIGHBOUR ROBOTS EACH REFERENCE FRAME
% %%%%%%%%%%%%%%%%%%%%
% subplot(2,number_of_robots,uIdx+number_of_robots)
% %                       figure(30+uIdx)
% 
% hold on;
% 
% 
% plot(0, 0, 'ro');
% % plot(robots(wIdx).pose_est_k(1,1),robots(wIdx).pose_est_k(1,2),'b*');
% % plot(robots(wIdx).pose_est_k(2,1),robots(wIdx).pose_est_k(2,2),'b*');
% r = 2;
% % Show Vector of Real Initial Orientation of Robots
% M = [r*cos(robots(wIdx).pos_hist(3,1)); r*sin(robots(wIdx).pos_hist(3,1))];
% plotv(M,'-')
% %                       if isempty(robots(wIdx).theta_wk_uk) ~= 1
% M = [r*cos(robots(wIdx).position(3)), robots(wIdx).distk*cos(robots(wIdx).theta_wk_uk(1)+robots(wIdx).epsi), robots(wIdx).distk*cos(robots(wIdx).theta_wk_uk(2)+robots(wIdx).epsi);...
%     r*sin(robots(wIdx).position(3)), robots(wIdx).distk*sin(robots(wIdx).theta_wk_uk(1)+robots(wIdx).epsi), robots(wIdx).distk*sin(robots(wIdx).theta_wk_uk(2)+robots(wIdx).epsi)];
% plotv(M,'-')
% %                       end
% 
% axis([-10, 10, -10, 10]);
% %               title ('Robot', wIdx);
% title(['Robot ',num2str(uIdx),' Vectors'])
% grid on;
%                       