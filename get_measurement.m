 
function z_measurement = get_measurement(pos,variance)
 
%   speed    = normrnd(movement(1), variance(1)*.25);
%   rotation = normrnd(movement(2), variance(2)*.25);
  
%   zx = normrnd(pos(1), variance(1)*0.25);
%   zy = normrnd(pos(2), variance(2)*0.25);
%   rotation = normrnd(pos(3), variance(3)*.25);
  
  delta = zeros(3,1);
  delta(1,1) = pos(1) + variance(1)* randn;
  delta(2,1) = pos(2) + variance(2)* randn;
  delta(3,1) = pos(3) + variance(3)* randn;

  z_measurement = delta;
  
%   newpos = [zx,zy];
  
%   speed    = normrnd(movement(1), variance(1)*.25);
%   rotation = normrnd(movement(2), variance(2)*.25);
% 
%   delta = zeros(3,1);
%   delta(1,1) = cos(pos(3)+rotation)*speed;
%   delta(2,1) = sin(pos(3)+rotation)*speed;
%   delta(3,1) = rotation;
% 
%   newpos = pos+delta;
end
