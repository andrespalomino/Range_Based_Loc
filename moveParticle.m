 % Compute how the robot should move from "pos" given the requested movement
% and some Gaussian random noise using the motion model. This
% method is used to move the simulated robot as well as each of the
% hypothetical particles.
function newpos = moveParticle(pos, movement, variance)
 
  speed    = normrnd(movement(1), variance(1)* 0.25);
  rotation = normrnd(movement(2), variance(2)* 0.25);

  delta = zeros(3,1);
  delta(1,1) = cos(pos(3)+rotation)*speed;
  delta(2,1) = sin(pos(3)+rotation)*speed;
  delta(3,1) = rotation;

  newpos = pos+delta;
end
