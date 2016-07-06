function y = param(x,a,b,c,d)


% od_a = a;
% od_b = b;
% 
% dist_j = c;
% dist_k = d;
% 
% y = norm(-od_a + [cos(x(1)) -sin(x(1));sin(x(1)) cos(x(1))]*od_b...
%     + dist_k*[cos(x(2));sin(x(2))]) - dist_j;

od_a = a;
od_b = b;

dist_j = c;
dist_k = d;

y = norm(-od_a + [cos(x(1)) -sin(x(1));sin(x(1)) cos(x(1))]*od_b...
    + dist_k*[cos(x(2));sin(x(2))]) - dist_j;
end