% Define the curface function
surface = @(x) x(1).*x(2) - x(1)^2 - x(2)^2 - 2*(x(1)) - 2*x(2) + 4
% Define the gradient vector
Gradient = @(x) [x(2) - 2*x(1) - 2;x(1) - 2*x(2)-2]
clf
d = .25;
%define initial point
x = [4;0];
norm(Gradient(x))
% Define the initial step-size
lambda = 1;
% Define the step-size multiplier
delta = 1.2;
%hold onto this point for calculations
x1 = [4; 0];
%define initial heading
heading1 = [0; 1];
delta = 1.2;
pub = rospublisher('/raw_vel');
msg = rosmessage(pub);
while norm(Gradient(x)) > 0.01
% Compute the angle between the next heading and current heading
g = Gradient(x)
costheta = dot(g,heading1)/(norm(g)*norm(heading1))
theta = acos(costheta)
heading1 = g
%Compute next point
x = x + (lambda/16).*Gradient(x);
deltax = x - x1;
% Tell robot to turn 
omega = (theta/lambda)*(d/2);
Vl = -omega
Vr = omega
msg.Data = [Vl, Vr];
send(pub, msg);
pause(lambda)
%Tell robot to drive at an increment
distance = sqrt(deltax(1)^2 + deltax(2)^2)/3.28
velocity = distance/lambda;
Vl = 0.1
Vr = 0.1
msg.Data = [Vl, Vr];
send(pub, msg);
pause(distance/0.1)
lambda = lambda .*delta;
x1 = x;
end
msg.Data = [0,0];
send(pub,msg)
plot(x(1), x(2), 'r*')
axis square

