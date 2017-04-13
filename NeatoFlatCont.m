%Define the functions 
syms x(t) y(t) 
A = [-2 1; 1 -2]
B = [-2; -2]
Vars = [x;y]
alpha = 0.1
%Solve the ODEs
ode = diff(Vars) == alpha.*(A*Vars + B);
cond = Vars(0) == [4;1];
[xSol(t), ySol(t)] = dsolve(ode, cond);

T = diff([xSol(t); ySol(t); 0])
%Compute tangent vectors
MagT = norm(T);
That = T./MagT;
%Calculating the normal vector
N = simplify(diff(That));
%Calculating angular velocity
w = cross(That, N);
w = w(3);
d = 0.83;
%calculating total velocity
vel = norm(T);
o = matlabFunction(w);
velocity = matlabFunction(vel);
%Making our ros objects 
pub = rospublisher('/raw_vel');
msg = rosmessage(pub);
%time object
time = tic;
run = 1;

while run == 1
%define t as our current time
t = toc(time);
    if t < 28
        omega = o(t)% angular velocity (radians/second)
        omega = (theta/lambda)*(d/2);
        Vl = -omega
        Vr = omega
        msg.Data = [Vl, Vr];
        send(pub, msg);
        pause(lambda)

    else
        run = 2;
    end

end
msg.Data = [0, 0];
send(pub, msg);