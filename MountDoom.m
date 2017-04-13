sub = rossubscriber('/accel');

%Define the functions 
d = 0.83;
lambda = 1/4;
%Making our ros objects 
pub = rospublisher('/raw_vel');
msg = rosmessage(pub);
accelMessage = receive(sub);
%time object
run = 1;
while run == 1
%define t as our current time
        accelMessage = receive(sub);
        gv = accelMessage.Data
        theta = atan(-gv(2)/gv(3))
        theta = theta 
        omega = (theta/lambda)*(d/2);
        Vl = omega
        Vr = -omega
        msg.Data = [Vl, Vr];
        send(pub, msg);
        pause(lambda)
        
        Vl = 0.09
        Vr = 0.09
        msg.Data = [Vl, Vr];
        send(pub, msg);
        
end
msg.Data = [0, 0];
send(pub, msg);