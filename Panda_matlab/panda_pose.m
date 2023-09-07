clear all
close all
clc

rosshutdown
rosinit('172.16.0.109');

r = panda_ros

r.ErrorRecovery


pub = rospublisher('/Panda_pose','geometry_msgs/Pose');
msg = rosmessage('geometry_msgs/Pose');

%%
r.SetCartesianStiffness([0,0,0,0,0,0]);
r.SetCollisionBehaviour([45,45,45],[45,45,45],[45,45,45,45,45,45,45]);

time = 100;
dt = 0.2;

iteracija = time/dt;
i = 1;

h = tic;
try
    while i<=iteracija
        P = r.p;
        Q = r.Q;
        msg.Position.X = P(1);
        msg.Position.Y = P(2);
        msg.Position.Z = P(3);
        Q = double(Q);
        msg.Orientation.W = Q(1);
        msg.Orientation.X = Q(2);
        msg.Orientation.Y = Q(3);
        msg.Orientation.Z = Q(4);
        i=i+1;
        send(pub,msg);
        toc(h);
        pause(i*dt - toc(h))
       

    end
catch
    disp('ROS subscriber terminated.');
end