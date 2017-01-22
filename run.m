% Cyrus Liu
% the Robotics Institute, Carnegie Mellon University
% 01/22/2017

% Vehicle Drifting Dynamics Simulation

% ----------------------------------------
% --------Initialize Visualization--------
% ----------------------------------------
figure(1)
P = [-0.15  -0.15  0.15  0.15  -0.15; -0.08  0.08  0.08  -0.08  -0.08; 1 1 1 1 1];
CoG = [0;0;1]
r_axle = [-0.15;0;1]
h = animatedline(P(1,:),P(2,:));
traj_cog = animatedline(CoG(1,:),CoG(2,:),'Color','g');
traj_r = animatedline(r_axle(1,:),r_axle(2,:),'Color','r');
axis([-3, 3, -3, 3])
axis equal

% --------Initialize Joystick--------
joy = vrjoystick(1)
x = [0,0,0,0,0,0];
dt = 0.03;


for t = 0:dt:10
    % ----------------------------------------
    % ----------Update Visualization----------
    % ----------------------------------------
    pos_x = x(1);
    pos_y = x(2);
    pos_phi = wrapToPi(x(3));
    A = [cos(pos_phi) -sin(pos_phi) pos_x; sin(pos_phi) cos(pos_phi) pos_y; 0 0 1];
    pos = A*P;
    CoG_n = A*CoG;
    rear_n = A*r_axle; 
    clearpoints(h);
    addpoints(h,pos(1,:),pos(2,:));
    addpoints(traj_cog,CoG_n(1,:),CoG_n(2,:));
    addpoints(traj_r,rear_n(1,:),rear_n(2,:));
    drawnow
    
    % --------Use Joystick Input--------
    throttle = max(0,-2*axis(joy,4));
    steer = -1.2*axis(joy,1);
    if button(joy,1)
        pause()   
    end 
    
    % ------Calculate Car Dynamics------
    u = [throttle,steer];
    new_x = dynamics_finite(x, u, dt);
    x = new_x;
end