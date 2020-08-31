%% Setup
clear;clc;
close all;

%import the model
lbr = importrobot('iiwa14.urdf');
lbr.DataFormat = 'row';
gripper = 'iiwa_link_ee_kuka';
lbr.Gravity = [0 0 -9.81];

params.segments = 6;
params.segment_time = 1;

% Initialize matricies to describe the waypoints
theta_waypoints    = []; % each row will give a waypoint in joint space
gripper_waypoints  = []; % each row will give a waypoint for the end-effector

% For plotting, we store the desired trajectory and its timing in the
% following variables
t_trajectory       = []; % 
theta_trajectory   = []; % each row gives joint angles at a snapshot in time
gripper_trajectory = []; % each row gives gripper position at a snapshot in time

theta_init = [deg2rad(90), deg2rad(-90), deg2rad(20), deg2rad(10), deg2rad(50), deg2rad(15), deg2rad(0)];  %angles of each joint
show(lbr, theta_init , 'PreservePlot', false); hold on
drawContext()

%% Part 1: Trajectory design [COMPLETE]

% Find joint angles for the first waypoint
% TODO: Call IK to get a configuration for the first waypoint
p_desired = getWayPoint(1); % Desired position for the gripper
R_desired = eul2rotm([0 deg2rad(105) 0],'ZYX'); % Desired orientation for the gripper. 
                 % Hint: you can use the function eul2rotm
T_desired = [R_desired p_desired';0 0 0 1]; % Desired homogenous transform for the gripper
theta_0 = IK(T_desired,theta_init,lbr,gripper,0); % Get the initial waypoint in joint space

theta_waypoints(1, :)  = theta_0;
gripper_waypoints(1,:) = p_desired;

show(lbr, theta_0 , 'PreservePlot', false); hold on
theta_init = theta_0;

t0 = 0;
t_waypoints = [0];

% Add waypoints for the segments
for n = 1:(params.segments)
    
    % TODO: Call IK to get joint angles for the waypoint at the end of
    %       the segment. Use the joint angles for the previous waypoint as
    %       an initial guess for the IK routine.
    p_desired = getWayPoint(n+1);
    R_desired = eul2rotm([0 deg2rad(105) 0],'ZYX'); % Desired orientation for the gripper 
    T_desired = [R_desired p_desired';0 0 0 1]; % Desired homogenous transform for the gripper
    theta_f = IK(T_desired,theta_0,lbr,gripper,0); % final joint angles for the spline in segment 'n'

    theta_waypoints(n+1,:) = theta_f; % IMPORTANT: DO NOT REMOVE
    gripper_waypoints(n+1,:) = p_desired;
    t_waypoints(n+1) = t0 + params.segment_time;
    
    for t = t0:0.1:(t0+params.segment_time)
        % TODO: Call evaluateCubicSpline to get the joint angles for the trajectory at time 't'
        theta = evaluateCubicSpline(theta_0,theta_f,t0,t_waypoints(n+1),t);
        
        % Get end effector position along trajectory
        T     = getTransform(lbr,theta, gripper); 
       
        theta_trajectory(end+1,:)   = theta;
        t_trajectory(end+1,:)       = t;
        gripper_trajectory(end+1,:) = T(1:3,4)';
        
        show(lbr, theta , 'PreservePlot', false); drawnow;
    end  
    
    theta_0 = theta_f; % The final position of this segment is the initial position for the next one
    t0 = t0+params.segment_time;
end

pause(1)
% TODO: Plot theta trajectory and waypoints
% figure(2); hold on
% plot(t_trajectory,rad2deg(theta_trajectory))
% plot(t_trajectory(1),rad2deg(theta_trajectory(1,:)),'bo')
% for i=1:6
%     plot(t_trajectory(i*11),rad2deg(theta_trajectory(i*11,:)),'bo')
% end
% xlabel('Time (s)')
% ylabel('Joint angles (^o)')
% %axis([0,1,0,90])
% legend("\theta_1","\theta_2","\theta_3","\theta_4","\theta_5","\theta_6","\theta_7")
% ax = gca;
% ax.FontSize = 16;
% ax.FontName = 'Times New Roman';
% 
% 
% % TODO: Plot end-effector trajectory and waypoints
% figure(3); hold on
% plot(t_trajectory,gripper_trajectory)
% plot(t_trajectory(1),gripper_trajectory(1,:),'bo')
% for i=1:6
%    plot(t_trajectory(i*11),gripper_trajectory(i*11,:),'bo')
% end
% xlabel('Time (s)')
% ylabel('Position (m)')
% %axis([0,1,0,90])
% legend("x","y","z")
% ax = gca;
% ax.FontSize = 16;
% ax.FontName = 'Times New Roman';
% figure(1);
% pause(1);
% % 

%% Part 2: Nonlinear Control

% Setup
params.theta_waypoints = theta_waypoints; % IMPORTANT: DO NOT REMOVE
t_end = params.segments * params.segment_time;
dt = 0.005;
dt_anim = 1/15;
t_last_anim = 0;
t_history = 0:dt:t_end;
theta_history = zeros(length(t_history), 7);
theta_dot_history = zeros(length(t_history), 7);
theta_des_history = zeros(length(t_history), 7);

% Initial Conditions for the simulation
theta = theta_init + ones(1,7)*.3;
theta_dot = 0*theta;

% Main simulation loop
k = 0;
for t = t_history
    k = k+1;
    
    % Find the joint acceleration and perform Euler integration
    [tau, theta_des] = controlLaw(lbr, t, theta, theta_dot,params);
    theta_ddot = forwardDynamics(lbr,theta,theta_dot,tau);
    
    theta = theta + dt*theta_dot;
    theta_dot = theta_dot + dt*theta_ddot;
    
    % Save a history of the trajectories
    theta_history(k,:) = theta;
    theta_dot_history(k,:) = theta_dot;
    theta_des_history(k,:) = theta_des;
    
    % If we haven't animated in a while, do so
    if t > (t_last_anim + dt_anim)
        show(lbr, theta , 'PreservePlot', false); drawnow;
        t_last_anim = t_last_anim +dt_anim;
        fprintf(1,'t=%f\n',t);
    end  
end

% TODO: Plot theta and theta_des vs. time
figure(4);clf; hold on
plot(t_history, rad2deg(theta_history),t_history, rad2deg(theta_des_history),'--')
xlabel('Time (s)')
ylabel('Joint angles (^o)')
legend("\theta_1","\theta_2","\theta_3","\theta_4","\theta_5","\theta_6",...
    "\theta_7","\theta_1_d","\theta_2_d","\theta_3_d","\theta_4_d",...
    "\theta_5_d","\theta_6_d","\theta_7_d",'NumColumns',7);
%axis([0 6 -75 300])
ax = gca;
ax.FontSize = 16;
ax.FontName = 'Times New Roman';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Begin Functions with TODO items
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% [theta] = evaluateCubicSpline(theta0, thetaf, t0, tf, t)
% Evaluate a cubic spline at time t where the spline satisfies 
% theta(t0) = theta0 and theta(tf) = thetaf
% and has zero starting and ending velocities
%
% Inputs: theta0    1x7 vector for initial configuration
%         thetaf    1x7 vector for final configuration
%         t0        intial time
%         tf        final time
%         t         time of interest
%
% Outputs: theta    1x7 vector for configruation at time t
%
function [theta, theta_dot, theta_ddot] = evaluateCubicSpline(theta0, thetaf, t0, tf, t)
    tf = tf-t0; % modify tf and t so you can treat everything as starting at t0=0 s
    t  = t-t0;
    
    %%TODO: compute a0-a3
    a0 =  theta0;
    a1 =  zeros(1,7);
    a2 =  (3/tf^2)*(thetaf-theta0);
    a3 = (-2/tf^3)*(thetaf-theta0);
    
    theta = a0 + a1*t + a2*t^2 + a3*t^3;
    theta_dot = a1+2*a2*t+3*a3*t^2;
    theta_ddot = 2*a2+6*a3*t;
end


%% [tau,theta_des]  = controlLaw(robot, t, theta, theta_dot,params)
% Evaluate the computed-torque control law
% 
% Inputs: robot     robot object
%         t         time
%         theta     1x7 vector for current joint angles
%         theta_dot 1x7 vector for current joint rates
%         params    parameter structure for desired trajectory
%
function [tau,theta_des]  = controlLaw(robot, t, theta, theta_dot,params)
    
    %% TODO: Uncomment
    [theta_des, theta_dot_des, theta_ddot_des] = getThetaDesired(t, params);
    M = massMatrix(robot, theta);
    V = velocityProduct(robot, theta, theta_dot);
    G = gravityTorque(robot, theta);
   
    %% TODO: Fill in computed torque control law
    kp = 100;
    kd = 20;
    theta_ddot_commanded =kp*(theta_des-theta)-kd*theta_dot;
    %theta_ddot_commanded =theta_ddot_des+kp*(theta_des-theta)+kd*(theta_dot_des-theta_dot);
    tau = (G' + V' + M*theta_ddot_commanded')';
    
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Helper Functions without TODO items
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% [theta_des, theta_dot_des, theta_ddot_des] = getThetaDesired(t, params)
% Return details on the desired trajectory at time t
%
% Inputs:  t            time at which you want to know the desired theta
%          params       structure of parameters for the desired trajectory
%          
% Outputs: theta_des      1x7 vector of the desired joints anlgles at time t
%          theta_dot_des  1x7 vector of the desired joint rates at time t
%          theta_ddot_des 1x7 vector of the desired joints accelerations at time t
%
function [theta_des, theta_dot_des, theta_ddot_des] = getThetaDesired(t, params)
    segment = floor(t)+1;
    theta_0 = params.theta_waypoints(segment,:);
    theta_f = params.theta_waypoints(min(segment+1,params.segments+1),:);
    if nargout == 1
        theta_des = evaluateCubicSpline(theta_0, theta_f, 0, 1, mod(t,1) );
    elseif nargout == 2
        [theta_des, theta_dot_des] = evaluateCubicSpline(theta_0, theta_f, 0, 1, mod(t,1) );
    else 
       [theta_des, theta_dot_des, theta_ddot_des] = evaluateCubicSpline(theta_0, theta_f, 0, 1, mod(t,1) );
    end
end

%% p = getWayPoint(n)
% Return the position of the n-th waypoint
%
% Inputs: n     the number of the waypoint of interest
%
% Ouptuts: p    a 1x3 vector for the position of the n-th waypoint
function p = getWayPoint(n) 
    separation = 0.18; %separation in y between waypoints
    y0 = -.5;          %initial y-position of waypoints
    y = (n-1)*separation + y0;
    if mod(n,2) == 0 % even numbered waypoints are on the fuselage
       p = getFuselagePosition(15*pi/180,  0 , 0);
    else %odd numbered waypoints are 0.25 m away
       p = getFuselagePosition(15*pi/180,  0, .25); 
    end
    p(2) = y;
end

%% e = computeError( T_desired, angleJ, robot, end_effector)  
% Function to compute the 6x1 vector of error
% 
% Inputs: T_desired     Desired homogenous transform from the base to
%                       the end effector (4 x 4)
%         theta         Joint angle vector for the manipulator (1 x N)
%         robot         Robotics Systems Toolbox Robot object 
%         end_effector  Name of end-effector
%
% Outputs: e    A 6x1 vector containing the angle-axis error and position error
%               between the desired and actual end-effector pose
%
function e = computeError( T_desired, theta, robot, end_effector)    
    % TODO: Extract the rotation from the desired homogenous transform
    R_desired = T_desired(1:3,1:3); 
    % TODO: Extract the position from the desired homogenous transform
    p_desired = T_desired(1:3,4);                 
    
    % Homogeneous transform from 0 to the end_effector
    T = getTransform(robot,theta, end_effector); 
    
    % TODO: Extract the actual rotation from the homogeneous transform
    R = T(1:3,1:3);  
    % TODO: Extract the actual position from the homogeneous transform
    p = T(1:3,4);               
    
    % TODO: Compute the position error
    position_error   = p_desired-p;  
    % TODO: Compute the cross product matrix of the angle-axis error
    S_angle_axis     = logm(R_desired*R'); 
    % TODO: Compute the angle-axis error
    angle_axis_error = [S_angle_axis(3,2);S_angle_axis(1,3);S_angle_axis(2,1)];    
    
    e = [angle_axis_error ; position_error ];
end

%% [angle_new, step_size] = IK_step(angle, T_desired, robot, end_effector)
% Function to take a step toward the desired end-effector pose
%
% Inputs: theta         Current joint angle vector for the manipulator (1 X N)
%         T_desired     Desired homogenous transform from the base to
%                       the end effector (4 x 4) 
%         robot         Robotics Systems Toolbox Robot object
%         end_effecotr  Name of end-effector
%
% Ouputs: theta_new     Updated joint angle after the step (1 X N)
%         reduction     Reduction in error from this step 

function [theta_new, reduction] = IK_step(theta, T_desired, robot, end_effector)
    
    e = computeError( T_desired, theta,robot, end_effector);
    initial_error = norm(e);
    J = geometricJacobian(robot,theta,end_effector);
    
    % Algorithm parameters
    lambda = 1e-4;                      % Damped-Least Squares Damping
    minimum_beta = 1e-15;               % Smallest step allowed
    accepted_reduction_percent = 0.01;  % Criteria to accept the step
    
    % Initial test step size
    beta = 1;
    
    % TODO: Compute ideal step with the damped-least-squares pseduoinverse
    delta_theta = J'*(J*J'+lambda*eye(6))^(-1)*e; 
    
    % Candidate for new joint angles
    theta_test = theta + delta_theta' * beta;
    
    e = computeError( T_desired, theta_test,robot, end_effector);
    % TODO: Compute reduction in error for the candidate step
    reduction           = initial_error-norm(e);
    % TODO: Compute the expected reduction in error for the candidate step (assumping linearization perfect)
    expected_reduction  = reduction*beta;       
    

    % Perform a backtracking line search over the step size parameter beta
    % Accept the step when either 
    %  a) the reduction in error is at least accepted_reduction_percent * expected_reduction or
    %  b) the step size is smaller than the minimum allowed beta
    % TODO: Fill in conditions on the termination of the loop
    while (reduction<accepted_reduction_percent*expected_reduction) && (beta>=minimum_beta) 
        %fprintf(1,'Rejected step with beta = %e, Actual Reduction = %e,  Expected Reduction = %e\n',beta, reduction, expected_reduction);
        beta = beta * .5;                           % Reduce beta
        theta_test = theta + delta_theta' * beta;   % Create a new test configuration
        e = computeError( T_desired, theta_test,robot, end_effector);
        % TODO: Fill in same as above
        reduction = initial_error-norm(e);     
        % TODO: Fill in same as above
        expected_reduction = reduction*beta;  
    end
    
    theta_new = theta_test;
    fprintf(1,'Accepted Step beta = %e, Actual Reduction = %e,  Expected Reduction = %e\n',beta, reduction, expected_reduction);
    
end
 
%% [theta, iteration_errors] = IK(T_desired, theta_0, robot, end_effector,animate_flag)
% Solve the numerical IK problem from a given initial guess
%
% Inputs: T_desired     Desired homogenous transform from the base to
%                       the end effector (4 x 4)
%         theta_0       Initial guess for the joint angle vector for the manipulator (1 x N)
%         robot         Robotics Systems Toolbox Robot object 
%         end_effector  Name of end-effector
%         animate_flag  Set to 1 to enable animation of each step
% Outputs: theta        Joint angle vector that solves the numerical IK problem
%          iteration_errors Vector of norm of e at each iteration
function [theta, iteration_errors] = IK(T_desired, theta_0, robot, end_effector, animate_flag)
    theta = theta_0; 
    e = computeError( T_desired, theta, robot, end_effector);
    error_threshold = 1e-6;
    iteration_errors = [norm(e)];
    
    % While the error has magnitude larger than some threshold
    % TODO: Complete while loop condition
    while (iteration_errors>error_threshold)
        [theta, reduction] = IK_step(theta, T_desired, robot, end_effector);
        e = computeError( T_desired, theta, robot, end_effector);
        iteration_errors(end+1) = norm(e);
        if animate_flag
            show(robot, theta , 'PreservePlot', false);
            pause(.5)
        end
    end  

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% From here down: Code you can ignore
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Helper function to return the a cartesian position along the fuselage from
% cylindrical coordniate inputs
% You don't need to know any of the details to complete the project
function p = getFuselagePosition(th, y, extra_r)
    r = 2;
    l = 4; 
    x0 = 2.5; 
    y0 = -2;
    z0 = 0;
    r = r+extra_r;
    p = [x0-r*cos(th) y0+l*y z0+r*sin(th)];
end

% Helper function to draw the fuselage and waypoints
% You don't need to know any of the details to complete the project
function drawContext()
    v = []; % verticies
    f = []; % faces
    c = []; % color
    n = []; % noramls

    num_rows = 1;
    num_cols = 40;
    % 
    spot = @(a,b) (a-1)*(num_cols+1)+b; 

    for i = 1:num_rows+1
        y = (i-1)/num_rows;
        for j = 1:num_cols+1
            th = (j-1)/num_cols*(pi/2+pi/3) - pi/3;
            v(end+1,:) = getFuselagePosition( th, y, 0); 
            n(end+1,:) = [-cos(th) 0 sin(th)];
        end
    end
    for i = 1:num_rows
        for j = 1:num_cols
           f(end+1,:) = [spot(i,j) spot(i,j+1) spot(i+1,j+1) spot(i+1,j)];
           c(end+1,:) = [.8 .8 .8];
        end 
    end

    figure(1);
    fg = gca;
    waypoints = [];
    for i = 1:7
        waypoints(i,:) = getWayPoint(i);
    end
    plot3(waypoints(:,1),waypoints(:,2), waypoints(:,3),'k.','MarkerSize',10);

    p = patch('Parent',fg,'faces', f, 'vertices', v, 'FaceVertexCData', c,  'FaceColor','Flat',...
           'EdgeColor','none', ...
           'VertexNormals',n,...
           'FaceLighting', 'gouraud', ...
           'BackFaceLighting', 'unlit');

   
   axis([-1.5 1.5 -1.5 1.5 -.75 2.5])
   
   l = light;
   l.Position = [-2 0 1];
   view(-172,18);
    a = gca;
    a.CameraPosition = [-3.3613 24.6817 8.4654];
    a.CameraTarget = [0.0776 0.2130 0.4369];
end
