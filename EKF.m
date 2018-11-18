%
% Author Patric Jensfelt
%
clear
figure(1)
clf

displayCorrelations = 1

% Sampling rate
dT = 0.1;

%% True model parameters
% Commanded speed in vehicle frame, corresponding to moving in 
% a circle of radius 5m
vx = 0.5;
vy = 0;
yawrate = 0.1;

% Process noise
xStd = 1e-9;
yStd = 1e-9;
yawStd = 1e-9;
vxStd = 0.001;
vyStd = 0.001;
yawrateStd = 0.001;

% Measurement noise
xMeasStd = 1;
yMeasStd = 1;

%% Filter model parameters
% Commanded speed in vehicle frame
vxF = 0.5;
vyF = 0;
yawrateF = 0.1;

% Process noise
xStdF = 1e-9;
yStdF = 1e-9;
yawStdF = 1e-9;
vxStdF = 0.001;
vyStdF = 0.001;
yawrateStdF = 0.001;

% Measurement noise
xMeasStdF = 1;
yMeasStdF = 1;

%% Run simulation

% Display handles to
h = [];

axis(20*[-1 1 -1 1])

% True state generated at random pose in 5m disc
startRadius = 5*rand(1,1);
startBearing = 2*pi*rand(1,1);
xt=startRadius*cos(startBearing); 
yt=startRadius*sin(startBearing); 
yawt=2*pi*rand(1,1);

% Initialize the estimate at 0,0,0 
X0 = zeros(3,1); % State estimate
P0 = 1e6*eye(3); % State covariance matrix

X = X0;
P = P0;

iter = 0;

% To experiment with what happens to the filter when we have no information 
% about the motion and the input is just noise
driveByPureNoise = 1
if driveByPureNoise
    vxF = 0;
    vyF = 0;
    yawrateF = 0;
    xStdF = 0.1;
    yStdF = 0.1;
    yawStdF = 0.1;
end

run = 1
while (run)
    
    % Noise command signals
    vxPN = (vx + vxStd*randn(1,1));
    vyPN = (vy + vyStd*randn(1,1));

    % Update true pose which includes some noise ontop of the commanded signals
    xt = xt + vxPN*dT*cos(yawt) - vyPN*dT*sin(yawt) + xStd*randn(1,1);
    yt = yt + vxPN*dT*sin(yawt) + vyPN*dT*cos(yawt) + yStd*randn(1,1);
    yawt = yawt + (yawrate + yawrateStd*randn(1,1))*dT + yawStd*randn(1,1);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Predict motion x=f(x)h
    X(1) = X(1) + vxF*dT*cos(X(3)) - vyF*dT*sin(X(3));
    X(2) = X(2) + vxF*dT*sin(X(3)) + vyF*dT*cos(X(3));
    X(3) = X(3) + yawrateF*dT;

    % Jacobian of system dynamics "f(x)" w.r.t. to state variables
    A = [ [ 1, 0, -vxF*dT*sin(X(3))-vyF*dT*cos(X(3))];
          [ 0, 1, vxF*dT*cos(X(3))-vyF*dT*sin(X(3))];
          [ 0, 0, 1] ];

    % Jacibian of system dynamics "f(x)" w.r.t. the noise variables with std
    % xStd, yStd, yawStd, vxStd, vyStd, yawrateStd
    W = [1 0 0 dT*cos(X(3)) -dT*sin(X(3)) 0; 0 1 0 dT*sin(X(3)) dT*cos(X(3)) 0; 0 0 1 0 0 dT];
    Q = (diag([xStdF yStdF yawStdF vxStdF vyStdF yawrateStdF])).^2;

    % Update state covariance matrix
    P = A*P*A' + W*Q*W';

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Measurement update assuming a measurement (x,y) = h(x)

    % Generate measurements from h(x)
    xMeas = xt + xMeasStd*randn(1,1);
    yMeas = yt + yMeasStd*randn(1,1);

    % Jacobian of the measurement function "h(x)"
    H = [1 0 0; 0 1 0];
    innov = [xMeas - X(1);yMeas - X(2)];
    R = (diag([xMeasStdF yMeasStdF])).^2;

    % Kalman filte rmeasurement update
    K = P*H'*inv(H*P*H' + R);
    X = X + K*innov;
    P = (eye(3) - K*H)*P;

    if ~isempty(h)
        delete(h);
    end
    hold on
    h = [plot(X(1,:),X(2,:),'b.') display_robot(xt,yt,yawt,'k',1)];
    h = [h plot(xMeas, yMeas,'xm')];
    h = [h plot_2dgauss(X(1:2), P(1:2,1:2), 'b', true)];
    dirLen = 2.5;
    h = [h plot([X(1)+[0 dirLen*cos(X(3))]], [X(2)+[0 dirLen*sin(X(3))]],'b')];
    h = [h plot([X(1)+[0 dirLen*cos(X(3)+3*sqrt(P(3,3)))]], [X(2)+[0 dirLen*sin(X(3)+3*sqrt(P(3,3)))]],'b')];
    hold off
    drawnow;
    
    if displayCorrelations
        figure(2)
        imagesc(P)
        title('Covariance matrix P')
        figure(1)

        if iter == 0
            disp('')
            disp('')
            disp('Make sure that the two figures are visible and do not overlap')
            disp('Press ENTER to start continue')
            pause
        end
    end
    
    iter = iter + 1;
end





