%
% Author Patric Jensfelt
%
clear
close all
figure(1)

% Load simulation parameters
simulation_parameters;

% Display handles to
h = [];

axis(20*[-1 1 -1 1])

% True state
xt=0; yt= 0; yawt = 0;
X0 = zeros(3,1); % State estimate
P0 = 1e-6*eye(3); % State covariance matrix

% Store state estimate and covariance matrix
X = X0;
P = P0;

firstIter = 0;

while (run)
    
    dnoise = (tspeed*dT*tdStd)*randn;
    arnoise = (rspeed*dT*rdaStd)*randn;
    atnoise = (tspeed*dT*tdStd)*randn;

    R =
    % Noise command signals
    vxPN = (vx + vxStd*randn(1,1));
    vyPN = (vy + vyStd*randn(1,1));

    % Update true pose which includes some noise ontop of the commanded signals
    xt = xt + vxPN*dT*cos(at) - vyPN*dT*sin(at) xStd*randn(1,1);
    yt = yt + vxPN*dT*sin(at) + vyPN*dT*cos(at) yStd*randn(1,1);
    yawt = yawt + (yawrate + yawrateStd*randn(1,1))*dt + yawStd*randn(1,1);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Predict motion x=f(x)
    X(1) = X(1) + vx*dT*cos(X(3)) - vy*dT*sin(X(3));
    X(2) = X(2) + vx*dT*sin(X(3)) + vy*dT*cos(X(3));
    X(3) = X(3) + yawrate*dT;

    % Jacobian of system dynamics "f(x)" w.r.t. to state variables
    A = [[ 1, 0, -vx*dT*sin(X(3))-vy*dT*cos(X(3))];
            [ 0, 1, vx*dT*cos(X(3))-vy*dT*sin(X(3))];
            [ 0, 0, 1]];

    % Jacibian of system dynamics "f(x)" w.r.t. the noise variables with std
    % xStd, yStd, yawStd, vxStd, vyStd, yawrateStd
    W = [1 0 0 dT*cos(X(3)) -dT*sin(X(3)) 0; 0 1 0 dT*sin(X(3)) dT*cos(X(3)) 0; 0 0 1 0 0 dT];
    Q = (diag([xStd yStd yawStd vxStd vyStd yawrateStd])).^2;

    % Update state covariance matrix
    P = A*P*A' + W*Q*W';

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Measurement update assuming a measurement (x,y) = h(x)

    % Generate measurements from h(x)
    xMeas = X(1) + xMeasStd*randn(1,1);
    yMeas = X(2) + yMeasStd*randn(1,1);

    % Jacobian of the measurement function "h(x)"
    H = [1 0 0; 0 1 0];
    innov = [X(1) - xMeas;X(2) - yMeas];
    R = (diag([xMeasStd yMeasStd])).^2;

    % Kalman filte rmeasurement update
    K = P*H'*inv(H*P*H' + R);
    X = X + K*innov;
    P = (eye(3) - K*H)*P;

    if ~isempty(h)
        delete(h);
    end
    hold on
    h = [plot(X(1,:),X(2,:),'b.') display_robot(xt,yt,at,'k',1)];
    h = [h plot([xMeas yMeas,'xm')];
    h = [h plot_2dgauss(X(1:2), P(1:2,1:2), 'b', true)];
    dirLen = 0.5;
    h = [h plot([X(1)+[0 dirLen*cos(X(3))]], [X(2)+[0 dirLen*sin(X(3))]],'b')];
    h = [h plot([X(1)+[0 dirLen*cos(X(3)+3*sqrt(P(3,3)))]], [X(2)+[0 dirLen*sin(X(3)+3*sqrt(P(3,3)))]],'b')];
    hold off
    drawnow;
end





