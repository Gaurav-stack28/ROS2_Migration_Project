function [sys,x0,str,ts,simStateCompliance] = Kalman_Filter_Estimation(t,x,u,flag)
%SFUNTMPL General MATLAB S-Function Template
%   With MATLAB S-functions, you can define you own ordinary differential
%   equations (ODEs), discrete system equations, and/or just about
%   any type of algorithm to be used within a Simulink block diagram.
%
%   The general form of an MATLAB S-function syntax is:
%       [SYS,X0,STR,TS,SIMSTATECOMPLIANCE] = SFUNC(T,X,U,FLAG,P1,...,Pn)
%
%   What is returned by SFUNC at a given point in time, T, depends on the
%   value of the FLAG, the current state vector, X, and the current
%   input vector, U.
%
%   FLAG   RESULT             DESCRIPTION
%   -----  ------             --------------------------------------------
%   0      [SIZES,X0,STR,TS]  Initialization, return system sizes in SYS,
%                             initial state in X0, state ordering strings
%                             in STR, and sample times in TS.
%   1      DX                 Return continuous state derivatives in SYS.
%   2      DS                 Update discrete states SYS = X(n+1)
%   3      Y                  Return outputs in SYS.
%   4      TNEXT              Return next time hit for variable step sample
%                             time in SYS.
%   5                         Reserved for future (root finding).
%   9      []                 Termination, perform any cleanup SYS=[].
%
%
%   The state vectors, X and X0 consists of continuous states followed
%   by discrete states.
%
%   Optional parameters, P1,...,Pn can be provided to the S-function and
%   used during any FLAG operation.
%
%   When SFUNC is called with FLAG = 0, the following information
%   should be returned:
%
%      SYS(1) = Number of continuous states.
%      SYS(2) = Number of discrete states.
%      SYS(3) = Number of outputs.
%      SYS(4) = Number of inputs.
%               Any of the first four elements in SYS can be specified
%               as -1 indicating that they are dynamically sized. The
%               actual length for all other flags will be equal to the
%               length of the input, U.
%      SYS(5) = Reserved for root finding. Must be zero.
%      SYS(6) = Direct feedthrough flag (1=yes, 0=no). The s-function
%               has direct feedthrough if U is used during the FLAG=3
%               call. Setting this to 0 is akin to making a promise that
%               U will not be used during FLAG=3. If you break the promise
%               then unpredictable results will occur.
%      SYS(7) = Number of sample times. This is the number of rows in TS.
%
%
%      X0     = Initial state conditions or [] if no states.
%
%      STR    = State ordering strings which is generally specified as [].
%
%      TS     = An m-by-2 matrix containing the sample time
%               (period, offset) information. Where m = number of sample
%               times. The ordering of the sample times must be:
%
%               TS = [0      0,      : Continuous sample time.
%                     0      1,      : Continuous, but fixed in minor step
%                                      sample time.
%                     PERIOD OFFSET, : Discrete sample time where
%                                      PERIOD > 0 & OFFSET < PERIOD.
%                     -2     0];     : Variable step discrete sample time
%                                      where FLAG=4 is used to get time of
%                                      next hit.
%
%               There can be more than one sample time providing
%               they are ordered such that they are monotonically
%               increasing. Only the needed sample times should be
%               specified in TS. When specifying more than one
%               sample time, you must check for sample hits explicitly by
%               seeing if
%                  abs(round((T-OFFSET)/PERIOD) - (T-OFFSET)/PERIOD)
%               is within a specified tolerance, generally 1e-8. This
%               tolerance is dependent upon your model's sampling times
%               and simulation time.
%
%               You can also specify that the sample time of the S-function
%               is inherited from the driving block. For functions which
%               change during minor steps, this is done by
%               specifying SYS(7) = 1 and TS = [-1 0]. For functions which
%               are held during minor steps, this is done by specifying
%               SYS(7) = 1 and TS = [-1 1].
%
%      SIMSTATECOMPLIANCE = Specifices how to handle this block when saving and
%                           restoring the complete simulation state of the
%                           model. The allowed values are: 'DefaultSimState',
%                           'HasNoSimState' or 'DisallowSimState'. If this value
%                           is not speficified, then the block's compliance with
%                           simState feature is set to 'UknownSimState'.


%   Copyright 1990-2010 The MathWorks, Inc.

%
% The following outlines the general structure of an S-function.
%
switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes;

  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1,
    sys=mdlDerivatives(t,x,u);

  %%%%%%%%%%
  % Update %
  %%%%%%%%%%
  case 2,
    sys=mdlUpdate(t,x,u);

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3,
    sys=mdlOutputs(t,x,u);

  %%%%%%%%%%%%%%%%%%%%%%%
  % GetTimeOfNextVarHit %
  %%%%%%%%%%%%%%%%%%%%%%%
  case 4,
    sys=mdlGetTimeOfNextVarHit(t,x,u);

  %%%%%%%%%%%%%
  % Terminate %
  %%%%%%%%%%%%%
  case 9,
    sys=mdlTerminate(t,x,u);

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end

% end sfuntmpl

%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes

%
% call simsizes for a sizes structure, fill it in and convert it to a
% sizes array.
%
% Note that in this example, the values are hard coded.  This is not a
% recommended practice as the characteristics of the block are typically
% defined by the S-function parameters.
%
sizes = simsizes;

sizes.NumContStates  = 0;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 1;
sizes.NumInputs      = 5;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

%
% initialize the initial conditions
%
x0  = [];

%
% str is always an empty matrix
%
str = [];

%
% initialize the array of sample times
%
ts  = [0 0];

% Specify the block simStateCompliance. The allowed values are:
%    'UnknownSimState', < The default setting; warn and assume DefaultSimState
%    'DefaultSimState', < Same sim state as a built-in block
%    'HasNoSimState',   < No sim state
%    'DisallowSimState' < Error out when saving or restoring the model sim state
simStateCompliance = 'UnknownSimState';

% end mdlInitializeSizes

%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%
function sys=mdlDerivatives(t,x,u)

sys = [];

% end mdlDerivatives

%
%=============================================================================
% mdlUpdate
% Handle discrete state updates, sample time hits, and major time step
% requirements.
%=============================================================================
%
function sys=mdlUpdate(t,x,u)

sys = [];

% end mdlUpdate

%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x,u)
     % global parameters for EKF
    global state_estimate_dd;
    global state_estimate_kd;
    global P_dd;
    global P_kd;
    
    % global vehicle parameters
    global v_y;
    global l_v;
    global l_h;
    global m;
    global c_v;
    global c_h;
    global theta;
    
    % global parameters for runing
    global analytical_solution_flag;
    global B_dd_func
    global yaw_rate_threshold;
    global vel_x_threshold;

   
    
    
    acc_x = u(1);
    acc_y = u(2);
    yaw_rate = u(3);
    v_x = u(4);
    steering_angle = u(5);


    % constant parameters in EKF
    Delta_T = 0.1;
    process_noise_v_kd = [0.01; 0.01];
    sensor_noise_w_kd = [0.01; 0.01];

    qk11 = rand;
    qk12 = rand * 0.1;
    qk21 = rand * 0.1;
    qk22 = rand;

    Q_kd = [qk11 0; 0 qk22];
    R_kd = [1 0; 0 1];

    I = [1 0; 0 1];
    dt = 0.1;

    process_noise_v_dd = [0.01; 0.01];
    sensor_noise_w_dd = [0.01; 0.01];

    qd11 = rand;
    qd12 = rand * 0.1;
    qd21 = rand * 0.1;
    qd22 = rand;

    Q_dd = [qd11 0; 0 qd22];
    R_dd = [1 0; 0 1];


    %Initializing priori means and estimate error covarience matrices

    if (abs(yaw_rate)<yaw_rate_threshold)
        state_estimate_dd = [0; 0];
        P_kd = [0.1 0; 0 0.1];
        P_dd = [0.1 0; 0 0.1];
        v_y = 0;
        state_estimate_kd = [v_x; 0];
        beta = 0;

     elseif(abs(v_x)<vel_x_threshold)
        state_estimate_dd = [0; yaw_rate];
        P_kd = [0.1 0; 0 0.1];
        P_dd = [0.1 0; 0 0.1];
        v_y = 0;
        state_estimate_kd = [v_x; 0];
        beta = 0;
    
   
    %Initializing all the states
    
% % %     if (abs(yaw_rate)<yaw_rate_threshold) &&  (abs(v_x)<vel_x_threshold)
% % %         state_estimate_dd = [0; yaw_rate];
% % %         state_estimate_kd = [v_x; 0];
% % %         P_kd = [0.1 0; 0 0.1];
% % %         P_dd = [0.1 0; 0 0.1];
% % %         v_y = 0;
% % %         beta = 0;
    
    else
        % calculate discretized state matrix A_d for Kinematics model
        A_k11 = (Delta_T ^ 4 * yaw_rate ^ 4) / 24 - (Delta_T ^ 2 * yaw_rate ^ 2) / 2 + 1;
        A_k12 = (Delta_T ^ 3 * yaw_rate ^ 3) / 6 - Delta_T * yaw_rate;
        A_k21 =  Delta_T * yaw_rate - (Delta_T ^ 3 * yaw_rate ^ 3) / 6 ;
        A_k22 = (Delta_T ^ 4 * yaw_rate ^ 4) / 24 - (Delta_T ^ 2 * yaw_rate ^ 2) / 2 + 1;
        A_kd = [A_k11  A_k12; A_k21 A_k22];

        % calculate discretized input matrix B_d
        B_k11 =  sin(Delta_T * yaw_rate) / yaw_rate;
        B_k12 =  cos(Delta_T * yaw_rate) / yaw_rate;
        B_k21 = -cos(Delta_T * yaw_rate) / yaw_rate;
        B_k22 =  sin(Delta_T * yaw_rate) / yaw_rate;
        B_kd = [B_k11  B_k12; B_k21  B_k22];
        
        % calculate measurement matrix
        C_kd = [1 0; 0 1];

        % set measurement matrix
        z_kd = [v_x; v_y];

        % set control vector
        u_kd = [acc_x; acc_y];

        % EKF update
        % state estimate
        state_estimate_kd = A_kd * state_estimate_kd + B_kd * u_kd + process_noise_v_kd;
        
        %Predicted covariance estimate
        P_kd = A_kd * P_kd * A_kd.' + Q_kd;

        % Measurement residual
        measurement_residual_y_k = z_kd - ((C_kd * state_estimate_kd) + sensor_noise_w_kd);
                
        % Residual covariance
        S_k = C_kd * P_kd * C_kd.' + R_kd;

        % Kalman gain
        K_k = P_kd * C_kd.' / S_k;

        % update state estimate
        state_estimate_kd = state_estimate_kd + K_k * measurement_residual_y_k;

        % update covariance of state estimate
        P_kd = P_kd - (K_k * C_kd * P_kd);
        
        % update value for dynamics model
        v_y = state_estimate_kd(2);
        
        % upate dynamics model's state estimation 
        state_estimate_dd(1) = state_estimate_kd(2); 
        
        % EKF for Dynamics model where d is noted for dynamics things

        A_11 = (-c_v-c_h)/(m*v_x);
        A_12 = v_x+(c_v*l_v-c_h*l_h)/(m*v_x);
        A_21 = (-l_v*c_v +l_h*c_h)/(theta*v_x);
        A_22 = (-c_v*l_v^2 - c_h*l_h^2)/(theta *v_x);
        A = [A_11 A_12; A_21 A_22];
        B = [(c_v/m); (l_v*c_v/theta)];
        A_dd = I + A*dt + 1/2*A^2*dt^2 + 1/6*A^3*dt^3 + 1/24*A^4*dt^4;

        % calculate the analytical solution of B_dd 
        % the analytical_solution_flag is to make the analytical solution calculated only once, optimising speed
        syms delta_t 'real';
        if analytical_solution_flag == false
            B_dd_func = int(expm(A*delta_t)*B, delta_t);
            analytical_solution_flag = true;
        end

        % Substitute values in the analytical solution
        B_dd = double(vpa(subs(B_dd_func, delta_t, dt)));
                
        C_dd = [((-c_v - c_h)/(m * v_x)) (-(l_v * c_v - l_h * c_h)/(m * v_x)); 0 1];
        D_dd = [(c_v/m); 0];

        % get the measurement output vector
        z_dd = [acc_y; yaw_rate] - D_dd * steering_angle;

        % control vector
        u_dd = steering_angle;

        % EKF update
        % state estimate
        state_estimate_dd = A_dd * state_estimate_dd + B_dd * u_dd + process_noise_v_dd;
        
        % Predicted covariance estimate
        P_dd = A_dd * P_dd * A_dd.' + Q_dd;

        % Measurement residual
        measurement_residual_y_k = z_dd - ((C_dd * state_estimate_dd) + sensor_noise_w_dd);

        % Residual covariance
        S_k = C_dd * P_dd * C_dd.' + R_dd;

        % Kalman gain
        K_k = P_dd * C_dd.' / S_k;

        % update state estimate
        state_estimate_dd = state_estimate_dd + K_k * measurement_residual_y_k;

        % update covariance of state estimate
        P_dd = P_dd - K_k * C_dd * P_dd;
        
        % update lateral velocity
        v_y = state_estimate_dd(1);
        
        % update kinematics model estimation for next time step
        state_estimate_kd(2) = v_y;

        % Sideslip Angle Estimation
        beta = atan(v_y /v_x);
    end      





sys = [beta];

% end mdlOutputs

%
%=============================================================================
% mdlGetTimeOfNextVarHit
% Return the time of the next hit for this block.  Note that the result is
% absolute time.  Note that this function is only used when you specify a
% variable discrete-time sample time [-2 0] in the sample time array in
% mdlInitializeSizes.
%=============================================================================
%
function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;

% end mdlGetTimeOfNextVarHit

%
%=============================================================================
% mdlTerminate
% Perform any end of simulation tasks.
%=============================================================================
%
function sys=mdlTerminate(t,x,u)

sys = [];

% end mdlTerminate
