scriptOptions.printProgress     = 1;  % Print progress every timestep
scriptOptions.printConvergence  = 0;  % Print convergence parameters every timestep
scriptOptions.saveWorkspace     = 1;  % Save complete workspace at the end of simulation
scriptOptions.savePath          = ['results/tmp']; % Destination folder of saved files

%% General settings
% Wind farm case to simulate
Wp.name = '2turb_adm_turb'; % Name of WFSim meshing (from '/WFSim/bin/core/meshing.m')

% Load a predefined random seed (for one-to-one comparisons between simulation cases)
strucObs.loadRandomSeed = true; 

% Estimate freestream conditions
strucObs.U_Inf.estimate  = false;  % Estimate freestream (inflow) u_Inf and v_Inf
strucObs.U_Inf.intFactor = 0.99;  % LPF gain (1: do not change, 0: instant change)
   
% Measurement definitions
strucObs.measPw = true;  % Use power measurements (SCADA) from turbines in estimates
    strucObs.measSigma.P = 1e4;   % Stand. dev. of artificial noise on Power measurements in [W]
strucObs.measFlow = false; % Use flow measurements (LIDAR) in estimates
    strucObs.measSigma.u = 1e-1;  % Stand. dev. of artificial noise on Flow measurements in [m/s]
    strucObs.measSigma.v = 1e-1;  % Stand. dev. of artificial noise on Flow measurements in [m/s]
    strucObs.sensorsPath = 'sensors_2turb_alm'; % Flow measurement setup filename (see '/setup_sensors/sensors_layouts')
        

%% Kalman filter settings
% State estimation settings
strucObs.se.enabled = true; % Estimate the model states (flow fields)?
    % Initial state error covariance matrix (diagonal values on P_0 matrix)
    strucObs.se.P0.u = 0.10; % Initial state covariance for long. flow
    strucObs.se.P0.v = 0.10; % Initial state covariance for lat. flow
    
    % Process noise covariance matrix (diagonal values on Q matrix)
    strucObs.se.Qk.u = 1e-2; % Autocovariance for long. flow process noise
    strucObs.se.Qk.v = 1e-4; % Autocovariance for lat.  flow process noise
    
    % Measurement noise covariance matrix (diagonal values on R matrix) 
    strucObs.se.Rk.P = 1e8;  % Autocovariance for turbine power measurements [if strucObs.measPw == 1]
    strucObs.se.Rk.u = 1e-2; % Autocovariance for long. flow measurements  [if strucObs.measFlow == 1]
    strucObs.se.Rk.v = 1e-2; % Autocovariance for lat.  flow measurements  [if strucObs.measFlow == 1]
    
    % Export pressure terms (recommended: false)
    scriptOptions.exportPressures = false; % Estimate pressure terms
        strucObs.se.P0.p = 0.00; % Initial state covariance for pressure terms
        strucObs.se.Qk.p = 0.00; % Autocovariance for pressure process noise

% Parameter estimation settings
strucObs.pe.enabled = true; % Estimate model parameters?
    strucObs.pe.vars = {'site.lmu'}; % If empty {} then no estimation
    strucObs.pe.P0   = [0.5]; % Initial state covariance(s) for model variable(s)
    strucObs.pe.Qk   = [1e-4]; % Autocovariance(s) process noise for model variable(s)
    strucObs.pe.lb   = [0.05]; % Lower bound(s) for model variable(s)
    strucObs.pe.ub   = [4.00]; % Upper bound(s) for model variable(s)
        
% Observer-specific settings
strucObs.filtertype = 'enkf'; 
switch lower(strucObs.filtertype)    
    case {'exkf'} % Extended Kalman filter (ExKF)
        % ... The ExKF does not have any specific model settings
        
    case {'smo'} % Sliding mode observer (SMO)   
        strucObs.alpha = 0.01; % tuning parameter
        % SMO does not use any covariance matrices, relies excl. on alpha
        
    case {'ukf'}  % Unscented Kalman filter (UKF)      
        % Sigma-point generation settings
        strucObs.alpha = 1.0; % Tuning parameter 
        strucObs.beta  = 2.0; % Tuning parameter (2 optimal for Gaussian distributions)
        strucObs.kappa = 0.0; % Tuning parameter (state est.: "0" or param est.: "3-L")

    case {'enkf'} % Ensemble Kalman filter (EnKF) 
        strucObs.nrens  = 50;        % Ensemble size
        strucObs.r_infl = 1.025;     % Covariance inflation factor (typically 1.00-1.20, no inflation: 1)
        strucObs.f_locl = 'gaspari'; % Localization method: 'off', 'gaspari' (Gaspari-Cohn 1999) or 'heaviside' (Heaviside step function: 0s or 1s)
        strucObs.l_locl = 131;       % Gaspari-Cohn: typically sqrt(10/3)*L with L the cut-off length. Heaviside: cut-off length (m).

    case {'sim'} % No filter, just open-loop simulation
        scriptOptions.exportPressures = true;  % Force 'true', required for sim case.

    otherwise
        error('not a valid filter/simulation specified.');
end