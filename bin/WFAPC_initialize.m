function [ Wp,sol,sys,strucObs,scriptOptions ] = WFAPC_initialize( configName )
% Load configuration file from the 'configurations' folder
run(configName);    

% Set default visualization settings
scriptOptions.plotMesh          = 0;  % Show meshing and turbine locations
scriptOptions.Animate           = 0;  % Show results every x iterations (0: no plots)
   scriptOptions.plotContour    = 0;  % Show flow fields
   scriptOptions.plotPower      = 0;  % Plot true and predicted power capture vs. time
    scriptOptions.powerForecast = 0;  % Plot power forecast (0 = disabled, x = number of steps) (only if plotPower = 1)
   scriptOptions.plotError      = 0;  % plot RMS and maximum error vs. time
   scriptOptions.plotCenterline = 0;  % Plot centerline speed of the wake (m/s)

% Decide whether we need to linearize WFSim
if strcmp(lower(strucObs.filtertype),'exkf') || strcmp(lower(strucObs.filtertype),'smo')
    % Calculate linearized system matrices: necessary for ExKF & SMO
    scriptOptions.Linearversion = true; 
else
    % Disable calculation of lin. matrices: not needed for EnKF & UKF
    scriptOptions.Linearversion = false; 
end

% Check KF settings compatibility
if strcmp(lower(strucObs.filtertype),'sim') == false
    if strucObs.se.enabled == 0 && strucObs.pe.enabled == 0
        error(['Please turn on state and/or parameter estimation. '...
            'Alternatively, select "sim" for open-loop simulations.']);
    end
    if strucObs.measFlow == 0 && strucObs.measPw == 0
        error(['Please turn on flow and/or power measurements. '...
            'Alternatively, select "sim" for open-loop simulations.']);
    end  
end
    
% load a default random seed for consistency
if strucObs.loadRandomSeed; load('randomseed'); rng(randomseed); clear randomseed; end;

% Default settings: following WFSim options are never used in WFObs
scriptOptions.Projection      = 0;    % Use projection
scriptOptions.exportLinearSol = 0;    % Export linear solution
scriptOptions.Derivatives     = 0;    % Calculate derivatives/gradients for system

if scriptOptions.printProgress
    disp([datestr(rem(now,1)) ' __  Initializing simulation model.']);
end;

[Wp,sol,sys] = InitWFSim(Wp,scriptOptions); % Initialize model

% Set default model convergence settings
scriptOptions.conv_eps     = 1e-6; % Convergence parameter
scriptOptions.max_it_dyn   = 1;    % Convergence parameter
if Wp.sim.startUniform
    scriptOptions.max_it = 1;   % Iteration limit for simulation start-up
else
    scriptOptions.max_it = 50;  % Iteration limit for simulation start-up
end

% Define what the system should predict (with or without pressures)
strucObs.size_state = Wp.Nu + Wp.Nv + Wp.Np;
if scriptOptions.exportPressures == 0
    strucObs.size_output = Wp.Nu + Wp.Nv;
else
    strucObs.size_output = Wp.Nu + Wp.Nv + Wp.Np;
end;

% Define measurement locations
if strucObs.measFlow
    sensorsfile        = load(strucObs.sensorsPath);
    strucObs.obs_array = unique([sensorsfile.sensors{1}.obsid; sensorsfile.sensors{2}.obsid]);
    
    % Calculate obs_array locations
    strucObs.obs_array_locu = struct('x',{},'y',{});
    strucObs.obs_array_locv = struct('x',{},'y',{});
    for j = 1:length(strucObs.obs_array)
        [ ~,locSensor,typeFlow ] = WFObs_s_sensors_nr2grid( strucObs.obs_array(j), Wp.mesh);
        if strcmp(typeFlow,'u')
            strucObs.obs_array_locu(end+1) = locSensor;
        else
            strucObs.obs_array_locv(end+1) = locSensor;
        end
    end
else
    strucObs.obs_array = [];
end;

% Create global RCM vector
[~, sysRCM] = WFSim_timestepping( sol, sys, Wp, scriptOptions );
sys.pRCM    = sysRCM.pRCM;

scriptOptions.klen = length(num2str(Wp.sim.NN));        % used for proper spacing in cmd output window
scriptOptions.tlen = length(num2str(Wp.sim.time(end))); % length

if scriptOptions.printProgress
    disp([datestr(rem(now,1)) ' __  Finished initialization sequence.']);
end;

% Load controller stuff ...
end