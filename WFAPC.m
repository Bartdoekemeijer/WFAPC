classdef WFAPC<handle
    properties
        Wp
        sol
        sys
        strucObs
        scriptOptions
    end
    methods
        %% Constructor function initializes default inputData
        function self = WFAPC(configName)
            
            addpath('bin'); % Model functions
            addpath('configurations'); % Add configurations folder
            addpath('WFObs'); % Estimator
            run('WFObs/WFObs_addpaths.m'); % Import libraries for WFObs & WFSim
                                 
            % Initialize WFSim, WFObs, and Controller
            [ self.Wp,self.sol,self.sys,self.strucObs,self.scriptOptions] = WFAPC_initialize( configName );
        end
        
                
        %% WFObs estimation cycle
        function [self] = estimate(self,measuredData)
            % measuredData format:
            % measuredData is a struct() containing:
            %   *.uq = [Nx x Ny elements, true flow field u]
            %   *.vq = [Nx x Ny elements, true flow field v]
            %   *.u  = [Nx x Ny elements, perturbed flow field u]
            %   *.v  = [Nx x Ny elements, perturbed flow field v]
            %   *.power = [NT x 1 elements, disturbed power signal]
            
            Wp            = self.Wp;
            sol           = self.sol;
            sys           = self.sys;
            strucObs      = self.strucObs;
            scriptOptions = self.scriptOptions;
            
            try
                measuredData.solq  = [vec(measuredData.uq(3:end-1,2:end-1)'); ... % True state vector
                                      vec(measuredData.vq(2:end-1,3:end-1)')];
                measuredData.sol   = [vec(measuredData.u(3:end-1,2:end-1)') ; ... % Dist. state vector
                                      vec(measuredData.v(2:end-1,3:end-1)') ];
            catch
                [measuredData.sol,measuredData.solq] = deal(zeros(strucObs.size_output,1));
            end
            
            % Start timer
            timerCPU = tic();
            
            % Forward timestep
            sol.k    = sol.k + 1;           % Timestep forward
            sol.time = Wp.sim.time(sol.k+1);% Timestep forward
    
            % Load measurements from input
            if nargin > 1
                sol.measuredData = measuredData;
            end
                       
            % Determine freestream inflow properties from SCADA data
            [ Wp,sol,sys,strucObs ] = WFObs_s_freestream(Wp,sol,sys,strucObs);

            % Calculate optimal solution according to filter of choice
            [Wp,sol,strucObs] = WFObs_o(strucObs,Wp,sys,sol,scriptOptions);

            % Display progress in the command window
            sol = WFObs_s_reporting(timerCPU,Wp,sol,strucObs,scriptOptions);

            self.Wp       = Wp;
            self.sol      = sol;
            self.sys      = sys;
            self.strucObs = strucObs;
        end
        
        
        %% Control optimization
        function [self] = optimize(self)            
            % Load variables
            Wp            = self.Wp;
            sol           = self.sol;
            scriptOptions = self.scriptOptions;
            
            % -- do optimization
            
            % Write variables back to self
            self.Wp       = Wp;
            self.sol      = sol;
        end

    end
end