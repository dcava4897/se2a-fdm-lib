function list_fp = flightPointListCreate(spec_fp)

% Conditions the list of flight points, trims aircraft at them, and returns the list of flight
% points and trim points

%% Condition flight point list and prepare the sweep
list_fp = [];

for i_fp_set = 1:length(spec_fp)
    spec_fp_tmp = spec_fp(i_fp_set);
    spec_fp_tmp = Reshape_SpecFP(spec_fp_tmp);
    % Complete the EAS/Mach list
    list_fp_EAS = [];
    list_fp_Mach = [];

    % Sweep 1: EAS
    if isfield(spec_fp_tmp,'EAS') && ~isempty(spec_fp_tmp.EAS)
        sweepFP_tmp = {'EAS' ,         spec_fp_tmp.EAS
                       'Altitude',     spec_fp_tmp.Altitude  
                       'MassCase',     spec_fp_tmp.MassCase
                       'Aircraft',     spec_fp_tmp.Aircraft
                       };

        list_fp_EAS = Assemble_FlightPointTrimSweep(sweepFP_tmp, spec_fp_tmp.DefType);

        sweepTrim_fp  = {'Y_Reqts',     'z_Kg',      -cell2mat(spec_fp_tmp.Altitude) 
                          'Y_Reqts',     'V_EAS',    cell2mat(spec_fp_tmp.EAS) 
                          'Y_Reqts',     'w_Kg',      0
                          'Xdot_Reqts',  'u_Kb',      0
                          'Xdot_Reqts',  'w_Kb',      0
                          'Xdot_Reqts',  'q_Kb',      0
                          'Xdot_Reqts',  '_eta_dt',      0
                          'Xdot_Reqts',  '_alpha_fus',0
                          'Xdot_Reqts',  'dHTP',      0
                          'Xdot_Reqts',  'dHTP_dt',      0
                          'Xdot_Reqts',  'thrust',      0
                          'XCont_Var',  'z__g',      [0]
                          'XCont_Var',  'u_Kb',      150
                          'XCont_Var',  'w_Kb',      0
                          'XCont_Var',  '_eta',      0
                          'XCont_Var',  'qt_2',      0
                          'XCont_Var',  '_alpha_fus',      0
                          'XCont_Var',  'dHTP',      0
                          'XCont_Var',  'dHTP_dt',      0
                          'XCont_Var',  'thrust',      1e5
                          'U_Var',      'thrust_cmd',      1e5
                          'U_Var',      'de_htp_cmd',      0};

        list_trim_tasks =  Assemble_TrimSweep(sweepTrim_fp, spec_fp_tmp.DefType);

        for ii_fp = 1:length(list_fp_EAS)
            for  jj = 1:length(list_trim_tasks)
                b_alt = (abs(list_fp_EAS(ii_fp).Altitude) == abs(list_trim_tasks{jj}.Y_Reqts{1,2}));
                b_spd = (abs(list_fp_EAS(ii_fp).EAS) == abs(list_trim_tasks{jj}.Y_Reqts{2,2}));

                if b_alt && b_spd
                    list_fp_EAS(ii_fp).Trim.Spec = list_trim_tasks{jj};
                end
            end

            %Calculate Mach/TAS
            atm_tmp = isAtmosphere(list_fp_EAS(ii_fp).Altitude);
            list_fp_EAS(ii_fp).TAS = sqrt(1.225/atm_tmp.rho)*list_fp_EAS(ii_fp).EAS;
            list_fp_EAS(ii_fp).Mach = list_fp_EAS(ii_fp).TAS/atm_tmp.a;
        end

    end

    % Sweep 2: Mach
    if isfield(spec_fp_tmp,'Mach') && ~isempty(spec_fp_tmp.Mach)
        sweepFP_tmp = {'Mach',         spec_fp_tmp.Mach
                       'Altitude',     spec_fp_tmp.Altitude  
                       'MassCase',     spec_fp_tmp.MassCase
                       'Aircraft',     spec_fp_tmp.Aircraft
                       };

        list_fp_Mach = Assemble_FlightPointTrimSweep(sweepFP_tmp, spec_fp_tmp.DefType);

        sweepTrim_fp  = {'Y_Reqts',     'z_Kg',      -cell2mat(spec_fp_tmp.Altitude) 
                          'Y_Reqts',     'Mach',    cell2mat(spec_fp_tmp.Mach) 
                          'Y_Reqts',     'w_Kg',      0
                          'Xdot_Reqts',  'u_Kb',      0
                          'Xdot_Reqts',  'w_Kb',      0
                          'Xdot_Reqts',  'q_Kb',      0
                          'Xdot_Reqts',  '_eta_dt',      0
                          'Xdot_Reqts',  '_alpha_fus',0
                          'Xdot_Reqts',  'dHTP',      0
                          'Xdot_Reqts',  'dHTP_dt',      0
                          'Xdot_Reqts',  'thrust',      0
                          'XCont_Var',  'z__g',      [0]
                          'XCont_Var',  'u_Kb',      150
                          'XCont_Var',  'w_Kb',      0
                          'XCont_Var',  '_eta',      0
                          'XCont_Var',  'qt_2',      0
                          'XCont_Var',  '_alpha_fus',      0
                          'XCont_Var',  'dHTP',      0
                          'XCont_Var',  'dHTP_dt',      0
                          'XCont_Var',  'thrust',      1e5
                          'U_Var',      'thrust_cmd',  1e5
                          'U_Var',      'de_htp_cmd',      0};

        list_trim_tasks =  Assemble_TrimSweep(sweepTrim_fp, spec_fp_tmp.DefType);

        for ii_fp = 1:length(list_fp_Mach)
            for  jj = 1:length(list_trim_tasks)
                b_alt = (abs(list_fp_Mach(ii_fp).Altitude) == abs(list_trim_tasks{jj}.Y_Reqts{1,2}));
                b_spd = (abs(list_fp_Mach(ii_fp).Mach) == abs(list_trim_tasks{jj}.Y_Reqts{2,2}));

                if b_alt && b_spd
                    list_fp_Mach(ii_fp).Trim.Spec = list_trim_tasks{jj};
                end
            end

            %Calculate EAS/TAS
            atm_tmp = isAtmosphere(list_fp_Mach(ii_fp).Altitude);
            list_fp_Mach(ii_fp).TAS = atm_tmp.a*list_fp_Mach(ii_fp).Mach;
            list_fp_Mach(ii_fp).EAS = sqrt(atm_tmp.rho/1.225)*list_fp_Mach(ii_fp).TAS;
        end

    end

    list_fp = [list_fp, list_fp_EAS, list_fp_Mach];
end

end

%% Local Functions

function listOfTasks = Assemble_TrimSweep(sweepData, sweepType)

    sweepCat      = sweepData(:,1);
    sweepNames    = sweepData(:,2);
    sweepValues   = sweepData(:,3);
    sweepSize     = length(sweepNames);

    for jj = 1:sweepSize
        sweepValuesSize(jj,1) = length(sweepValues{jj,1}); 
    end

    if (sweepType == 2) 
        n_val = numel(unique(sweepValuesSize(sweepValuesSize~=1)));
        if (n_val>1) || (n_val==1 && numel(sweepValuesSize(sweepValuesSize>1))<2) %(sum(sweepValuesSize) ~= sweepValuesSize(1)*sweepSize) )
            error('For parallel sweeps, all value vectors must be the same length');
        elseif n_val == 1 %Replicate all values with dimension 1 (if the parallel sweep dimension> 1)
            for i_rm = 1:sweepSize
                if sweepValuesSize(i_rm, 1) == 1
                    sweepValues{i_rm, 1} = repmat(sweepValues{i_rm, 1}, 1, max(sweepValuesSize));
                end
            end
        end
    end

    listOfTasks={};
    taskIdx = 0;
    % Sweep engine
    sweepComplete = false;

    sweepIdxs = ones(sweepSize,1); 
    sweepPoint = zeros(sweepSize,1);

    while ~sweepComplete
        taskIdx = taskIdx + 1;
        % Assign values to variables

        listOfTasks{taskIdx}.Y_Reqts    = {};
        listOfTasks{taskIdx}.Xdot_Reqts = {};
        listOfTasks{taskIdx}.XCont_Var  = {};
        listOfTasks{taskIdx}.XDisc_Var  = {};
        listOfTasks{taskIdx}.U_Var      = {};       

        for jj = 1:sweepSize
    %             sweepPoint(ii,1) = sweepValues{ii,1}(sweepIdxs(ii));
            listOfTasks{taskIdx}.(sweepCat{jj})(end+1,:) = {sweepNames{jj}, sweepValues{jj,1}(sweepIdxs(jj))};
        end

        %Update the index vector
        for updateRow = 1:sweepSize

            sweepIdxs(updateRow) = sweepIdxs(updateRow)+1;
            if sweepIdxs(updateRow)>sweepValuesSize(updateRow) %Index has reached max
                sweepIdxs(updateRow) = 1; %Reset to 1, and allow loop to update next value
            else
                if sweepType == 1
                    break; %Update complete, break the update loop
                end
            end

        end

        %Sweep is complete when all indices have been reset to 1
        if sum(sweepIdxs) == sweepSize
            sweepComplete = true;
        end
    end

end

function listOfFlightPtSpecs = Assemble_FlightPointTrimSweep(sweepData, sweepType)

%     sweepCat      = sweepData(:,1);
    sweepNames    = sweepData(:,1);
    sweepValues   = sweepData(:,2);
    sweepSize     = length(sweepNames);

    for jj = 1:sweepSize
        sweepValuesSize(jj,1) = length(sweepValues{jj,1}); 
    end

    if (sweepType == 2) && (sum(sweepValuesSize) ~= sweepValuesSize(1)*sweepSize)
        error('For parallel sweeps, all value vectors must be the same length');
    end

    listOfFlightPtSpecs={};
    taskIdx = 0;
    % Sweep engine
    sweepComplete = false;

    sweepIdxs = ones(sweepSize,1); 
    sweepPoint = zeros(sweepSize,1);

    while ~sweepComplete
        taskIdx = taskIdx + 1;
        % Assign values to variables

%         listOfFlightPtSpecs{taskIdx}.Y_Reqts    = {};
%         listOfFlightPtSpecs{taskIdx}.Xdot_Reqts = {};
%         listOfFlightPtSpecs{taskIdx}.XCont_Var  = {};
%         listOfFlightPtSpecs{taskIdx}.XDisc_Var  = {};
%         listOfFlightPtSpecs{taskIdx}.U_Var      = {};
%         listOfFlightPtSpecs(end+1,:) = [];

        for jj = 1:sweepSize
    %             sweepPoint(ii,1) = sweepValues{ii,1}(sweepIdxs(ii));
            listOfFlightPtSpecs(taskIdx).(sweepNames{jj}) = sweepValues{jj,1}{sweepIdxs(jj)};
        end

        %Update the index vector
        for updateRow = 1:sweepSize

            sweepIdxs(updateRow) = sweepIdxs(updateRow)+1;
            if sweepIdxs(updateRow)>sweepValuesSize(updateRow) %Index has reached max
                sweepIdxs(updateRow) = 1; %Reset to 1, and allow loop to update next value
            else
                if sweepType == 1
                    break; %Update complete, break the update loop
                end
            end

        end

        %Sweep is complete when all indices have been reset to 1
        if sum(sweepIdxs) == sweepSize
            sweepComplete = true;
        end
    end

end


function trim_point = trimPointSetStaticDownwash( trim_point, model_description, Gamma_trim )

    fnames = model_description.States.Continuous(:,1);

    trim_point.IndvStates.delay_downwash = Gamma_trim; %w_trim;
    trim_point.State(find(startsWith(fnames, 'delay_downwash'))) = Gamma_trim; %w_trim;

end

function spec_fp_tmp = Reshape_SpecFP(spec_fp_tmp)
    %Reshapes FP spec, s.t. numeric values are stored as a cell row array 
    fn = fieldnames(spec_fp_tmp);
    for ii = 1:length(fn)

        if iscell(spec_fp_tmp.(fn{ii}))
        % Case 1: if it is stored as a numeric vector in a cell, convert it to
        % a cell row array
            c2m_tmp = cell2mat(spec_fp_tmp.(fn{ii}));
            if isnumeric(c2m_tmp)
                spec_fp_tmp.(fn{ii}) = num2cell(c2m_tmp);
            end
        elseif isnumeric(spec_fp_tmp.(fn{ii})) && ismember(fn{ii}, {'Mach', 'EAS', 'Altitude'})
        % Case 2: if it is just a numeric array (or scalar), and is
        % supposed to be a cell row array, store it in a cell row array
            spec_fp_tmp.(fn{ii}) = num2cell(spec_fp_tmp.(fn{ii}));
        
        elseif (ischar(spec_fp_tmp.(fn{ii})) || isstring(spec_fp_tmp.(fn{ii})))...
                && ismember(fn{ii}, {'Aircraft', 'MassCase'})
        % Case 3: if it is just a char/string row or array, and is
        % supposed to be a cell row array, store it in a cell row array    
            spec_fp_tmp.(fn{ii}) = cellstr(spec_fp_tmp.(fn{ii}));
        end
            
        
    end
        
end
