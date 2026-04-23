function [StatesMDL] = load_StatesMDL(model_name)
%Load states model description list.
%Columns: 1 - Name
%         2 - Unit
%         3 - 'Plottable' name
%         4 - Description
%         5 - Path to block containing state
%
%   This function dynamically constructs the model description list
%   based on the states found in the model. This is done primarily by
%   comparing the 'path' to the blocks to which each state belongs (defined in column 5). 
%   For blocks containing multiple states, either:
%   - The user can list them individually in a cell column array, and
%     the function will separate them into individual states automatically. 
%     In this case, the number listed states must equal the actual states, and 
%     at least the first column must be defined. For columns 2,3, and 4,
%     the entry must either be a) empty, b) contain one value (which is
%     then copied to all the states), or c) defined as the first column,
%     with an equal number of entries. 
%  -  A single entry can be specified. The function will then copy and
%     number the entry by appending '__##', where ## is the number. Note: 
%     the number of digits in this number is not fixed, and no 0's are added.
%     E.g.: eta__1, eta__2, ..., eta__10, eta__11, ..., eta__100, eta__101
%     This numbering is used only for columns 1 and 3. All other columns
%     are simply copied. 
%  If a state is found which is not defined in this function, it will be
%  automatically assigned a 'generic' name, and numbered according to its
%  position in the full state vector, i.e. 'x__##'


% available aircraft subsystem names (library blocks)
aircraft_names = ...
    { ...
    'Rigid aircraft with steady aerodynamics'; ...
    'Rigid aircraft with unsteady aerodynamics'; ...
    'Flexible aircraft with steady aerodynamics'; ...
    'Flexible aircraft with unsteady aerodynamics'; ...
    };

% available environment subsystem names (library blocks)
envir_names = ...
    { ...
    ['Environment',newline,'with successive 1-cos gusts']; ...
    ['Environment',newline,'with single 1-cos gusts']; ...
    ['Environment',newline,'with single external wind input']; ...
    ['Environment',newline,'with single external wind input linear']; ...
    ['Environment',newline,'with zonal external wind input linear'];...
    ['Environment',newline,'with Lidar WindEst'];...
    % add other environment library block description lists here
    };

% available environment subsystem names (library blocks)
controller_names = ...
    { ...
    'INDI Controller' ...
    % add other controller library block description lists here
    };


% find subsystems in Simulink model
aircraft_name   = findSubModel(model_name,aircraft_names,'aircraft');
envir_name      = findSubModel(model_name,envir_names,'environment');
% controller_name = findSubModel(model_name,controller_names,'INDI Controller');

% load parts of the model description lists
aircraft_states = aircraftLoadMdlStates(model_name,aircraft_name);
envir_states    = envirLoadMdlStates(model_name,envir_name);
% controller_states = controllerLoadMdlStates(model_name,controller_name);

% merge aircraft and environment states
cont_state_list = ...
    [ ...
    aircraft_states.Continuous; ...
    envir_states.Continuous; ...
%     controller_states.Continuous; ...
    ];
disc_state_list = ...
    [ ...
    aircraft_states.Discrete; ...
    envir_states.Discrete; ...
%     controller_states.Discrete; ...
    ];

% print info
num_cont = length(cont_state_list);
num_disc = length(disc_state_list);
disp(['load_StatesMDL: Looking for ',num2str(num_cont),' continuous and ', ...
    num2str(num_disc),' discrete state blocks...']);

%% Get list of existing states, compare them to the list above, and build the final MDL

[numStates, ~, statePathList] = feval(model_name,  [], [], [], 'sizes'); % See documentation for 'model' function.
disp(['load_StatesMDL: Found ',num2str(numStates(1)),' continuous and ',num2str(numStates(2)),' discrete states.']);

cont_statePathList = statePathList(1:numStates(1),1); % List of paths for continuous states
disc_statePathList = statePathList(numStates(1)+1:end); % List of paths for discrete states

contStateList_final = mdlConstructState(cont_statePathList, cont_state_list, numStates(1)); 
discStateList_final = mdlConstructState(disc_statePathList, disc_state_list, numStates(2), numStates(1)+1); 

%%

StatesMDL.Continuous = contStateList_final;
StatesMDL.Discrete = discStateList_final;

disp('load_StatesMDL: Finished!');
    
end


function subsystem_name = findSubModel(model_name,subsystem_names,identifier)
subsystem_name = 0;
is_found = false;
for i = 1:length(subsystem_names)
    try
        block_path = find_system([model_name,'/',subsystem_names{i}],'SearchDepth',1,'BlockType','SubSystem');
        subsystem_name = subsystem_names{i};
        is_found = true;
        break;
    catch
        continue;
    end
end
if ~is_found
    warning(['Could not find valid ',identifier,' model.']);
end
end

