function [trimPoint] = trim_flexible_steady(modelName, model_description, varargin) 
%% Trim the aircraft to specifications. 
% 2 input modes: 
%   trim_flexible_steady(modelName, model_description, Y_Reqts, Xdot_Reqts, XCont_Var, XDisc_Var, U_Var, trimPointPre)
%       Trim reqts and variables are given as separate cell arrays, each of
%       which has the variable names in column 1 and the initial
%       values/target values in column 2, or:
%   trim_flexible_steady(modelName, model_description, trimTask, trimPointPre)
%       Trim reqts and variables are contained in trimTask, a struct in
%       whic each field corresponds to the cell arrays specified above, and
%       initial conditions are taken from trimPointPre, a previous
%       trimPoint (optional)

%% Process input
if numel(varargin)>=5
    Y_Reqts = varargin{1}; 
    Xdot_Reqts = varargin{2};
    XCont_Var = varargin{3};
    XDisc_Var = varargin{4};
    U_Var = varargin{5};
elseif numel(varargin)>=1 && isstruct(varargin{1})
    trimTask = varargin{1};
    
    Y_Reqts = trimTask.Y_Reqts;
    Xdot_Reqts = trimTask.Xdot_Reqts; 
    XCont_Var = trimTask.XCont_Var;
    XDisc_Var = trimTask.XDisc_Var;
    U_Var = trimTask.U_Var;
else
    error('Trim function inputs defined incorrectly');
end

if ismember(numel(varargin), [2,6])
    trimPointPre = varargin{end};
    b_retrim = true;
else
    b_retrim = false;
end

%% Remove external input and initial states
load_system(modelName);
set_param(modelName,'LoadInitialState','off');
set_param(modelName,'LoadExternalInput','off');


%% Obtain initial state and model sizes:

[modelSizes, x_0] = feval(modelName,  [], [], [], 'sizes'); % See documentation for 'model' function.

numContStates   = modelSizes(1);
numDiscStates   = modelSizes(2);
numStates       = modelSizes(1) + modelSizes(2);
numOutputs      = modelSizes(3);
numInputs       = modelSizes(4);

%% Specify trim problem:
% Specifications are expressed as a cell array with two columns, in which
% the left column contains the variable name (as defined in the model
% description lists) and the right column contains the value.

% If the first character is '_', it is treated as a 'numbered' variable,
% i.e. (potentially) more than one variable begins with that variable name, 
% and is followed by two underscores ('__') and numbers. Note that the 
% trailing underscores should NOT be included in the spec. This is particularly useful for
% specifying, for example, flexible modes. If individual members of this
% set are also specified individually, e.g. 'eta__11', the individual
% specification will take priority. 

% Trim requirements:

% Y_Reqts = {... %Requirements on outputs
%     };
% 
% %   Requirements on state derivatives. The names used to specify state
% %   derivatives should be the same as state names. '_dot' is automatically appended afterwards. 
% %   NB: Only continuous states can be specified in Xdot.
% Xdot_Reqts = {...      
%     };
% 
% 
% % Trim variables:
% % Values assigned to trim variables are treated as 'initial guesses'. There
% % must be AT LEAST as many trim variables as requirements.
% XCont_Var = {...}
%
% 
% XDisc_Var = {...
%     };
% 
% U_Var = {...
%     };

%% Process the trim specifications
Y_Reqts    = Process_TrimSpec(Y_Reqts, model_description.MDL_Outputs);
Xdot_Reqts = Process_TrimSpec(Xdot_Reqts, model_description.States.Continuous);

XCont_Var = Process_TrimSpec(XCont_Var, model_description.States.Continuous);
XDisc_Var = Process_TrimSpec(XDisc_Var, model_description.States.Discrete);
U_Var     = Process_TrimSpec(U_Var, model_description.MDL_Inputs);

%% Check number of trim requirements vs variables
numReqts = size(Y_Reqts,1) + size(Xdot_Reqts,1);
numVar   = size(XCont_Var,1) + size(XDisc_Var,1) + size(U_Var, 1);

if (numVar - numReqts) ~= 0
    error('Number of trim variables/trim requirements do not match!');
end

%% Build up full vectors
%Find indices in model description lists
if ~isempty(Y_Reqts)
    Y_Reqts_Idxs    = dl2idx(model_description.MDL_Outputs, Y_Reqts(:,1)); 
else
    Y_Reqts_Idxs    = [];
end

if ~isempty(Xdot_Reqts)
    Xdot_Reqts_Idxs = dl2idx(model_description.States.Continuous, Xdot_Reqts(:,1));
else
    Xdot_Reqts_Idxs = [];
end

if ~isempty(XCont_Var)
    XCont_Var_Idxs = dl2idx(model_description.States.Continuous,XCont_Var(:,1));
else
    XCont_Var_Idxs = [];
end
% XDisc_Var_Idxs = dl2idx(model_description.States.Discrete, XDisc_Var(:,1));
XDisc_Var_Idxs = []; % Nothing has been specified for discrete states, so this is left empty.
X_Var_Idxs     = [XCont_Var_Idxs; (XDisc_Var_Idxs + numContStates)]; % Combined continuous and discrete

if ~isempty(U_Var)
    U_Var_Idxs     = dl2idx(model_description.MDL_Inputs, U_Var(:,1));
else
    U_Var_Idxs     = [];
end


Y_Names     = model_description.MDL_Outputs(:,1);
Xdot_Names  = strcat(model_description.States.Continuous(:,1), '_dot');
X_Names     = [model_description.States.Continuous(:,1); model_description.States.Discrete(:,1)];
U_Names     = model_description.MDL_Inputs(:,1);

%   Values of outputs/state derivatives which are not specified in the 
%   requirements are not used, so set to zero by default
y_0 = zeros(numOutputs,1);
xdot_0 = zeros(numContStates,1);

u_0 = zeros(numInputs,1); % Inputs are assumed set to 0 unless used for trimming.

if b_retrim
    % Override initial values with trimPoint:
    [x_0, xdot_0, y_0, u_0] = Extract_InitTrimPoint(model_description, trimPointPre);
else
    if ~isempty(U_Var)
        u_0(U_Var_Idxs)         = cell2mat(U_Var(:,2));
    end
    if ~isempty(XCont_Var)
        x_0(XCont_Var_Idxs)     = cell2mat(XCont_Var(:,2));
    end
end

if ~isempty(Y_Reqts)
    y_0(Y_Reqts_Idxs)       = cell2mat(Y_Reqts(:,2));
end
if ~isempty(Xdot_Reqts)
    xdot_0(Xdot_Reqts_Idxs) = cell2mat(Xdot_Reqts(:,2)); 
end

% x_0(XDisc_Var_Idxs + numContStates) = cell2mat(XDisc_Var(:,2));

% x_0 = converge_unsteadyaero(x_0, u_0, model_description);

%% Run trim
trim_opts = [ 300; 1e-10 ];%[ 300; 2e-13 ];

[x_tr, u_tr, d_tr, y_tr, trim_fail] = jj_trim(modelName, x_0, u_0, xdot_0, y_0, X_Var_Idxs, U_Var_Idxs, Xdot_Reqts_Idxs, Y_Reqts_Idxs, ...
        X_Names, U_Names, Xdot_Names, Y_Names, [],[],1e-6*(1 + abs(x_0)),1e-6*(1 + abs(u_0)),trim_opts);
    
if trim_fail
    warning('Trim unsuccessful!');
else
    % close jj_trim message box
    hv_figure_all = findall(0, 'Type', 'Figure');
    for i = 1:length(hv_figure_all)
        if isequal(hv_figure_all(i).Tag,'Msgbox_Success')
            delete(hv_figure_all(i));
        end
    end
end


% assignin('base', 'x_tr_orig', x_tr);
% x_tr_rc = converge_unsteadyaero(x_tr, u_tr, model_description);
% delta_x = sum(abs(x_tr - x_tr_rc))
% x_tr = x_tr_rc; 


%% Set up trim point

trimPoint.State = x_tr;
trimPoint.Input = u_tr;
trimPoint.Output = y_tr;
trimPoint.Deriv = d_tr;

trimPoint.mdl_names = mdlGetMdlNames(model_description);


end

function trimSpec_out = Process_TrimSpec(trimSpec_in, model_desc_sub)
       
    if isempty(trimSpec_in)
        numberedVarIdx = [];
    else   
        numberedVarIdx = find(strncmp('_', trimSpec_in(:,1),1)); % Find 
    end
    
    trimSpec_out = trimSpec_in;
    trimSpec_tmp = []; % To contain rows to be added to trimSpec
    
    rowsToBeDeleted = []; %Array of rows to be removed from trimSpec_out later on. 
    
    for in = 1:size(numberedVarIdx,1) % Loop through the specified 'numbered variables' which have been found
        nVarName  = trimSpec_in{numberedVarIdx(in),1}; % 
        nVarValue = trimSpec_in{numberedVarIdx(in),2};
        
        nVarMDLIdxs = dl2idx_single(model_desc_sub, [nVarName(2:end), '__'],1);
        
        rowsToBeDeleted = [rowsToBeDeleted; numberedVarIdx(in)];
        
        for jn = 1:size(nVarMDLIdxs,1) % Loop through the set of variables connected to the 'numbered variable'
            varName_tmp = model_desc_sub{nVarMDLIdxs(jn), 1};
            
            indSpecVarIdx = find(ismember(trimSpec_in(:,1), varName_tmp)); % Has this specific variable name been individually specified?
            
            if isempty(indSpecVarIdx) %It has not been individually specified
                trimSpec_tmp = [trimSpec_tmp; {varName_tmp, nVarValue}]; % Add a row with the 'generic' value
            else
                trimSpec_tmp = [trimSpec_tmp; {varName_tmp, trimSpec_in{indSpecVarIdx,2}}]; % Add a row with the individually specified value
                
                rowsToBeDeleted = [rowsToBeDeleted; indSpecVarIdx]; % Make sure the individually specified row will be removed
            end
        end
                       
    end
    
    trimSpec_out(rowsToBeDeleted,:) = [];
    trimSpec_out = [trimSpec_out; trimSpec_tmp];

end


function [x_0, xdot_0, y_0, u_0] = Extract_InitTrimPoint(model_description, trimPoint)
    x_tmp = trimPoint.State; x_0 = zeros(size(x_tmp));
    xdot_tmp = trimPoint.Deriv; xdot_0 = zeros(size(xdot_tmp));
    y_tmp = trimPoint.Output; y_0 = zeros(size(y_tmp));
    u_tmp = trimPoint.Input; u_0 = zeros(size(u_tmp));
    
    cont_names_tp = trimPoint.mdl_names.States.Continuous;
    state_names_tp = [cont_names_tp; trimPoint.mdl_names.States.Discrete];
    out_names_tp = trimPoint.mdl_names.MDL_Outputs;
    in_names_tp = trimPoint.mdl_names.MDL_Inputs;
    
    out_names_mdl   = model_description.MDL_Outputs(:,1);
    cont_names_mdl  = model_description.States.Continuous(:,1);
    state_names_mdl = [model_description.States.Continuous(:,1); model_description.States.Discrete(:,1)];
    in_names_mdl    = model_description.MDL_Inputs(:,1);
    
    for ii = 1:length(x_tmp)
        idx_tmp = dl2idx(state_names_mdl, state_names_tp(ii));
        x_0(idx_tmp) = x_tmp(ii);
    end
    
    for ii = 1:length(xdot_tmp)
        idx_tmp = dl2idx(cont_names_mdl, cont_names_tp(ii));
        xdot_0(idx_tmp) = xdot_tmp(ii);
    end
    
    for ii = 1:length(y_tmp)
        idx_tmp = dl2idx(out_names_mdl, out_names_tp(ii));
        y_0(idx_tmp) = y_tmp(ii);
    end
    
    for ii = 1:length(u_tmp)
        idx_tmp = dl2idx(in_names_mdl, in_names_tp(ii));
        u_0(idx_tmp) = u_tmp(ii);
    end
end

