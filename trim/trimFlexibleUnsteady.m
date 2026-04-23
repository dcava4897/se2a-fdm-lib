function tp_unsteady = trimFlexibleUnsteady(aircraft,controller,fp_spec,model_name)
% trimFlexibleUnsteady trim flexible aircraft with unsteady aerodynamics

%% backup base workspace because we can not use the function workspace
base_workspace_backup = baseWorkspaceBackup();

%% trim with steady aerodynamics first
aircraft = aircraftUnsteady2Steady( aircraft );

% fp_spec.Altitude    = {fp_spec.Altitude};
% fp_spec.EAS         = {fp_spec.EAS};
if ~isfield(fp_spec, 'MassCase')
    fp_spec.MassCase    = {'unknown'};
end
if ~isfield(fp_spec, 'Aircraft')
    fp_spec.Aircraft    = {'unknown'};
end
if ~isfield(fp_spec, 'DefType')
    fp_spec.DefType     = 1; 
end
fp_list             = flightPointListCreate(fp_spec);

trim_spec = fp_list(1).Trim.Spec;

simTaskInitBaseWorkspace(aircraft,0,0);

model_name_steady = 'lin_flexible_steady';

model_description = load_ModelDescriptionLists(model_name_steady);

% trim without downwash delay
trimPoint = trim_flexible_steady(model_name_steady, model_description, trim_spec.Y_Reqts,...
                    trim_spec.Xdot_Reqts, trim_spec.XCont_Var, trim_spec.XDisc_Var, trim_spec.U_Var);

ic = tpGenerateIC(trimPoint);

assignin('base','trimPoint',trimPoint);
assignin('base','ic',ic);

load_system(model_name_steady);

% set model initial state and input
set_param(model_name_steady,'SaveFormat','Array');
set_param(model_name_steady,'LoadInitialState','on','InitialState','trimPoint.State');
set_param(model_name_steady,'LoadExternalInput','on','ExternalInput','[0,trimPoint.Input'']')

% to do: the sim command could be avoided by defining all SimOutputs as
% top level outputs, which should then appear in trimPoint.Outputs
trimPoint.SimOutputs = sim(model_name_steady, 'StopTime', '0');

close_system(model_name_steady,0);

trimPoint = tpSetState(trimPoint,'delay_downwash', ...
    trimPoint.SimOutputs.Gamma_mean );

% at this point the Simulink model lin_flexible_steady_dev can be 
% initialized at the specified trim point

%% convert trim point to unsteady aerodynamics
aircraft = aircraftSteady2Unsteady( aircraft );

simTaskInitBaseWorkspace(aircraft,controller,trimPoint);

model_description = load_ModelDescriptionLists(model_name);

tp_unsteady = convert_trim_s2us(trimPoint, trimPoint.SimOutputs, model_description, aircraft.downwash);

%% restore base workspace
baseWorkspaceRestore(base_workspace_backup);

end
