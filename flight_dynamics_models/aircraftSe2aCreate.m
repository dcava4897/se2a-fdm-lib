function [aircraft,structure] = aircraftSe2aCreate( varargin )

p = inputParser;

default_n_panel_wing = 40;
default_n_panel_htp = 16;
default_n_panel_vtp = 8;
default_n_seg_fuse = 15;

default_n_modeshapes = 30;

mode_damp = zeros(default_n_modeshapes,1);
% mode_damp(1) = 0.01;
% mode_damp(3) = 0.03;

valid_panel_number = @(x) isnumeric(x) && isscalar(x) && (x>1) && (mod(x,1)==0);

addParameter(p,'cpacsfilename','SE2A_AC_Design_MR_V4_BwdSweep_CPACS2_Turbulent_twist.xml',@ischar);
addParameter(p,'pchfilename','na_Se2A-MR-Ref-v4-twist_GFEM_MTOAa_S103_DMIG.pch',@ischar);
addParameter(p,'gridfoldername','GRID_SE2A_MR_BWD_swept_V4_twist',@ischar);
addParameter(p,'ncssfilename','',@ischar); % NeoCASS file containing, at a minimum, the beam_model struct
addParameter(p,'actuatorsfilename','actuator_dynamics_params_se2a',@ischar);
addParameter(p,'external_structure',false);
addParameter(p,'mode_damp',mode_damp);

addParameter(p,'numpanelwing',default_n_panel_wing,valid_panel_number);
addParameter(p,'numpanelhtp',default_n_panel_htp,valid_panel_number);
addParameter(p,'numpanelvtp',default_n_panel_vtp,valid_panel_number);
addParameter(p,'numsegfuse',default_n_seg_fuse,valid_panel_number);

addParameter(p,'nummodeshapes',default_n_modeshapes,valid_panel_number);

addParameter(p,'ControlsMainFile','wingControls_params_mainDefault');
addParameter(p,'ControlsHtpFile','wingControls_params_htpDefault');
addParameter(p,'ControlsVtpFile','wingControls_params_vtpDefault');

addParameter(p,'unsteady',false,@islogical);
addParameter(p,'flexible',false,@islogical);
addParameter(p,'stall',false,@islogical);
addParameter(p,'le_shock',false,@islogical);

addParameter(p,'Mach',0);
addParameter(p,'Alt',0);

addParameter(p,'AdjustJigTwist',true,@islogical);



parse(p,varargin{:})

%% get handles from TiXI and TiGL
% tixiHandle = tixiOpenDocumentTry( ... 
%     which ( p.Results.cpacsfilename ) );
% tiglHandle = tiglOpenCPACSConfigurationTry( tixiHandle );

% Replace tixi/tigl with mtigl:
cpacs_struct = xml2struct(p.Results.cpacsfilename);
tiglHandle = mtigl(cpacs_struct.cpacs);

%% load structure 
axis_reversed = [ -1; 1; -1 ];
if isstruct(p.Results.external_structure) %External
    structure = p.Results.external_structure;
elseif ~isempty(p.Results.ncssfilename) %From NeoCASS data
    structure = ncssStructureCreate(p.Results.ncssfilename, axis_reversed);
else % from NASTRAN file
    structure = structureCreateFromNastran( ...
        p.Results.pchfilename,p.Results.gridfoldername,axis_reversed);
end

%% init rigid body
body.m = structureGetTotalMass( structure );
body.I = structureGetTotalInertia( structure );

%% flexible aircraft
if p.Results.flexible
    % reduce order of structure
    structure_red = structureGetReduced( structure, p.Results.nummodeshapes );
    structure_red = structureSetDamp( structure_red, p.Results.mode_damp );
end

%% configuration parameters
% center of gravity in body-fixed c frame (NED located at the nose)
config.xyz_cg_c = structureGetCg( structure );
% reference point of rigid body state
config.xyz_ref_c = config.xyz_cg_c;
if p.Results.flexible
    % transformation matrix from structure state to reference point
    config.T_ref_s = structureGetRefPointTrafo(structure_red,config.xyz_ref_c);
else
    config.T_ref_s = zeros( 6, 1 );
end

Delta_jig_twist = getDeltaJigTwist(p.Results.pchfilename);

%% aerodynamics
if p.Results.flexible
    wing_main = wingCreateWithCPACS( tiglHandle, 1, p.Results.numpanelwing, 'spacing', 'like_chord', 'airfoil_method', 'analytic', 'is_unsteady', p.Results.unsteady, 'stall', p.Results.stall, 'le_shock', p.Results.le_shock, 'flexible', structure_red, 'ControlsFilename',p.Results.ControlsMainFile, 'Mach', p.Results.Mach, 'Alt', p.Results.Alt, 'AdjustJigTwist', Delta_jig_twist );
    wing_htp = wingCreateWithCPACS( tiglHandle, 2, p.Results.numpanelhtp, 'spacing', 'like_chord', 'airfoil_method', 'analytic', 'is_unsteady', p.Results.unsteady, 'stall', p.Results.stall, 'le_shock', p.Results.le_shock, 'flexible', structure_red, 'ControlsFilename',p.Results.ControlsHtpFile, 'Mach', p.Results.Mach, 'Alt', p.Results.Alt );
    wing_vtp = wingCreateWithCPACS( tiglHandle, 3, p.Results.numpanelvtp, 'spacing', 'like_chord', 'airfoil_method', 'analytic', 'is_unsteady', p.Results.unsteady, 'stall', p.Results.stall, 'le_shock', p.Results.le_shock, 'flexible', structure_red, 'ControlsFilename',p.Results.ControlsVtpFile, 'Mach', p.Results.Mach, 'Alt', p.Results.Alt );
    if p.Results.unsteady
        fuselage = fuselageCreateWithCpacs( tiglHandle, 'Fuse', axis_reversed, p.Results.numsegfuse, 'flexible', structure_red, 'unsteady' );
    else
        fuselage = fuselageCreateWithCpacs( tiglHandle, 'Fuse', axis_reversed, p.Results.numsegfuse, 'flexible', structure_red );
    end
else
    wing_main = wingCreateWithCPACS( tiglHandle, 1, p.Results.numpanelwing, 'spacing', 'like_chord', 'airfoil_method', 'analytic', 'is_unsteady', p.Results.unsteady, 'stall', p.Results.stall, 'le_shock', p.Results.le_shock, 'ControlsFilename',p.Results.ControlsMainFile, 'Mach', p.Results.Mach );
    wing_htp = wingCreateWithCPACS( tiglHandle, 2, p.Results.numpanelhtp, 'spacing', 'like_chord', 'airfoil_method', 'analytic', 'is_unsteady', p.Results.unsteady, 'stall', p.Results.stall, 'le_shock', p.Results.le_shock, 'ControlsFilename',p.Results.ControlsHtpFile, 'Mach', p.Results.Mach );
    wing_vtp = wingCreateWithCPACS( tiglHandle, 3, p.Results.numpanelvtp, 'spacing', 'like_chord', 'airfoil_method', 'analytic', 'is_unsteady', p.Results.unsteady, 'stall', p.Results.stall, 'le_shock', p.Results.le_shock, 'ControlsFilename',p.Results.ControlsVtpFile, 'Mach', p.Results.Mach );
    if p.Results.unsteady
        fuselage = fuselageCreateWithCpacs( tiglHandle, 'Fuse', axis_reversed, p.Results.numsegfuse, 'unsteady' );
    else
        fuselage = fuselageCreateWithCpacs( tiglHandle, 'Fuse', axis_reversed, p.Results.numsegfuse );
    end
end

%% wing downwash
downwash = downwashUnstFromVlmWings( wing_main, wing_htp, 50 );

%% engines
if p.Results.flexible
    engines = enginesCreate( p.Results.gridfoldername, structure_red, axis_reversed );
end

%% actuator dynamics
act = loadParams(p.Results.actuatorsfilename);

%% assign
aircraft.body = body;
aircraft.wing_main = wing_main;
aircraft.wing_htp = wing_htp;
aircraft.wing_vtp = wing_vtp;
aircraft.fuselage = fuselage;
if p.Results.flexible
    aircraft.eom_flexible.structure_red = structure_red;
    aircraft.eom_flexible.node_mass = structureGetNodeMass(structure,1:length(structure.idx_node_struct));
    aircraft.eom_flexible.cg_nodes = structureGetNodeCg(structure,1:length(structure.idx_node_struct));
    aircraft.eom_flexible.body = body;
else
    aircraft.eom_rigid = body;
end
aircraft.downwash = downwash;
if p.Results.flexible
    aircraft.engines = engines;
end
aircraft.actuators = act;
aircraft.config = config;

aircraft.vtp_sideslip_factor = 1.7;

%% Outputs

% Loads
% aircraft.outputs.loads = 


end

function Delta_jig_twist = getDeltaJigTwist(pchfilename)

Delta_jig_twist = zeros( 2, 40 );
Delta_jig_twist(1,:) = [ -21.3280  -20.7444  -20.1254  -19.4686  -18.7715  -18.0317  -17.2465  -16.4131  -15.5287  -14.5904  -13.5954  -12.5404  -11.4219  -10.2359   -8.9786   -7.6455   -6.2302   -4.6851   -2.9566   -1.0218    1.0218    2.9566    4.6851    6.2302    7.6455    8.9786   10.2359   11.4219   12.5404   13.5954   14.5904   15.5287   16.4131   17.2465   18.0317   18.7715   19.4686   20.1254   20.7444   21.3280 ];
if contains(pchfilename,'na_Se2A-MR-Ref-v4-twist_GFEM')
    Delta_jig_twist(2,:) = [ 0.0582    0.0582    0.0577    0.0570    0.0559    0.0545    0.0525    0.0499    0.0465    0.0424    0.0378    0.0327    0.0273    0.0215    0.0158    0.0100    0.0054    0.0016   -0.0004   -0.0016   -0.0016   -0.0004    0.0015    0.0054    0.0100    0.0158    0.0216    0.0274    0.0328    0.0379    0.0425    0.0467    0.0502    0.0529    0.0550    0.0564    0.0576    0.0583    0.0588    0.0588 ];
elseif contains(pchfilename,'na_Se2A-MR-Ref-v4-twist_25g_GFEM')
    Delta_jig_twist(2,:) = [ 0.0454    0.0454    0.0450    0.0442    0.0431    0.0417    0.0397    0.0372    0.0344    0.0317    0.0296    0.0270    0.0230    0.0182    0.0135    0.0088    0.0050    0.0017   -0.0002   -0.0014   -0.0014   -0.0002    0.0017    0.0050    0.0088    0.0135    0.0182    0.0230    0.0271    0.0297    0.0318    0.0345    0.0374    0.0400    0.0421    0.0436    0.0447    0.0455    0.0460    0.0460 ];
else
    warning('There is no data for jig twist adjustment.')
end

end