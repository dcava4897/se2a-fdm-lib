function [structure] = ncssStructureCreate(beam_model_filename,axis_reversed)
%NCSSAIRCRAFTCREATE Loads aircraft and structural data in the format
%exported by NeoCASS and outputs structs in the expected SE2A FDM format.
%   NB: Currently assumes filename points to a .MAT containing the exported
%   'ac' and 'beam_model' structs (with computed structural matrices, etc.)
%   Future implementation may  read XML, DAT, TXT, etc.

% Load data
ncss_data = load(beam_model_filename);

%% assemble structure
K_tmp = ncss_data.beam_model.Res.Struct.K;
M_tmp = ncss_data.beam_model.Res.Struct.M;
% TODO: is the D matrix related?

node_xyz_tmp = ncss_data.beam_model.Node.Coord;

%Due to the rigid elements connecting wings, HTP, and VTP to the fuselage,
%as well as the "rib" nodes which are not part of the structural model, the
%number of structural degrees of freedom in K,M are not simply equal to
%6*n_nodes.

% From beam_model.Node.DOF2 we can find which row/col idx of K/M
% corresponds to which DOF (columnwise) in which node (rowwise), i.e., 
% beam_model.Node.DOF2(idx_node, idx_DOF)

% Step 1: remove all nodes whose DOFs are not part of the structural model
% (entire row is 0) 
idx_node_nz = find(sum(ncss_data.beam_model.Node.DOF2,2)~=0);
node_xyz_tmp = node_xyz_tmp(idx_node_nz,:)';

[nodes_sec_tmp] = ncssIdentifyStructNodeSectionIdxs(node_xyz_tmp);

% Step 2: Find which nodes are fully represented in the structural model.
%         We want to give the option to consider only nodes with 
%         all the normal properties (e.g., mass) in 6DOF, because many
%         other functions rely on connecting the elements of the structural
%         matrices with the 6DOFs of the individual nodes. 
%         So this typically excludes: 
%         - The 2 wing nodes which are connected to the fuselage, and which
%         have their own degrees of freedom only in pitch and yaw; the remaining 
%         DOFS are rigidly connected to a fuselage node.
%         Note: These are actually the wing ROOT nodes, further inboard are
%         the carrythrough wing box nodes, which are independent.
%         - One node each for HTP and VTP, which are fully rigidly connected
%         to the fuselage. 

%   We look at the rowwise difference among DOF2 indices. It should be 
%   monoton. increasing, so we expect a difference of 6 from row-to-row. 
%   For a (semi-) rigid connection, we expect a bigger jump (as it refers to
%   fuselage nodes).
%   Assuming that the fuselage nodes (which everything is connected to) come
%   first, we should see first a negative jump, then a positive one. 
%   The positive jump coincides with the rigidly-connected node.
node_struct_idx = ncss_data.beam_model.Node.DOF2(idx_node_nz,:);

dof2_diff_mean_tmp = mean(diff(node_struct_idx,1),2);

idx_node_struct = find(dof2_diff_mean_tmp<=6); %Indices of 'normal' structural nodes
idx_node_rbe    = find(dof2_diff_mean_tmp>6); %Indices of nodes with reduced DOF due to rigid connections

% Pick out individual DOFs which are eliminated due to rigid connections
dof2_diff_tmp = diff(reshape(node_struct_idx', [], 1));
%Sets of rigidly connected DOFs start with a negative jump, and end with a
%positive jump
idx_start_rdof = find(dof2_diff_tmp < 1)+1; % + 1 due to diff
idx_end_rdof = find(dof2_diff_tmp > 1);
%In case the last DOF is rigidly connected (so there's no 'jump back' before the end):
if numel(idx_end_rdof) < numel(idx_start_rdof)
    idx_end_rdof(end+1) = numel(dof2_diff_tmp);
end
idx_dof_rbe = [];
for i_dof = 1:numel(idx_start_rdof)
    idx_dof_rbe = [idx_dof_rbe; [idx_start_rdof(i_dof):idx_end_rdof(i_dof)]'];
end

idx_dof_struct = reshape(node_struct_idx(idx_node_struct,:)', [],1);%find(~ismember([1:numel(node_struct_idx)]', idx_dof_rbe));

structure = struct;
structure.K = K_tmp;
structure.M = M_tmp;
structure.xyz = node_xyz_tmp;
structure.idx_node_struct = idx_node_struct;
structure.idx_node_rbe = idx_node_rbe;
structure.idx_dof_struct = idx_dof_struct; %NB: this is NOT just the inverse of idx_node_rbe!

if nargin > 1
    sign_vector     = repmat( repmat(axis_reversed,2,1), length(structure.xyz), 1 );
    sign_matrix     = sign_vector * sign_vector';
    sign_matrix     = sign_matrix(~ismember(1:end, idx_dof_rbe),:); % Remove rigidly connected DOF rows
    sign_matrix     = sign_matrix(:,~ismember(1:end, idx_dof_rbe)); % Remove rigidly connected DOF cols
    structure.M     = sign_matrix .* structure.M;
    structure.K     = sign_matrix .* structure.K;
    structure.xyz   = axis_reversed.*structure.xyz;

%%
end

