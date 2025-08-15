function [nodes_sec] = ncssIdentifyStructNodeSectionIdxs(node_xyz)
% This function identifies the main aircraft sections (Fuselage, Left/right
% Wing, etc.) from the node coordinates based on typical characteristics of
% the NeoCASS outputs. 
% Coordinates are assumed to include only valid nodes (no 'wing rib' or
% 'tube' nodes)
% 
% Note: this function is very approximate, and relies on a standard
%   structure. Always check if it is working correctly! 
% Note: Current implementation assigns the innermost node of the wing and
%   HTP always to teh right side

% Assumptions: 
% - Fuselage nodes have y = 0, and x increases monotonically
% - Sections appear in following order: 
%       First, Bar-end nodes: FU, WR, WL, VT, HR, HL, NR, NL
%       Then, Mid-bar nodes:  FU, WR, WL, VT, HR, HL, NR, NL
% - For all sections except NR/NL, there is one less mid-bar node than
%       bar-end. Wings and HTP are counted tip-to-tip. 
%       Engines are mounted directly to the wing, so they are equal for NR/NL

if (size(node_xyz, 2) > size(node_xyz,1)) && (size(node_xyz,1) == 3)
    % Set nodes row-wise
    node_xyz = node_xyz';
end
% Compute expected number of nodes
n_tot = size(node_xyz,1); %length yields longest dimension
% n_ends = (n_tot+4)/2; % Fus + Wing + HTP + VTP = 4
% n_mids = n_ends-4; 

sec_names = {'FU'; 'WR'; 'WL'; 'VT'; 'HR'; 'HL'; 'NR'; 'NL'};
field_seq = [sec_names; sec_names];

%% Sort nodes into sections

nodes_sec = struct;
for i_sec = 1:numel(sec_names)
    nodes_sec.(sec_names{i_sec}) = [];
end

i_field = 1;

% Length of distances between subsequent nodes
node_dist = vecnorm(diff(node_xyz,1),2,2);
for i_node = 1:n_tot
    % Check based on the distance from the previous node to this one
    % whether we have passed to a new section
    if i_node~=1 && node_dist(i_node-1) > 10 %10 is set as threshold experimentally, to be revisited
        i_field = i_field + 1; 
    end
    sec_str = field_seq{i_field};
    
    nodes_sec.(sec_str) = [nodes_sec.(sec_str); i_node];
    
end



