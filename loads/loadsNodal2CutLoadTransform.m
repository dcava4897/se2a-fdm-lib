function [T_cg, load_labels] = loadsNodal2CutLoadTransform(structure, node_ref)
% Calculates the nodal- to cut-load transformation matrix
% Following Reschke, Integrated Flight Loads Modelling and Analysis for
% Flexible Transport Aircraft
% Uses structural matrices to automatically calculate structural load paths among nodes.
%
% Inputs:
% - structure: Same as 'structure' struct in init. Should contain the K
%               matrix and the nodal xyz coordinates
% - node_ref: 'Central' reference node coordinates, typically the CG 


    nodes_xyz = structure.xyz(:,structure.idx_node_struct);

    node_ref(2) = 0; % We assume the ref node is on the plane of symmetry
    num_nodes = size(nodes_xyz,2);

    nodes_sec = structure.nodes_sec;%Load_StructNodeIdxs('v4_twist');
    
    %Convert sec node indices removing non-struct nodes
    secnames = fieldnames(nodes_sec);
    for i_sec = 1:numel(secnames)
        idx_node_rm = []; %Nodes to be removed
        for i_node = 1:numel(nodes_sec.(secnames{i_sec}))
            idx_tmp = find(ismember(structure.idx_node_struct, nodes_sec.(secnames{i_sec})(i_node)));
            if ~isempty(idx_tmp)
                nodes_sec.(secnames{i_sec})(i_node) = idx_tmp;
            else
                idx_node_rm(end+1) = i_node;
            end
        end
        nodes_sec.(secnames{i_sec})(idx_node_rm) = [];
    end

    %Find map of node connections
    nodes_map = mapNodeConnections(structure.K(structure.idx_dof_struct,structure.idx_dof_struct), nodes_xyz, nodes_sec);

    %Find set of 'cut-off' nodes for each load station
    cutoff_nodes = findCutoffNodes(nodes_map, nodes_xyz, node_ref);

    % Assemble nodal- to cut-load transformation matrix
    % Following Reschke, Integrated Flight Loads Modelling and Analysis for
    % Flexible Transport Aircraft
    T_cg = zeros(6*num_nodes,6*num_nodes); 

    for idx_node = 1:size(cutoff_nodes,1)
        idx_node_full = 6*(idx_node-1)+1;
        for idx_cutoff = 1:size(cutoff_nodes,2)
            if cutoff_nodes(idx_node,idx_cutoff) == 0
                continue;
            else
                idx_co_full = 6*(idx_cutoff-1)+1;
                T_cg(idx_node_full:(idx_node_full+5),idx_co_full:(idx_co_full+5) ) = ...
                    [eye(3) zeros(3);
                    assembleSkewSymMat(nodes_xyz(:,idx_cutoff)-nodes_xyz(:,idx_node)) eye(3)];
            end
        end
    end

    % Rearrange T_cg to separate the 6 different loads:
    T_cg = [T_cg(6*(0:num_nodes-1)+1,:); T_cg(6*(0:num_nodes-1)+2,:); T_cg(6*(0:num_nodes-1)+3,:);...
            T_cg(6*(0:num_nodes-1)+4,:); T_cg(6*(0:num_nodes-1)+5,:); T_cg(6*(0:num_nodes-1)+6,:)];

    %Load station/load names
    load_station_names = [];
    for idx_node = 1:num_nodes
        sec_str = findNodeSection(idx_node, nodes_sec);
        load_station_names{end+1} = strcat(sec_str,'_',num2str(idx_node));
    end

    load_labels = [strcat('Tx_', load_station_names');
                   strcat('Ty_', load_station_names');
                   strcat('Tz_', load_station_names');
                   strcat('Mx_', load_station_names');
                   strcat('My_', load_station_names');
                   strcat('Mz_', load_station_names')];
end


%% Local functons

function nodes_map = mapNodeConnections(K, nodes_xyz, nodes_sec)
    num_nodes = size( nodes_xyz, 2 );
     
    K_3d = reshape( full(K), 6, num_nodes, 6, num_nodes );
    
    nodes_map = zeros(num_nodes, num_nodes);
    
    for ii_n = 1:num_nodes
        nodes_map(ii_n, findConnectedNodes(K_3d, ii_n, nodes_sec, nodes_xyz)) = 1;
    end
    
    %Enforce 1 connection between sections: (mainly a problem for
    %wings/vtp)
    for i_node = 1:num_nodes
        if sum(nodes_map(i_node,:)) > 4 %Are there many connections at this node?
            sec_node = findNodeSection(i_node, nodes_sec); %Find section
            idx_cn = find(nodes_map(i_node,:)); %Find all connected nodes
            
            sec_names = fieldnames(nodes_sec);

            for j_names = 1:length(sec_names) % iterate on sections
                if strcmp(sec_names{j_names},sec_node); continue; end %Ignore connections with own section
                idx_rm_tmp = idx_cn(ismember(idx_cn, nodes_sec.(sec_names{j_names}))); %Get all connected nodes in this section
                vec_nodes = nodes_xyz(:,idx_rm_tmp)-repmat(nodes_xyz(:,i_node), 1, length(idx_rm_tmp)); % Vectors from node to connected nodes
                if sum(strcmp(sec_names{j_names}, {'NR', 'NL'})) && strcmp(sec_node, 'FU') %Remove direct connection between FU and Nacelles
                    idx_sort = [];
                else
                  [~, idx_sort] = sort(sum(vec_nodes.^2,1), 'ascend'); %Order from closest to furthest

                end
                
                if ~isempty(idx_sort)
                    idx_rm_tmp = removeEntries(idx_rm_tmp, idx_rm_tmp(idx_sort(1))); % Keep the closest node...
                    
                end
                nodes_map(i_node,idx_rm_tmp) = 0; % ...and remove the rest (from the section)
                nodes_map(idx_rm_tmp,i_node) = 0; % both row-wise and column-wise
            end
            
        end   
    end
end

function cutoff_nodes = findCutoffNodes(nodes_map, nodes_xyz, ref_xyz)
    num_nodes = length(nodes_xyz);
    G_nodes = graph(nodes_map);
    
    %Find nodes adjacent to ref node (CG):  (note: assumes it is a fuselage
    %node, so CG lies between two)
    % TODO: find one fore and aft to avoid including the CG in the
    % calculation?
    
    [~, idx_ref1] = min(sum((nodes_xyz-repmat(ref_xyz, 1, num_nodes)).^2,1));
    idx_ref_n = neighbors(G_nodes,idx_ref1);
    [~,idx_tmp] = min(sum((nodes_xyz(:,idx_ref_n)-repmat(ref_xyz, 1, length(idx_ref_n) )).^2,1));
    idx_ref2 = idx_ref_n(idx_tmp);
    clear idx_tmp idx_ref_n
    
%     idx_tips = find(degree(G_nodes)==1); % List of 'tip' nodes
    
    cutoff_nodes = zeros(num_nodes, num_nodes);
    idx_done = [];
%     b_reset = true;
    %Strategy: at each node, find path to the reference point, and then
    %path to all other nodes. Throw away all nodes whsoe paths touch the
    %reference path; the rest are effectively 'cut off'
    idx_curr = 1;
    while length(idx_done)<num_nodes
%         if b_reset == true %Find a new 'tip' node
%             idx_curr = idx_tips(~ismember(idx_tips, idx_done));
%             idx_curr = idx_curr(1);
%         end
        
        path_ref = {shortestpath(G_nodes, idx_curr, idx_ref1); % Path from cut station to furthest reference node
                    shortestpath(G_nodes, idx_curr, idx_ref2)};
        [~, max_path] = max([length(path_ref{1}),length(path_ref{2})]);
        cutoff_curr = [];
        pathtree_curr = shortestpathtree(G_nodes, idx_curr, 'OutputForm', 'cell');
        for idx_path=1:length(pathtree_curr)
            if sum(ismember(pathtree_curr{idx_path}, path_ref{max_path}(2:end)))== 0 %&& (idx_path~=idx_curr)
                cutoff_curr(end+1) = idx_path;
            end
        end
        
        cutoff_nodes(idx_curr, cutoff_curr) = 1;
        idx_done(end+1) = idx_curr;
        
        idx_curr = idx_curr + 1;
        
    end
end

function idx = removeEntries( idx, idx_remove )
    for i = 1:length(idx_remove)
        idx(idx==idx_remove(i)) = [];
    end
    idx = unique(idx);
end

function idx_connected_nodes = findConnectedNodes(K_3d, idx, nodes_sec, nodes_xyz)
    num_nodes = size(K_3d,2);
    stiffness_col = reshape( squeeze( K_3d(:,idx(end),:,:)), 6*6, num_nodes )';
    K_max = max(abs(stiffness_col));
    [idx_connected_nodes,~] = find( abs(stiffness_col)>zeros(size(stiffness_col) ));%repmat(1e-8*K_max,size(stiffness_col,1),1) );
%         [idx_connected_nodes,~] = find( abs(stiffness_col)>repmat(1e-3*K_max,size(stiffness_col,1),1) );

    idx_connected_nodes = unique(idx_connected_nodes);
    idx_connected_nodes = removeEntries(idx_connected_nodes,idx);

    sec_str = findNodeSection(idx(end), nodes_sec);
    
    if sum(strcmp(sec_str, {'WR', 'WL', 'HR','HL', 'VT'}))
        %We assume that each node in each section can be connected to max. 2
        %other nodes within section
        idx_sec_local = find(ismember( idx_connected_nodes,nodes_sec.(sec_str))); %Any other nodes in same section?
        vec_sec = nodes_xyz(:,idx_connected_nodes(idx_sec_local))-repmat(nodes_xyz(:,idx(end)), 1, length(idx_sec_local)); %Vectors from selected node to others in same section

        %     [~,idx_1] = min(sum(vec_sec.^2,1)); 
    %     [~,idx_2] = min(sum(vec_sec(:,setdiff(1:end,idx_1)).^2,1)); % Find 2nd-nearest
        [dist_sort, idx_sort] = sort(sum(vec_sec.^2,1), 'ascend');
        idx_1 = idx_sort(1); %Find nearest
        if length(dist_sort)>1 %Is there more than one conneted?
            idx_2 = idx_sort(2); %Find 2nd-nearest
    %     if ~isempty(idx_2)
            angle_12 = atan2(norm(cross(vec_sec(:,idx_1),vec_sec(:,idx_2))),dot(vec_sec(:,idx_1),vec_sec(:,idx_2))); % Angle between two nearest
            if angle_12 < pi/2 %if angle < 90°, we ignore it
                idx_2 = [];
            end
        else
            idx_2 = [];
        end
        idx_sec_rm = removeEntries(idx_sec_local, idx_sec_local([idx_1, idx_2]));

        idx_connected_nodes = removeEntries(idx_connected_nodes,idx_connected_nodes(idx_sec_rm));
    end
    
end

function sec_str = findNodeSection(idx_node, nodes_sec)
    sec_names = fieldnames(nodes_sec);
    sec_str = [];
    for ii = 1:length(sec_names)
        if ismember(idx_node, nodes_sec.(sec_names{ii}))
            sec_str = sec_names{ii};
        end
    end
end

function skew_mat = assembleSkewSymMat(x)
    % Assembles skew-symmetric matrix to produce cross-product (3D vector)
    skew_mat = [ 0   -x(3) x(2);
                 x(3) 0   -x(1);
                -x(2) x(1) 0   ];
end