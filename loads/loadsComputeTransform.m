function [loads] = loadsComputeTransform(loads_selection, aircraft, structure)
%COMPUTE_LOADSTRANSFORM Calculates load transformation matrices and
%reduces them according to specs


%Calculate load transformation matrices:
[T_cg, load_labels] = loadsNodal2CutLoadTransform(structure, aircraft.config.xyz_cg_c);

phi_loads = T_cg * structure.K(structure.idx_dof_struct,structure.idx_dof_struct) *  aircraft.eom_flexible.structure_red.modal.T(:,7:end);


phi_out = [];
load_labels_red = [];
for ii = 1:size(loads_selection, 1) % Section
    sec_tmp = loads_selection{ii,1};
    
    for jj = 1:length(loads_selection{ii,2}) % Load type

        load_tmp = loads_selection{ii,2}{jj};
        
        if (numel(loads_selection{ii, 3}) == 1) && (loads_selection{ii, 3} == -1) %Take all nodes within the specified section
            idx_sec_load = find(contains(load_labels, load_tmp) .* contains(load_labels, sec_tmp));
        else % use only specified nodes
            idx_nodes = zeros(size(load_labels));
            nodes_tmp = loads_selection{ii, 3};
            for kk = 1:length(nodes_tmp)
                idx_nodes = idx_nodes + contains(load_labels, num2str(nodes_tmp(kk)));
            end
            idx_sec_load = find(contains(load_labels, load_tmp) .* contains(load_labels, sec_tmp) .* idx_nodes);
        end
        
        phi_out = [phi_out; phi_loads(idx_sec_load,:)];
        load_labels_red = [load_labels_red; load_labels(idx_sec_load)];
%         loadStations.(sec_tmp).(load_tmp) = [];
%         loadStations.(sec_tmp).(load_tmp).phi = phi_loads(idx_sec_loads,:);
        
    end
        
end

loads.phi= phi_out;
loads.labels = load_labels_red;
loads.spec = loads_selection;


end

