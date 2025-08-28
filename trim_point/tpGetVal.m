function [vals,idx] = tpGetVal(trim_point,type,val_names)

if ~isa(val_names,'cell')
    val_names = {val_names};
end

vals = [];
idx  = [];

sub_names = mdlGetTypeNames(trim_point.mdl_names,type);
sub_array = tpGetType(trim_point,type);

for i = 1:length(val_names)
    if endsWith(val_names{i},'__')%strcmp(val_names{i}(end-1:end),'__')
        k = 0;
        for j = 1:length(sub_names)
            val_name = {[val_names{i},num2str(j)]};
            idx_tmp = dl2idx(sub_names, val_name,1,false);
            if idx_tmp ~= 0
                idx(end+1) = idx_tmp;
                vals(end+1) = tpGetVal(trim_point,type,{[val_names{i},num2str(j)]});
                k = k+1;
            end
        end
        if k == 0
            warning([val_names{i},' was not found.']);
        end
    else
        idx(end+1) = dl2idx(sub_names, val_names(i), 1, false);
        if idx(end) == 0
            % term not found
            vals(end+1) = 0;
        else
            vals(end+1) = sub_array(idx(end));
        end
    end
end

end

