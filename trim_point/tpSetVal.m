function trim_point = tpSetVal(trim_point,type,val_names,vals)

if ~isa(val_names,'cell')
    val_names = {val_names};
end

sub_names = mdlGetTypeNames(trim_point.mdl_names,type);
sub_array = tpGetType(trim_point,type);

for i = 1:length(val_names)
    if endsWith(val_names{i},'__')%strcmp(val_names{i}(end-1:end),'__')
        for j = 1:length(vals)
            trim_point = tpSetVal(trim_point,type,{[val_names{i},num2str(j)]},vals(j));
        end
    else
        idx = dl2idx(sub_names, val_names(i));
        sub_array(idx) = vals(i);
        trim_point = tpSetType(trim_point,type,sub_array);
    end
end

end

