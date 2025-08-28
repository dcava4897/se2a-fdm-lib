function [outputs] = aircraftConfigureOutputs(output_specs, aircraft, structure)
%AIRCRAFTCONFIGUREOUTPUTS Configure the necessary transformation matrices
%for the required outputs based on the output specification struct.
outputs = [];
out_types = fieldnames(output_specs);

for ii = 1:length(out_types)
    switch out_types{ii}
        case 'loads'
            outputs.loads = loadsComputeTransform(output_specs.(out_types{ii}), aircraft, structure);
            
%         case 'local_inertial'
    end
end


end

