function [outputSpecs] = outputSpecsInit()
% Loads standard specifications for configurable model outputs.


outputSpecs = [];

%% Loads
%Matrix for selecting loads: | Section | Load Type | Indices |  
% --> TODO:figure out how to define 'indices' intelligently
outputSpecs.loads = { 'WL', {'Mx', 'My', 'Tz'}, -1; ...
                    'WR', {'Mx', 'My', 'Tz'}, -1; ...
                    'HL', {'Mx', 'My', 'Tz'}, -1; ...
                    'HR', {'Mx', 'My', 'Tz'}, -1; ...
                    'FU', {'My', 'Tz'}, -1 ...
                    };
 
%% Local inertial sensors
% outputSpecs.local_inertial = 
                
                
                
end