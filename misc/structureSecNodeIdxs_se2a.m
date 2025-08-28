function [nodes_sec] = Load_StructNodeIdxs_SE2A(structure_version)
%LOAD_STRUCTNODEIDXS Load indices of the nodes contained in each section of
%the aircraft
switch structure_version
    case 'v1'  
        nodes_sec.FU = 1:41;
        nodes_sec.NL = 42:52;
        nodes_sec.NR = 53:63;
        nodes_sec.VT = 64:73;
        nodes_sec.HL = 74:81;
        nodes_sec.HR = 82:89;
        nodes_sec.WL = 90:119;
        nodes_sec.WR = 120:149;
    case 'v4'
        nodes_sec.FU = 1:40;
        nodes_sec.NL = 41:49;
        nodes_sec.NR = 50:58;
        nodes_sec.VT = 59:68;
        nodes_sec.HL = 69:76;
        nodes_sec.HR = 77:84;
        nodes_sec.WL = 85:114;
        nodes_sec.WR = 115:144;
    case 'v4_twist'
        nodes_sec.FU = 1:40;
        nodes_sec.NL = 41:44;
        nodes_sec.NR = 45:48;
        nodes_sec.VT = 49:58;
        nodes_sec.HL = 59:66;
        nodes_sec.HR = 67:74;
        nodes_sec.WL = 75:104;
        nodes_sec.WR = 105:134;
end

end

