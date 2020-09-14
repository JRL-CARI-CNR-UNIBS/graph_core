clear all;close all;clc

for idx=1:30
    m = memmapfile(sprintf('global_robot6_2conf_%d.dirrt',idx-1),...
        'Format',{'double',[1,1],'time';...
        'uint32', [1,1],'iteration';...
        'uint32',[1,1],'nodes';...
        'double', [1,1],'local_probability';...
        'double',[1,1],'best_cost'});
    
    data{idx}=struct2table(m.Data);
end

C = colororder;
hf=graph_dirrt(data,C(1,:));
 
local=load('local_robot6_2conf.mat');
graph_dirrt(local.data,C(2,:),hf);