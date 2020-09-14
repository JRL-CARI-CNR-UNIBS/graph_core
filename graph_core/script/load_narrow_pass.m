clear
logs=dir('~/.ros/narrow_pass*.dirrt');
for idx=1:length(logs)
    log_name=logs(idx).name(1:end-6);
    log_info=split(log_name,'__');
    log_params=str2double(cellfun(@(x) x(regexp(x,'[^a-zA-Z_]')),log_info,'un',0));
    
    test.type=log_info{1};
    test.algorithm=log_info{2};
    test.cylinder_radius=log_params(3);
    test.cylinder_width=log_params(4);
    test.measures_ratio=log_params(5);
    test.tube_radius=log_params(6);
    test.forgetting_factor=log_params(7);
    test.dof=log_params(8);
    test.trial=log_params(9);
    test.name=[logs(idx).folder,filesep,logs(idx).name];
    tests(idx)=test;
end
test_table=struct2table(tests);

algoritms=test_table(and(test_table.trial==0,test_table.dof==2),1:7);

%%
figure
for dof=3
    for ialg=1%[1 5] %1:size(algoritms,1)
        ialg
        idxs=strcmpi(test_table.algorithm,algoritms.algorithm{ialg}) & ...
            test_table.cylinder_radius==algoritms.cylinder_radius(ialg) & ...
            test_table.cylinder_width==algoritms.cylinder_width(ialg) & ...
            test_table.measures_ratio==algoritms.measures_ratio(ialg) & ...
            test_table.tube_radius==algoritms.tube_radius(ialg) & ...
            test_table.forgetting_factor==algoritms.forgetting_factor(ialg);
        
        
        cylinder_width=algoritms.cylinder_width(ialg);
        cylinder_radius=algoritms.cylinder_radius(ialg);
        measures_ratio=algoritms.measures_ratio(ialg);
        hole_radius=cylinder_radius*(measures_ratio^(1.0/(dof-1.0)));
        b=cylinder_width*0.6;
        a=(cylinder_radius+3*hole_radius)/4.0;
        
        global_minimum = cylinder_width+2*sqrt((0.1*cylinder_width^2  + (hole_radius-a)^2));
        local_minimum  = cylinder_width+2*sqrt((0.1*cylinder_width)^2 + (cylinder_radius-a)^2);
        
        trials=test_table(idxs & test_table.dof==dof,:);
        analyze_test(trials.name,global_minimum,local_minimum);
        drawnow
    end
    semilogx(xlim,local_minimum*[1 1],'-b','LineWidth',2)
    semilogx(xlim,local_minimum*[1 1],'-k','LineWidth',2)
    
end