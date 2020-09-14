function result=analyze_test(test_names,global_minimum,local_minimum)
result=[];
max_iterations=0;
for idx=1:length(test_names)
    m = memmapfile(test_names{idx},...
        'Format',{'double',[1,1],'time';...
        'uint32', [1,1],'iteration';...
        'uint32',[1,1],'nodes';...
        'double', [1,1],'local_probability';...
        'double',[1,1],'best_cost'});
    
    data{idx}=struct2table(m.Data);
    max_iterations=max(max_iterations,length(data{idx}.iteration));
end


for idx=1:length(test_names)
    newlines=max_iterations-length(data{idx}.iteration);
    last_part=repmat(data{idx}(end,:),newlines,1);
    last_part.iteration=data{idx}.iteration(end)+uint32((1:newlines)');
    data{idx}=[data{idx};last_part];
end

iters=(1:max_iterations)';
costs=zeros(length(iters),length(test_names));
for idx=1:length(test_names)
    costs(:,idx)=data{idx}.best_cost;
end

h=semilogx(iters,median(costs,2));

hold on
semilogx(iters,prctile(costs,5,2),'-.','Color',h.Color)
semilogx(iters,prctile(costs,95,2),'--','Color',h.Color)
