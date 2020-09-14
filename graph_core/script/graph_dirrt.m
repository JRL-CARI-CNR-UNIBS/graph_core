function hf=graph_dirrt(data,color,hf)
if nargin<3
    hf=figure;
else
    figure(hf);
end

complete=1;

iters=min(cellfun(@(x)size(x,1),data));
costs=zeros(iters,1);
for idx=1:length(data)
    costs(:,idx)=data{idx}.best_cost(1:iters);
end
mean_cost=mean(costs,2);
std_cost=std(costs,1,2);
for idx=1:length(data)
    if complete
        h1=subplot(411);
    end
    l=semilogx(data{idx}.iteration,(data{idx}.best_cost),'Color',[color 0.5]);
    h1.YScale='log';
    h1.XScale='log';
    hold on
    grid on
    ylabel('cost');
    xlabel('iteration');
    if complete
        h2=subplot(412);
        semilogx(data{idx}.iteration,data{idx}.time,'Color',color)
        h2.YScale='log';
        h2.XScale='log';
        
        hold on
        grid on
        ylabel('time');
        xlabel('iteration');
        
        h3=subplot(413);
        semilogx(data{idx}.time,data{idx}.best_cost,'Color',color)
        h3.YScale='log';
        h3.XScale='log';
        
        hold on
        grid on
        ylabel('cost');
        xlabel('time');
        
        
        h4=subplot(414);
        semilogx(data{idx}.iteration,data{idx}.local_probability,'Color',color)
        h3.YScale='Linear';
        h3.XScale='log';
        
        hold on
        grid on
        ylabel('local probability');
        xlabel('iteration');
    end
end

if complete
    subplot(411)
end
plot(1:iters,mean_cost,'LineWidth',4,'Color',color)
plot(1:iters,mean_cost+2*std_cost,'--','LineWidth',4,'Color',color)
if complete
    linkaxes([h1 h2 h4],'x')
end

