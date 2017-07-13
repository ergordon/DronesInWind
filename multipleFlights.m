% Number of flights
nFlights = 100;

%Vars
Xf = zeros(1,nFlights);
xx = []
tt = []
% Loop over each flight
tic
for i=1:nFlights
% Run simulation without graphics and save data
DesignProblemSummer2017('Controller')% Load data
load('data.mat');
% Get t and x
t = processdata.timeEnd;
tt = [tt t];
ii = [ii i];
% Do analysis...
%
Xf(i) = x(end);
end
toc
histogram(Xf)
    set(gca,'fontsize',14);
    xlabel('Distance (x)');
    ylabel('Frequency');
    set(gcf,'paperorientation','landscape');
    set(gcf,'paperunits','normalized');
    set(gcf,'paperposition',[0 0 1 1]);
    title('Frequency Distribution of 1000 Simulated Flights')
    print(gcf,'-dpdf','flights.pdf');
mean(Xf)
median(Xf)

m = matfile(xFlight1,'Writable',isWritable)
save(xFlight,Xf)