function AnalysisOfOptimalTrajectory

load traj
topt = t;
xopt = x;
uopt = u;

load data
tsim = processdata.t;
xsim = [processdata.x; processdata.xdot; processdata.z; processdata.zdot; processdata.theta];
usim = [controllerdata.actuators.pitchrate; controllerdata.actuators.thrust];

clf;

% Get MATLAB's standard list of colors
colorList = get(gca,'ColorOrder');

plot(topt,xopt,'-','linewidth',1);
hold on;
for i=1:size(xopt,1)
    plot(tsim,xsim(i,:),'.','color',colorList(i,:));
end

legend('x','xdot','z','zdot','theta');

xlim([topt(1) topt(end)]);


end