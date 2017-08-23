% Constants to be applied
clear; 
xFinal = 5;
yFinal = 0;
zFinal = 5;
timeDensity = 400;
runTime = 4;

gravity = 9.81;
mass = 1;
Ix = 1;
Iy = 1;
Iz = .5;

maxPitchRate = 10;
% maxRollRate = 10;
% maxYawRate = 10;
maxThrust = 20/4;
minThrust = 1/4;

save('runOptions.mat');

%Run the four scripts
eoms()
planOptimalTrajectory()
DesignProblemSummer2017('Controller','datafile','data.mat')
AnalysisOfOptimalTrajectory()