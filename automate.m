% Constants to be applied
clear; 
xFinal = 5;
yFinal = 5;
zFinal = 5;
timeDensity = 400;
runTime = 4;


gravity = 9.81;
mass = 1;

maxPitchRate = 10;
maxRollRate = 10;
maxYawRate = 10;
maxThrust = 20;
minThrust = 1;

save('runOptions.mat');

%Run the three scripts
planOptimalTrajectory()
DesignProblemSummer2017('Controller','datafile','data.mat')
AnalysisOfOptimalTrajectory()