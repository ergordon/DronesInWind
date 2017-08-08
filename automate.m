% Constants to be applied
clear; 
xFinal = 5;
zFinal = 5;
timeDensity = 400;
runTime = 2;


gravity = 9.81;
mass = 1;

maxPitchRate = 10;
maxThrust = 20;
minThrust = 1;

save('runOptions.mat');

%Run the three scripts
planOptimalTrajectory()
DesignProblemSummer2017('Controller','datafile','data.mat')
AnalysisOfOptimalTrajectory()