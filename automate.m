% Constants to be applied
clear; 
xFinal = 1;
zFinal = 0;
timeDensity = 500;
runTime = 1;
save('runOptions.mat');

%Run the three scripts
planOptimalTrajectory()
DesignProblemSummer2017('Controller','datafile','data.mat')
AnalysisOfOptimalTrajectory()