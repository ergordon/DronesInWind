% Constants to be applied
clear; 
xFinal = 2;
yFinal = 4;
zFinal = 2;
timeDensity = 500;
runTime = 4;

gravity = 9.81;
mass = 1;
Ix = 1;
Iy = 1;
Iz = 1;%.5;
dim_1 = .651;
dim_2 = .651;
dim_3 = .188;

maxThrust = 20/4;
minThrust = 1/4;

save('runOptions.mat');

%Run the four scripts
eoms()
planOptimalTrajectory()
DesignProblemSummer2017('Controller','datafile','data.mat')
AnalysisOfOptimalTrajectory()