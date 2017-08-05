function DesignProblemSummer2017(controller,varargin)
% DesignProblemSummer2017   run simulation of quadrotor in 2D
%
%   DesignProblemSummer2017('FunctionName') uses the controller defined by
%       the function 'FunctionName' - for example, in a file with
%       the name FunctionName.m - in the simulation.
%
%   DesignProblemSummer2017('FunctionName','P1',V1,'P2','V2',...) optionally
%       defines a number of parameter values:
%
%           'team' : a name (e.g., 'FirstName LastName') that, if defined,
%                    will appear on the figure window
%
%           'datafile' : a filename (e.g., 'data.mat') where, if defined,
%                        data will be logged and saved
%
%           'moviefile' : a filename (e.g., 'movie.mp4') where, if defined,
%                         a movie of the simulation will be saved
%
%           'snapshotfile' : a filename (e.g., 'snap.pdf') where, if
%                            defined, a PDF with a snapshot of the last
%                            frame of the simulation will be saved
%
%           'controllerdatalog' : a cell array (e.g., {'y','xhat'}) with
%                                 the names of fields in controller.data -
%                                 if 'datafile' is defined (so data is
%                                 logged), then values in these fields will
%                                 also be logged and saved
%
%           'perturbmodel' : a flag - true or false (default) - that, if
%                            true, results in some perturbation (not yet
%                            implemented...)
%
%   Regardless of how the function is called, it will clear the current
%   figure and will show the simulation. To quit, type 'q' when this figure
%   is in the foreground.

% Parse the arguments
% - Create input parser
p = inputParser;
% - Parameter names must be specified in full
p.PartialMatching = false;
% - This argument is required, and must be first
addRequired(p,'controller',@ischar);
% - These parameters are optional, and can be in any order
addParameter(p,'team',[],@ischar);
addParameter(p,'datafile',[],@ischar);
addParameter(p,'moviefile',[],@ischar);
addParameter(p,'snapshotfile',[],@ischar);
addParameter(p,'controllerdatatolog',[],@iscell);
addParameter(p,'perturbmodel',false,@islogical);
% - Apply input parser
parse(p,controller,varargin{:});
% - Extract parameters
process = p.Results;
% - Check that the 'controller' function exists
if (exist(process.controller,'file')~=2)
    error('Controller ''%s'' does not exist.',process.controller);
end

% Setup the simulation
[process,controller] = SetupSimulation(process);

% Run the simulation
RunSimulation(process,controller);


end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% FUNCTIONS THAT WILL CHANGE FOR DIFFERENT PROCESSES
%

function [process,controller] = SetupSimulation(process)

load('runOptions')
% DEFINE CONSTANTS

% Constants related to simulation.
% - State time.
process.tStart = 0;
% - Stop time.
process.tStop = runTime; % MODIFICATION: Runtime
% - Time step.
process.tStep = 1/timeDensity; % MODIFICATION: Timestep
% - Names of things to log in datafile, if desired
process.processdatatolog = {'t','x','xdot','z','zdot','theta'};

% Constants related to physical properties.
% - Acceleration of gravity.
process.g = 9.81;
% - Mass
process.m = 1.0;
% - Maximum thrust
process.maxthrust = 15;
% - Maximum pitch rate
process.maxpitchrate = 5;
% - Geometry
process.sparLength = 0.5;
process.sparWidth = 0.01;
process.rotorLength = 0.25;
process.rotorWidth = 0.05;

% DEFINE VARIABLES

% Time
process.t = 0;
% x, xdot - position and velocity
process.x = 0;
process.xdot = 0;
% z, zdot - position and velocity
process.z = 0;
process.zdot = 0;
% theta - angle
process.theta = 0;

% DEFINE CONTROLLER

% Functions
% - get handles to user-defined functions 'init' and 'run'
controller = eval(process.controller);
controller.name = process.controller;
% Parameters
% - define a list of constants that will be passed to the controller
names = {'tStep','g','m','maxthrust','maxpitchrate'};
% - loop to create a structure with only these constants
controller.parameters = struct;
for i=1:length(names)
    controller.parameters.(names{i}) = process.(names{i});
end
% Storage
controller.data = struct;
% References
controller.references = struct;
% Sensors
controller.sensors = GetSensors(process);
% Actuators
controller.running = true;
tic
try
    [controller.actuators,controller.data] = ...
        controller.init(controller.sensors, ...
                           controller.references, ...
                           controller.parameters, ...
                           controller.data);
catch exception
    warning(['The ''init'' function of controller\n     ''%s''\n' ...
             'threw the following error:\n\n' ...
             '==========================\n' ...
             '%s\n', ...
             '==========================\n\n' ...
             'Turning off controller and setting all\n' ...
             'actuator values to zero.\n'],controller.name,getReport(exception));
	controller.actuators = ZeroActuators();
    controller.running = false;
end
if (~isstruct(controller.actuators) || ~CheckActuators(controller.actuators))
    warning(['The ''init'' function of controller\n     ''%s''\n' ...
             'did not return a structure ''actuators'' with the right\n' ...
             'format. Turning off controller and setting all\n' ...
             'actuator values to zero.\n'],controller.name);
    controller.actuators = ZeroActuators();
    controller.running = false;
end
controller.tComputation = toc;

end


function sensors = GetSensors(process)
sensors.t = process.t;
sensors.x = process.x;
sensors.xdot = process.xdot;
sensors.z = process.z;
sensors.zdot = process.zdot;
sensors.theta = process.theta;
% Add noise
%   (nothing)
end

function [t,x] = Get_TandX_From_Process(process)
t = process.t;
x = [process.x; process.xdot; process.z; process.zdot; process.theta];
end

function u = GetInput(process,actuators)
% Copy input from actuators
u = [actuators.pitchrate; actuators.thrust];

% Initialize inputs to zero
u = zeros(2,1);

% Copy pitch rate from actuators
if actuators.pitchrate<-process.maxpitchrate
    u(1) = -process.maxpitchrate;
elseif actuators.pitchrate>process.maxpitchrate
    u(1) = process.maxpitchrate;
else
    u(1) = actuators.pitchrate;
end

% Copy thrust from actuators
if actuators.thrust<0
    u(2) = 0;
elseif actuators.thrust>process.maxthrust
    u(2) = process.maxthrust;
else
    u(2) = actuators.thrust;
end

% Add disturbance
%   (nothing)
end

function process = Get_Process_From_TandX(t,x,process)
process.t = t;
process.x = x(1,1);
process.xdot = x(2,1);
process.z = x(3,1);
process.zdot = x(4,1);
process.theta = x(5,1);
end

function XDOT = GetXDot(T,X,U,process)
% unpack X and U
x = X(1,1);
xdot = X(2,1);
z = X(3,1);
zdot = X(4,1);
theta = X(5,1);
w = U(1,1);
f = U(2,1);
% compute rates of change
d_x = xdot;
d_xdot = (f/process.m)*sin(theta) + 0; %MODIFICATION: Wind
d_z = zdot;
d_zdot = (f/process.m)*cos(theta)-process.g;
d_theta = w;
% pack XDOT
XDOT = [d_x; d_xdot; d_z; d_zdot; d_theta];
end

function iscorrect = CheckOneActuatorValue(val)
iscorrect = isnumeric(val)&&isscalar(val)&&~isnan(val)&&~isinf(val);
end

function iscorrect = CheckActuators(actuators)
iscorrect = false;
if isfield(actuators,{'thrust'})&&isfield(actuators,{'pitchrate'})&&(length(fieldnames(actuators))==2)
    if CheckOneActuatorValue(actuators.thrust)&&CheckOneActuatorValue(actuators.pitchrate)
        iscorrect = true;
    end
end
end

function actuators = ZeroActuators()
actuators = struct('thrust',0,'pitchrate',0);
end

function fig = UpdateFigure(process,controller,fig)
if (isempty(fig))
    % CREATE FIGURE
    
    % Clear the current figure.
    clf;
    
    % Create an axis for text (it's important this is in the back,
    % so you can rotate the view and other stuff!)
    fig.text.axis = axes('position',[0 0 1 1]);
    axis([0 1 0 1]);
    hold on;
    axis off;
    fs = 14;
    if (controller.running)
        status = 'ON';
        color = 'g';
    else
        status = 'OFF';
        color = 'r';
    end
    fig.text.status=text(0.05,0.975,...
        sprintf('CONTROLLER: %s',status),...
        'fontweight','bold','fontsize',fs,...
        'color',color,'verticalalignment','top');
    fig.text.time=text(0.05,0.12,...
        sprintf('t = %6.2f / %6.2f\n',process.t,process.tStop),...
        'fontsize',fs,'verticalalignment','top','fontname','monaco');
    fig.text.teamname=text(0.05,0.06,...
        sprintf('%s',process.team),...
        'fontsize',fs,'verticalalignment','top','fontweight','bold');
    
    % Create an axis for the view from frame 0.
    fig.view0.axis = axes('position',[0 0 1 1]);
    set(gcf,'renderer','opengl');
    set(gcf,'color','w');
    axis equal;
    axis([-4 4 -3 3]);
    axis manual;
    hold on;
    axis off;
    box on;
    
    % Grid
    xmax = 4;
    for x = -xmax:1:xmax
        line([x x],[-xmax xmax],'color',0.75*[1 1 1],'linewidth',0.5);
        line([-xmax xmax],[x x],'color',0.75*[1 1 1],'linewidth',0.5);
    end
    
    % Quadcopter
    fig.view0.spar = DrawBox([],process.x,process.z,-process.theta,...
                                process.sparLength,process.sparWidth);
    fig.view0.leftRotor = DrawBox([],process.x-0.5*process.sparLength*cos(process.theta),...
                                     process.z+0.5*process.sparLength*sin(process.theta),...
                                     -process.theta,process.rotorLength,process.rotorWidth);
	fig.view0.rightRotor = DrawBox([],process.x+0.5*process.sparLength*cos(process.theta),...
                                      process.z-0.5*process.sparLength*sin(process.theta),...
                                      -process.theta,process.rotorLength,process.rotorWidth);
    
    % Make the figure respond to key commands.
    set(gcf,'KeyPressFcn',@onkeypress);
else
    % UPDATE FIGURE
    
    % Text
    set(fig.text.time,'string',sprintf('t = %6.2f / %6.2f\n',process.t,process.tStop));
    if (controller.running)
        status = 'ON';
        color = 'g';
    else
        status = 'OFF';
        color = 'r';
    end
    set(fig.text.status,'string',sprintf('CONTROLLER: %s',status),'color',color);
    
    % Quadcopter
    fig.view0.spar = DrawBox(fig.view0.spar,...
                             process.x,process.z,-process.theta,...
                             process.sparLength,process.sparWidth);
    fig.view0.leftRotor = DrawBox(fig.view0.leftRotor,...
                                  process.x-0.5*process.sparLength*cos(process.theta),...
                                  process.z+0.5*process.sparLength*sin(process.theta),...
                                  -process.theta,process.rotorLength,process.rotorWidth);
	fig.view0.rightRotor = DrawBox(fig.view0.rightRotor,...
                                   process.x+0.5*process.sparLength*cos(process.theta),...
                                   process.z-0.5*process.sparLength*sin(process.theta),...
                                   -process.theta,process.rotorLength,process.rotorWidth);
    
end
drawnow;
end

function box = DrawBox(box,x,y,theta,boxLength,boxWidth)
R = [cos(theta) -sin(theta);
     sin(theta) cos(theta)];
o = [x;y];
p = 0.5*[-boxLength boxLength boxLength -boxLength -boxLength;
         -boxWidth -boxWidth boxWidth boxWidth -boxWidth];
p = repmat(o,1,size(p,2))+R*p;
if isempty(box)
    box = fill(p(1,:),p(2,:),'r');
else
    set(box,'xdata',p(1,:),'ydata',p(2,:));
end
end

function [p,f]=GetRobotModel(filename)
load(filename);
end

%
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% FUNCTIONS THAT (I HOPE) WILL REMAIN THE SAME FOR ALL PROCESSES
%

function RunSimulation(process,controller)

% START-UP

% Create empty figure.
fig = [];

% Flag to stop simulation on keypress.
global done
done = false;

% Start making movie, if necessary.
if (~isempty(process.moviefile))
    myV = VideoWriter(process.moviefile,'MPEG-4');
    myV.Quality = 100;
    myV.FrameRate = 1/process.tStep;
    open(myV);
end

% LOOP

% Loop until break.
tStart = tic;
while (1)
    
    % Update figure (create one if fig is empty).
    fig = UpdateFigure(process,controller,fig);
    
    % Update data.
    if (~isempty(process.datafile) && controller.running)
        [process,controller] = UpdateDatalog(process,controller);
    end
    
    % If making a movie, store the current figure as a frame.
    if (~isempty(process.moviefile))
        frame = getframe(gcf);
        writeVideo(myV,frame);
    end
    
    % Stop if time has reached its maximum.
    if ((process.t>=process.tStop)||done)
        break;
    end
    
    % Update process (integrate equations of motion).
    [process,controller] = UpdateProcess(process,controller);
    
    % Wait if necessary, to stay real-time.
    while (toc(tStart)<process.t-process.tStart)
        % Do nothing
    end
    
end

% SHUT-DOWN

% Close and save the movie, if necessary.
if (~isempty(process.moviefile))
    for i=1:myV.FrameRate
        frame = getframe(gcf);
        writeVideo(myV,frame);
    end
    close(myV);
end

% Save the data.
if (~isempty(process.datafile))
    processdata = process.log.process; %#ok<NASGU>
    controllerdata = process.log.controller; %#ok<NASGU>
    save(process.datafile,'processdata','controllerdata');
end

% Save the snapshot, if necessary.
if (~isempty(process.snapshotfile))
    set(gcf,'paperorientation','landscape');
    set(gcf,'paperunits','normalized');
    set(gcf,'paperposition',[0 0 1 1]);
    print(gcf,'-dpdf',process.snapshotfile);
end

end


function [process,controller] = UpdateDatalog(process,controller)
% Create data log if it does not already exist.
if (~isfield(process,'log'))
    process.log = struct('process',struct,...
                         'controller',struct('tComputation',[],...
                                             'sensors',struct,...
                                             'actuators',struct,...
                                             'data',struct),...
                         'count',0);
end
% Increment log count.
process.log.count = process.log.count+1;
% Write data to log.
for i=1:length(process.processdatatolog)
    name = process.processdatatolog{i};
    process.log.process.(name)(:,process.log.count) = process.(name);
end
process.log.controller.tComputation(:,process.log.count) = ...
    controller.tComputation;
names = fieldnames(controller.sensors);
for i=1:length(names)
    name = names{i};
    process.log.controller.sensors.(name)(:,process.log.count) = ...
        controller.sensors.(name);
end
names = fieldnames(controller.actuators);
for i=1:length(names)
    name = names{i};
    process.log.controller.actuators.(name)(:,process.log.count) = ...
        controller.actuators.(name);
end
for i=1:length(process.controllerdatatolog)
    name = process.controllerdatatolog{i};
    try
        process.log.controller.data.(name)(:,process.log.count) = ...
            controller.data.(name);
    catch exception
        warning(['Saving element ''%s'' of data for controller\n',...
                 '     ''%s''',...
                 'threw the following error:\n\n' ...
                 '==========================\n' ...
                 '%s\n', ...
                 '==========================\n\n' ...
                 'Turning off controller and setting all\n' ...
                 'actuator values to zero.\n'],...
                 name,controller.name,getReport(exception));
        controller.actuators = ZeroActuators();
        controller.running = false;
        return
    end
end
end


function [process,controller] = UpdateProcess(process,controller)
% Integrate equations of motion
[t0,x] = Get_TandX_From_Process(process);
u = GetInput(process,controller.actuators);
[t,x] = ode45(@(t,x) GetXDot(t,x,u,process),[t0 t0+process.tStep],x);
process = Get_Process_From_TandX(t(end),x(end,:)',process);

% Get sensor values
controller.sensors = GetSensors(process);

% Get actuator values (run controller)
if (controller.running)
    tic
    try
        [controller.actuators,controller.data] = ...
            controller.run(controller.sensors, ...
                              controller.references, ...
                              controller.parameters, ...
                              controller.data);
    catch exception
        warning(['The ''run'' function of controller\n     ''%s''\n' ...
                 'threw the following error:\n\n' ...
                 '==========================\n' ...
                 '%s\n', ...
                 '==========================\n\n' ...
                 'Turning off controller and setting all\n' ...
                 'actuator values to zero.\n'],controller.name,getReport(exception));
        controller.actuators = ZeroActuators();
        controller.running = false;
    end
    if (~isstruct(controller.actuators) || ~CheckActuators(controller.actuators))
        warning(['The ''run'' function of controller\n     ''%s''\n' ...
                 'did not return a structure ''actuators'' with the right\n' ...
                 'format. Turning off controller and setting all\n' ...
                 'actuator values to zero.\n'],controller.name);
        controller.actuators = ZeroActuators();
        controller.running = false;
    end
    controller.tComputation = toc;
else
    controller.tComputation = 0;
end

end

%
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% HELPER FUNCTIONS
%

function onkeypress(src,event)
global done
if event.Character == 'q'
    done = true;
end
end

%
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%