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
process.processdatatolog = {'t','x','xdot','y','ydot','z','zdot','theta','phi','psi'};

% Constants related to physical properties.
% - Acceleration of gravity.
process.g = gravity;
% - Mass
process.m = mass;
% - Maximum thrust
process.maxthrust = maxThrust;
% - Minimum thrust
process.minthrust = minThrust;
% - Maximum pitch rate
process.maxpitchrate = maxPitchRate;
% - Maximum roll rate
process.maxrollrate = maxRollRate;
% - Maximum yaw rate
process.maxyawrate = maxYawRate;

% DEFINE GEOMETRY
% Geometry of quadrotor
[process.pRobot_in1,process.fRobot]=GetRobotModel('quadmodel.mat');


%===== Comment back in to include axis of room and quadrotor axis
% Geometry of coordinate axes
pFrame = [0 1 0 0;
          0 0 1 0;
          0 0 0 1];
process.pRoomFrame_in0 = pFrame;
process.pRobotFrame_in1 = pFrame;

% DEFINE VARIABLES

% Time
process.t = 0;
% x, xdot - position and velocity
process.x = 0;
process.xdot = 0;
% y, ydot - position and velocity
process.y = 0;
process.ydot = 0;
% z, zdot - position and velocity
process.z = 0;
process.zdot = 0;
% theta - angle
process.theta = 0;
% phi - angle
process.phi = 0;
% psi - angle
process.psi = 0;

% DEFINE CONTROLLER

% Functions
% - get handles to user-defined functions 'init' and 'run'
controller = eval(process.controller);
controller.name = process.controller;
% Parameters
% - define a list of constants that will be passed to the controller
names = {'tStep','g','m','maxthrust','maxpitchrate','maxrollrate','maxyawrate'};
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
sensors.y = process.y;
sensors.ydot = process.ydot;
sensors.z = process.z;
sensors.zdot = process.zdot;
sensors.theta = process.theta;
sensors.phi = process.phi;
sensors.psi = process.psi;
% Add noise
%   (nothing)
end

function [t,x] = Get_TandX_From_Process(process)
t = process.t;
x = [process.x; process.xdot;
     process.y; process.ydot; 
     process.z; process.zdot;
     process.theta; 
     process.phi;
     process.psi];
end

function u = GetInput(process,actuators)
% Copy input from actuators
u = [actuators.pitchrate; actuators.rollrate; actuators.yawrate; actuators.thrust];

% Initialize inputs to zero
u = zeros(4,1);

% Copy pitch rate from actuators
%==========================================================
if actuators.pitchrate < -process.maxpitchrate
    u(1) = -process.maxpitchrate;
elseif actuators.pitchrate > process.maxpitchrate
    u(1) = process.maxpitchrate;
else
    u(1) = actuators.pitchrate;
end

% Copy roll rate from actuators
%==========================================================
if actuators.rollrate< -process.maxrollrate
    u(2) = -process.maxrollrate;
elseif actuators.rollrate > process.maxrollrate
    u(2) = process.maxrollrate;
else
    u(2) = actuators.rollrate;
end

% Copy yaw rate from actuators
%==========================================================
if actuators.yawrate< -process.maxyawrate
    u(3) = -process.maxyawrate;
elseif actuators.yawrate > process.maxyawrate
    u(3) = process.maxyawrate;
else
    u(3) = actuators.yawrate;
end

% Copy thrust from actuators
%==========================================================
if actuators.thrust<process.minthrust
    u(4) = process.minthrust;
elseif actuators.thrust > process.maxthrust
    u(4) = process.maxthrust;
else
    u(4) = actuators.thrust;
end
%==========================================================
% Add disturbance
%   (nothing)
end

function process = Get_Process_From_TandX(t,x,process)
process.t = t;
process.x = x(1,1);
process.xdot = x(2,1);
process.y = x(3,1);
process.ydot = x(4,1);
process.z = x(5,1);
process.zdot = x(6,1);
process.theta = x(7,1);
process.phi = x(8,1);
process.psi = x(9,1);


% Just did this to shorten, typing. But this will marginally slow sim
phi = process.phi;
psi = process.psi;
theta = process.theta;

% Frame in origin frame
o_1in0 = [process.x;process.y;process.z];       % - copied for convenience


Rz = [cos(psi) -sin(psi) 0;
      sin(psi) cos(psi) 0;
      0 0 1];

Ry = [cos(theta) 0 sin(theta);
      0 1 0;
      -sin(theta) 0 cos(theta)];
  
Rx = [1 0 0;
      0 cos(phi) -sin(phi);
      0 sin(phi) cos(phi)];

% Rotation Matrix  
R_1in0= Rz*Ry*Rx;
         
% Coordinate transformation for robot and then axis
process.pRobot_in0 = R_1in0*process.pRobot_in1 +repmat(o_1in0,1,length(process.pRobot_in1));        % - coordinate transformation from process.pRobot_in1
process.pRobotFrame_in0 = R_1in0* process.pRobotFrame_in1 +repmat(o_1in0,1,length(process.pRobotFrame_in1));  % - coordinate transformation from process.pRobotFrame_in1

end

function XDOT = GetXDot(T,X,U,process)
% unpack X and U
x = X(1,1);
xdot = X(2,1);
y = X(3,1);
ydot = X(4,1);
z = X(5,1);
zdot = X(6,1);
theta = X(7,1);
phi = X(8,1);
psi = X(9,1);

w = U(1,1);
p = U(2,1);
r = U(3,1);
f = U(4,1);
% compute rates of change
d_x = xdot;
d_xdot = (f/process.m)*(cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi)) + 0; %MODIFICATION: Wind

d_y = ydot;
d_ydot = (f/process.m)*(sin(psi)*sin(theta)*cos(phi)-cos(psi)*sin(phi));

d_z = zdot;
d_zdot = (f/process.m)*cos(theta)*cos(phi)-process.g;

d_theta = w;
d_phi = p;
d_psi = r;

% pack XDOT
XDOT = [d_x; d_xdot; 
        d_y; d_ydot; 
        d_z; d_zdot; 
        d_theta;
        d_phi;
        d_psi];
end

function iscorrect = CheckOneActuatorValue(val)
iscorrect = isnumeric(val)&&isscalar(val)&&~isnan(val)&&~isinf(val);
end

function iscorrect = CheckActuators(actuators)
iscorrect = false;
if isfield(actuators,{'thrust'})&&isfield(actuators,{'pitchrate'})...
        &&isfield(actuators,{'rollrate'})&&isfield(actuators,{'yawrate'})...
        &&(length(fieldnames(actuators))==4)
    if CheckOneActuatorValue(actuators.thrust)&&CheckOneActuatorValue(actuators.pitchrate)...
       &&CheckOneActuatorValue(actuators.rollrate)&&CheckOneActuatorValue(actuators.yawrate)
        iscorrect = true;
    end
end
end

function actuators = ZeroActuators()
actuators = struct('thrust',0,'pitchrate',0,'rollrate',0,'yawrate',0);
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
%     set(gcf,'renderer','opengl');
    axis equal;
    axis([-7 7 -7 7 -7 7]);
    
    axis manual;
    hold on;
    axis on;
    grid on
    grid minor
    xlabel('X')
    ylabel('Y')
    zlabel('Z')
    box on;
    
%     view([0 0])  % XZ Plane
%     view([0,90]) % XY Plane
%     view([90,0]) %YZ Plane
     view([45,20])
      % Draws room axis: 
      fig.roomframe = DrawFrame([],process.pRoomFrame_in0);
    

      % Draws robot and then robot axis
      fig.robot = DrawMesh([],process.pRobot_in0,process.fRobot,[.9 .7 .9],1);
      fig.robotframe = DrawFrame([],process.pRobotFrame_in0);
    
    
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
    
    
      % Draws robot and then robot axis
      fig.robot = DrawMesh(fig.robot,process.pRobot_in0,process.fRobot);
      fig.robotframe = DrawFrame(fig.robotframe,process.pRobotFrame_in0);
    
end
drawnow;
end


    % Got rid of this function:
% function box = DrawBox(box,x,y,theta,boxLength,boxWidth)

function [p,f]=GetRobotModel(filename)
load(filename);
end

% Coppied in this function
function mesh = DrawMesh(mesh,p,f,color,alpha)
if isempty(mesh)
    mesh = patch('Vertices',p','Faces',f,...
                 'FaceColor',color,'FaceAlpha',alpha,'EdgeAlpha',alpha);
else
    set(mesh,'vertices',p');
end
end
% Also Copied in this function
function frame = DrawFrame(frame,p)
if isempty(frame)
    frame.x = plot3(p(1,[1 2]),p(2,[1 2]),p(3,[1 2]),'r-','linewidth',3);
    frame.y = plot3(p(1,[1 3]),p(2,[1 3]),p(3,[1 3]),'g-','linewidth',3);
    frame.z = plot3(p(1,[1 4]),p(2,[1 4]),p(3,[1 4]),'b-','linewidth',3);
else
    set(frame.x,'xdata',p(1,[1 2]),'ydata',p(2,[1 2]),'zdata',p(3,[1 2]));
    set(frame.y,'xdata',p(1,[1 3]),'ydata',p(2,[1 3]),'zdata',p(3,[1 3]));
    set(frame.z,'xdata',p(1,[1 4]),'ydata',p(2,[1 4]),'zdata',p(3,[1 4]));
end
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
    
    % Update process (integrate equations of motion).
    [process,controller] = UpdateProcess(process,controller);  
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
