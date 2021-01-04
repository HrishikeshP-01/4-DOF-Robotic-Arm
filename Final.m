clear
clc
addpath(genpath(strcat(pwd,'\Dependencies')))
robot=createRigidBodyTree;
axes=show(robot);
axes.CameraPositionMode='auto';

x=input('Enter waypoint type: ','s');
switch x
    case 'simple'
        wayPoints=[0.2 -0.2 0.02;0.25 0 0.15; 0.2 0.2 0.02];
        wayPointVels = [0 0 0;0 0.1 0;0 0 0];
    case 'complex'
        wayPoints = [0.2 -0.2 0.02;0.15 0 0.28;0.15 0.05 0.2; 0.15 0.09 0.15;0.1 0.12 0.1; 0.04 0.1 0.2;0.25 0 0.15; 0.2 0.2 0.02];
        wayPointVels = [0 0 0; 0 0.05 0; 0 0.025 -0.025; 0 0.025 -0.025;-0.1 0 0;0.05 0 0;0 -0.1 0;0 0 0];
end
exampleHelperPlotWayPoints(wayPoints);

numTotalPoints=size(wayPoints,1)*10;
waypointTime=4;
trajType=input('Enter trajectory type: ','s');
switch trajType
    case 'trapezoidal'
        trajectory=trapveltraj(wayPoints',numTotalPoints,'EndTime',waypointTime);
    case 'cubic'
        wpTimes=(0:size(wayPoints,1)-1)*waypointTime;
        trajTimes=linspace(0,wpTimes(end),numTotalPoints);
        trajectory=cubicpolytraj(wayPoints',wpTimes,trajTimes,'VelocityBoundaryCondition',wayPointVels');
end
hold on
plot3(trajectory(1,:),trajectory(2,:),trajectory(3,:),'r-','LineWidth',2);

ik=robotics.InverseKinematics('RigidBodyTree',robot);
weights=[0.1 0.1 0 1 1 1];
initialguess=robot.homeConfiguration;
for idx=1:size(trajectory,2)
    tform=trvec2tform(trajectory(:,idx)');
    configSoln(idx,:)=ik('end_effector',tform,weights,initialguess);
    initialguess=configSoln(idx,:);
end

title('Robot waypoint tracking visualization')
axis([-0.1 0.4 -0.35 0.35 0 0.35]);
for idx=1:size(trajectory,2)
    show(robot,configSoln(idx,:),'PreservePlot',false,'Frames','off');
    pause(0.1)
end
hold off

function [robot, homeConfig] = createRigidBodyTree
try
    robot=importrobot('open_manipulator.urdf');
    disp('Loading model...');
    gravityVec=[0 0 -9.80665];
    robot.Gravity=gravityVec;
    eeOffset=0.12;
    eeBody=robotics.RigidBody('end_effector');
    setFixedTransform(eeBody.Joint,trvec2tform([eeOffset 0 0]));
    addBody(robot,eeBody,'link5');
catch
    warning('Error importing URDF file. Loading presaved object. ');
    load openManipulatorDescription robot
end
homeConfig=robot.homeConfiguration;
end

function exampleHelperPlotSpheres(size,pointPosition)
[X,Y,Z]=sphere(20);
X=size*X+pointPosition(1);
Y=size*Y+pointPosition(2);
Z=size*Z+pointPosition(3);
s=patch(surf2patch(X,Y,Z));
s.FaceColor='blue';
s.FaceLighting='gouraud';
s.EdgeAlpha=0;
lightObj=findobj(gca,'Type','Light');
lightObj.Position=[1 1 1];
end

function exampleHelperPlotWayPoints(wayPoints)
for idx=1:size(wayPoints,1)
    exampleHelperPlotSpheres(0.01,wayPoints(idx,:));
end
end


