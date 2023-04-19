% biped_walk_animation.m
% This script is to visualize the MPC results with the biped robot model.
%% 
addpath ./visualizationFuncs

global uLINK
SetupBipedRobot;

% flag_movie = 1;
if flag_movie    
    try
        name = 'LMPC_BipedWalking1.mp4';
        vidfile = VideoWriter(name,'MPEG-4');
    catch ME
        name = 'LMPC_BipedWalking';
        vidfile = VideoWriter(name,'Motion JPEG AVI');
    end
    vidfile.FrameRate = 10;
    vidfile.Quality = 100;
    open(vidfile);
end

fig = figure(1001); clf;
fig.Color = 'w';
fig.Position = [200 100 1000 750];
set(0, 'DefaultFigureRenderer', 'opengl');
set(gcf, 'Color', 'white');

%% 
for i = 1:desired_walking_time
    % compute the inverse kinematics
    uLINK(BODY).p = [X_total(i, 1), Y_total(i, 1), h]';
    uLINK(BODY).R = eye(3);

    Rfoot.p = rfoot_tra(i, :)';
    Rfoot.R = eye(3);

    Lfoot.p = lfoot_tra(i, :)';
    Lfoot.R = eye(3);

    %%% Analytical inverse kinematics solution
    qR2 = IK_leg(uLINK(BODY), -half_step_width, 0.32, 0.32, Rfoot);
    qL2 = IK_leg(uLINK(BODY),  half_step_width, 0.32, 0.32, Lfoot);

    for n=0:5
        uLINK(RLEG_J0+n).q = qR2(n+1);
        uLINK(LLEG_J0+n).q = qL2(n+1);
    end
    
    clf;
    ForwardKinematics(1);
    DrawAllJoints(1);

    view(29.78, 13.54)
    axis equal
    AX = [X_total(1,1)-1.0, X_total(end,1)+1.0];    AY = [-0.5, 0.5];   AZ = [-0.1  1.5];
    xlim(AX);    ylim(AY);    zlim(AZ);
    grid on

    % ground
    xBnd = AX(1):0.01:AX(2);
    yBnd = AY(1):0.01:AY(2);

    [Gnd.x, Gnd.y] = meshgrid(xBnd, yBnd);
    Gnd.z = zeros(size(Gnd.x));
    GndColor = [255 228 181] / 255;

    surf(Gnd.x, Gnd.y, Gnd.z, 'EdgeColor', 'none', 'FaceColor', GndColor);
    
    xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
    
    formatSpec = 'Biped walking -- time: %4.2f s';
    
    title(sprintf(formatSpec, t(i)));
    
    set(gca, 'FontSize', 20);
    
    if flag_movie
        writeVideo(vidfile, getframe(gcf));
    end
    
    drawnow;
    
    pause(0.01);
    
end

if flag_movie
    close(vidfile);
end

rmpath ./visualizationFuncs