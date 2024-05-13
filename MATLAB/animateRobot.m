function animateRobot(robot, jointAnglesSequence)
    numSteps = size(jointAnglesSequence, 1);
    hFig = figure;
    hold on;
    axis tight manual; % this ensures that getframe() returns a consistent size
    filename = 'robot_animation.gif'; % Name of the GIF file

    % Set viewing parameters
    view(3); % Standard 3D view
    axis([-0.5 0.5 -0.5 0.5 0 1]); % Adjust these values to zoom in or out
    daspect([1 1 1]); % Ensures equal aspect ratio along all axes
    camzoom(1.25); % Zooms the camera by 1.5 times, adjust as necessary
    for step = 1:numSteps
        config = homeConfiguration(robot);
        for i = 1:length(config)
            config(i).JointPosition = jointAnglesSequence(step, i);
        end
        
        % Clear previous animations and show the current configuration
        cla;
        show(robot, config, 'Frames', 'off');
        drawnow;
   
        
      
        
        % Capture the plot as an image 
        frame = getframe(hFig);
        im = frame2im(frame);
        [imind, cm] = rgb2ind(im, 256);
        
        % Write to the GIF File 
        if step == 1
            imwrite(imind, cm, filename, 'gif', 'Loopcount', inf, 'DelayTime', 0.1);
        else
            imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append', 'DelayTime', 0.1);
        end
    end
end
