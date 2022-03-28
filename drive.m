function ~ = drive(timeBounds, leftSpeed, rightSpeed)
    pub = rospublisher('raw_vel');

    %Here we are creating a ROS message
    message = rosmessage(pub);  

    %Set the right and left wheel velocities 
    message.Data = [leftSpeed, rightSpeed]; 

    % Send the velocity commands to the NEATO
    send(pub, message);
    
    pause(0.01);

end
