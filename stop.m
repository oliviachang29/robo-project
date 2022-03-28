function [] = stop()
    pub = rospublisher('raw_vel');
    stopMsg = rosmessage(pub);
    stopMsg.Data = [0 0];
    send(pub, stopMsg);
end

