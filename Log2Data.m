%%% Format log for Matlab MHE


%% Import log file
% Assumes there are two folders in the current directory/path:
% 'Logs/': contains the raw crazyflie logs in csv format
% 'Data/': contains .mat files loaded by the estimator
logName = 'square';
logLocation = 'Logs/';
file_in = strcat(logLocation,logName,'.csv');
file_out = strcat('Data/', logName,'.mat');
log = readtable(file_in,'PreserveVariableNames',true);

%% Generate data structure
mode = rangingModes.twr;
% The variable names in the 'log' table are the headers of the .csv file
data.time = (log.timeTick - log.timeTick(1))/1000;

data.x = log.cfX;
data.y = log.cfY;
data.z = log.cfZ;

data.roll = log.roll * 2 * pi / 360;
data.pitch = log.pitch * 2 * pi / 360;
data.yaw = log.yaw * 2 * pi / 360;

% Anchor ids and positions
%anchors(1) = Anchor(0,[-4.66, -4.75, 1.76]);
%anchors(2) = Anchor(1,[-4.83,  4.59, 1.70]);
%anchors(3) = Anchor(2,[ 4.84,  4.61, 0.88]);
%anchors(4) = Anchor(3,[ 4.74, -4.73, 1.61]);

anchors(1) = Anchor(0,[-4.66,  4.75, -1.76]);
anchors(2) = Anchor(1,[-4.83, -4.59, -1.70]);
anchors(3) = Anchor(2,[ 4.84, -4.61, -0.88]);
anchors(4) = Anchor(3,[ 4.74,  4.73, -1.61]);

if mode == rangingModes.twr
    dist_corrector = -0.161; % correction due to miscalibration of uwb
    log.twr0 = log.twr0 - dist_corrector;
    log.twr1 = log.twr1 - dist_corrector;
    log.twr2 = log.twr2 - dist_corrector;
    log.twr3 = log.twr3 - dist_corrector;

    j = 1;
    for i=2:length(log.timeTick)
        if log.twr0(i) ~= log.twr0(i-1)
            data.uwb(j) = TWRPacket(anchors(1), log.twr0(i), data.time(i));
            j = j+1;
        end
        if log.twr1(i) ~= log.twr1(i-1)
            data.uwb(j) = TWRPacket(anchors(2), log.twr1(i), data.time(i));
            j = j+1;
        end
        if log.twr2(i) ~= log.twr2(i-1)
            data.uwb(j) = TWRPacket(anchors(3), log.twr2(i), data.time(i));
            j = j+1;
        end
        if log.twr3(i) ~= log.twr3(i-1)
            data.uwb(j) = TWRPacket(anchors(4), log.twr3(i), data.time(i));
            j = j+1;
        end
    end
elseif mode == rangingModes.tdoa
    dist_corrector = -0.161; % correction due to miscalibration of uwb
    log.tdoa01 = log.tdoa01 - dist_corrector;
    log.tdoa02 = log.tdoa02 - dist_corrector;
    log.tdoa03 = log.tdoa03 - dist_corrector;
    log.tdoa12 = log.tdoa12 - dist_corrector;
    log.tdoa13 = log.tdoa13 - dist_corrector;
    log.tdoa23 = log.tdoa23 - dist_corrector;

    j = 1;
    for i=2:length(log.timeTick)
        if log.tdoa01(i) ~= log.tdoa01(i-1)
            data.uwb(j) = TDOAPacket(anchors(1),anchors(2), log.tdoa01(i), data.time(i));
            j = j+1;
        end
        if log.tdoa02(i) ~= log.tdoa02(i-1)
            data.uwb(j) = TDOAPacket(anchors(1),anchors(3), log.tdoa02(i), data.time(i));
            j = j+1;
        end
        if log.tdoa03(i) ~= log.tdoa03(i-1)
            data.uwb(j) = TDOAPacket(anchors(1),anchors(4), log.tdoa03(i), data.time(i));
            j = j+1;
        end
        if log.tdoa12(i) ~= log.tdoa12(i-1)
            data.uwb(j) = TDOAPacket(anchors(2),anchors(3), log.tdoa12(i), data.time(i));
            j = j+1;
        end
        if log.tdoa13(i) ~= log.tdoa13(i-1)
            data.uwb(j) = TDOAPacket(anchors(2),anchors(4), log.tdoa13(i), data.time(i));
            j = j+1;
        end
        if log.tdoa23(i) ~= log.tdoa23(i-1)
            data.uwb(j) = TDOAPacket(anchors(3),anchors(4), log.tdoa23(i), data.time(i));
            j = j+1;
        end
    end
end
    
%% Save data
save(file_out,"data");