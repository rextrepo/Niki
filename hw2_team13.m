function hw2_v1(r)
neo = strcmp(class(r), 'CreateRobot');

ONLINE = 1;
BACKING = 2;
TURNINGTOWALL = 3;
ONWALL = 4;
MAYBELOSTWALL = 5;
SPIRALLING = 6;
statenames = 'LBTOMS';
state = ONLINE;

theta = 0; % current angle (radians) robot is facing
x = 0;
y = 0;

xhist = [0];
yhist = [0];
pmin = -1;
pmax = 1;

xStart = 0;
yStart = 0;
xFinal = .5;
yFinal = 0;

% y = mx + b
m = (yFinal - yStart) / (xFinal - xStart);
b = yStart - m * xStart;
threshold = .1;

firstBumpX = inf;
firstBumpY = inf;
firstBumpTheta = inf;

distTraveled = 0;
iterationsSinceFirstBump = -inf;

if neo
    threshratio=20;
else
    threshratio=10;
end

beenstraight = 0; % How long we've been parellel to the wall (if high enough, speed up)
touchingWhileTurning=0; % Used in TURNING state, indicates whether we should be touching the wall
turnedforcnt=0; % How many iterations we've been in state TURNING
turnedforangle=0; % How many degrees we've been in state TURNING
angledeg=0; % Angle we most recently turned in degrees
% When spiraling, we move in a series of straight lines (aka 'segments'), each longer than the one before
spiralseg=0; % What segment of the spiral we're in
spiralcnt=0; % What iteration within the segment we're in (while spiraling)

% In the simulator, Wall sensor works best if we're not moving, so we want to pause,read,resume
% These let us save our current motion so that we can resume
curV=inf;
curTH=inf;
    function SaveSetFwdVelAngVelCreate(r,v,th)
        curV=v;
        curTH=th;
        SetFwdVelAngVelCreate(r,v,th);
    end

% Turn by dth degrees ccw without going back to the main loop, handling odometry, then go forward at newspeed
    function myTurn(dth,newspeed)
        SaveSetFwdVelAngVelCreate(r,0,0);
        pause(.01);
        odometry();
        turnAngle(r,.1,dth);
        SaveSetFwdVelAngVelCreate(r,0,0);
        pause(.01);
        odometry();
        SaveSetFwdVelAngVelCreate(r,newspeed,0);
    end

% Read Angle and Distance sensors, then calculate where we are
    function odometry()
        Angle = AngleSensorRoomba(r);
        Dist = DistanceSensorRoomba(r);
        % Compensate for systematic bias in simulator (at least as it was configured for us)
        if neo
            Angle = Angle / 0.98;
            Dist = Dist / 0.98;
        end
        distTraveled = distTraveled + Dist;
        angledeg=Angle*180/pi;
        theta = theta+Angle;
        x = x + Dist * cos(theta);
        y = y + Dist * sin(theta);
        % Print debugging info
        sprtxt=''; % Extra state descriptor if we're in state SPIRALLING
        if (state == SPIRALLING)
            sprtxt=sprintf(' <%d/%d>', spiralcnt, spiralseg);
        end
        disp(sprintf('state=%s%s bump=%d%d%d w=%d moved=%.2f turned=%.1f now at %.2f,%.2f facing %.1f',statenames(state),sprtxt,BumpLeft,BumpFront,BumpRight,Wall,Dist,angledeg,x,y,theta*180/pi));
        % If we're running in real life, keep track of where we've been and plot it
        % Don't do this in the simulator because it interferes
        if ~neo
            xhist = [xhist, x];
            yhist = [yhist, y];
            plot([xhist],[yhist]);
            pmax = max([pmax, x, y]);
            pmin = min([pmin, x, y]);
            axis([pmin, pmax, pmin, pmax]);
        end
    end

% function change?
% Transition to a new state, issuing commands to the robot as needed
    function change(ns)
        switch ns
            case ONLINE
                endAngle = pi + atan((y-yStart) / (x-xStart));
                diffAngle = (endAngle - mod(theta, 2 * pi)) * 180 / pi;
                if(diffAngle > 180)
                    diffAngle = diffAngle - 360;
                end
                
                myTurn(diffAngle, .05);
                SaveSetFwdVelAngVelCreate(r, .1, 0);
                
            case BACKING
                SaveSetFwdVelAngVelCreate(r,-.01,0);
            case TURNINGTOWALL
                odometry;
                if neo
                    SaveSetFwdVelAngVelCreate(r,0,0.1);
                else
                    SaveSetFwdVelAngVelCreate(r,0,0.4);
                end
                turnedforcnt=0;
                turnedforangle=0;
                touchingWhileTurning=(state~=BACKING);
            case ONWALL
                beenstraight = 0;
                if iterationsSinceFirstBump == -inf
                    iterationsSinceFirstBump = 0;
                end
                SaveSetFwdVelAngVelCreate(r,0.05,0);
            case MAYBELOSTWALL
                lostfor=0;
            case SPIRALLING
                spiralseg=0;
                spiralcnt=1;
        end
        state=ns;
    end


AngleSensorRoomba(r);
DistanceSensorRoomba(r);

BumpRight = 0;
BumpLeft = 0;
BumpFront = 0;
Wall = 0;

% Calculate angle toward final position
%endAngle = pi + atan((y-yFinal) / (x-xFinal));
endAngle = atan2(yFinal - y, xFinal - x);
diffAngle = (endAngle - mod(theta, 2 * pi)) * 180 / pi;
if(diffAngle > 180)
    diffAngle = diffAngle - 360;
end

myTurn(diffAngle, .05);
SaveSetFwdVelAngVelCreate(r, .1, 0);

while(1)
    if sqrt((x-xFinal)^2 + (y-yFinal)^2) < distTraveled/threshratio
        SaveSetFwdVelAngVelCreate(r,0,0);
        disp('WE HAVE ARRIVED!');
        
        break
    end
    
    
    disp('starting');
    pause(0.0001 + neo*.01); % In reality, reading the sensors is sufficient pause
    % Read bump and wall sensors
    [BumpRight,BumpLeft,WheDropRight,WheDropLeft,WheDropCaster,BumpFront] = BumpsWheelDropsSensorsRoomba(r);
    
    if neo % In simulator, WallSensor is more accurate if we stop moving.
        SetFwdVelAngVelCreate(r,0,0)
    end
    Wall = WallSensorReadRoomba(r);
    if neo
        SetFwdVelAngVelCreate(r,curV,curTH)
    end
    
    
    if state == ONLINE
        % if bumped, go to wall following algorithm
        
        if BumpRight | BumpLeft | BumpFront | Wall
            change(ONWALL)
            disp('triggered sensor');
        else
            disp('still on line?');
            % otherwise keep going
            
            endAngle = atan2(yFinal - y, xFinal - x);
            diffAngle = (endAngle - mod(theta, 2 * pi)) * 180 / pi;
            if(diffAngle > 180)
                diffAngle = diffAngle - 360;
            end
            
            myTurn(diffAngle, .05);
            SaveSetFwdVelAngVelCreate(r, .1, 0);
            
        end
    end
    if state==ONWALL
        iterationsSinceFirstBump = iterationsSinceFirstBump + 1;
        % We don't actually set firstBump the first time we bump, but a few ONWALL iterations afterwards, to make sure we've really bumped properly
        if (neo && iterationsSinceFirstBump==10) || (~neo && iterationsSinceFirstBump==2)
            firstBumpX = x;
            firstBumpY = y;
            firstBumpTheta = theta;
            distTraveled = 0;
            disp(sprintf('First Bump! (%.2f, %.2f facing %.1f)',firstBumpX,firstBumpY,theta*180/pi));
        end
        % Do odometry periodically while moving forward, so that we'll know when we're done
        if mod(iterationsSinceFirstBump,20)==0
            odometry;
        end
    end
    
    % Check if we're done.  The further we've gone, the less accurate our odometry, so increase the threshold
    % Also, expect better accuracy in the simulator
    
    % If we're close enough to full circumnavigation, and have travelled a respectable distance, and are facing close to our original direction
    % TODO: make it check perpendicular distance D:
    % ALSO... Dude, need to make sure you're not hitting the obstacle still
    %   especially at start... Possibly start a counter?
    if abs(y - (m * x + b)) < threshold && x > firstBumpX
        SaveSetFwdVelAngVelCreate(r,0,0);
        
        state=ONLINE;
        
        
    elseif sqrt((x-firstBumpX)^2 + (y-firstBumpY)^2) < distTraveled/threshratio && distTraveled > 1 && angleDiff(theta,firstBumpTheta) < pi/2
        SaveSetFwdVelAngVelCreate(r,0,0);
        disp('FAILED!');
        
        break
    end
    
    
    % Check if we need to change states
    % We've bumped
    if ((BumpFront && state~=ONWALL) || (state==ONLINE && BumpLeft))
        if neo
            % In the simulator, the wall sensor doesn't work well at range, so turn immediately
            change(TURNINGTOWALL);
        else
            % In reality, turning while touching a wall hurts odometry due to friction, so back away before turning
            change(BACKING);
        end
        % If this is the first time we've bumped, start counting for firstBump
        if iterationsSinceFirstBump == -inf
            iterationsSinceFirstBump = 0;
        end
        % If we've bumped *and* we're on a wall (i.e. we've hit an interior corner) we need to turn now to avoid an infinite loop
    elseif (state==ONWALL && BumpFront)
        myTurn(16,.05);
        % We found the wall
    elseif (state==ONLINE && Wall)
        change(ONWALL);
        iterationsSinceFirstBump = 0;
        % We've finished backing away from a wall
    elseif (state==BACKING && ~BumpFront)
        change(TURNINGTOWALL);
        % We're turning.  This takes more thought.
    elseif (state==TURNINGTOWALL)
        % We're done turning
        if ((touchingWhileTurning && ~BumpRight) || (~touchingWhileTurning && Wall))
            odometry;
            change(ONWALL);
        else
            % Remember to check odometry periodically
            turnedforcnt = turnedforcnt + 1;
            if (turnedforcnt > 20)
                odometry;
                turnedforangle = turnedforangle + angledeg;
                % We've made a complete circle and then some without getting parallel.
                % Instead of spinning forever, go into a spiral.
                if turnedforangle > 450
                    state = SPIRALLING;
                end
            end
        end
        % Stuff to do on the wall
    elseif (state==ONWALL)
        % We bumped the wall, turn away
        if  (BumpRight)
            change(TURNINGTOWALL);
            % We lost the wall
        elseif (~Wall)
            change(MAYBELOSTWALL);
            % We've entered a narrow crevice, turn to get out
        elseif (BumpLeft)
            myTurn(90,0);
            change(TURNINGTOWALL);
        else
            beenstraight = beenstraight + 1;
            % If we've been straight for a while, then we must be really parallel, so speed up
            if (beenstraight > 5)
                SaveSetFwdVelAngVelCreate(r,0.2,0);
            end
        end
        % Stuff to do when we might have lost the wall
    elseif (state==MAYBELOSTWALL)
        % Found it again
        if (BumpRight)
            myTurn(2,.05);
            change(ONWALL);
        elseif (Wall)
            change(ONWALL);
        end
        % Count how long we've been lost
        lostfor = lostfor + 1;
        % Turn a little toward the wall
        if (lostfor == 15 + 15*neo)
            myTurn(-2,.05);
        end
        % Give up, go find the wall for real
        if (lostfor == 30 + 30*neo)
            myTurn(-90,.05);
            change(SPIRALLING);
        end
        % Stuff to do while spiralling
    elseif (state==SPIRALLING)
        % Found the wall
        if (BumpLeft || BumpRight)
            change(BACKING);
        end
        % Count how long we've been spiralling
        spiralcnt = spiralcnt + 1;
        % Time for a turn and a new segment
        if (spiralcnt > 30*spiralseg)
            spiralcnt = 0;
            spiralseg = spiralseg + 1;
            myTurn(-40,.05);
        end
    end
end
end

% Read the wall sensor, using appropriate code for physical robot or simulator
function [wall_sensor] = WallSensorReadRoomba(serPort)
if strcmp(class(serPort), 'serial')
    try
        set(serPort,'timeout',.01);
        %Flush buffer
        N = serPort.BytesAvailable();
        while(N~=0)
            fread(serPort,N);
            N = serPort.BytesAvailable();
        end
    catch
    end
    fwrite(serPort, [142 8]);
    wall_sensor = fread(serPort, 1);
else
    wall_sensor = genIR(serPort);
end
end

% Absolute value difference between two angles, subtracting 2pi as needed for smallest correct value
function [d] = angleDiff(a,b)
d = mod(a-b, 2*pi);
if d > pi
    d = d - 2*pi;
end
d = abs(d);
end

function b= online(x,y)
d= (x+y)/(sqrt((x-10)^2 + (y-0)^2));
if( d<2)
    b=true;  
else
    b=false;
end
end

function turnToward(serPort, x,y)
 thet=180*((atan(y/x))/pi); 
 turnAngle(serPort,.1,thet) 
end

end

