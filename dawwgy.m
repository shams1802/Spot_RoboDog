% SPOT_CIRCLE_ANIMATION (script)
% Single-file MATLAB script to animate a Spot-like quadruped trotting
% around a circle. No toolboxes or external assets required.
%
% Usage: open this file and press Run.

%% -------- Parameters (tunable) --------
params = struct();

% Body geometry (meters)
params.bodyLength = 0.349;  % from CAD body_v1 (approx 349 mm)
params.bodyWidth  = 0.108;  % from CAD body_v1 (approx 108 mm)
params.bodyHeight = 0.050;  % from CAD body_v1 (approx 50 mm)

% Hip locations relative to body center (x forward, y left, z up)
params.hipForwardX = 0.17;  % ~ bodyLength/2 - small margin
params.hipLateralY = 0.054; % ~ bodyWidth/2
params.hipHeightZ  = -0.00; % motors approximately centered in body height

% Leg link lengths (planar sagittal IK)
params.upperLen = 0.206;  % upper leg rotor-to-rotor from CAD (~206 mm)
params.lowerLen = 0.208;  % lower leg rotor-to-foot center from CAD (~208 mm)

% Gait and path
params.circleRadius   = 0.60;     % meters
params.walkSpeed_mps  = 0.25;     % nominal tangential speed
params.stepFrequency  = 1.8;      % Hz (per leg)
params.dutyFactor     = 0.6;      % stance fraction [0..1]
params.stepHeight     = 0.03;     % meters
params.clearanceDrop  = 0.005;    % extra drop at start/end of stance

% Pose dynamics
params.bodyYawLead  = 0.0;        % radians lead vs tangent
params.bodyPitchAmp = 3*pi/180;   % small periodic pitch
params.bodyRollAmp  = 2*pi/180;   % small periodic roll

% Timing
params.totalTime  = 18;   % seconds
params.fps        = 45;   % frames per second

% Colors
params.colorBody  = [0.92 0.73 0.23]; % Boston-ish mustard
params.colorBlack = [0.08 0.08 0.10];
params.colorJoints= [0.20 0.20 0.22];
params.bgColor    = [0.97 0.98 1.00]; % background color

% Rendering
params.lineWidthLeg = 3.5;
params.markerSize   = 10;

%% -------- Derived values --------
T = 1/params.fps;
N = floor(params.totalTime * params.fps);

strideLen = params.walkSpeed_mps / params.stepFrequency; % meters per cycle

% Leg names and diagonal pair phase offsets (trot gait)
% Order: FL, FR, RL, RR
legNames = {"FL","FR","RL","RR"};
phaseOffsets = [0, 0.5, 0.5, 0];  % FL/RR in phase, FR/RL opposite

% Hip positions in body frame
hipPosBody = [
    params.hipForwardX,  params.hipLateralY,  params.hipHeightZ;  % FL
    params.hipForwardX, -params.hipLateralY,  params.hipHeightZ;  % FR
   -params.hipForwardX,  params.hipLateralY,  params.hipHeightZ;  % RL
   -params.hipForwardX, -params.hipLateralY,  params.hipHeightZ]; % RR

%% -------- Figure setup --------
fig = figure('Name','Spot-like Quadruped - Circle Trot','Color',params.bgColor);
ax = axes(fig); hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
xlabel(ax,'X (m)'); ylabel(ax,'Y (m)'); zlabel(ax,'Z (m)');
view(ax, 35, 20);
set(ax,'Color',params.bgColor);

% Ground and circle visualization
theta = linspace(0, 2*pi, 300);
xc = params.circleRadius * cos(theta);
yc = params.circleRadius * sin(theta);
zc = zeros(size(theta));
plot3(ax, xc, yc, zc, '--', 'Color',[0.55 0.55 0.65]);

% Ground plane under the path
planeR = params.circleRadius + 0.8;
[Xp,Yp] = meshgrid(linspace(-planeR,planeR,2), linspace(-planeR,planeR,2));
Zp = zeros(size(Xp)) - 0.001;
surf(ax, Xp, Yp, Zp, 'FaceColor',[0.92 0.93 0.95], 'EdgeColor','none', 'FaceAlpha',1.0);
lighting gouraud; camlight('headlight');

% Pre-create graphics objects for speed
[bodyVerts, bodyFaces] = makeBodyMesh(params);
bodyPatch = patch('Faces',bodyFaces,'Vertices',bodyVerts, ...
    'FaceColor',params.colorBody,'EdgeColor','none','FaceAlpha',1.0, ...
    'FaceLighting','gouraud','SpecularStrength',0.3);
bodyEdges = plotBodyEdges(ax, params, params.colorBlack);

legLines = gobjects(4,2); % [upper, lower]
footMarkers = gobjects(4,1);
hipMarkers  = gobjects(4,1);
for i = 1:4
    legLines(i,1) = plot3(ax, [0 0], [0 0], [0 0], '-', 'Color', params.colorBlack, 'LineWidth', params.lineWidthLeg);
    legLines(i,2) = plot3(ax, [0 0], [0 0], [0 0], '-', 'Color', params.colorBlack, 'LineWidth', params.lineWidthLeg);
    footMarkers(i) = plot3(ax, 0,0,0, 'o', 'MarkerFaceColor', params.colorBlack, 'MarkerEdgeColor','none', 'MarkerSize', params.markerSize);
    hipMarkers(i)  = plot3(ax, 0,0,0, 'o', 'MarkerFaceColor', params.colorJoints, 'MarkerEdgeColor','none', 'MarkerSize', 6);
end

axis(ax, [-(params.circleRadius+0.5) (params.circleRadius+0.5) -(params.circleRadius+0.5) (params.circleRadius+0.5) -0.35 0.35]);

%% -------- Animation loop --------
time = 0;
for k = 1:N
    time = time + T;

    % Body pose following circle (world frame)
    ang = wrapToPi((params.walkSpeed_mps * time) / params.circleRadius); % angle along circle
    posWorld = [params.circleRadius * cos(ang), params.circleRadius * sin(ang), 0.15];
    yawWorld = ang + pi/2 + params.bodyYawLead; % tangent facing
    pitch = params.bodyPitchAmp * sin(2*pi*0.5*time);
    roll  = params.bodyRollAmp  * sin(2*pi*0.7*time + pi/4);

    Rwb = eulZYX(yawWorld, pitch, roll); % world<-body rotation

    % Body vertices transform
    V = bodyVerts * Rwb.' + posWorld; %#ok<NASGU> (for reference)
    set(bodyPatch, 'Vertices', (bodyVerts * Rwb.' + posWorld));
    updateBodyEdges(bodyEdges, params, Rwb, posWorld);

    % Feet nominal stance placement in body frame (rectangle under hips)
    stanceRectX = [ params.hipForwardX+0.05,  params.hipForwardX+0.05, -params.hipForwardX-0.05, -params.hipForwardX-0.05];
    stanceRectY = [ params.hipLateralY,      -params.hipLateralY,       params.hipLateralY,      -params.hipLateralY];
    nominalFootBody = [stanceRectX(:), stanceRectY(:), repmat(-0.18,4,1)]; % fixed height under body

    % For each leg: compute foot trajectory world frame, do IK per hip frame
    for iLeg = 1:4
        phase = phaseOffsets(iLeg);
        s = mod(params.stepFrequency * time + phase, 1.0); % 0..1

        % Decompose path displacement along circle tangent for the foot
        tangentDirWorld = [-sin(ang), cos(ang), 0];
        lateralDirWorld = [-cos(ang), -sin(ang), 0]; % toward circle center

        % Base placement = nominal foot directly under the body, projected to ground (z=0)
        footNomWorld = nominalFootBody(iLeg,:) * Rwb.' + posWorld;
        footNomWorld(3) = 0; % ensure baseline on ground

        % Stance moves backward relative to body; swing moves forward and up
        [deltaAlong, zLift] = footCycleDisplacement(s, strideLen, params);

        % Slight radial bias so path is circular; inner legs slightly more inside
        radialBias = 0.02 * (iLeg==2 || iLeg==3) - 0.02 * (iLeg==1 || iLeg==4); % FR/RL inward, FL/RR outward

        footWorld = footNomWorld + deltaAlong * tangentDirWorld + radialBias * lateralDirWorld;
        % vertical component with no ground penetration
        footWorld(3) = max(0, footNomWorld(3) + zLift);

        % Hip world position
        hipWorld = hipPosBody(iLeg,:) * Rwb.' + posWorld;

        % Transform target to hip local frame (hip x forward, y left, z up)
        pHipLocal = (footWorld - hipWorld) / Rwb.'; % since Rwb rotates body->world

        % Use planar IK in x-z plane; keep y offset as-is for ab/adduction look
        targetXZ = [pHipLocal(1), pHipLocal(3)];
        [th1, th2] = twoLinkIK(targetXZ, params.upperLen, params.lowerLen);

        kneeLocal  = [params.upperLen*cos(th1), 0, params.upperLen*sin(th1)];
        footLocal  = [kneeLocal(1)+params.lowerLen*cos(th1+th2), pHipLocal(2), kneeLocal(3)+params.lowerLen*sin(th1+th2)];
        kneeLocal(2) = pHipLocal(2); % keep the same lateral offset for a slight 3D feel

        % Convert joints back to world
        kneeWorld = kneeLocal * Rwb.' + hipWorld;
        footWorld = footLocal * Rwb.' + hipWorld;

        % Draw hip, leg segments, and feet
        set(hipMarkers(iLeg), 'XData', hipWorld(1), 'YData', hipWorld(2), 'ZData', hipWorld(3));
        set(legLines(iLeg,1), 'XData', [hipWorld(1) kneeWorld(1)], 'YData', [hipWorld(2) kneeWorld(2)], 'ZData', [hipWorld(3) kneeWorld(3)]);
        set(legLines(iLeg,2), 'XData', [kneeWorld(1) footWorld(1)], 'YData', [kneeWorld(2) footWorld(2)], 'ZData', [kneeWorld(3) footWorld(3)]);
        set(footMarkers(iLeg), 'XData', footWorld(1), 'YData', footWorld(2), 'ZData', footWorld(3));
    end

    drawnow limitrate;
end



%% -------- Helper functions (same file) --------
function R = eulZYX(yaw, pitch, roll)
    cy = cos(yaw);  sy = sin(yaw);
    cp = cos(pitch); sp = sin(pitch);
    cr = cos(roll);  sr = sin(roll);
    R = [ cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr; 
          sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr; 
            -sp,            cp*sr,            cp*cr ];
end

function [th1, th2] = twoLinkIK(targetXZ, l1, l2)
    x = targetXZ(1);
    z = targetXZ(2);
    r2 = x^2 + z^2;
    r = sqrt(max(r2, 1e-9));
    % Clamp to reachable workspace
    r = min(max(r, abs(l1-l2)+1e-6), l1+l2-1e-6);
    % Law of cosines
    cosTh2 = (r2 - l1^2 - l2^2) / (2*l1*l2);
    cosTh2 = max(min(cosTh2, 1), -1);
    th2 = acos(cosTh2) - pi; % choose knee bend backwards
    k1 = l1 + l2*cos(th2);
    k2 = l2*sin(th2);
    th1 = atan2(z, x) - atan2(k2, k1);
end

function [deltaAlong, zLift] = footCycleDisplacement(s, strideLen, params)
    % s in [0,1): 0..duty is stance, remainder is swing
    duty = params.dutyFactor;
    if s < duty
        % Stance: foot moves backward relative to body
        frac = s / duty; % 0..1
        deltaAlong = -strideLen * (frac - 0.5); % center at 0 to reduce slip
        % Small extra vertical compliance at edges
        edge = min(frac, 1-frac);
        zLift = -params.clearanceDrop * (0.5 - edge) * 2;
    else
        % Swing: move forward and lift following a smooth cycloid
        frac = (s - duty) / (1 - duty); % 0..1
        deltaAlong = strideLen * (frac - 0.5);
        zLift = params.stepHeight * sin(pi*frac);
    end
end

function [V,F] = makeBodyMesh(params)
    L = params.bodyLength; W = params.bodyWidth; H = params.bodyHeight;
    % Skewed/tapered box to look more like Spot (narrower head/front)
    taperTop  = 0.55;  % top front width factor
    taperBottom = 0.75; % bottom front width factor

    % Base box vertices centered at origin
    V = [
        -L/2, -W/2, -H/2;  % 1 back-bottom-left
        +L/2, -W/2, -H/2;  % 2 front-bottom-left (will taper)
        +L/2, +W/2, -H/2;  % 3 front-bottom-right (will taper)
        -L/2, +W/2, -H/2;  % 4 back-bottom-right
        -L/2, -W/2, +H/2;  % 5 back-top-left
        +L/2, -W/2, +H/2;  % 6 front-top-left (will taper more)
        +L/2, +W/2, +H/2;  % 7 front-top-right (will taper more)
        -L/2, +W/2, +H/2]; % 8 back-top-right

    % Apply taper to front face (x = +L/2)
    % Bottom front points 2,3
    V(2,2) = V(2,2) * taperBottom;
    V(3,2) = V(3,2) * taperBottom;
    % Top front points 6,7
    V(6,2) = V(6,2) * taperTop;
    V(7,2) = V(7,2) * taperTop;

    F = [
        1 2 3 4; % bottom
        5 6 7 8; % top
        1 2 6 5; % left side
        2 3 7 6; % front
        3 4 8 7; % right side
        4 1 5 8]; % back
end

function edges = plotBodyEdges(ax, params, color)
    L = params.bodyLength; W = params.bodyWidth; H = params.bodyHeight;
    % Twelve edges of the box; stored as line handles to update each frame
    verts = [
        -L/2, -W/2, -H/2;  % 1
        +L/2, -W/2, -H/2;  % 2
        +L/2, +W/2, -H/2;  % 3
        -L/2, +W/2, -H/2;  % 4
        -L/2, -W/2, +H/2;  % 5
        +L/2, -W/2, +H/2;  % 6
        +L/2, +W/2, +H/2;  % 7
        -L/2, +W/2, +H/2]; % 8
    edgesIdx = [
        1 2; 2 3; 3 4; 4 1; % bottom
        5 6; 6 7; 7 8; 8 5; % top
        1 5; 2 6; 3 7; 4 8]; % pillars
    edges = gobjects(size(edgesIdx,1),1);
    for i = 1:size(edgesIdx,1)
        p = verts(edgesIdx(i,:), :);
        edges(i) = plot3(ax, p(:,1), p(:,2), p(:,3), '-', 'Color', color, 'LineWidth', 1);
    end
end

function updateBodyEdges(edges, params, Rwb, pos)
    L = params.bodyLength; W = params.bodyWidth; H = params.bodyHeight;
    verts = [
        -L/2, -W/2, -H/2;  % 1
        +L/2, -W/2, -H/2;  % 2
        +L/2, +W/2, -H/2;  % 3
        -L/2, +W/2, -H/2;  % 4
        -L/2, -W/2, +H/2;  % 5
        +L/2, -W/2, +H/2;  % 6
        +L/2, +W/2, +H/2;  % 7
        -L/2, +W/2, +H/2]; % 8
    edgesIdx = [
        1 2; 2 3; 3 4; 4 1; % bottom
        5 6; 6 7; 7 8; 8 5; % top
        1 5; 2 6; 3 7; 4 8]; % pillars
    Vw = verts * Rwb.' + pos;
    for i = 1:size(edgesIdx,1)
        p = Vw(edgesIdx(i,:), :);
        set(edges(i), 'XData', p(:,1), 'YData', p(:,2), 'ZData', p(:,3));
    end
end


