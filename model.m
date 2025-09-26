% DAWWGY_MODEL (script)
% Renders a static Dawwgy quadruped model using dimensions inferred
% from CAD scripts in `quad_cad/quad_designs`.
%
% Usage: open this file and press Run.

%% -------- Parameters (CAD-derived) --------
params = struct();

% Body geometry (meters)
% From body_v1.py:
% outer_rect_w = 40.2; inner_rect_w = 29; inner_rect_l = 42.5; l_offset = 10;
% hole_plate_w = (outer_rect_w-inner_rect_w)/2 = 5.6;
% outer_rect_l = inner_rect_l + hole_plate_w = 48.1;
% LENGTH = outer_rect_l - l_offset = 38.1;
% BREADTH = outer_rect_w + 10 = 50.2; HEIGHT = BREADTH;
% xl = HEIGHT - LENGTH = 12.1; wz = HEIGHT - motor_inner_thickness (34.5) = 15.7;
% size = (xl+LENGTH-wz)*2/sqrt(3) ≈ 39.8; zl = 3.5*size + 10 ≈ 159.3;
% Body print: l = HEIGHT + 2*zl ≈ 368.8 mm, b = 2*l_offset + 2*LENGTH + xl ≈ 108.3 mm
params.bodyLength = 0.3688; % meters (≈ 368.8 mm)
params.bodyWidth  = 0.1083; % meters (≈ 108.3 mm)
params.bodyHeight = 0.0502; % meters (≈ 50.2 mm)

% Hip locations relative to body center (x forward, y left, z up)
params.hipForwardX = 0.1844; % ≈ bodyLength/2
params.hipLateralY = params.bodyWidth/2; % exact half width from CAD
params.hipHeightZ  = 0.000;  % motor axis approx at body center height

% Leg link lengths (rotor-to-rotor; rotor-to-foot)
% From leg_upper_v1 and v3 (rotor-to-rotor length ~206 mm) and leg_lower_v1
params.upperLen = 0.2058;   % meters (≈ 205.8 mm)
params.lowerLen = 0.2080;   % meters (≈ 208.0 mm)

% Neutral stance target foot height below body center
% Neutral foot height chosen so feet touch ground at world z=0 when body z=0.18
params.footZ_world = -0.18;

% Colors
params.colorBody   = [0.92 0.73 0.23];
params.colorBlack  = [0.08 0.08 0.10];
params.colorJoints = [0.20 0.20 0.22];
params.bgColor     = [0.97 0.98 1.00];

% Rendering
params.lineWidthLeg = 3.5;
params.markerSize   = 10;

% Use exact CAD meshes if available
params.useMeshes = true;
params.partsDir = fullfile(pwd, 'quad_cad', 'assets', 'quad_parts');
params.meshScale = 1e-3; % STL in mm -> meters

%% -------- Derived --------
% Hip positions in body frame (x forward, y left, z up)
hipPosBody = [
    params.hipForwardX,  params.hipLateralY,  params.hipHeightZ;  % FL
    params.hipForwardX, -params.hipLateralY,  params.hipHeightZ;  % FR
   -params.hipForwardX,  params.hipLateralY,  params.hipHeightZ;  % RL
   -params.hipForwardX, -params.hipLateralY,  params.hipHeightZ]; % RR

% Neutral body world pose
posWorld = [0 0 0.18];
yawWorld = 0; pitch = 0; roll = 0;
Rwb = eulZYX(yawWorld, pitch, roll);

%% -------- Figure setup --------
fig = figure('Name','Dawwgy - Static Model','Color',params.bgColor);
ax = axes(fig); hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
xlabel(ax,'X (m)'); ylabel(ax,'Y (m)'); zlabel(ax,'Z (m)');
view(ax, 35, 20);
set(ax,'Color',params.bgColor);

% Ground plane
planeR = 0.8;
[Xp,Yp] = meshgrid(linspace(-planeR,planeR,2), linspace(-planeR,planeR,2));
Zp = zeros(size(Xp));
surf(ax, Xp, Yp, Zp, 'FaceColor',[0.92 0.93 0.95], 'EdgeColor','none', 'FaceAlpha',1.0);
lighting gouraud; camlight('headlight');

% Body mesh
if params.useMeshes
    bodyFile = pickExisting({
        fullfile(params.partsDir,'body_with_motor_v1.stl'),
        fullfile(params.partsDir,'body_v1.stl')
    });
    if ~isempty(bodyFile)
        [F,V] = safeStlRead(bodyFile);
        V = V * params.meshScale; % mm -> m
        % Recentre body mesh roughly around origin for placement
        V = recentreVertices(V);
        bodyPatch = patch('Faces',F,'Vertices',(V * Rwb.' + posWorld), ...
            'FaceColor',params.colorBody,'EdgeColor','none','FaceAlpha',1.0, ...
            'FaceLighting','gouraud','SpecularStrength',0.3);
        bodyEdges = gobjects(0);
    else
        [bodyVerts, bodyFaces] = makeBodyMesh(params);
        bodyPatch = patch('Faces',bodyFaces,'Vertices',(bodyVerts * Rwb.' + posWorld), ...
            'FaceColor',params.colorBody,'EdgeColor','none','FaceAlpha',1.0, ...
            'FaceLighting','gouraud','SpecularStrength',0.3);
        bodyEdges = plotBodyEdges(ax, params, params.colorBlack);
        updateBodyEdges(bodyEdges, params, Rwb, posWorld);
    end
else
    [bodyVerts, bodyFaces] = makeBodyMesh(params);
    bodyPatch = patch('Faces',bodyFaces,'Vertices',(bodyVerts * Rwb.' + posWorld), ...
        'FaceColor',params.colorBody,'EdgeColor','none','FaceAlpha',1.0, ...
        'FaceLighting','gouraud','SpecularStrength',0.3);
    bodyEdges = plotBodyEdges(ax, params, params.colorBlack);
    updateBodyEdges(bodyEdges, params, Rwb, posWorld);
end

% Pre-create graphics
legLines = gobjects(4,2); % [upper, lower]
footMarkers = gobjects(4,1);
hipMarkers  = gobjects(4,1);
for i = 1:4
    legLines(i,1) = plot3(ax, [0 0], [0 0], [0 0], '-', 'Color', params.colorBlack, 'LineWidth', params.lineWidthLeg);
    legLines(i,2) = plot3(ax, [0 0], [0 0], [0 0], '-', 'Color', params.colorBlack, 'LineWidth', params.lineWidthLeg);
    footMarkers(i) = plot3(ax, 0,0,0, 'o', 'MarkerFaceColor', params.colorBlack, 'MarkerEdgeColor','none', 'MarkerSize', params.markerSize);
    hipMarkers(i)  = plot3(ax, 0,0,0, 'o', 'MarkerFaceColor', params.colorJoints, 'MarkerEdgeColor','none', 'MarkerSize', 6);
end
upperPatches = gobjects(4,1);
lowerPatches = gobjects(4,1);
upperFile = pickExisting({
    fullfile(params.partsDir,'leg_upper_with_motor_v1.5.stl'), ...
    fullfile(params.partsDir,'leg_upper_v1.5.stl'), ...
    fullfile(params.partsDir,'upper_leg_v3.stl'), ...
    fullfile(params.partsDir,'leg_upper_v1.stl')
});
lowerFile = pickExisting({
    fullfile(params.partsDir,'leg_lower_v3.stl'), ...
    fullfile(params.partsDir,'lower_leg_v1.stl')
});

% Nominal rectangle under hips for feet
stanceRectX = [ params.hipForwardX+0.05,  params.hipForwardX+0.05, -params.hipForwardX-0.05, -params.hipForwardX-0.05];
stanceRectY = [ params.hipLateralY,      -params.hipLateralY,       params.hipLateralY,      -params.hipLateralY];
nominalFootBody = [stanceRectX(:), stanceRectY(:), repmat(params.footZ_world,4,1)];

% Draw each leg in neutral pose
for iLeg = 1:4
    hipWorld = hipPosBody(iLeg,:) * Rwb.' + posWorld;
    footWorld = nominalFootBody(iLeg,:) * Rwb.' + posWorld;
    % Solve IK in hip local X-Z plane
    pHipLocal = (footWorld - hipWorld) / Rwb.';
    targetXZ = [pHipLocal(1), pHipLocal(3)];
    [th1, th2] = twoLinkIK(targetXZ, params.upperLen, params.lowerLen);
    kneeLocal  = [params.upperLen*cos(th1), 0, params.upperLen*sin(th1)];
    footLocal  = [kneeLocal(1)+params.lowerLen*cos(th1+th2), pHipLocal(2), kneeLocal(3)+params.lowerLen*sin(th1+th2)];
    kneeLocal(2) = pHipLocal(2);

    kneeWorld = kneeLocal * Rwb.' + hipWorld;
    footWorld = footLocal * Rwb.' + hipWorld;

    set(hipMarkers(iLeg), 'XData', hipWorld(1), 'YData', hipWorld(2), 'ZData', hipWorld(3));
    set(legLines(iLeg,1), 'XData', [hipWorld(1) kneeWorld(1)], 'YData', [hipWorld(2) kneeWorld(2)], 'ZData', [hipWorld(3) kneeWorld(3)]);
    set(legLines(iLeg,2), 'XData', [kneeWorld(1) footWorld(1)], 'YData', [kneeWorld(2) footWorld(2)], 'ZData', [kneeWorld(3) footWorld(3)]);
    set(footMarkers(iLeg), 'XData', footWorld(1), 'YData', footWorld(2), 'ZData', footWorld(3));

    % Mesh rendering for legs if available
    if params.useMeshes && ~isempty(upperFile)
        [Fu,Vu] = safeStlRead(upperFile);
        Vu = Vu * params.meshScale;
        Vu = recentreVertices(Vu);
        % Upper leg local frame: X forward, Y left, Z up; rotate by th1 in X-Z plane
        Ru = rotY(0) * rotZ(0) * rotY(0); %#ok<NASGU>
        Rhip = Rwb; % body->world
        Rupper = axRotY(0) * axRotY(0); %#ok<NASGU>
        Rhl = Rwb; % simplify as body-aligned; apply planar rotation around Y
        Ry = [cos(th1) 0 sin(th1); 0 1 0; -sin(th1) 0 cos(th1)];
        VuW = (Vu * Ry.') + hipWorld;
        if isempty(upperPatches(iLeg)) || ~isgraphics(upperPatches(iLeg))
            upperPatches(iLeg) = patch('Faces',Fu,'Vertices',VuW, 'FaceColor',[0.6 0.6 0.65],'EdgeColor','none','FaceAlpha',1.0,'FaceLighting','gouraud');
        else
            set(upperPatches(iLeg), 'Vertices', VuW);
        end
    end
    if params.useMeshes && ~isempty(lowerFile)
        [Fl,Vl] = safeStlRead(lowerFile);
        Vl = Vl * params.meshScale;
        Vl = recentreVertices(Vl);
        % Lower leg rotation by (th1+th2) about hip local Y, positioned at knee
        Ry2 = [cos(th1+th2) 0 sin(th1+th2); 0 1 0; -sin(th1+th2) 0 cos(th1+th2)];
        VlW = (Vl * Ry2.') + kneeWorld;
        if isempty(lowerPatches(iLeg)) || ~isgraphics(lowerPatches(iLeg))
            lowerPatches(iLeg) = patch('Faces',Fl,'Vertices',VlW, 'FaceColor',[0.35 0.35 0.38],'EdgeColor','none','FaceAlpha',1.0,'FaceLighting','gouraud');
        else
            set(lowerPatches(iLeg), 'Vertices', VlW);
        end
    end
end

axis(ax, [-0.5 0.5 -0.5 0.5 -0.05 0.4]);

drawnow;

%% -------- Helper functions --------
function file = pickExisting(candidates)
    file = '';
    for i = 1:numel(candidates)
        if exist(candidates{i}, 'file')
            file = candidates{i};
            return;
        end
    end
end

function [F,V] = safeStlRead(path)
    % Try common signatures of stlread
    try
        [F,V] = stlread(path);
    catch
        try
            tri = stlread(path); % triangulation object or struct
            if isa(tri, 'triangulation')
                F = tri.ConnectivityList;
                V = tri.Points;
            elseif isstruct(tri)
                if isfield(tri,'Faces') && isfield(tri,'Vertices')
                    F = tri.Faces;
                    V = tri.Vertices;
                else
                    error('Unsupported STL struct format for %s', path);
                end
            else
                error('Unsupported stlread return type for %s', path);
            end
        catch innerErr
            error('stlread not available or failed for %s: %s', path, innerErr.message);
        end
    end
    % Normalize vertices to N-by-3
    if size(V,2) ~= 3 && size(V,1) == 3
        V = V.';
    end
end

function Vc = recentreVertices(V)
    c = mean(V,1);
    Vc = V - c;
end

function R = axRotY(theta)
    R = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
end
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
    r = min(max(r, abs(l1-l2)+1e-6), l1+l2-1e-6);
    cosTh2 = (r2 - l1^2 - l2^2) / (2*l1*l2);
    cosTh2 = max(min(cosTh2, 1), -1);
    th2 = acos(cosTh2) - pi;
    k1 = l1 + l2*cos(th2);
    k2 = l2*sin(th2);
    th1 = atan2(z, x) - atan2(k2, k1);
end

function [V,F] = makeBodyMesh(params)
    L = params.bodyLength; W = params.bodyWidth; H = params.bodyHeight;
    taperTop  = 0.55;  % top front width factor
    taperBottom = 0.75; % bottom front width factor

    V = [
        -L/2, -W/2, -H/2;  % 1 back-bottom-left
        +L/2, -W/2, -H/2;  % 2 front-bottom-left (tapered)
        +L/2, +W/2, -H/2;  % 3 front-bottom-right (tapered)
        -L/2, +W/2, -H/2;  % 4 back-bottom-right
        -L/2, -W/2, +H/2;  % 5 back-top-left
        +L/2, -W/2, +H/2;  % 6 front-top-left (more taper)
        +L/2, +W/2, +H/2;  % 7 front-top-right (more taper)
        -L/2, +W/2, +H/2]; % 8 back-top-right

    V(2,2) = V(2,2) * taperBottom;
    V(3,2) = V(3,2) * taperBottom;
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
    verts = [
        -L/2, -W/2, -H/2;
        +L/2, -W/2, -H/2;
        +L/2, +W/2, -H/2;
        -L/2, +W/2, -H/2;
        -L/2, -W/2, +H/2;
        +L/2, -W/2, +H/2;
        +L/2, +W/2, +H/2;
        -L/2, +W/2, +H/2];
    edgesIdx = [
        1 2; 2 3; 3 4; 4 1;
        5 6; 6 7; 7 8; 8 5;
        1 5; 2 6; 3 7; 4 8];
    edges = gobjects(size(edgesIdx,1),1);
    for i = 1:size(edgesIdx,1)
        p = verts(edgesIdx(i,:), :);
        edges(i) = plot3(ax, p(:,1), p(:,2), p(:,3), '-', 'Color', color, 'LineWidth', 1);
    end
end

function updateBodyEdges(edges, params, Rwb, pos)
    L = params.bodyLength; W = params.bodyWidth; H = params.bodyHeight;
    verts = [
        -L/2, -W/2, -H/2;
        +L/2, -W/2, -H/2;
        +L/2, +W/2, -H/2;
        -L/2, +W/2, -H/2;
        -L/2, -W/2, +H/2;
        +L/2, -W/2, +H/2;
        +L/2, +W/2, +H/2;
        -L/2, +W/2, +H/2];
    edgesIdx = [
        1 2; 2 3; 3 4; 4 1;
        5 6; 6 7; 7 8; 8 5;
        1 5; 2 6; 3 7; 4 8];
    Vw = verts * Rwb.' + pos;
    for i = 1:size(edgesIdx,1)
        p = Vw(edgesIdx(i,:), :);
        set(edges(i), 'XData', p(:,1), 'YData', p(:,2), 'ZData', p(:,3));
    end
end
