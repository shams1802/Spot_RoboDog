% Animate the quad_military URDF trotting around a circle
% This script does not modify any STL/URDF files.

urdfPath = fullfile(pwd, 'urdf', 'quad_military.urdf');

% Load URDF using rigidBodyTree
useRoboticsTB = exist('importrobot','file') == 2;
if useRoboticsTB
    try
        robot = importrobot(urdfPath, 'DataFormat','row');
        robot.Gravity = [0 0 -9.81];
    catch err
        useRoboticsTB = false;
        warning('importrobot failed, using fallback renderer: %s', err.message);
    end
end

% Parameters
circleRadius   = 0.6;    % m
walkSpeed_mps  = 0.25;   % m/s
stepFrequency  = 1.8;    % Hz
fps            = 45;     % frames per second
dutyFactor     = 0.6;    % stance fraction
stepHeight     = 0.03;   % m
bodyZ          = 0.18;   % nominal height

% Timebase
T = 1/fps; totalTime = 18; N = floor(totalTime * fps);
strideLen = walkSpeed_mps / stepFrequency;

% Gait phases FL, FR, RL, RR
phaseOffsets = [0, 0.5, 0.5, 0];

% Visualization
fig = figure('Name','quad\_military - Circle Trot','Color',[0.97 0.98 1.00]);
ax = axes(fig); hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
xlabel(ax,'X (m)'); ylabel(ax,'Y (m)'); zlabel(ax,'Z (m)');
view(ax, 35, 20);

% Ground and path
th = linspace(0,2*pi,300); plot3(ax, circleRadius*cos(th), circleRadius*sin(th), zeros(size(th)),'--','Color',[0.6 0.6 0.7]);
[Xp,Yp] = meshgrid(linspace(-circleRadius-0.8,circleRadius+0.8,2)); Zp = 0*Xp;
surf(ax, Xp, Yp, Zp, 'FaceColor',[0.92 0.93 0.95], 'EdgeColor','none', 'FaceAlpha',1.0);
lighting gouraud; camlight('headlight');

% Initial pose
if useRoboticsTB
    q = homeConfiguration(robot);
else
    q = [];
end
axes(ax);

% Helper for foot cycle displacement
footDisp = @(s) deal( ...
    (s < dutyFactor) .* (-strideLen * ((s./dutyFactor) - 0.5)) + ...
    (s >= dutyFactor) .* ( strideLen * (((s - dutyFactor)./(1 - dutyFactor)) - 0.5)), ...
    (s >= dutyFactor) .* ( stepHeight * sin(pi*((s - dutyFactor)./(1 - dutyFactor))) ) + ...
    (s < dutyFactor)  .* ( 0 ) );

% Axis limits
axis(ax, [-(circleRadius+0.5) (circleRadius+0.5) -(circleRadius+0.5) (circleRadius+0.5) -0.1 0.5]);

% Animation
ang = 0; time = 0;
for k = 1:N
    time = time + T;
    ang = mod((walkSpeed_mps * time) / circleRadius, 2*pi);
    posWorld = [circleRadius * cos(ang), circleRadius * sin(ang), bodyZ];
    yawWorld = ang + pi/2; % tangent facing
    Rwb = eulZYX(yawWorld, 0, 0);

    % Render at base pose; for simplicity, we keep joints at home config here
    % (full IK to match foot trajectories would require leg frame mapping from URDF)
    if useRoboticsTB
        show(robot, q, 'Parent', ax, 'Visuals','on','PreservePlot',false);
        camtarget(ax, posWorld);
        campos(ax, posWorld + (Rwb.'*[1.2 -1.2 0.7]'));
        camup(ax, (Rwb.'*[0 0 1]'));
    else
        cla(ax);
        Tbase = eye(4); Tbase(1:3,1:3)=Rwb; Tbase(1:3,4)=posWorld(:);
        renderUrdfVisuals(urdfPath, ax, Tbase);
        camtarget(ax, posWorld);
        campos(ax, posWorld + (Rwb.'*[1.2 -1.2 0.7]'));
        camup(ax, (Rwb.'*[0 0 1]'));
    end

    drawnow limitrate;
end

function R = eulZYX(yaw, pitch, roll)
    cy = cos(yaw);  sy = sin(yaw);
    cp = cos(pitch); sp = sin(pitch);
    cr = cos(roll);  sr = sin(roll);
    R = [ cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr; 
          sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr; 
            -sp,            cp*sr,            cp*cr ];
end

% Fallback URDF visual renderer reused from model.m
function renderUrdfVisuals(urdfPath, ax, Tbase)
    if nargin < 3, Tbase = eye(4); end
    doc = xmlread(urdfPath);
    robotNode = doc.getDocumentElement();
    linkNodes = robotNode.getElementsByTagName('link');
    visualsByLink = struct();
    for i=0:linkNodes.getLength()-1
        ln = linkNodes.item(i);
        name = char(ln.getAttribute('name'));
        visNodes = ln.getElementsByTagName('visual');
        visuals = {};
        for j=0:visNodes.getLength()-1
            vn = visNodes.item(j);
            org = vn.getElementsByTagName('origin');
            if org.getLength()>0
                on = org.item(0);
                xyz = parseVecAttr(on,'xyz',[0 0 0]);
                rpy = parseVecAttr(on,'rpy',[0 0 0]);
            else
                xyz = [0 0 0]; rpy = [0 0 0];
            end
            geo = vn.getElementsByTagName('geometry');
            meshPath = '';
            scale = [1 1 1];
            if geo.getLength()>0
                gn = geo.item(0).getElementsByTagName('mesh');
                if gn.getLength()>0
                    mn = gn.item(0);
                    meshPath = char(mn.getAttribute('filename'));
                    sc = strtrim(char(mn.getAttribute('scale')));
                    if ~isempty(sc)
                        vals = sscanf(sc,'%f');
                        if numel(vals)==3, scale = vals'; end
                    end
                end
            end
            visuals{end+1} = struct('xyz',xyz,'rpy',rpy,'mesh',meshPath,'scale',scale); %#ok<AGROW>
        end
        visualsByLink.(matlab.lang.makeValidName(name)) = visuals;
    end
    % Render all link visuals with base transform (no joint articulation)
    linkNames = fieldnames(visualsByLink);
    for i=1:numel(linkNames)
        visuals = visualsByLink.(linkNames{i});
        for k=1:numel(visuals)
            v = visuals{k};
            if isempty(v.mesh), continue; end
            meshFile = resolveMeshPath(v.mesh);
            if isempty(meshFile) || ~exist(meshFile,'file'), continue; end
            [F,V] = safeStlRead(meshFile);
            V = ensureNx3(V);
            S = diag([v.scale 1]);
            Tv = makeHT(v.xyz, v.rpy);
            Vh = [V, ones(size(V,1),1)] * (S*Tv*Tbase).';
            patch('Faces',F,'Vertices',Vh(:,1:3), 'FaceColor',[0.7 0.7 0.72], 'EdgeColor','none', 'FaceAlpha',1.0,'Parent',ax);
        end
    end
end

function T = makeHT(xyz, rpy)
    cx = cos(rpy(1)); sx = sin(rpy(1));
    cy = cos(rpy(2)); sy = sin(rpy(2));
    cz = cos(rpy(3)); sz = sin(rpy(3));
    Rx = [1 0 0; 0 cx -sx; 0 sx cx];
    Ry = [cy 0 sy; 0 1 0; -sy 0 cy];
    Rz = [cz -sz 0; sz cz 0; 0 0 1];
    R = Rz*Ry*Rx;
    T = eye(4); T(1:3,1:3)=R; T(1:3,4)=xyz(:);
end

function v = parseVecAttr(node, attr, def)
    s = strtrim(char(node.getAttribute(attr)));
    if isempty(s), v = def; return; end
    vals = sscanf(s,'%f');
    if numel(vals)==3, v = vals'; else, v = def; end
end

function pathOut = resolveMeshPath(meshURI)
    if startsWith(meshURI,'package://')
        pathOut = '';
    else
        if meshURI(1)==filesep || (~isempty(regexp(meshURI,'^[A-Za-z]:','once')))
            pathOut = meshURI;
        else
            pathOut = fullfile(pwd, meshURI);
        end
    end
end

function V = ensureNx3(V)
    if size(V,2)==3, return; end
    if size(V,1)==3, V = V.'; return; end
    error('Unexpected vertex array shape');
end

function [F,V] = safeStlRead(path)
    try
        [F,V] = stlread(path);
    catch
        try
            tri = stlread(path);
            if isa(tri, 'triangulation')
                F = tri.ConnectivityList; V = tri.Points;
            elseif isstruct(tri) && isfield(tri,'Faces') && isfield(tri,'Vertices')
                F = tri.Faces; V = tri.Vertices;
            else
                error('Unsupported stlread output');
            end
        catch innerErr
            error('stlread failed for %s: %s', path, innerErr.message);
        end
    end
    if size(V,2)~=3 && size(V,1)==3, V = V.'; end
end
