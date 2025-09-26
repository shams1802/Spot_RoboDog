% Load and render a stationary model from URDF (quad_military.urdf)
% This script does not modify any STL/URDF files.

urdfPath = fullfile(pwd, 'urdf', 'quad_military.urdf');

useRoboticsTB = exist('importrobot','file') == 2;
if useRoboticsTB
    try
        robot = importrobot(urdfPath, 'DataFormat','row');
        robot.Gravity = [0 0 -9.81];
        q = homeConfiguration(robot);
    catch err
        useRoboticsTB = false; % fallback to our renderer
        warning('importrobot failed, using fallback renderer: %s', err.message);
    end
end

% Figure
fig = figure('Name','quad\_military - Stationary','Color',[0.97 0.98 1.00]);
ax = axes(fig); hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
xlabel(ax,'X (m)'); ylabel(ax,'Y (m)'); zlabel(ax,'Z (m)');
view(ax, 35, 20);

% Ground plane
planeR = 1.0;
[Xp,Yp] = meshgrid(linspace(-planeR,planeR,2), linspace(-planeR,planeR,2));
Zp = zeros(size(Xp));
surf(ax, Xp, Yp, Zp, 'FaceColor',[0.92 0.93 0.95], 'EdgeColor','none', 'FaceAlpha',1.0);
lighting gouraud; camlight('headlight');

if useRoboticsTB
    show(robot, q, 'Parent', ax, 'Visuals','on','PreservePlot',false);
else
    renderUrdfVisuals(urdfPath, ax);
end
axis(ax, [-0.8 0.8 -0.8 0.8 0 0.8]);

%% ----------------- Fallback URDF visual renderer -----------------
function renderUrdfVisuals(urdfPath, ax)
    doc = xmlread(urdfPath);
    robotNode = doc.getDocumentElement();
    % Build link map
    linkNodes = robotNode.getElementsByTagName('link');
    links = struct();
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
        links.(matlab.lang.makeValidName(name)) = struct('name',name,'visuals',{visuals});
    end
    % Build joint list and parent graph
    jointNodes = robotNode.getElementsByTagName('joint');
    joints = struct();
    parents = containers.Map();
    children = containers.Map();
    origins = containers.Map();
    for i=0:jointNodes.getLength()-1
        jn = jointNodes.item(i);
        jname = char(jn.getAttribute('name'));
        parentNode = jn.getElementsByTagName('parent').item(0);
        childNode  = jn.getElementsByTagName('child').item(0);
        parent = char(parentNode.getAttribute('link'));
        child  = char(childNode.getAttribute('link'));
        org = jn.getElementsByTagName('origin');
        if org.getLength()>0
            on = org.item(0);
            xyz = parseVecAttr(on,'xyz',[0 0 0]);
            rpy = parseVecAttr(on,'rpy',[0 0 0]);
        else
            xyz = [0 0 0]; rpy = [0 0 0];
        end
        parents(child) = parent;
        children(parent) = child; %#ok<NASGU>
        origins(child) = struct('xyz',xyz,'rpy',rpy);
        joints.(matlab.lang.makeValidName(jname)) = struct('parent',parent,'child',child,'xyz',xyz,'rpy',rpy);
    end
    % Find base link (one that is never a child)
    linkNames = fieldnames(links);
    isChild = false(size(linkNames));
    for i=1:numel(linkNames)
        isChild(i) = parents.isKey(links.(linkNames{i}).name);
    end
    baseIdx = find(~isChild,1,'first');
    if isempty(baseIdx), baseIdx = 1; end
    baseName = links.(linkNames{baseIdx}).name;
    % Traverse tree and render
    drawLinkRecursive(baseName, eye(4), links, parents, origins, ax);
end

function drawLinkRecursive(linkName, Tparent, links, parents, origins, ax)
    L = links.(matlab.lang.makeValidName(linkName));
    % Draw all visuals of this link at Tparent
    for k=1:numel(L.visuals)
        v = L.visuals{k};
        if isempty(v.mesh), continue; end
        meshFile = resolveMeshPath(v.mesh);
        if isempty(meshFile) || ~exist(meshFile,'file'), continue; end
        [F,V] = safeStlRead(meshFile);
        V = ensureNx3(V);
        % Apply scale (URDF uses meters; meshes here likely in mm -> keep as-is)
        S = diag([v.scale 1]);
        Tv = makeHT(v.xyz, v.rpy);
        Vh = [V, ones(size(V,1),1)] * (S*Tv*Tparent).';
        patch('Faces',F,'Vertices',Vh(:,1:3), 'FaceColor',[0.7 0.7 0.72], 'EdgeColor','none', 'FaceAlpha',1.0,'Parent',ax);
    end
    % Recurse into child if any
    childNames = parents.keys;
    for i=1:numel(childNames)
        child = childNames{i};
        if strcmp(parents(child), linkName)
            if origins.isKey(child)
                org = origins(child);
                Tc = makeHT(org.xyz, org.rpy) * Tparent;
            else
                Tc = Tparent;
            end
            drawLinkRecursive(child, Tc, links, parents, origins, ax);
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
    % Supports relative paths, package:// style not handled here
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
