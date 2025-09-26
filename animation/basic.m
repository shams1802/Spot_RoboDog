% basic.m - Minimal loader for quad_military.urdf (as-is)
% Uses importrobot if available; otherwise falls back to a simple URDF visual renderer.

urdfPath = fullfile(pwd, 'urdf', 'quad_military.urdf');

useRoboticsTB = exist('importrobot','file') == 2;

fig = figure('Name','quad\_military - basic','Color',[1 1 1]);
ax = axes(fig); hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
xlabel(ax,'X (m)'); ylabel(ax,'Y (m)'); zlabel(ax,'Z (m)');
view(ax, 35, 20);

% Ground plane
[Xp,Yp] = meshgrid(linspace(-1,1,2)); Zp = zeros(size(Xp));
surf(ax, Xp, Yp, Zp, 'FaceColor',[0.95 0.96 0.98], 'EdgeColor','none');
lighting gouraud; camlight('headlight');

if useRoboticsTB
    try
        robot = importrobot(urdfPath, 'DataFormat','row');
        show(robot, robot.homeConfiguration, 'Parent', ax, 'Visuals','on','PreservePlot',false);
    catch err
        warning('importrobot failed (%s). Using fallback visual renderer.', err.message);
        renderUrdfVisuals(urdfPath, ax);
    end
else
    renderUrdfVisuals(urdfPath, ax);
end

axis(ax, [-0.8 0.8 -0.8 0.8 0 0.8]);

a=1; %#ok<NASGU> % keep figure from closing in some environments

% --- Fallback visual renderer ---
function renderUrdfVisuals(urdfPath, ax)
    doc = xmlread(urdfPath);
    robotNode = doc.getDocumentElement();
    linkNodes = robotNode.getElementsByTagName('link');
    for i=0:linkNodes.getLength()-1
        ln = linkNodes.item(i);
        visNodes = ln.getElementsByTagName('visual');
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
                        vals = sscanf(sc,'%f'); if numel(vals)==3, scale = vals'; end
                    end
                end
            end
            if isempty(meshPath), continue; end
            meshFile = resolveMeshPath(meshPath);
            if isempty(meshFile) || ~exist(meshFile,'file'), continue; end
            [F,V] = safeStlRead(meshFile);
            if size(V,2)~=3 && size(V,1)==3, V = V.'; end
            S = diag([scale 1]);
            T = makeHT(xyz, rpy);
            Vh = [V, ones(size(V,1),1)] * (S*T).';
            patch('Faces',F,'Vertices',Vh(:,1:3), 'FaceColor',[0.7 0.72 0.75], 'EdgeColor','none', 'FaceAlpha',1.0,'Parent',ax);
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
end
