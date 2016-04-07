function boxes = detect(input, model, thresh)

% Keep track of detected boxes and features
debugcnt=0;
BOXCACHESIZE = 100000;
cnt = 0;
boxes.s  = 0;
boxes.c  = 0;
boxes.xy = 0;
boxes.level = 0;
boxes(BOXCACHESIZE) = boxes; 

% Compute the feature pyramid and prepare filters

pyra = featpyramid(input,model);

[components,filters,resp]  = modelcomponents(model,pyra);
%if(~exist('myParams.mat','file'))
 %delete('myParams.mat');
%end;
c  = randperm(length(components));
%save('myParams','model','pyra','components','filters','resp','c');
%disp(['pyra .scale size' num2str(pyra.scale)]);
tic 
%model,pyra,components,filters,resp
%for c  = randperm(length(components)),
for c  =1:length(components)
    
    minlevel = model.interval+1;%model ¼ä¸ô
    levels   = minlevel:length(pyra.feat);
    %for rlevel = levels(randperm(length(levels))),
    for rlevel = levels(1):levels(length(levels))
        parts    = components{c};
        numparts = length(parts);
        
        % Local part scores
        for k = 1:numparts,
            f     = parts(k).filterid;
            level = rlevel-parts(k).scale*model.interval;
            %disp(['level -rlevel:' num2str(level -rlevel)]);
            
            
            if isempty(resp{level}),
                resp{level} = fconv(pyra.feat{level},filters,1,length(filters));
               
            end
            parts(k).score = resp{level}{f};
           %parts(k).score(:,1)
            parts(k).level = level;
            
        end
        
        % Walk from leaves to root of tree, passing message to parent
        % Given a 2D array of filter scores 'child', shiftdt() does the following:
        % (1) Apply distance transform
        % (2) Shift by anchor position (child.startxy) of part wrt parent
        % (3) Downsample by child.step
        
         debugcnt=debugcnt+1;
        for k = numparts:-1:2,
            child = parts(k);
            
           
            
            par   = child.parent;
            [Ny,Nx,foo] = size(parts(par).score);
            [msg,parts(k).Ix,parts(k).Iy] = shiftdt(child.score, child.w(1),child.w(2),child.w(3),child.w(4), ...
                child.startx, child.starty, Nx, Ny, child.step);
            
            
            parts(par).score = parts(par).score + msg;
            
            
            
        end
        
            % Add bias to root score
        rscore = parts(1).score + parts(1).w;
       
       
        
        
        [Y,X] = find(rscore >= thresh);
        
        if ~isempty(X)
            XY = backtrack( X, Y, parts, pyra);
        end
        
        % Walk back down tree following pointers
        %disp(['length(X)' num2str(length(X))]);
        for i = 1:length(X)
           
            x = X(i);
            y = Y(i);
            
            if cnt == BOXCACHESIZE
                b0 = nms_face(boxes,0.3);
                clear boxes;
                boxes.s  = 0;
                boxes.c  = 0;
                boxes.xy = 0;
                boxes.level = 0;
                boxes(BOXCACHESIZE) = boxes;
                cnt = length(b0);
                boxes(1:cnt) = b0;
            end
            
            cnt = cnt + 1;
            boxes(cnt).c = c;
            boxes(cnt).s = rscore(y,x);
            boxes(cnt).level = rlevel;
            boxes(cnt).xy = XY(:,:,i);
           %  boxes(cnt).xy(:,1);
        end
    end
end

boxes = boxes(1:cnt);
time_pass=toc;
 fprintf('feater detection took %.1f seconds\n',time_pass);


% Backtrack through dynamic programming messages to estimate part locations
% and the associated feature vector
function box = backtrack(x,y,parts,pyra)
numparts = length(parts);
ptr = zeros(numparts,2,length(x));
box = zeros(numparts,4,length(x));
k   = 1;
p   = parts(k);
ptr(k,1,:) = x;
ptr(k,2,:) = y;
% image coordinates of root
scale = pyra.scale(p.level);
padx  = pyra.padx;
pady  = pyra.pady;
box(k,1,:) = (x-1-padx)*scale + 1;
box(k,2,:) = (y-1-pady)*scale + 1;
box(k,3,:) = box(k,1,:) + p.sizx*scale - 1;
box(k,4,:) = box(k,2,:) + p.sizy*scale - 1;
%disp('box(:,3,1)');
%box(k,3,:)
%disp('box(:,4,1)');
%box(k,4,:)
for k = 2:numparts,
    p   = parts(k);
    par = p.parent;
    x   = ptr(par,1,:);
    y   = ptr(par,2,:);
   % disp('xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx');
    %x
  %  disp('yyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyy');
  %  y
    inds = sub2ind(size(p.Ix), y, x);
    sum(p.Ix(:,1))
   % if k==65 && length(x)>=77
       % [row col]=size(p.Ix);
       % disp(['size(p.Ix) (' num2str(row) ',' num2str(col) ')']);
       % disp(['x:' num2str(x(77))]);
       % disp(['y:' num2str(y(77))]);
    %end;
    ptr(k,1,:) = p.Ix(inds);
    ptr(k,2,:) = p.Iy(inds);
    % image coordinates of part k
    scale = pyra.scale(p.level);
    box(k,1,:) = (ptr(k,1,:)-1-padx)*scale + 1;
    box(k,2,:) = (ptr(k,2,:)-1-pady)*scale + 1;
    box(k,3,:) = box(k,1,:) + p.sizx*scale - 1;
    box(k,4,:) = box(k,2,:) + p.sizy*scale - 1;
   % disp('ptr(k,1,:)................................');
  % ptr(k,1,:)
  %  disp('ptr(k,1,:)................................');
 %  ptr(k,1,:)
    
end

% Cache various statistics from the model data structure for later use
function [mycomponents,filters,resp] = modelcomponents(model,pyra)
mycomponents = cell(length(model.components),1);
for c = 1:length(model.components),
    for k = 1:length(model.components{c}),
        p = model.components{c}(k);
        x = model.filters(p.filterid);
        [p.sizy p.sizx foo] = size(x.w);
        p.filterI = x.i;
        x = model.defs(p.defid);
        p.defI = x.i;
        p.w    = x.w;
        
        % store the scale of each part relative to the component root
        par = p.parent;
        assert(par < k);
        ax  = x.anchor(1);
        ay  = x.anchor(2);
        ds  = x.anchor(3);
        if par > 0,
            p.scale = ds + mycomponents{c}(par).scale;
        else
            assert(k == 1);
            p.scale = 0;
        end
        % amount of (virtual) padding to hallucinate
        step     = 2^ds;
        virtpady = (step-1)*pyra.pady;
        virtpadx = (step-1)*pyra.padx;
        % starting points (simulates additional padding at finer scales)
        p.starty = ay-virtpady;
        p.startx = ax-virtpadx;
        p.step   = step;
        p.level  = 0;
        p.score  = 0;
        p.Ix     = 0;
        p.Iy     = 0;
        mycomponents{c}(k) = p;
    end
end

resp    = cell(length(pyra.feat),1);
filters = cell(length(model.filters),1);
for i = 1:length(filters),
    filters{i} = model.filters(i).w;
end

