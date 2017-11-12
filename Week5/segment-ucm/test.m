% Set-up UCM
% cd MCG-PreTrained
% install
% cd ..
addpath(genpath('MCG-PreTrained/'))
ima_name = '00.000948.jpg';
ima = imread(ima_name);

tic;
seg = segment_ucm(ima, 0.8);
toc
imshow(label2color(seg));
% imwrite(label2color(seg), '00.000948_seg.png');

%%

labels = unique(seg(:))';

[MinSize,FRmin,FRmax,FFmin,FFmax] = deal(900,0.55,1,0.43,1.5);

for i = 21
    mask = double(seg ==i);
    Concomp = bwconncomp(mask,4);
    BBox=regionprops(Concomp,'BoundingBox');
    
    
    % Initialise number of filtered detection (amount of accepted Bboxes in the same image)    
    index=1;

    % In case there are no boxes
    FinalBBox = []; 

    % Looping over each box detected
    for j=1:length(BBox)
        % Rounding positions of bounding box
        pos=round(BBox(j).BoundingBox);

        % Cropping the region wihtin the bounding box
        box=mask(pos(2):pos(2)+pos(4)-1,pos(1):pos(1)+pos(3)-1);
        
        
        
        % Calculating area of non-zero pixels
        maskArea=sum(box(:));
        
        % Calculating area of the Bbox
        BBoxArea=BBox(j).BoundingBox(3)*BBox(j).BoundingBox(4);

        % Calculating filling ration
        fillRatio=maskArea./BBoxArea;

        % Calculating Form Factor
        formFactor= BBox(j).BoundingBox(3)./BBox(j).BoundingBox(4);

        % Setting conditions for signals
        if BBoxArea>MinSize && fillRatio>FRmin && fillRatio<=FRmax && formFactor<FFmax && formFactor>FFmin 
            %rectangle('Position',BBox(i).BoundingBox,'EdgeColor','r','LineWidth',2)
            % If the condition is satisfied adding Bounding box to the vector
            FinalBBox(index,:)=BBox(j).BoundingBox;
            index = index+1;
        end
    end
    
    
    
end

