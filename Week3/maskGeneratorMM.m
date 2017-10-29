%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
%                       Master In Computer Vision                         %
%               M1 Introduction to Human and Computer Vision              %
%                               Project                                   %
%                                                                         %
% STUDENTS:                                                               %
%   Arnau Vallve                                                          %
%   Guillermo Torres                                                      %
%   Yevgeniy Kadranov                                                     %
%   Santiago Barbarisi                                                    %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% INPUT:
%       pathToDir           = the Path to get to the training folder, where 
%                             the images, anotations and ground truth are 
%                             stored.
%       ImagesName          = The name of all the images that are on the
%                             validation split.
%       model               = the model to be used to create the masks
%       regionModel         = the model to be used to find the bounding
%                             boxes over a mask image
% 
% OUTPUT:
%       ValidationMasks     = A multidimensional matrix that has every mask
%                             from the validation set.
%       BoundingBoxes       = The list of boxes found for each image
%
%   This function get the mask for all the validation images that where
%   splited on the splitData function.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [ValidationMasks,BoundingBoxes] = maskGeneratorMM(pathToDir,ImagesName,model,regionModel)

    path = pathToDir;

    % Thresholds for HSV colorspace
    red = [20 340];
    blue = [195 240];
    sat = 0.45;
    val = [0.05 0.95];
    
    % Structural Elements for morphological filtering
    sm = strel('square',5);
    bm = strel('square',13);
    
    ValidationMasks = zeros(size(imread([path ImagesName{1}])));
    
    time = 0;
    switch model
        case 'HSV'
            % Method 1
            tic;
            for i = 1:length(ImagesName)
                image = imread([path ImagesName{i}]);

                imageHSV   = rgb2hsv(image);
                imageHSV(:,:,1) = imageHSV(:,:,1)*360;

                maskimage =  ((imageHSV(:,:,1)>=blue(1) & imageHSV(:,:,1)<=blue(2)) ...
                                | (imageHSV(:,:,1)>=red(2) | imageHSV(:,:,1)<=red(1))) ...
                                & (imageHSV(:,:,2)>=sat) ...
                                & (imageHSV(:,:,3)>=val(1) & imageHSV(:,:,3)<=val(2));
                
                ValidationMasks(:,:,i) = imfill(imclose(imopen(maskimage,sm),bm),'holes');
                BoundingBoxes(i,1) = RegionModel(ValidationMasks(:,:,i),regionModel);
                ValidationMasks(:,:,i) = isolateBoxes(ValidationMasks(:,:,i),BoundingBoxes(i,1));
            end
            time = toc;
        
        case 'HSV+YCbCr'
            % Method 2
            % Thresholds for YCbCr colorspace
            Cb = [132 180];
            Cr = [136 180];
            
            tic;
            for i = 1:length(ImagesName)
                image = imread([path ImagesName{i}]);

                imageHSV   = rgb2hsv(image);
                imageHSV(:,:,1) = imageHSV(:,:,1)*360;

                imageYCbCr = rgb2ycbcr(image);
                
                maskimage = (((imageHSV(:,:,1)>=blue(1) & imageHSV(:,:,1)<=blue(2)) ...
                                & (imageYCbCr(:,:,2)>=Cb(1) & imageYCbCr(:,:,2)<=Cb(2))) ...
                                | ((imageHSV(:,:,1)>=red(2) | imageHSV(:,:,1)<=red(1)) ...
                                & (imageYCbCr(:,:,3)>=Cr(1) & imageYCbCr(:,:,3)<=Cr(2)))) ...
                                & imageHSV(:,:,2)>=sat ...
                                & (imageHSV(:,:,3)>=val(1) & imageHSV(:,:,3)<=val(2));
                
                ValidationMasks(:,:,i) = imfill(imclose(imopen(maskimage,sm),bm),'holes');
                BoundingBoxes(i,1) = RegionModel(ValidationMasks(:,:,i),regionModel);
                ValidationMasks(:,:,i) = isolateBoxes(ValidationMasks(:,:,i),BoundingBoxes(i,1));
            end
            time = toc;
    end
    fprintf('The mean time for processing the images is: %.3f \n',time/length(ImagesName))

end

function boxes = RegionModel(toFilter,regionModel)
    switch regionModel
        case 'CCL'
            % Features to be satisfy by bounding boxes with CCL
            [MinSize,FRmin,FRmax,FFmin,FFmax] = deal(900,0.55,1,0.43,1.5);
            im = toFilter;
            boxes{1,1}= BBoxDetect(im, MinSize,FRmin,FRmax,FFmin,FFmax); %calling the BBox function with signal parameters calculated in week1


        case 'Sliding Window'
            FBox = [];
            index = 1;
            scaleFact = 0.2;
            h = 28;
            w = 28;
            windowArea = h*w;
            step = 20;
            mask = toFilter;
            
            while scaleFact <= 1
                
                [ni,nj] = deal(round(size(mask,1)*scaleFact),round(size(mask,2)*scaleFact));
                im = imresize(mask, [ni, nj], 'Nearest');

                for n = 1:step:ni-h
                    for m = 1:step:nj-w
                        box = im(n:n+h-1,m:m+w-1);
                        maskArea = sum(box(:));
                        if maskArea/windowArea>0.5;
                            window = [m,n,w,h];
                            FBox(index,:) = window./scaleFact;
                            index = index+1;
                        end
                    end
                end
                scaleFact = scaleFact+0.1;
            end
            if ~isempty(FBox)
                boxes{1,1} = filterBoxes(FBox,[size(mask,1) size(mask,2)]);
            else
                boxes{1,1} = [];
            end
        case 'Integral Image'
            mask = toFilter;
            FBox = [];
            index = 1;
            scaleFact = 0.2;
            h = 28;
            w = 28;
            windowArea = h*w;
            step = 20;
            while scaleFact <= 1
                [ni,nj] = deal(round(size(mask,1)*scaleFact),round(size(mask,2)*scaleFact));
                im = imresize(mask, [ni, nj], 'Nearest');
                im = cumsum(cumsum(im),2);
                for n = 1:step:ni-h
                    for m = 1:step:nj-w
                        if m==1 && n==1
                            maskArea = im(n+h-1,m+w-1);
                        elseif n==1
                            maskArea = im(n+h-1,m+w-1) - im(n+h-1,m-1);
                        elseif m==1
                            maskArea = im(n+h-1,m+w-1) - im(n-1,m+w-1);
                        else
                            maskArea = im(n+h-1,m+w-1) - im(n+h-1,m-1) - im(n-1,m+w-1) + im(n-1,m-1);
                        end
                        if maskArea/windowArea>0.5;
                            window = [m,n,w,h];
                            FBox(index,:) = window./scaleFact;
                            index = index+1;
                        end
                    end
                end
                scaleFact = scaleFact + 0.1;
            end
            if ~isempty(FBox)
                boxes{1,1} = filterBoxes(FBox,[size(mask,1) size(mask,2)]);
            else
                boxes{1,1} = [];
            end
        case 'Convolution'
            mask = toFilter;
            FBox = [];
            index = 1;
            scaleFact = 0.2;
            h = 28;
            w = 28;
            windowArea = h*w;
            step = 20;
            convEl = ones(h,w);
            while scaleFact <= 1
                [ni,nj] = deal(round(size(mask,1)*scaleFact),round(size(mask,2)*scaleFact));
                im = imresize(mask, [ni, nj], 'Nearest');
                im = conv2(im,convEl,'same');
                rowC = ceil((h - 1) / 2);
                colC = ceil((w - 1) / 2);
                for n = 1:step:ni-h
                    for m = 1:step:nj-w
                        maskArea=im(n + rowC - 1,m + colC - 1);
                        if maskArea/windowArea>0.5;
                            window = [m,n,w,h];
%                             box = [round(window(1:2)/scaleFact) min((window(3)+window(1)-1),nj)/scaleFact min((window(4)+window(2)-1),ni)/scaleFact];
%                             FBox(index,:) = [box(1:2) box(3:4)-box(1:2)];
                            FBox(index,:) = window./scaleFact;
                            index = index+1;
                        end
                    end
                end
                scaleFact = scaleFact+0.1;
            end
            if ~isempty(FBox)
                boxes{1,1} = filterBoxes(FBox,[size(mask,1) size(mask,2)]);
            else
                boxes{1,1} = [];
            end
    end
end

function FinalBBox = BBoxDetect(im, MinSize,FRmin,FRmax,FFmin,FFmax)
    % Detecting connected components (default connectivity of 8)
    Concomp = bwconncomp(im); 
    
    % Extracting bounding boxes from the connected components
    BBox=regionprops(Concomp,'BoundingBox'); 
 
    % Initialise number of filtered detection (amount of accepted Bboxes in the same image)    
    j=1;
    
    % In case there are no boxes
    FinalBBox = []; 
    
    % Looping over each box detected
    for i=1:length(BBox)
        % Rounding positions of bounding box
        pos=round(BBox(i).BoundingBox);
        
        % Cropping the region wihtin the bounding box
        box=imcrop(im,[pos]);
        
        % Calculating area of non-zero pixels
        maskArea=sum(box(:));
        
        % Calculating area of the Bbox
        BBoxArea=BBox(i).BoundingBox(3)*BBox(i).BoundingBox(4);
        
        % Calculating filling ration
        fillRatio=maskArea./BBoxArea;
        
        % Calculating Form Factor
        formFactor= BBox(i).BoundingBox(3)./BBox(i).BoundingBox(4);
        
        % Setting conditions for signals
        if BBoxArea>MinSize && fillRatio>FRmin && fillRatio<=FRmax && formFactor<FFmax && formFactor>FFmin 
            %rectangle('Position',BBox(i).BoundingBox,'EdgeColor','r','LineWidth',2)
            % If the condition is satisfied adding Bounding box to the vector
            FinalBBox(j,:)=BBox(i).BoundingBox;
            j = j+1;
        end
    end
end

function Boxes = filterBoxes(FBox,sz)
    % As the 3 and 4 column gives the size of the box we want the center so its
    % the half of them
    toCenter = FBox(:,3:4)./2;

    % We add that to the upper left points and normalize [0,1]
    center = (FBox(:,1:2) + toCenter)./repmat([sz(2) sz(1)],size(FBox,1),1);

    % Number of boxes
    numBoxes = size(FBox,1);

    % Threshold to filter the centroid's neighbors
    threshold = 0.1;

    % Until the condition doesn't satisfy the threshold, will remain iterating
    condThreshold = 0.85;
    condition = false;

    % initialize the k means number of centroids
    k = 1;
    while ~condition
        % will contain the number of centroid's that are at a certain distance
        % to their centroid
        conTrue = 0;
        % The position of the elements that fullfill the condition
        posTrue = [];
        % Apply k means algorithm to the center of the boxes with k centroids
        [ind,Cent] = kmeans(center,k);
        for i = 1:k
            % Find the indexes from cluster k
            pos = find(ind==i);
            % Find the centers from cluster k
            centerValues = center(pos,:);
            % Calculate cluster's distance to its centroid
            neighbors = pdist2(Cent(i,:),centerValues);
            % Which of them are at least at a certain distance
            conCheck = find(neighbors<threshold);
            % Store the indexes that fullfill the radius condition
            posTrue = [posTrue;pos(conCheck)];
            % Number of elements within a cluster that fullfill the raduis
            % condition
            conTrue = conTrue + numel(conCheck);
        end
        if conTrue/numBoxes >= condThreshold
            condition = true;
        else
            k = k + 1;
        end
    end
    
    ind = ind(posTrue);
    FBox = FBox(posTrue,:);

    clusters = unique(ind);
    Boxes = [];
    
    for i=clusters'
        boxes = FBox(ind==i,:);
        boxes = [min(boxes(:,1:2),[],1) max(boxes(:,1:2)+boxes(:,3:4),[],1)];
        Boxes = [Boxes;boxes(:,1:2) boxes(:,3:4)-boxes(:,1:2)];
    end
end

function mask = isolateBoxes(image,boxes)
    
    mask = zeros(size(image));
    
    if size(boxes,1)==0
        mask = image;
        return
    end
    
    for i = 1:size(boxes,1)
        box = boxes{i,:};
        for j = 1:size(box,1)
            bb = box(j,:);
%             part = imcrop(image,bb);
            part = image(ceil(bb(2)):floor(bb(2)+bb(4))-1,ceil(bb(1)):floor(bb(1)+bb(3))-1);
            mask(ceil(bb(2)):floor(bb(2)+bb(4))-1,ceil(bb(1)):floor(bb(1)+bb(3))-1) = part;
        end
    end
end

