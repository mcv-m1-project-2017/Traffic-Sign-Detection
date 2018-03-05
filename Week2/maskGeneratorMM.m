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
% 
% OUTPUT:
%       ValidationMasks     = A multidimensional matrix that has every mask
%                             from the validation set.
%
%   This function get the mask for all the validation images that where
%   splited on the splitData function.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function ValidationMasks = maskGeneratorMM(pathToDir,ImagesName,model)

    path = pathToDir;

    % Thresholds for HSV colorspace
    red = [20 340];
    blue = [195 240];
    sat = 0.45;
    val = [0.05 0.95];

    % Thresholds for YCbCr colorspace
    Cb = [132 180];
    Cr = [136 180];
    
    % Structural Elements for morphological filtering
    sm = strel('square',5);
    bm = strel('square',13);
    
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
            end
            time = toc;
        
        case 'HSV+YCbCr'
            % Method 2
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

            end
            time = toc;
            
        case 'BackProjection_v1'
            % Method 3
            load BackProkectionDistributions.mat
            
            threshold = 0.09;
            nbins = [size(BMeanHist,1) size(BMeanHist,2)];

            tic;
            for k = 1:length(ImagesName)
                image   = double(rgb2hsv(imread([path ImagesName{k}])));
                h = image(:,:,1);
                s = image(:,:,2);

                Blue = zeros(size(h));
                Red = zeros(size(h));
                BlueRed = zeros(size(h));

                for i = 1:size(s,1)
                    for j = 1:size(s,2)
                        Xidx = round(h(i,j)*(nbins(1)-1))+1;
                        Yidx = round(s(i,j)*(nbins(2)-1))+1;

                        Blue(i,j) = BMeanHist(Xidx,Yidx);
                        Red(i,j) = RMeanHist(Xidx,Yidx);
                        BlueRed(i,j) = BRMeanHist(Xidx,Yidx);
                    end
                end

                mask1 = Blue>threshold*max(Blue(:));
                mask2 = Red>threshold*max(Red(:));
                mask3 = BlueRed>threshold*max(BlueRed(:));

                mask = mask1 + mask2 + mask3;
                ValidationMasks(:,:,k) = imfill(imclose(imopen(mask,sm),bm),'holes');
            end
            time = toc;
            
        case 'BackProjection_v2'
            % Method 4
            load BackProkectionDistributions.mat
            
            threshold = 0.00005;
            tic;
            nbins = [size(BMeanHist,1) size(BMeanHist,2)];

            radio = 4;

            disk = strel('disk',radio);
            disk = double(disk.Neighborhood);

            tic;
            for k = 1:length(ImagesName)
                image   = double(rgb2hsv(imread([path ImagesName{k}])));
                h = image(:,:,1);
                s = image(:,:,2);

                Ih = hist3([h(:) s(:)],nbins);

                Rblue = BMeanHist./Ih;
                Rred = RMeanHist./Ih;
                Rbr = BRMeanHist./Ih;

                Blue = zeros(size(h));
                Red = zeros(size(h));
                BlueRed = zeros(size(h));

                for i = 1:size(s,1)
                    for j = 1:size(s,2)
                        Xidx = round(h(i,j)*(nbins(1)-1))+1;
                        Yidx = round(s(i,j)*(nbins(2)-1))+1;

                        Blue(i,j) = Rblue(Xidx,Yidx);
                        Red(i,j) = Rred(Xidx,Yidx);
                        BlueRed(i,j) = Rbr(Xidx,Yidx);
                    end
                end

                Blue = min(Blue,1);
                Blue = conv2(Blue,disk);
                Blue = Blue/max(max(Blue));
                Blue = Blue>threshold;

                Red = min(Red,1);
                Red = conv2(Red,disk);
                Red = Red/max(max(Red));
                Red = Red>threshold;

                BlueRed = min(BlueRed,1);
                BlueRed = conv2(BlueRed,disk);
                BlueRed = BlueRed/max(max(BlueRed));
                BlueRed = BlueRed>threshold;

                mask = Blue + Red + BlueRed;
                ValidationMasks(:,:,k) = imfill(imclose(imopen(mask(radio:end-radio+1,radio:end-radio+1),sm),bm),'holes');
            end
            time = toc;
    end
    
    fprintf('The mean time for processing the images is: %.3f \n',time/length(ImagesName))

end
