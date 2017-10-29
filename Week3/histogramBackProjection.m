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
%       pathToDir       = the Path to get to the training folder, where the
%                         images, anotations and ground truth are stored.
%       ImagesName      = The name of all the images that are on the
%                         validation split.
%       BMeanHist       = 2D Probability distibution for blue signals
%       RMeanHist       = 2D Probability distibution for red signals
%       BEMeanHist      = 2D Probability distibution for blue/red signals
%       threshold       = threshold to filter the probabilities on the
%                         image
% 
% OUTPUT:
%       ValidationMasks     = A multidimensional matrix that has every mask
%                             from the validation set.
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function ValidationMasks = histogramBackProjection(pathToDir,imagesNames,BMeanHist,RMeanHist,BRMeanHist,threshold)

path = pathToDir;
nbins = [size(BMeanHist,1) size(BMeanHist,2)];

tic;
for k = 1:length(imagesNames)
    image   = double(rgb2hsv(imread([path imagesNames{k}])));
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
    
    ValidationMasks(:,:,k) = mask1 + mask2 + mask3;
end
time = toc;

fprintf('The mean time for processing the images is: %.3f \n',time/length(imagesNames))
end