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

function ValidationMasks = histogramBackProjectionPaper(pathToDir,imagesNames,BMeanHist,RMeanHist,BRMeanHist,threshold)

path = pathToDir;
nbins = [size(BMeanHist,1) size(BMeanHist,2)];

radio = 4;

disk = strel('disk',radio);
disk = double(disk.Neighborhood);

tic;
for k = 1:length(imagesNames)
    image   = double(rgb2hsv(imread([path imagesNames{k}])));
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
    ValidationMasks(:,:,k) = mask(radio:end-radio+1,radio:end-radio+1);
    
end
time = toc;

fprintf('The mean time for processing the images is: %.3f \n',time/length(imagesNames))
end