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
%       Validation          = The name of all the images that are on the
%                             validation split.
%       redBound            = The limits to filter the red color.
%       blueBound           = The limits to filter the blue color.
%       satBound            = The limit to filter the saturation.
%       valBound            = The limits to filter the value.
% 
% OUTPUT:
%       ValidationMasks     = A multidimensional matrix that has every mask
%                             from the validation set.
%
%   This function get the mask for all the validation images that where
%   splited on the splitData function.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function ValidationMasks = maskGenerator(pathToDir,Validation,redBound,blueBound,satBound,valBound)

    red = redBound;
    blue = blueBound;
    sat = satBound;
    val = valBound;

    path = pathToDir;

    % Here it iterates over the validation images and their correspondance
    % ground truth.
    tic;
    for i = 1:length(Validation)
        image   = rgb2hsv(imread([path Validation{i}]));
        image(:,:,1) = image(:,:,1)*360;

        % Here it fillters the images
        ValidationMasks(:,:,i) =  ((image(:,:,1)>=blue(1) & image(:,:,1)<=blue(2)) ...
            | (image(:,:,1)>=red(2) | image(:,:,1)<=red(1))) ...
            & (image(:,:,2)>=sat) ...
            & (image(:,:,3)>=val(1) & image(:,:,3)<=val(2));
    end
    time = toc;
    fprintf('The mean time for processing the images is: %.3f',time/length(Validation))
end
