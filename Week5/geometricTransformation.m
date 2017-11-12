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
%       movingImg       = image to transform, it is the distorted image.
%       fixedImg        = it is the base image.
%       transformType   = indicates type of transformation to use to
%                         calculate the transformation matrix. Supported
%                         types are: 
%                                   'translation', 
%                                   'rigid',
%                                   'similarity', and
%                                   'affine'.
% 
% OUTPUT:
%       transformedImg  = it is a geometric transformation image that maps 
%                         movingImg to fixedImg.
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [ transformedImg ] = geometricTransformation( movingImg, fixedImg, transformType )

    metric = registration.metric.MeanSquares;
    optimizer = registration.optimizer.RegularStepGradientDescent;
    optimizer.MaximumIterations = 300;
    optimizer.MinimumStepLength = 5e-4;

    % calculating the transform matrix    
    tform = imregtform(movingImg, fixedImg, transformType, optimizer, metric);

    % applying the transform matrix calculated to the distorted image.
    transformedImg = imwarp(fixedImg,tform,'OutputView',imref2d(size(fixedImg)));
end

