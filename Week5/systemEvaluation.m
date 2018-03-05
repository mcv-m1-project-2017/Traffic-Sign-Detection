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
%       ValidationMask      = The mask the system found on the validation
%                             set.
%       ValidationPosition  = The position of the image on the dataset.
%
%   In this function the sistem is evaluated according to the mask it has
%   found and the ground truth of the validation set.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [pixelPrecision, pixelAccuracy, pixelSpecificity, pixelSensitivity] = systemEvaluation(pathToDir,ValidationMasks,ValidationPosition)

    TotalTP = 0;
    TotalFP = 0;
    TotalFN = 0;
    TotalTN = 0;

    mskDir = [pathToDir 'mask/'];

    direc = dir([mskDir '*.png']);

    for i = 1:length(ValidationPosition)
        pos = ValidationPosition(i);
        GTmasks(:,:,i) = imread([mskDir direc(pos).name]);

        [pixelTP, pixelFP, pixelFN, pixelTN] = PerformanceAccumulationPixel(ValidationMasks(:,:,i), GTmasks(:,:,i));
        TotalTP = TotalTP + pixelTP;
        TotalFP = TotalFP + pixelFP;
        TotalFN = TotalFN + pixelFN;
        TotalTN = TotalTN + pixelTN;

    end

    [pixelPrecision, pixelAccuracy, pixelSpecificity, pixelSensitivity] = PerformanceEvaluationPixel...
        (TotalTP, TotalFP, TotalFN, TotalTN);
    
    fprintf('TP: %i \n',TotalTP)
    fprintf('FP: %i \n',TotalFP)
    fprintf('FN: %i \n',TotalFN)
    
    
    fprintf('The Precision of the system is: %.3f %% \n',pixelPrecision*100)
    fprintf('The Accuracy of the system is: %.3f %% \n',pixelAccuracy*100)
    fprintf('The Specificity of the system is: %.3f %% \n',pixelSpecificity*100)
    fprintf('The Sensitivity of the system is: %.3f %% \n',pixelSensitivity*100)

    fprintf('The F1-score of the system is: %.3f %% \n',2*(pixelPrecision * pixelSensitivity)/(pixelPrecision + pixelSensitivity)*100)
end