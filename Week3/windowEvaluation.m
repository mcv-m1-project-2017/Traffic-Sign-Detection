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
%       BoundingBoxes       = The boxes that the system found on the validation
%                             set.
%       ValidationPosition  = The position of the image on the dataset.
%
%   In this function the sistem is evaluated according to the mask it has
%   found and the ground truth of the validation set.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [precision, sensitivity, accuracy] = windowEvaluation(pathToDir,BoundingBoxes,ValidationPosition)

    TP = 0;
    FP = 0;
    FN = 0;

    parth = [pathToDir 'gt/'];
    txt = dir([parth '*.txt']);
    
    for i = 1:length(ValidationPosition)
        
        annotations = [];
        pos = ValidationPosition(i);
        windowAnotation = LoadAnnotations([parth txt(pos).name]);
        
        boxes = BoundingBoxes{i,:};

        for j=1:size(boxes,1)
            box = boxes(j,:);
            if ~isempty(box)
                annotations = [annotations;struct('x',box(1),'y',box(2),'w',box(3),'h',box(4))];
            end
        end
        [pixelTP, pixelFP, pixelFN] = PerformanceAccumulationWindow(annotations, windowAnotation);
        TP = TP + pixelTP;
        FP = FP + pixelFP;
        FN = FN + pixelFN;
        
    end

    [precision, sensitivity, accuracy] = PerformanceEvaluationWindow(TP, FN, FP);
    
    fprintf('TP: %i \n',TP)
    fprintf('FP: %i \n',FP)
    fprintf('FN: %i \n',FN)
    
    fprintf('The Precision of the window system is: %.3f %% \n',precision*100)
    fprintf('The Accuracy of the window system is: %.3f %% \n',accuracy*100)
    fprintf('The Sensitivity of the window system is: %.3f %% \n',sensitivity*100)

    fprintf('The F1-score of the system is: %.3f %% \n',2*(precision * sensitivity)/(precision + sensitivity)*100)

end