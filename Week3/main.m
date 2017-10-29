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
% 
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clearvars
close all

addpath('evaluation/')

% Here it goes the ABSOLUTE path to the folder with the training images
pathToDir = '../../train/';

% load information from the database
load TrainingValidation.mat
load TrainingInfo.mat sigPos sigColor

% Flags
plotFlag = 0;
saveFlag = 0;

%% Task 1
% image and mask to test our implementation
% Possible models to choose: {'HSV','HSV+YCbCr'}
model = 'HSV';
% Possible models to choose: {'CCL','Sliding Window','Integral Image','Convolution'}
regionModel = 'CCL';
% Get the masks and bounding boxes for the desired method
[ValidationMasks,BoundingBoxes] = maskGeneratorMM(pathToDir,Validation,model,regionModel);

% Analyse results
[Precision, Accuracy, Specificity, Sensitivity] = systemEvaluation(pathToDir,ValidationMasks,ValidationPosition);

% Analyse window results
[precision, sensitivity, accuracy] = windowEvaluation(pathToDir,BoundingBoxes,ValidationPosition);
%% Task 2
% image and mask to test our implementation
% Possible models to choose: {'HSV','HSV+YCbCr'}
model = 'HSV';
% Possible models to choose: {'CCL','Sliding Window','Integral Image','Convolution'}
regionModel = 'Sliding Window';
% Get the masks and bounding boxes for the desired method
[ValidationMasks,BoundingBoxesSW] = maskGeneratorMM(pathToDir,Validation,model,regionModel);

% Analyse results
[Precision, Accuracy, Specificity, Sensitivity] = systemEvaluation(pathToDir,ValidationMasks,ValidationPosition);

% Analyse window results
[precision, sensitivity, accuracy] = windowEvaluation(pathToDir,BoundingBoxesSW,ValidationPosition);
% Task 3
% image and mask to test our implementation
% Possible models to choose: {'HSV','HSV+YCbCr'}
model = 'HSV';

% Possible models to choose: {'CCL','Sliding Window','Integral Image','Convolution'}
regionModel = 'Integral Image';

% Get the masks and bounding boxes for the desired method
[ValidationMasks,BoundingBoxesII] = maskGeneratorMM(pathToDir,Validation,model,regionModel);

% Analyse results
[Precision, Accuracy, Specificity, Sensitivity] = systemEvaluation(pathToDir,ValidationMasks,ValidationPosition);

% Analyse window results
[precision, sensitivity, accuracy] = windowEvaluation(pathToDir,BoundingBoxesII,ValidationPosition);

% Task 5
% image and mask to test our implementation
% Possible models to choose: {'HSV','HSV+YCbCr'}
model = 'HSV';

% Possible models to choose: {'CCL','Sliding Window','Integral Image','Convolution'}
regionModel = 'Convolution';

% Get the masks and bounding boxes for the desired method
[ValidationMasks,BoundingBoxesCnv] = maskGeneratorMM(pathToDir,Validation,model,regionModel);

% Analyse results
[Precision, Accuracy, Specificity, Sensitivity] = systemEvaluation(pathToDir,ValidationMasks,ValidationPosition);

% Analyse window results
[precision, sensitivity, accuracy] = windowEvaluation(pathToDir,BoundingBoxesCnv,ValidationPosition);