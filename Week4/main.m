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

%% Task 1 A
% image and mask to test our implementation
% Possible models to choose: {'HSV','HSV+YCbCr'}
model = 'HSV';
% Possible models to choose: {'CCL','Global','Sliding Window','Integral Image','Convolution'}
regionModel = 'Global';

tmpMatch = false;

% Get the masks and bounding boxes for the desired method
[ValidationMasks,BoundingBoxes] = maskGeneratorMM(pathToDir,Validation,model,regionModel,tmpMatch);

% Analyse results
[Precision, Accuracy, Specificity, Sensitivity] = systemEvaluation(pathToDir,ValidationMasks,ValidationPosition);

% Analyse window results
[precision, sensitivity, accuracy] = windowEvaluation(pathToDir,BoundingBoxes,ValidationPosition);

%% Task 1 B
% image and mask to test our implementation
% Possible models to choose: {'HSV','HSV+YCbCr'}
model = 'HSV';
% Possible models to choose: {'CCL','Global','Sliding Window','Integral Image','Convolution'}
regionModel = 'CCL';
% Possible models to choose: {'Correlation', 'Distance Transform'}
tmpMatch = 'Correlation';

% Get the masks and bounding boxes for the desired method
[ValidationMasks,BoundingBoxes] = maskGeneratorMM(pathToDir,Validation,model,regionModel,tmpMatch);

% Analyse results
[Precision, Accuracy, Specificity, Sensitivity] = systemEvaluation(pathToDir,ValidationMasks,ValidationPosition);

% Analyse window results
[precision, sensitivity, accuracy] = windowEvaluation(pathToDir,BoundingBoxes,ValidationPosition);

%% Task 1 C
% image and mask to test our implementation
% Possible models to choose: {'HSV','HSV+YCbCr'}
model = 'HSV';
% Possible models to choose: {'CCL','Global','Sliding Window','Integral Image','Convolution'}
regionModel = 'Integral Image';
% Possible models to choose: {'Correlation', 'Distance Transform'}
tmpMatch = 'Correlation';

% Get the masks and bounding boxes for the desired method
[ValidationMasks,BoundingBoxes] = maskGeneratorMM(pathToDir,Validation,model,regionModel,tmpMatch);

% Analyse results
[Precision, Accuracy, Specificity, Sensitivity] = systemEvaluation(pathToDir,ValidationMasks,ValidationPosition);

% Analyse window results
[precision, sensitivity, accuracy] = windowEvaluation(pathToDir,BoundingBoxes,ValidationPosition);


%% Task 2
% image and mask to test our implementation
% Possible models to choose: {'HSV','HSV+YCbCr'}
model = 'HSV';
% Possible models to choose: {'CCL','Global','Sliding Window','Integral Image','Convolution'}
regionModel = 'Integral Image';
% Possible models to choose: {'Correlation', 'Distance Transform'}
tmpMatch = 'Distance Transform';

% Get the masks and bounding boxes for the desired method
[ValidationMasks,BoundingBoxes] = maskGeneratorMM(pathToDir,Validation,model,regionModel,tmpMatch);

% Analyse results
[Precision, Accuracy, Specificity, Sensitivity] = systemEvaluation(pathToDir,ValidationMasks,ValidationPosition);

% Analyse window results
[precision, sensitivity, accuracy] = windowEvaluation(pathToDir,BoundingBoxes,ValidationPosition);
