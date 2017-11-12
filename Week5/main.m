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
addpath(genpath('segment-ucm/'))

% Here it goes the ABSOLUTE path to the folder with the training images
pathToDir = '../../train/';

% load information from the database
load TrainingValidation.mat
load TrainingInfo.mat sigPos sigColor

%% Method 1
% image and mask to test our implementation
% Possible models to choose: {'HSV','UCM'}
model = 'HSV';
% Possible models to choose: {'CCL','Global','Sliding Window','Integral Image','Convolution'}
regionModel = 'Integral Image';
% Possible models to choose: {'Hough','Correlation', 'Distance Transform'}
shapeModel = 'Distance Transform';

% Get the masks and bounding boxes for the desired method
[ValidationMasks,BoundingBoxes] = maskGeneratorMM(pathToDir,Validation,model,regionModel,shapeModel);

% Analyse results
[Precision, Accuracy, Specificity, Sensitivity] = systemEvaluation(pathToDir,ValidationMasks,ValidationPosition);

% Analyse window results
[precision, sensitivity, accuracy] = windowEvaluation(pathToDir,BoundingBoxes,ValidationPosition);

%% Method 2
% image and mask to test our implementation
% Possible models to choose: {'HSV','UCM'}
model = 'HSV';
% Possible models to choose: {'CCL','Global','Sliding Window','Integral Image','Convolution'}
regionModel = 'Global';
% Possible models to choose: {'Hough','Correlation', 'Distance Transform'}
shapeModel = '';

% Get the masks and bounding boxes for the desired method
[ValidationMasks,BoundingBoxes] = maskGeneratorMM(pathToDir,Validation,model,regionModel,shapeModel);

% Analyse results
[Precision, Accuracy, Specificity, Sensitivity] = systemEvaluation(pathToDir,ValidationMasks,ValidationPosition);

% Analyse window results
[precision, sensitivity, accuracy] = windowEvaluation(pathToDir,BoundingBoxes,ValidationPosition);

%% Method 3
% image and mask to test our implementation
% Possible models to choose: {'HSV','UCM'}
model = 'HSV';
% Possible models to choose: {'CCL','Global','Sliding Window','Integral Image','Convolution'}
regionModel = 'Integral Image';
% Possible models to choose: {'Hough','Correlation', 'Distance Transform'}
shapeModel = 'Hough';

% Get the masks and bounding boxes for the desired method
[ValidationMasks,BoundingBoxes] = maskGeneratorMM(pathToDir,Validation,model,regionModel,shapeModel);

% Analyse results
[Precision, Accuracy, Specificity, Sensitivity] = systemEvaluation(pathToDir,ValidationMasks,ValidationPosition);

% Analyse window results
[precision, sensitivity, accuracy] = windowEvaluation(pathToDir,BoundingBoxes,ValidationPosition);

%% Method 4
% image and mask to test our implementation
% Possible models to choose: {'HSV','UCM'}
model = 'UCM';
% Possible models to choose: {'CCL','Global','Sliding Window','Integral Image','Convolution'}
regionModel = 'Integral Image';
% Possible models to choose: {'Hough','Correlation', 'Distance Transform'}
shapeModel = 'Distance Transform';

% Get the masks and bounding boxes for the desired method
[ValidationMasks,BoundingBoxes] = maskGeneratorMM(pathToDir,Validation,model,regionModel,shapeModel);

% Analyse results
[Precision, Accuracy, Specificity, Sensitivity] = systemEvaluation(pathToDir,ValidationMasks,ValidationPosition);

% Analyse window results
[precision, sensitivity, accuracy] = windowEvaluation(pathToDir,BoundingBoxes,ValidationPosition);
