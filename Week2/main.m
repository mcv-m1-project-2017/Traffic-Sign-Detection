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
pathToDir = '../train/';

% load information from the database
load TrainingValidation.mat
load TrainingInfo.mat sigPos sigColor

% Flags
plotFlag = 0;
saveFlag = 0;

%% Task 1 and 2
% image and mask to test our implementation
image = imread('../../train/00.000948.jpg');
mask = strel('disk',3);

testOperator(image,mask)

compareMethods(image)

%% Task 3
% UNCOMMENT for color analysis
% colorAnalysisYCbCr(pathToDir,Training,TrainingPosition,ImageSigDist,sigPos,sigColor);
% colorAnalysisHVS(pathToDir,Training,TrainingPosition,ImageSigDist,sigPos,sigColor);

% Possible models to choose: {'HSV','HSV+YCbCr'}
model = 'HSV';

% Get the masks for the desired method
ValidationMasks = maskGeneratorMM(pathToDir,Validation,model);

% Analyse results
[Precision, Accuracy, Specificity, Sensitivity] = systemEvaluation(pathToDir,ValidationMasks,ValidationPosition);

%% Task 4
% UNCOMMENT for color analysis
% colorAnalysis3DH(pathToDir,Training,TrainingPosition,ImageSigDist,sigPos,sigColor);

% Possible models to choose: {'BackProjection_v1','BackProjection_v2'}
model = 'BackProjection_v2';

% Get the masks for the desired method
ValidationMasks = maskGeneratorMM(pathToDir,Validation,model);

% Analyse results
[Precision, Accuracy, Specificity, Sensitivity] = systemEvaluation(pathToDir,ValidationMasks,ValidationPosition);

%% Test Mask Generation
pathToDir = '../../test/';

testEls = dir([pathToDir '*.jpg']);

for i = 1:size(testEls,1)
    Test(i,1) = {testEls(i).name};
end

blueBound = [195 240];
redBound = [20 340];
satBound = 0.45;
valBound = [0.05 0.95];

model = '';

TestMasks = maskGeneratorMM(pathToDir,Test,model);

% It's supposed to be a foder named 'resultsTests'
 for i=1:size(TestMasks,3)
    name = strcat('mask.', Test(i,1));
    cad = strcat('../resultsTests/',  name, '.png');
    imwrite(uint8(TestMasks(:,:,i)),cad{1});
 end
