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
%   This script is to run the first block of the project for the M1 subject
%   from the Master in Computer Vision. To use it this script must be in
%   the icv-m1-2017-master folder. IMPORTANT: also the user must asign the
%   absolute path to arrive the training images in the pathToDir variable
%   below.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clearvars
close all

addpath('evaluation/')

saveFlag = 0;

% Here it goes the ABSOLUTE path to the folder with the training images
pathToDir = '../train/';

% Task 1
[sigColor,sigPos,sigShape,sigType] = DatasetAnalysis(pathToDir,saveFlag);

% Task 2
[Training,TrainingPosition,Validation,ValidationPosition, ...
    ImageSigDist,TrValDist] = splitData(pathToDir);

% Task 3
colorAnalysis(pathToDir,Training,TrainingPosition,ImageSigDist,sigPos,sigColor)

blueBound = [195 240];
redBound = [20 340];
satBound = 0.45;
valBound = [0.05 0.95];

ValidationMasks = maskGenerator(pathToDir,Validation,redBound,blueBound,satBound,valBound);

% Task 4
[Precision, Accuracy, Specificity, Sensitivity] = systemEvaluation(pathToDir,ValidationMasks,ValidationPosition);


%%%%%%%%%%%%%%%%%
%save ValidationMasksTeam1.mat ValidationMasks

% It's supposed to be a foder named 'results'
%  for i=1:size(ValidationMasks,3)
%     name = Validation(i,1);
%     cad = strcat('../results/', name, '.png');
%     imwrite(uint8(ValidationMasks(:,:,i)),cad{1});
%  end
