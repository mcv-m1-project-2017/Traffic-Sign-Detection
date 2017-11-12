clearvars

addpath(genpath('MCG-PreTrained/'))
load ../TrainingValidation.mat

path = '../../../train/';


for i = 1:size(Validation,1)
    image = imread([path Validation{i}]);
    seg = im2ucm(image,'fast');
    save (['SegmentedData/' Validation{i}(1:end-4) '.mat'],'seg');
end