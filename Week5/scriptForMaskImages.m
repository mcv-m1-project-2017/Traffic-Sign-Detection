%% Test Mask Generation
pathToDir = '../../test/';
addpath(genpath('segment-ucm/'))

testEls = dir([pathToDir '*.jpg']);

for i = 1:size(testEls,1)
    Test(i,1) = {testEls(i).name};
end
dbstop if error
model = 'UCM';
% Possible models to choose: {'CCL','Global','Sliding Window','Integral Image','Convolution'}
regionModel = 'Integral Image';
% Possible models to choose: {'Hough','Correlation','Distance Transform'}
tmpMatch = 'Distance Transform';
[TestMasks,BoundingBoxes] = maskGeneratorMM(pathToDir,Test,model,regionModel,tmpMatch);

% It's supposed to be a foder named 'resultsTests'
 for i=1:size(TestMasks,3)
    name = strcat('mask.', testEls(i).name(1:end-4));
    imwrite(uint8(TestMasks(:,:,i)),['../Method 3/' name '.png'])
    mat = BoundingBoxes{i};
    strct = [];
    if ~isempty(mat)
        for j=1:size(mat,1)
            strct = [strct;struct('x', mat(j,1), 'y', mat(j,2), 'w', mat(j,3), 'h', mat(j,4))];
        end
    else
       strct.x = [];
       strct.y = [];
       strct.w = [];
       strct.h = [];
    end
    windowCandidates = strct;
    save(['../Method 3/' name '.mat'], 'windowCandidates')
 end
 
%  save ../'Method 4'/windowCandidates.mat windowCandidates
 
 