%% Test Mask Generation
pathToDir = '../../test/';

testEls = dir([pathToDir '*.jpg']);

for i = 1:size(testEls,1)
    Test(i,1) = {testEls(i).name};
end

model = 'HSV';
% Possible models to choose: {'CCL','Sliding Window','Integral Image','Convolution'}
regionModel = 'Convolution';
[TestMasks,BoundingBoxes] = maskGeneratorMM(pathToDir,Test,model,regionModel);

% It's supposed to be a foder named 'resultsTests'
 for i=1:size(TestMasks,3)
    name = strcat('mask.', testEls(i).name(1:end-4));
    imwrite(uint8(TestMasks(:,:,i)),['../Method 4/' name '.png'])
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
    save(['../Method 4/' name '.mat'], 'windowCandidates')
 end
 
%  save ../'Method 4'/windowCandidates.mat windowCandidates
 
 