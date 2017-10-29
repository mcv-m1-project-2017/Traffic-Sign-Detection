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
%       Training            = The name of all the images that are on the
%                             training split.
%       TrainingPosition    = The position of the image on the dataset.
%       ImageSigDist        = A matrix of nx6, where n is the number
%                             of images and one column for each type of 
%                             signal, where it counts the number of signals
%                             in every image.
%       sigPos              = position of the bounding box of every signal 
%                             on the database.
%       sigColor            = label vector of every signal color on the 
%                             database.
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function integralImageAnalysis(pathToDir,Training,TrainingPosition,ImageSigDist,sigPos,sigColor)

    path = pathToDir;
    
    sigXim = sum(ImageSigDist,2);
    ind = 1;
    for i = 1:size(TrainingPosition,1)
        % Using images from the training split.
%         image   = double(rgb2gray(imread([path Training{i}])));
        image   = double(imread([path 'mask/mask.' Training{i}(1:end-4) '.png']));
        pos     = TrainingPosition(i);

        for j=1:sigXim(pos)
            bbindex = sum(sigXim(1:pos-1)) + j;
            bboxpos = round(sigPos(bbindex,:));
            im      = image(bboxpos(1):bboxpos(3),bboxpos(2):bboxpos(4));
            intIm = cumsum(cumsum(im),2);
            
            intImVal(ind) = intIm(end,end);
            if (strcmp(sigColor(bbindex),'Blue'))
                % 
                color(ind,1) = {'Blue'};
            elseif (strcmp(sigColor(bbindex),'Red'))
                % 
                color(ind,1) = {'Red'};
            elseif (strcmp(sigColor(bbindex),'Blue/Red')) 
                % 
                color(ind,1) = {'BlueRed'};
            end
            ind = ind + 1;
        end
    end
    
    
    figure;
    boxplot(double(intImVal),color), grid on
    set(gca,'Fontsize',15)
    title('Integral Image for each color signal')
    
end