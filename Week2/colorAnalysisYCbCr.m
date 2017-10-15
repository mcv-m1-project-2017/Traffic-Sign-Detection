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
%   The purpose of this function is to get the values the system will use
%   to filter the images into the yCbCr color space. The final plot figure
%   shows the blue chroma for blue signals, the red chroma for red signals
%   and both of them for Blue/Red signals
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function colorAnalysisYCbCr(pathToDir,Training,TrainingPosition,ImageSigDist,sigPos,sigColor)

    path = pathToDir;

    % Loop counter initializers.
    blueCont = 1;
    redCont = 1;
    blueRedCont = 1;

    sigXim = sum(ImageSigDist,2);
    histNum = 256;

    BHist = [];
    RHist = [];
    BRHist = [];

    for i = 1:size(Training,1)
        % Using images from the training split.
        image   = imread([path Training{i}]);
        pos     = TrainingPosition(i);

        for j=1:sigXim(pos)
            bbindex = sum(sigXim(1:pos-1)) + j;
            bboxpos = round(sigPos(bbindex,:));
            im      = double(rgb2ycbcr(image(bboxpos(1):bboxpos(3),bboxpos(2):bboxpos(4),:)));

            rows = numel(im(:,:,1));
            if (strcmp(sigColor(bbindex),'Blue'))
                % Save all the blue chroma values in the matrix ox histograms.
                BHist = [BHist; reshape(im(:,:,2),rows,1)];

            elseif (strcmp(sigColor(bbindex),'Red'))
                % Save all the red chroma values in the matrix ox histograms.
                RHist = [RHist; reshape(im(:,:,3),rows,1)];

            elseif (strcmp(sigColor(bbindex),'Blue/Red')) 
                % Save all the blue/red chroma values in the matrix ox histograms.
                BRHist = [BRHist; [reshape(im(:,:,2),rows,1) reshape(im(:,:,3),rows,1)]];
                
            end
        end
    end

    % Plot the histogram for the signals
    figure;
    subplot(2,2,1);
    hist(BHist,histNum)
    title('Blue chroma histogram for Blue signals')
    xlabel('Chroma values')
    ylabel('Number of pixels')
    grid on;
    set(gca,'FontSize',15)

    subplot(2,2,2);
    hist(RHist,histNum)
    title('Red chroma histogram for Red signals')
    xlabel('Chroma values')
    ylabel('Number of pixels')
    grid on;
    set(gca,'FontSize',15)

    subplot(2,2,3);
    hist(BRHist(:,1),histNum)
    title('Blue chroma histogram for Blue/Red signals')
    xlabel('Chroma values')
    ylabel('Number of pixels')
    grid on;
    set(gca,'FontSize',15)

    subplot(2,2,4);
    hist(BRHist(:,2),histNum)
    title('Red chroma histogram for Blue signals')
    xlabel('Chroma values')
    ylabel('Number of pixels')
    grid on;
    set(gca,'FontSize',15)
    

end