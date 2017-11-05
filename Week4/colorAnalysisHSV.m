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
%   to filter the images into the HSV color space. The final plot figure
%   shows the Hue mean value of the Blue, Red and Blue/Red signals.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function colorAnalysisHSV(pathToDir,Training,TrainingPosition,ImageSigDist,sigPos,sigColor)

    path = pathToDir;

    % Loop counter initializers.
    blueCont = 1;
    redCont = 1;
    blueRedCont = 1;

    sigXim = sum(ImageSigDist,2);
    histNum = 360;

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
            im      = rgb2hsv(image(bboxpos(1):bboxpos(3),bboxpos(2):bboxpos(4),:));

            rows = numel(im(:,:,1));
            if (strcmp(sigColor(bbindex),'Blue'))
                % Save all the blue histograms in the matrix ox histograms.
                BHist(:, blueCont,1) = hist(reshape(im(:,:,1),rows,1),histNum);
                BHist(:, blueCont,2) = hist(reshape(im(:,:,2),rows,1),histNum);
                BHist(:, blueCont,3) = hist(reshape(im(:,:,3),rows,1),histNum);
                blueCont = blueCont + 1;

            elseif (strcmp(sigColor(bbindex),'Red'))
                % Save all the blue histograms in the matrix ox histograms.
                RHist(:, redCont,1) = hist(reshape(im(:,:,1),rows,1),histNum);
                RHist(:, redCont,2) = hist(reshape(im(:,:,2),rows,1),histNum);
                RHist(:, redCont,3) = hist(reshape(im(:,:,3),rows,1),histNum);
                redCont = redCont + 1;


            elseif (strcmp(sigColor(bbindex),'Blue/Red')) 
                % Save all the blue/red histograms in the matrix ox histograms.
                BRHist(:, blueRedCont,1) = hist(reshape(im(:,:,1),rows,1),histNum);
                BRHist(:, blueRedCont,2) = hist(reshape(im(:,:,2),rows,1),histNum);
                BRHist(:, blueRedCont,3) = hist(reshape(im(:,:,3),rows,1),histNum);
                blueRedCont = blueRedCont + 1;
            end
        end
    end

    % Compute the average histogram for each signal color group.
    BMeanHist = mean(BHist, 2);
    RMeanHist = mean(RHist, 2);
    BRMeanHist = mean(BRHist, 2);

    % Plot the Hue mean value for the signals.
    figure;
    subplot(2,2,1);
    bar(BMeanHist(:,:,1))
    title('Hue average histogram for Blue signals')
    xlabel('Hue Angle (Degrees)')
    ylabel('Number of pixels')
    grid on;
    set(gca,'FontSize',15)

    subplot(2,2,2);
    bar(RMeanHist(:,:,1))
    title('Hue average histogram for Red signals')
    xlabel('Hue Angle (Degrees)')
    ylabel('Number of pixels')
    grid on;
    set(gca,'FontSize',15)

    subplot(2,2,3.5);
    bar(BRMeanHist(:,:,1))
    title('Hue average histogram for Blue/Red signals')
    xlabel('Hue Angle (Degrees)')
    ylabel('Number of pixels')
    grid on;
    set(gca,'FontSize',15)


end