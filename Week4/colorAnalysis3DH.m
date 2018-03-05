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
%   The purpose of this function is to get 2D histogram distribution over
%   the HS space of the Blue, Red and Blue/Red signals. It also show them
%   in a plot.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function colorAnalysis3DH(pathToDir,Training,TrainingPosition,ImageSigDist,sigPos,sigColor)

    path = pathToDir;

    % Loop counter initializers.
    blueCont = 1;
    redCont = 1;
    blueRedCont = 1;

    sigXim = sum(ImageSigDist,2);
    histNum = 360;
    bins = [50 50];
    
    BHist = zeros(bins);
    RHist = zeros(bins);
    BRHist = zeros(bins);

    for i = 1:size(Training,1)
        % Using images from the training split.
        image   = imread([path Training{i}]);
        pos     = TrainingPosition(i);

        for j=1:sigXim(pos)
            bbindex = sum(sigXim(1:pos-1)) + j;
            bboxpos = round(sigPos(bbindex,:));
            im      = rgb2hsv(image(bboxpos(1):bboxpos(3),bboxpos(2):bboxpos(4),:));
            h = im(:,:,1);
            s = im(:,:,2);
            
            rows = numel(im(:,:,1));
            if (strcmp(sigColor(bbindex),'Blue'))
                % Acumulate the HS histograms for blue signals
                BHist = BHist + hist3(double([h(:) s(:)]),bins);

            elseif (strcmp(sigColor(bbindex),'Red'))
                % Acumulate the HS histograms for red signals
                RHist = RHist + hist3(double([h(:) s(:)]),bins);

            elseif (strcmp(sigColor(bbindex),'Blue/Red')) 
                % Acumulate the HS histograms for blue/red signals
                BRHist = BRHist + hist3(double([h(:) s(:)]),bins);

            end
        end
    end

    % Compute the probability distribution for each signal color group.
    BMeanHist = BHist./sum(BHist(:));
    RMeanHist = RHist./sum(RHist(:));
    BRMeanHist = BRHist./sum(BRHist(:));
    
    % Plot Probability distribution for the signals.
    figure;
    subplot(2,2,1);
    imagesc(BMeanHist)
    title('Probability distribution for Blue signals')
    xlabel('Hue Values')
    ylabel('Saturation Values')
    grid on;
    set(gca,'FontSize',15)

    subplot(2,2,2);
    imagesc(RMeanHist)
    title('Probability distribution for Red signals')
    xlabel('Hue Values')
    ylabel('Saturation Values')
    grid on;
    set(gca,'FontSize',15)

    subplot(2,2,3.5);
    imagesc(BRMeanHist)
    title('Probability distribution for Blue/Red signals')
    xlabel('Hue Values')
    ylabel('Saturation Values')
    grid on;
    set(gca,'FontSize',15)

    % Store for further requirement
    save BackProkectionDistributions.mat BMeanHist RMeanHist BRMeanHist
end