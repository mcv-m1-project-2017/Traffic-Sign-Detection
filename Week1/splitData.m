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
%       pathToDir   = the Path to get to the training folder, where the
%                   images, anotations and ground truth are stored.
% 
% OUTPUT:
%       Training            = The name of all the images that are on the
%                             training split
%       TrainingPosition    = The position of the image on the dataset
%       Validation          = The name of all the images that are on the
%                             validation split
%       ValdationPosition   = The position of the image on the dataset
%       ImageSigDist        = A matrix of nx6, where n is the number
%                             of images and one column for each type of 
%                             signal, where it counts the number of signals
%                             in every image.
%       TrValDist
% 
% This function split the training information from the database into
% training/validation subsets. The default split is 70-30%.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
function [Training,TrainingPosition,Validation,ValidationPosition, ...
    ImageSigDist,TrValDist] = splitData(pathToDir)

    % Path to the images and to the text anotations
    jpgDir = pathToDir;
    txtDir = [pathToDir 'gt/'];

    % This gets a struct with the information on the train folder where the
    % .jpg files are stored.
    jpgEls = dir ([jpgDir '*.jpg']);

    % This gets a struct with the information on the gt folder where the .txt
    % files for the masks are stored.
    txtEls = dir ([txtDir '*.txt']);

    % Percentage of signals we want to keep on the training part.
    spl = 0.7;

    % This matrix indicates for every image on the database (rows) how many of
    % the posible signals (columns) do they have.
    ImageSigDist = zeros(size(jpgEls,1),6);

    % These for loops read the text files for every images and update the
    % values on the imageSigDist matrix. Also it storages the name of every
    % image.
    index = 1;
    for i = 1:size(jpgEls,1)    
        % Read again the .txt file to get the number of signals on the image.
        file    = fopen([txtDir txtEls(i).name],'r');
        A       = fscanf(file,'%f %f %f %f %s');

        for j = 1:length(A)/5
            ImageSigDist(i,A(5*j)-64) = ImageSigDist(i,A(5*j)-64) + 1;
        end

        imName(i,1) = {jpgEls(i).name};

        fclose(file);
    end

    % This variable has the amount of each type of signals on the database
    sigAp = sum(ImageSigDist)';

    % In this matrix we keep the number of signals that the training and the
    % validation set must have, according to the split percentage.
    TrValDist = [floor(sigAp*spl) sigAp-floor(sigAp*spl)];
    TrValAux = [floor(sigAp*spl) sigAp-floor(sigAp*spl)];

    % Amount of signals per image.
    sigXim = sum(ImageSigDist,2);

    % Initialize the variables that will sotre the name of the indexation of
    % the images that are going to be part of the training and validation sets.
    Training = [];
    TrainingPosition = [];
    Validation = [];
    ValidationPosition = [];

    for i = max(sigXim):-1:1
        % Case where an image has more than 1 signal.
        if i>1
            % Find the images that have i signals and then randomise the order,
            % so we can get different data sets in every excecution of this
            % script.
            imPos = find(sigXim==i);
            imPos = imPos(randperm(size(imPos,1)));

            % This matrix gets the values of the imageSigDist for the case of
            % more than 1 signal per image.
            mat = ImageSigDist(imPos,:);

            % The t value split the images that have more than 1 signal into
            % two, so one part is asigned to the training and the other to the
            % validation set.
            t = ceil(size(mat,1)/2);
            names = imName(imPos);

            % The name of the images is stored in the Training and Validation
            % variables, and also their position on the database. The TrValAux
            % is updated substracting the amount of signals that are on the 
            % images we assign for the sets.
            Training = [Training;names(1:t,1)];
            TrValAux(:,1) = TrValAux(:,1) - sum(mat(1:t,:),1)';
            TrainingPosition = [TrainingPosition;imPos(1:t,1)];

            Validation = [Validation;names(t+1:end,1)];
            TrValAux(:,2) = TrValAux(:,2) - sum(mat(t+1:end,:),1)';
            ValidationPosition = [ValidationPosition;imPos(t+1:end,1)];

        else
            % For every image that has only 1 signal, the for loop gets the
            % images for every signal type and asign to the training or
            % validation according to the remaining values on the TrValAux
            % matrix.
            for j = 1:6
                % Here it looks for the images that has a signal of the j
                % column and it's the only one on that image
                imPos = find(ImageSigDist(:,j)==1 & sigXim==1);
                imPos = imPos(randperm(size(imPos,1)));
                mat = ImageSigDist(imPos,:);

                ind = TrValAux(j,1);
                names = imName(imPos);

                Training = [Training; names(1:ind,1)];
                TrValAux(:,1) = TrValAux(:,1) - sum(mat(1:ind,:),1)';
                TrainingPosition = [TrainingPosition;imPos(1:ind,1)];

                Validation = [Validation; names(ind+1:end,1)];
                TrValAux(:,2) = TrValAux(:,2) - sum(mat(ind+1:end,:),1)';
                ValidationPosition = [ValidationPosition;imPos(ind+1:end,1)];
            end
        end
    end

    % The resulting vectors are rearranged to get them into an ascending
    % direction
    [~, index] = sort(TrainingPosition);
    Training = Training(index);
    TrainingPosition = TrainingPosition(index);

    [~, index] = sort(ValidationPosition);
    Validation = Validation(index);
    ValidationPosition = ValidationPosition(index);

    save TrainingValidation.mat Training TrainingPosition Validation ...
        ValidationPosition ImageSigDist TrValDist

end