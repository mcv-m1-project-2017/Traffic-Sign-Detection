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
%       saveFlag    = saves or not an excel file with the tables (0/1).
%       plotFlag    = plot or not the figures from the function (0/1).
% 
% OUTPUT:
%       sigColor    = label vector of every signal color on the database
%       sigPos      = position of the bounding box of every signal on the
%                     database
%       sigShape    = label vector of every signal shape on the database
%       sigType     = label vector of every signal type on the database
%
% This function gets the information from the training dataset to achieve 
% the task 1 of the project, creating 3 tables with information grouping 
% the signals by TYPE (A-F), SHAPE (triangles, circles, squares) and COLOR
% (blue, red, blue/red). These tables are then stored on an excel file if 
% saveFlag = 1. Thetables gives the following information:
%
%   Min_Size        = the minimum area of a bounding box of a
%                     type/shape/color
%   Max_Size        = the maximum area of a bounding box of a
%                     type/shape/color
%   Min_FF          = the min value found of the form factor within the
%                     inliers
%   Max_FF          = the max value found of the form factor within the
%                     inliers
%   Min_FR          = the min value found of the filling ratio within the
%                     inliers
%   Max_FR          = the max value found of the filling ratio within the
%                     inliers
%   Frecuency       = frequency of appeareance of the type/shape/color on
%                     the training dataset
%
% The function saves a .mat file 'TrainingInfo.mat' where there are
% variables related to the signals. These variables are usefull for the
% image segmentation analysis and they are stored acording to their 
% apearence on the database. The stored values are the same as the output,  
% plus the sigAp.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [sigColor,sigPos,sigShape,sigType] = DatasetAnalysis(pathToDir,saveFlag,plotFlag)

    % Path to text anotations and masks of the images on the dataset.
    txtDir = [pathToDir 'gt/'];
    mskDir = [pathToDir 'mask/'];

    % This gets a struct with the information on the gt folder where the .txt
    % files for the masks are stored.
    txtEls      = dir ([txtDir '*.txt']);

    % This gets a struct with the information on the mask folder where the mask
    % images are stored.
    imEls       = dir([mskDir '*.png']);

    % index to iterate over the already calculated matrix.
    index = 1;

    mat = [];

    for i = 1:size(imEls,1)
        % Pic the mask.
        im      = imread([mskDir imEls(i).name]);

        % Read the .txt file to get the number of signals on the image and the
        % bounding box of them.
        file    = fopen([pathToDir 'gt/' txtEls(i).name],'r');
        A       = fscanf(file,'%f %f %f %f %s');
        mat     = [mat reshape(A',[5,length(A)/5])];

        % iterate over the signals on the images and calculate the area of the
        % white part in the mask.
        for j = 1:length(A)/5
            pos     = round(mat(1:4,index));
            box     = im(pos(1):pos(3),pos(2):pos(4));
            maskArea(index,1) = sum(box(:));
            index   = index + 1;
        end

        fclose(file);

    end

    % Separate the bounding box positions and the type of the signal.
    sigType     = cellstr(char(mat(5,:))');
    sigPos      = mat(1:4,:)';

    % Unique signal type.
    unSig       = unique(sigType);

    % Frequency of appearences of every signal type.
    sigAp = getFrequency(unSig,sigType);

    % Weight and Height for every signal on the dataset.
    sigWH       = [sigPos(:,4) - sigPos(:,2), sigPos(:,3) - sigPos(:,1)];

    % Area for every signal on the dataset.
    sigSize     = sigWH(:,1).*sigWH(:,2);

    % Form Factor for every signal on the dataset.
    formFactor  = sigWH(:,1)./sigWH(:,2);

    % Filling ratio for every signal on the dataset.
    fillingRatio = maskArea./sigSize;

    % Get the min and max values of different variables. For the formFactor and
    % fillingRatio the only the inliers in the 25-75% are considered. By signal
    % type.
    [minV,maxV,minFF,maxFF,minFR,maxFR] = getMinMax(unSig,sigType,sigSize,formFactor,fillingRatio);

    % Creates a table with the values.
    tableByType = table(strcat(repmat('Type_',size(unSig)), unSig),minV,maxV,minFF,maxFF,minFR,maxFR,sigAp,...
        'VariableNames',{'Type','Min_Size','Max_Size','Min_FF','Max_FF','Min_FR','Max_FR','Frequency'});

    % Creates a vector that assign a shape for every signal type.
    sigShape                        = cell(size(sigType));
    sigShape(strcmp(sigType,'A'))   = {'Triangle'};
    sigShape(strcmp(sigType,'B'))   = {'Triangle'};
    sigShape(strcmp(sigType,'C'))   = {'Circle'};
    sigShape(strcmp(sigType,'D'))   = {'Circle'};
    sigShape(strcmp(sigType,'E'))   = {'Circle'};
    sigShape(strcmp(sigType,'F'))   = {'Square'};

    % Unique values of shape.
    unSha = unique(sigShape);

    % Get the min and max values of different variables. For the formFactor and
    % fillingRatio the only the inliers in the 25-75% are considered. By signal
    % shape.
    [minV,maxV,minFF,maxFF,minFR,maxFR] = getMinMax(unSha,sigShape,sigSize,formFactor,fillingRatio);

    % Frequency of appearences of every signal shape.
    shaAp = getFrequency(unSha,sigShape);

    % Creates a table with the values
    tableByShape = table(unSha,minV,maxV,minFF,maxFF,minFR,maxFR,shaAp,...
        'VariableNames',{'Shape','Min_Size','Max_Size','Min_FF','Max_FF','Min_FR','Max_FR','Frequency'});

    % Creates a vector that assign a color for every signal type
    sigColor                      = cell(size(sigType));
    sigColor(strcmp(sigType,'A')) = {'Red'};
    sigColor(strcmp(sigType,'B')) = {'Red'};
    sigColor(strcmp(sigType,'C')) = {'Red'};
    sigColor(strcmp(sigType,'D')) = {'Blue'};
    sigColor(strcmp(sigType,'E')) = {'Blue/Red'};
    sigColor(strcmp(sigType,'F')) = {'Blue'};

    % Unique values of colors
    unCol = unique(sigColor);

    % Get the min and max values of different variables. For the formFactor and
    % fillingRatio the only the inliers in the 25-75% are considered. By signal
    % color.
    [minV,maxV,minFF,maxFF,minFR,maxFR] = getMinMax(unCol,sigColor,sigSize,formFactor,fillingRatio);

    % Frequency of appearences of every signal color.
    colAp = getFrequency(unCol,sigColor);

    % Creates a table with the values
    tableByColor = table(unCol,minV,maxV,minFF,maxFF,minFR,maxFR,colAp,...
        'VariableNames',{'Color','Min_Size','Max_Size','Min_FF','Max_FF','Min_FR','Max_FR','Frequency'});

    % If desirable an excel file is created with the 3 tables.
    if saveFlag
        % Create excel book with the tables created before
        writetable(tableByType,'DatasetAnalysis.xlsx','WriteVariableNames',true,'Sheet',1)
        writetable(tableByShape,'DatasetAnalysis.xlsx','WriteVariableNames',true,'Sheet',2)
        writetable(tableByColor,'DatasetAnalysis.xlsx','WriteVariableNames',true,'Sheet',3)
    end

    % Store for further requirement
    save TrainingInfo.mat sigPos sigType sigAp sigColor sigShape

    % This vector is used to plot the boxplots below
    aa=sum([strcmp(sigShape,'Triangle') 2*strcmp(sigShape,'Square') 3*strcmp(sigShape,'Circle')],2);
    if plotFlag
        figure
        boxplot(fillingRatio,aa), grid on
        title('Filling Ratio')
        set(gca,'XTickLabel',{'Triangle','Square','Circle'},'FontSize',15)

        figure
        boxplot(formFactor,aa), grid on
        title('Form Factor')
        set(gca,'XTickLabel',{'Triangle','Square','Circle'},'FontSize',15)
    end
end

function frequency = getFrequency(unVals,featVector)
    % Counting apearence on the dataset
    frequency = zeros(size(unVals));
    for i = 1:length(unVals)
        frequency(i,1) = sum(strcmp(featVector,unVals(i)));
    end
end

function [minV,maxV,minFF,maxFF,minFR,maxFR] = getMinMax(unVals,featVector,sizeVector,formFactor,fillingRatio)
    % Initialize variables
    [minV,maxV,minFF,maxFF,minFR,maxFR] = deal(zeros(size(unVals)));

    % Calculation of the parameters by type of signal
    for i = 1:length(unVals)
        query = strcmp(featVector,unVals(i));

        minV(i,1)   = min(sizeVector(query));
        maxV(i,1)   = max(sizeVector(query));
        minFF(i,1)  = min(outliersOut(formFactor(query)));
        maxFF(i,1)  = max(outliersOut(formFactor(query)));
        minFR(i,1)  = min(outliersOut(fillingRatio(query)));
        maxFR(i,1)  = max(outliersOut(fillingRatio(query)));

    end
end

function [inliers,outliers] = outliersOut(vector)
    % Finds the inliers and outliers of a vector, within the 25-75% of the
    % data from vector.
    auxMedia = sort(vector);
    n = length(auxMedia);
    p=0.25;
    np = ceil(n*p);
    n1_p=ceil(n*(1-p));

    Qinf = auxMedia(np);
    Qsup = auxMedia(n1_p);

    IQR = Qsup-Qinf;

    i=Qinf-1.5*IQR;
    s=Qsup+1.5*IQR;

    crit = vector(:,1)>=i & vector(:,1)<=s;
    inliers = vector(crit,:);
    outliers = vector(~crit,:);
end