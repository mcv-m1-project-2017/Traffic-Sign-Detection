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
%       image           = the image that the function will use to test the
%                         operators
%       mask            = the mask to filter the image
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function testOperator(image,mask)

    % Binary mask from the mask object
    mask = mask.Neighborhood;

    % Dilation
    dilImage = ImageOperators(image,mask,'dilation');

    % Erosion
    eroImage = ImageOperators(image,mask,'erosion');

    % Close
    clImage = ImageOperators(dilImage,mask,'erosion');

    % Open
    opImage = ImageOperators(eroImage,mask,'dilation');

    % Top Hat
    topImage = image - opImage;

    % Bottom Hat
    botImage = clImage - image;

    % Show results
    figure
    subplot(1,3,1)
    imshow(image)
    title('Original Image')
    set(gca,'FontSize',15)

    subplot(1,3,2)
    imshow(dilImage)
    title('Dilated Image')
    set(gca,'FontSize',15)

    subplot(1,3,3)
    imshow(eroImage)
    title('Eroded Image')
    set(gca,'FontSize',15)

    figure
    subplot(2,2,1)
    imshow(opImage)
    title('Opening')
    set(gca,'FontSize',15)

    subplot(2,2,2)
    imshow(clImage)
    title('Closing')
    set(gca,'FontSize',15)

    subplot(2,2,3)
    imshow(topImage)
    title('Top Hat')
    set(gca,'FontSize',15)

    subplot(2,2,4)
    imshow(botImage)
    title('Dual Top Hat/Bottom Hat')
    set(gca,'FontSize',15)
end