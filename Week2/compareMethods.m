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
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function compareMethods(image)
    
    % Mask for matlab implementation of the morphological filters
    mask = strel('square',11);
    
    % Matlab dilation
    tic;
    mtDil = imdilate(image,mask);
    time1 = toc;
    
    % Matlab erosion
    tic;
    mtEro = imerode(image,mask);
    time2 = toc;
    
    % Our dilation
    tic;
    oiDil = ImageOperators(image,mask.Neighborhood,'dilation');
    time3 = toc;
    
    % Our erosion
    tic;
    oiEro = ImageOperators(image,mask.Neighborhood,'erosion');
    time4 = toc;
    
    % display time results and efficiency
    fprintf('The time of execution of Matlab dilation is: %.3f seconds.\n',time1)
    fprintf('The time of execution of our dilation is: %.3f\n seconds.\n',time3)
    fprintf('The time of execution of Matlab erosion is: %.3f seconds.\n',time2)
    fprintf('The time of execution of our erosion is: %.3f seconds.\n',time4)
    
    fprintf('The efficiency of our dilate is: %.3f %%\n',time3/time1*100)
    fprintf('The efficiency of our erosion is: %.3f %%\n',time4/time2*100)

end