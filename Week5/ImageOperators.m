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
%       filtype         = the operation we want to apply to the image 
%                         (dilation or erosion)
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function transformedImage = ImageOperators(image,mask,filtype)
    
    [ni,nj,nd] = size(image);
    [mi,mj] = size(mask);
    
    mi = floor(mi/2);
    mj = floor(mj/2);
    image = double(image);
    
    if strcmp(filtype,'dilation')
        mask = flip(flip(mask,2),1);
        boundedImage = padarray(image,[mi mj],-Inf,'both');
        fnc = @(x) max(x);
        
    elseif strcmp(filtype,'erosion')
        mask = mask;
        boundedImage = padarray(image,[mi mj],Inf,'both');
        fnc = @(x) min(x);
    else
        disp('Choose between ''dilation'' or ''erosion''')
        return
    end
    
    ImageToFill = zeros(size(boundedImage));
    
    for ch = 1:nd
        for i = mi+1:ni+mi
            for j = mj+1:nj+mj
                toFilter = boundedImage(i-mi:i+mi,j-mj:j+mj,ch);
                ImageToFill(i,j,ch) = fnc(toFilter(mask));
            end
        end
    end
    
    transformedImage = uint8(ImageToFill(mi+1:ni+mi,mj+1:nj+mj,:));
    
end