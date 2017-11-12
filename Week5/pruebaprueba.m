function tmp = pruebaprueba(sz)

if size(sz,2)==1
    sz = [sz sz];
end

rsz = round(sz*0.85);

mrg = sz - rsz;

switch 'square'
    case 'circle'
        if rsz(1)==rsz(2)
            ax = linspace(-rsz(1)/2+1,rsz(1)/2-1,rsz(1));
            [x,y] = meshgrid(ax,ax);
            tmp = x.^2 + y.^2 <= (rsz(1)/2)^2;
        else
            [mval,in] = min(rsz);
            ax = linspace(-mval/2+1,mval/2-1,mval);
            [x,y] = meshgrid(ax,ax);
            tmp = double(x.^2 + y.^2 <= (mval/2)^2);
            if rsz(in) == rsz(1)
                tmp = [zeros(rsz(in),floor((rsz(2)-rsz(1))/2)) tmp zeros(rsz(in),ceil((rsz(2)-rsz(1))/2))];
            elseif rsz(in) == rsz(2)
                tmp = [zeros(floor((rsz(1)-rsz(2))/2),rsz(in)); tmp; zeros(ceil((rsz(1)-rsz(2))/2),rsz(in))];
            end
            
        end
    case 'square'
        tmp = ones(rsz);
        
    otherwise
        zmat = zeros(rsz);
        rst = 1;
        
        for i = rsz(1):-1:1
            zmat(i,round(rst)+1:end-round(rst)+1) = 1;
            rst = rst + rsz(2)/rsz(1)/2;
        end
        
        switch 'inverted triangle'
            case 'triangle'
                tmp = zmat;
                
            case 'inverted triangle'
                tmp = zmat(end:-1:1,end:-1:1);
        end
        
end
mat = zeros(sz);
mrg = ceil(mrg./2);
mat(mrg(1):mrg(1)+rsz(1)-1,mrg(2):mrg(2)+rsz(2)-1) = tmp;
tmp = mat;

% imagesc(tmp)

end