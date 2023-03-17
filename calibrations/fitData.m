function [p1,p2, mdl] = fitData(x,y)
    
    % check if data > 1
    if size(x,1) < 1 || size(y,1) < 1
        disp('fitData: no data loaded')
    elseif size(x) ~= size(y)
        disp('fitData: check size of input data')
    else
        x = x(~isnan(x));
        y = y(~isnan(y));
        
        mdl = fitlm(x, y);
        result = mdl.Coefficients.Estimate';
        p1 = result(1);
        p2 = result(2);
    
        if mdl.Rsquared.Ordinary < 0.995
            disp('Linear Regression R Sqaured below 0.995')
        end
    end
end