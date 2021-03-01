function[btt,tt]=plot_bsplines(b,knot)

% sample the time interval
% if nargin<3
tt=linspace(knot(1), knot(end),1000);
% else
%     tt=varargin{1};
% end
% since internally we have stored the bsplines in cells, one per each
% interval, we need to do the plotting piece by piece

isBasis=iscell(b{1});
if nargout>0
    isOutput=1;
    if isBasis
        btt={};
        for i=1:length(b)
            btt{i}=[];
        end
    else
        btt=[];
    end
else
    isOutput=0;
end

for jj=1:length(knot)-1
    % if the knot values are equal there is nothing to plot
    if knot(jj)==knot(jj+1)
        continue;
    end
    % get the time sampling which corresponds to the current interval (the
    % jj-th)
    tj=tt(logical((tt<knot(jj+1)).*(tt>=knot(jj))));
    
    % plot the d-1 order bspline and the combination of d-order bsplines on
    % that interval
    if isBasis
        for i=1:length(b)
            tmp=double(subs(b{i}{jj},tj));
            if isOutput
                btt{i}=[btt{i} tmp];
            else
                plot(tj,tmp,'b');
            end
        end
    else
        tmp=double(subs(b{jj},tj));
        if isOutput            
            btt=[btt tmp];
        else
            plot(tj,tmp,'b');
        end
    end
end
% % add the last point
% if isBasis
%     for i=1:length(b)
%         tmp=double(subs(b{i}{jj},tj));
%         if isOutput
%             btt{i}=[btt{i} btt{i}(end)];
%         end
%     end
% else
%     if isOutput
%         btt=[btt btt(end)];
%     end
% end
if isOutput
else
    scatter(knot,zeros(size(knot)),'filled')
end
