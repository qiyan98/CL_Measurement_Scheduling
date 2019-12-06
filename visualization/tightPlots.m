function ha = tightPlots(Nh, Nw, w, AR, gap, marg_h, marg_w, units)

% Theodoros Michelis, 15 February 2015
% TUDelft, Aerospace Engineering, Aerodynamics
% t.michelis@tudelft.nl
%
% -------------------------------------------------------------------------
% D E S C R I P T I O N:
% tightPlots is a function that allows making single plots or subplots with
% controlled size and margins. The desired (page) width, aspect ratio and
% margins must be input, with which the appropriate height is calculated.
% 
% This workflow allows for all figures to maintain font formatting when
% they are inserted into a document/publication. Adjust the width to match
% the document page width and fix the font size. Narrow plots can then be
% created by setting wide side margins.
%
% The function additionally creates a bounding box equal to the figure size
% on the screen. As a result, what is seen on the monitor can be printed in
% both raster (.png) and vector (.eps, .pdf) formats with the same result.
%
% tightPlots has been influenced by the "tight_subplot" function of
% Pekka Kumpulainen, hence, parts of the code and parameter names were kept
% the same.
%
% Thanks for using tightPlots!!!
%
% I N P U T:
% ha = tightPlots(Nh, Nw, w, AR, gap, marg_h, marg_w, units)
%        Nh:     Number of axes in the vertical direction
%        Nw:     Number of axes in the horizontal direction
%        w:      Figure width in appropriate units (see units)
%        AR:     [w h] Axis aspect ratio
%        gap:    [gap] Gap between axes in appropriate units
%                   or [gap_h gap_w] for different gaps in height and width
%        marg_h: [mh] Bottom and top margins in appropriate units
%                   or [lower upper] for different lower and upper margins 
%        marg_w: [mw] Left and right margind in appropriate units
%                   or [left right] for different left and right margins
%        units:  Centimeters, inches, points or pixels. The function is not
%                intended to work with normalised units.
%
% I M P O R T A N T: 
% All dimensions are converted into 'points' within the function. This is
% particularly necessary when the input unit is 'pixels', which cannot be
% defined on paper! It is also necessary for versions of MATLAB 2014b and
% above as the rendering settings have been changed. Thus, when printing in
% raster formats, the resolution is controlled through the dpi setting
% while saving (see examples further down).
% 
% S A V I N G  F I G U R E:
% In order to save the figure use the print command as shown in the
% examples below. The '-loose' option ensures that the figure bounding box
% is not clipped while exporting to a vector format. For MATLAB versions of
% 2014b and above, do not forget to set the renderer to '-painters' as the
% default has been switched to '-opengl'.
%
% E X A M P L E  1:
%   ha = tightPlots(2, 2, 15, [2 1], [0.4 0.4], [0.6 0.7], [0.8 0.3], 'centimeters');
%   f = [1 5 10 15]; x = 0:0.05:10;
%   for i = 1:length(f)
%       y = sin(f(i) * x);
%       axes(ha(i)); plot(x, y)
%   end
%   set(ha(1:4), 'fontname', 'Times', 'fontsize', 10)
%   set(ha(1:2), 'xticklabel', '');
%   set([ha(2) ha(4)], 'yticklabel', '');
%   axes(ha(1));  title('Title 1');
%   axes(ha(2));  title('Title 2');
%
%   print(gcf, 'Example1.png', '-dpng', '-r200', '-opengl');
%   print(gcf, 'Example1.eps', '-depsc2', '-painters', '-loose');
%   print(gcf, 'Example1.pdf', '-dpdf', '-painters', '-loose');
% 
% E X A M P L E  2:
%   ha = tightPlots(1, 2, 15, [1 1], 0.25, 0.25, 0.25, 'centimeters');
%   f = [1 5]; x = 0:0.05:10;
%   for i = 1:length(f)
%       y = sin(f(i) * x);
%       axes(ha(i)); plot(x, y)
%   end
%   set(ha(1:2), 'XtickLabel', '', 'YtickLabel', '')
%
%   print(gcf, 'Example2.png', '-dpng', '-r200', '-opengl');
%   print(gcf, 'Example2.eps', '-depsc2', '-painters', '-loose');
%   print(gcf, 'Example2.pdf', '-dpdf', '-painters', '-loose');
% -------------------------------------------------------------------------

% Default values if no input
if nargin<3; w = 15; end
if nargin<4; AR = [2 1]; end
if nargin<5 || isempty(gap); gap = 0.8; end
if nargin<6 || isempty(marg_h); marg_h = [ 0.8 0.4 ]; end
if nargin<7 || isempty(marg_w); marg_w = [ 0.8 0.4 ]; end
if nargin<8; units = 'centimeters'; end
if numel(gap)==1; gap = [gap gap]; end
if numel(marg_w)==1; marg_w = [marg_w marg_w]; end
if numel(marg_h)==1; marg_h = [marg_h marg_h]; end

% Ensure appropriate unit input
if strcmp(units, 'centimeters') + strcmp(units, 'inch') + ...
        strcmp(units, 'points') + strcmp(units, 'pixels') == 0;
    error('Units must be centimeters ''inch'' ''points'' or ''pixels''');
end

% Unit conversion to points
if strcmp(units, 'centimeters') == 1; 
    con = 72/2.54;
elseif strcmp(units, 'inch') == 1
    con = 72;
elseif strcmp(units, 'pixels') == 1
    con = 0.75;
else
    con = 1;
end

units = 'points';
w = w * con;
gap = gap * con;
marg_h = marg_h * con;
marg_w = marg_w * con;

% Calculation of axis width and height
axw = (w-sum(marg_w)-(Nw-1)*gap(2))/Nw;
axh = AR(2)/AR(1)*axw;

% Calculation of figure height
h = Nh*axh + (Nh-1)*gap(1) + sum(marg_h);

% Obtain screen dimensions to place figure in the centre
set(0, 'units', units);
screensize = get(0, 'screensize');
figSize = [ screensize(3)/2 - w/2  screensize(4)/2 - h/2 w h];

% Set the same figure dimensions on screen and paper.
set(gcf, 'Units', units, 'Resize', 'off')
set(gcf, 'Position', figSize)
set(gcf, 'PaperUnits', units, 'PaperSize', [w h])
set(gcf, 'PaperPositionMode', 'manual', 'PaperPosition', [0 0 w h])

% Build axes in figure space
py = h-marg_h(2)-axh; 

ha = zeros(Nh*Nw,1);
ii = 0;
for ih = 1:Nh
    px = marg_w(1);
    
    for ix = 1:Nw
        ii = ii+1;
        
        ha(ii) = axes;
        set(gca, 'Units', units, 'Position', [px py axw axh])
%         set(gca, 'XTickLabel', '', 'YTickLabel', '')
        
        px = px+axw+gap(2);
    end
    
    py = py-axh-gap(1);
end

end