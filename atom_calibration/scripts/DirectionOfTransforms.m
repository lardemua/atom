% Test transformations direction
% Miguel Oliveira

% The goal is to have a simple test that clears once and for all which
% direction the transformations should have.
% Lets us a 2D Transformation (without a Z coordinate), just to simplify.
clc; clear all; close all;


% Two reference systems, A and B. B is in fron of A and 
% rotated 90 degrees. Thus, the transformations is:
I = getTransform(0, 0, 0);
A_T_B =  getTransform(0, 3, 2)*getTransform(pi/4, 0, 0);;
B_T_A = inv(A_T_B);

% Draw using refA as figure frame
% All drawings are on the reference frame A
figure; subplot(1,2,1); al = 7;
axis([-al al -al al]); hold on;
grid on; axis square; 
title('Figure frame is A' )

% Draw reference frame A
drawReferenceFrame(2, I, 'A')

% Draw reference frame B
drawReferenceFrame(2, A_T_B, 'B')


% Now lets have a point P1 defined in the refA
P1_in_A = [2 1 1]'; % homogeneous coorditanes defined in refA
plotPoint(P1_in_A, 'P1-in-A', 'bo', I);

% Point P2  is defined in the refB instead!!!
P2_in_B = [2 1 1]'; % homogeneous coorditanes defined in refA
plotPoint(P2_in_B, 'P2-in-B', 'bo', A_T_B);


% Draw using refB as figure frame
% All drawings are on the reference frame A
subplot(1,2,2); al = 7;
axis([-al al -al al]); hold on;
grid on; axis square; 
title('Figure frame is B')

% Draw reference frame A
drawReferenceFrame(2, B_T_A, 'A')

% Draw reference frame B
drawReferenceFrame(2, I, 'B')


% Now lets have a point P1 = (1,2) defined in the refA
plotPoint(P1_in_A, 'P1-in-A', 'bo', B_T_A);

% Point P2 = (1,2) is defined in the refB instead!!!
plotPoint(P2_in_B, 'P2-in-B', 'bo', I);


function plotPoint(P, name, marker,T)

P_figure_frame = T*P;
x = P_figure_frame(1);
y = P_figure_frame(2); 

plot(x,y, marker)
text(x,y, [name '=(' num2str(x) ',' num2str(y) ')'])

end


function T = getTransform(theta, dx, dy)

T = [cos(theta) -sin(theta) dx
     sin(theta)  cos(theta) dy
         0           0       1];

end

function drawReferenceFrame(scale, T, name)
% scale: the size of the axis
% T: a transformation from frame to draw to the 
% reference frame of the drawing (of the plot)

Po_to_draw_frame = [0 0 1]'; % the origin
Px_to_draw_frame = [scale 0 1]'; % the tip of the x axis
Py_to_draw_frame = [0 scale 1]'; % the tip of the y axis

Po_figure_frame = T*Po_to_draw_frame;
Px_figure_frame = T*Px_to_draw_frame;
Py_figure_frame = T*Py_to_draw_frame;

plot([Po_figure_frame(1) Px_figure_frame(1)], ...
     [Po_figure_frame(2) Px_figure_frame(2)], 'r-')

plot([Po_figure_frame(1) Py_figure_frame(1)], ...
     [Po_figure_frame(2) Py_figure_frame(2)], 'g-')
 
text(Po_figure_frame(1), Po_figure_frame(2), name) 

end