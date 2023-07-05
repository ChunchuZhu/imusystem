clear
clc

beep = audioread('beep.wav');
start = imread("start.jpg");
relax = imread("relax.jpg");
left = imread("Left.jpg");
right = imread("Right.jpg");
BeReady = imread("BeReady.jpg");
sit = imread("sit.jpg");
stand = imread("stand.jpg");

%% Impulse response

figure(1)
clf
imshow(start)
pause(5)

for i =1:10
    clf
    imshow(BeReady)
    sound(beep,48000);
    pause(5)

    clf
    imshow(stand)
    pause(5)

    clf
    imshow(sit)
    pause(3)
end
id