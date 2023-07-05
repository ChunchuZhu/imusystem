clear
clc

beep = audioread('beep.wav');
start = imread("start.jpg");
relax = imread("relax.jpg");
left = imread("Left.jpg");
right = imread("Right.jpg");
BeReady = imread("BeReady.jpg");

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
    
    id(i) = randi([1,2]);
    if id(i) == 1
        clf
        imshow(left)
        pause(5)
    end

    if id(i) == 2
        clf
        imshow(right)
        pause(5)
    end
    clf
    imshow(relax)
    pause(5)
end
id