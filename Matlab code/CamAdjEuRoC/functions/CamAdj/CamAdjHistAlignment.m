function [img1, img2] = CamAdjHistAlignment(img1, img2)
hist1 = imhist(img1);
img2 = histep(img2, hist1);
return