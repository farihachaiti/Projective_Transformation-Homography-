function mainfunction
  clc
  
  #reading images 
  printf("Reading images\n")
  
  A = imread('a.jpg');  
  
  B = imread("b.jpg"); 
  
  C = imread("c.jpg");
  
  #showing image 1 and 2 and picking corresponding points
  printf("Showing image 1 and 2 and picking corresponding points\n")
  
 

 [Ax1,Ax2,Ax3,Ax4,Ay1,Ay2,Ay3,Ay4]=take_points(A)
 
 
 [Aprimex1,Aprimex2,Aprimex3,Aprimex4,Aprimey1,Aprimey2,Aprimey3,Aprimey4]=take_points(B)
 
 
   #Measuring corresponding points of the images 1 and 2
  printf("Measuring corresponding points of the images 1 and 2\n")
 
  [Ax1y1, Ax2y2, Ax3y3, Ax4y4,T1] = measure_points(Ax1,Ay1,Ax2,Ay2,Ax3,Ay3,Ax4,Ay4)
[Aprimex1y1, Aprimex2y2, Aprimex3y3, Aprimex4y4, T2] = measure_points(Aprimex1,Aprimey1,Aprimex2,Aprimey2,Aprimex3,Aprimey3,Aprimex4,Aprimey4)

  #Condittioned Coordinates
  printf("Conditioned Coordinatesn")
 
 AX1Y1 = design_matrix(Ax1y1,Aprimex1y1)
 AX2Y2 = design_matrix(Ax2y2,Aprimex2y2)
 AX3Y3 = design_matrix(Ax3y3,Aprimex3y3)
 AX4Y4 = design_matrix(Ax4y4,Aprimex4y4)
 
   #2D Homography of images 1 and 2
  printf("2D Homography of images 1 and 2\n")
 
 i1 = twoD_homography(AX1Y1,AX2Y2,AX3Y3,AX4Y4,A,B,T1,T2);
 
   #showing image 3 and the rectified image i1 and picking corresponding points
  printf("Showing image 3 and the rectified image i1 and picking corresponding points\n")
   
 
  [Bx1,Bx2,Bx3,Bx4,By1,By2,By3,By4]=take_points(C)
 
  [Bprimex1,Bprimex2,Bprimex3,Bprimex4,Bprimey1,Bprimey2,Bprimey3,Bprimey4]=take_points(i1)
 
 #Measuring corresponding points of the images 3 and the rectified image i1
  printf("Measuring corresponding points of the images 3 and the rectified image i1\n")
 
 [Bx1y1, Bx2y2, Bx3y3, Bx4y4, T3] = measure_points(Bx1,By1,Bx2,By2,Bx3,By3,Bx4,By4)
[Bprimex1y1, Bprimex2y2, Bprimex3y3, Bprimex4y4,T4] = measure_points(Bprimex1,Bprimey1,Bprimex2,Bprimey2,Bprimex3,Bprimey3,Bprimex4,Bprimey4)
 
 BX1Y1 = design_matrix(Bx1y1,Bprimex1y1)
 BX2Y2 = design_matrix(Bx2y2,Bprimex2y2)
 BX3Y3 = design_matrix(Bx3y3,Bprimex3y3)
 BX4Y4 = design_matrix(Bx4y4,Bprimex4y4)
 
 
   #2D Homography of images 3 and rectified image
  printf("2D Homography of images 3 and rectified image\n")
 
 
 i2 = twoD_homography(BX1Y1,BX2Y2,BX3Y3,BX4Y4,C,i1,T3,T4);
 figure; imshow(i2);
 
 
 endfunction


#function [y,c]=define_function(x)
function [Y1,Y2,Y3,Y4,T]=measure_points(x1,y1,x2,y2,x3,y3,x4,y4)
  
   #Conditioning: Translation
  printf("Conditioning: Translation\n")
  
  
  X1Y1 = [x1; y1; 1]
  X2Y2 = [x2; y2; 1]
  X3Y3 = [x3; y3; 1]
  X4Y4 = [x4; y4; 1]
  XYmat = [abs(X1Y1) abs(X2Y2) abs(X3Y3) abs(X4Y4)]
  
  t = mean(XYmat,2)
  t(1,1)
  t(2,1)
  t(3,1)
  
   #Conditioning: Scalling
  printf("Conditioning: Scalling\n")
  
 X1final = x1 - t(1,1)
 Y1final = y1 - t(2,1)
 X2final = x2 - t(1,1)
 Y2final = y2 - t(2,1)
 X3final = x3 - t(1,1)
 Y3final = y3 - t(2,1)
 X4final = x4 - t(1,1)
 Y4final = y4 - t(2,1)
  
 X1Y1final = [X1final; Y1final; 1]
 X2Y2final = [X2final; Y2final; 1]
 X3Y3final = [X3final; Y3final; 1]
 X4Y4final = [X4final; Y4final; 1]
 
 XYmat2 = [abs(X1Y1final) abs(X2Y2final) abs(X3Y3final) abs(X4Y4final)]
 
 s = mean(XYmat2,2)
 s(1,1)
 s(2,1)
 s(3,1)
 
  #Coordinate Transformation
  printf("Coordinate Transformation\n")
 
 
 T = [1/s(1,1) 0 0; 0 1/s(2,1) 0; 0 0 1]*[ 1 0 -t(1,1); 0 1 -t(2,1); 0 0 1] 
 
  #Conditioned Coordinates
  printf("Conditioned Coordinates\n")
 
 
 Y1 = T*X1Y1
 Y2 = T*X2Y2
 Y3 = T*X3Y3
 Y4 = T*X4Y4
 
endfunction

function y=design_matrix(xy,primexy)
   y = [-primexy(3,1)*xy(1,1)  -primexy(3,1)*xy(2,1) -primexy(3,1)*xy(3,1) 0 0 0 primexy(1,1)*xy(1,1) primexy(1,1)*xy(2,1) primexy(1,1)*xy(3,1); 0 0 0 -primexy(3,1)*xy(1,1) -primexy(3,1)*xy(2,1) -primexy(3,1)*xy(3,1) primexy(2,1)*xy(1,1) primexy(2,1)*xy(2,1) primexy(2,1)*xy(3,1)]
  
  endfunction

  function i=twoD_homography(AX1Y1, AX2Y2, AX3Y3, AX4Y4,A,B,T1,T2)
    
 
  #Design Matrix A1
  printf("Design Matrix A1\n")
 
 A1 = [AX1Y1; AX2Y2; AX3Y3; AX4Y4; 0 0 0 0 0 0 0 0 0]
 
  #Singular Value Decompisition
  printf("Singular Value Decompisition\n")
 
 [U,D,V] = svd(A1)
 #H12 = min(svd(A1))
 H12 = [V(:,9)]
 
  #Reshaping to get the Homography Matrix
  printf("Reshaping to get the Homography Matrix\n")
 
 H12final = transpose(reshape(H12,[3,3]))
 
 

   #Reverse Conditioning
  printf("Reverse Conditioning\n")
 
 H1 = inverse(T2)*H12final*T1
 
   #Normaliation
  printf("Normalization\n")
 
 H1 = H1/H1(3,3)
 
   #Generating 2D Homography of images 
  printf("2D Homography of images\n")
 
 i = geokor(H1,A,B);
 #figure; imshow(i1);
  endfunction

  function [x1,x2,x3,x4,y1,y2,y3,y4]=take_points(A)
    imshow(A)
    
 [xinput,yinput,mouse_button] = ginput(1)
 x1 = xinput
 y1 = yinput
 [xinput,yinput,mouse_button] = ginput(1)
 x2 = xinput
 y2 = yinput
 [xinput,yinput,mouse_button] = ginput(1)
 x3 = xinput
 y3 = yinput
 [xinput,yinput,mouse_button] = ginput(1)
 x4 = xinput
 y4 = yinput 
 
 close(gcf)
    
  endfunction
