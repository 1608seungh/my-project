function y = TwoD_RTP(Matrix,Point)

Point_1x3 = [Point(1) Point(2) 1];
MxV = (Matrix*Point_1x3')'

y = [MxV(1) MxV(2)];

end