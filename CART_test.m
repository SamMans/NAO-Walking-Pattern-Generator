% CART table model
px = [];
py = [];
G = 9.81;
for i = 1:size(COMtot,2)
    px = [px, COMtot(1,i)-((COMtot(3,i)*COMtot(7,i))/(G+COMtot(9,i)))];
    py = [py, COMtot(4,i)-((COMtot(6,i)*COMtot(7,i))/(G+COMtot(9,i)))];
end