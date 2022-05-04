################################# Parking scenario geometric sizes
param SL == 6.75;
param SW == 2;
param CL == 3.5;

# It is of great importance to note that, when re-setting SL, the setting should also involve the last second line of this file. 
# E.g., if one sets SL == 7.1415, then "param SL == 7.1415;" and "2 1 7.1415".

################################# Simultaneous approach relevant parameters
var tf >= 1;
param NE == 40;
var hi = tf/NE;
set I :={1..NE};
set I1:={1..NE-1};
set J :={1..3};
set K :={0..3};
param tauj{j in K};
param dljtauk{j in K,k in J};
param omega{j in J};
param PPP{ii in {1..2}, m in {1..2}};

################################# Initial conditions
var x0;
param y0 == 1.5;
param theta0 == 0;
param v0 == 0;
param vtf == 0;

################################# Boundary restrictions
param amax == 0.75;
param vmax == 2;
param jerkmax == 0.5;
param dcurmax = 0.6;
param phymax == 0.576;

################################# Vehicle geometric sizes
param n == 0.839;
param l == 2.588;
param m == 0.657;
param b == 0.8855;

################################# States and controls
var xij{i in I,j in K};
var yij{i in I,j in K};
var thetaij{i in I,j in K};
var vij{i in I,j in K};
var aij{i in I,j in K};
var jerkij{i in I,j in J};
var phyij{i in I,j in K};
var wij{i in I,j in J};

var AXij{i in I,j in K};
var BXij{i in I,j in K};
var CXij{i in I,j in K};
var DXij{i in I,j in K};

var AYij{i in I,j in K};
var BYij{i in I,j in K};
var CYij{i in I,j in K};
var DYij{i in I,j in K};

################################# Opt. objective
minimize f:
tf;

#s.t. time:
#tf <= 10;


######### x0 = m + SL
s.t. x0_issue:
x0 = m + SL;

################################# Differential equations converted to NLPs : part 1

s.t. DIFF_dxdt {i in I,k in J}:
sum{j in K}(dljtauk[j,k]*xij[i,j])-hi*vij[i,k]*cos(thetaij[i,k])=0;

s.t. DIFF_dydt {i in I,k in J}:
sum{j in K}(dljtauk[j,k]*yij[i,j])-hi*vij[i,k]*sin(thetaij[i,k])=0;

s.t. DIFF_dvdt {i in I,k in J}:
sum{j in K}(dljtauk[j,k]*vij[i,j])-hi*aij[i,k]=0;

s.t. DIFF_dthetadt {i in I,k in J}:
sum{j in K}(dljtauk[j,k]*thetaij[i,j])-hi*(tan(phyij[i,k]))*vij[i,k]/l=0;

s.t. DIFF_dphydt {i in I,k in J}:
sum{j in K}(dljtauk[j,k]*phyij[i,j])-hi*wij[i,k]=0;

s.t. DIFF_dadt {i in I,k in J}:
sum{j in K}(dljtauk[j,k]*aij[i,j]) - hi*jerkij[i,k]=0;


################################# Differential equations converted to NLPs : part 2

s.t. EQ_diffx {i in I1}:
xij[i+1,0]=sum{j in K}((prod{k in K:k<>j}((1-tauj[k])/(tauj[j]-tauj[k])))*xij[i,j]);

s.t. EQ_diffy {i in I1}:
yij[i+1,0]=sum{j in K}((prod{k in K:k<>j}((1-tauj[k])/(tauj[j]-tauj[k])))*yij[i,j]);

s.t. EQ_diffv {i in I1}:
vij[i+1,0]=sum{j in K}((prod{k in K:k<>j}((1-tauj[k])/(tauj[j]-tauj[k])))*vij[i,j]);

s.t. EQ_difftheta {i in I1}:
thetaij[i+1,0]=sum{j in K}((prod{k in K:k<>j}((1-tauj[k])/(tauj[j]-tauj[k])))*thetaij[i,j]);

s.t. EQ_diffphy {i in I1}:
phyij[i+1,0]=sum{j in K}((prod{k in K:k<>j}((1-tauj[k])/(tauj[j]-tauj[k])))*phyij[i,j]);

s.t. EQ_diffa {i in I1}:
aij[i+1,0]=sum{j in K}((prod{k in K:k<>j}((1-tauj[k])/(tauj[j]-tauj[k])))*aij[i,j]);


################################# Boundary constraints

s.t. EQ_starting_x :
xij[1,0] = x0;

s.t. EQ_starting_y :
yij[1,0] = y0;

s.t. EQ_starting_v :
vij[1,0] = v0;

s.t. EQ_starting_theta :
thetaij[1,0] = theta0;

s.t. EQ_starting_phy :
phyij[1,0] = 0;

s.t. EQ_starting_a :
aij[1,0] = 0;

s.t. EQ_end_v :
vij[NE,3] = 0;

s.t. EQ_end_a :
aij[NE,3] = 0;

################################# Terminal conditions
s.t. EQ_AYtf02 :
AYij[NE,3] <= 0;

s.t. EQ_BYtf02 :
BYij[NE,3] <= 0;

s.t. EQ_CYtf02 :
CYij[NE,3] <= 0;

s.t. EQ_DYtf02 :
DYij[NE,3] <= 0;
################################# 

s.t. Bonds_v {i in I,j in J}:
(vij[i,j])^2 <= vmax^2;

s.t. Bonds_w {i in I,j in J}:
(wij[i,j]/(((cos(phyij[i,j]))^2)*l))^2 <= (dcurmax)^2;

s.t. Bonds_a {i in I,j in J}:
(aij[i,j])^2 <= (amax)^2;

s.t. Bonds_phy {i in I,j in J}:
phyij[i,j]^2 <= (phymax)^2;

s.t. Bonds_da {i in I,j in J}:
jerkij[i,j]^2 <= (jerkmax)^2;


################################# ABCD  OE  relationships
s.t. RELATIONSHIP_AX {i in I,j in K}:
AXij[i,j] = xij[i,j] + (l + n) * cos(thetaij[i,j]) - b * sin(thetaij[i,j]);

s.t. RELATIONSHIP_BX {i in I,j in K}:
BXij[i,j] = xij[i,j] + (l + n) * cos(thetaij[i,j]) + b * sin(thetaij[i,j]);

s.t. RELATIONSHIP_CX {i in I,j in K}:
CXij[i,j] = xij[i,j] - m * cos(thetaij[i,j]) + b * sin(thetaij[i,j]);

s.t. RELATIONSHIP_DX {i in I,j in K}:
DXij[i,j] = xij[i,j] - m * cos(thetaij[i,j]) - b * sin(thetaij[i,j]);

s.t. RELATIONSHIP_AY {i in I,j in K}:
AYij[i,j] = yij[i,j] + (l + n) * sin(thetaij[i,j]) + b * cos(thetaij[i,j]);

s.t. RELATIONSHIP_BY {i in I,j in K}:
BYij[i,j] = yij[i,j] + (l + n) * sin(thetaij[i,j]) - b * cos(thetaij[i,j]);

s.t. RELATIONSHIP_CY {i in I,j in K}:
CYij[i,j] = yij[i,j] - m * sin(thetaij[i,j]) - b * cos(thetaij[i,j]);

s.t. RELATIONSHIP_DY {i in I,j in K}:
DYij[i,j] = yij[i,j] - m * sin(thetaij[i,j]) + b * cos(thetaij[i,j]);



################################# Step functions

s.t. COMPARISON_A {i in I,j in K}:
AYij[i,j] >=  ( floor(AXij[i,j] / SL) - floor(0.5 * AXij[i,j] / SL) + floor( -0.25 * AXij[i,j] / SL ) ) * SW;

s.t. COMPARISON_B {i in I,j in K}:
BYij[i,j] >=  ( floor(BXij[i,j] / SL) - floor(0.5 * BXij[i,j] / SL) + floor( -0.25 * BXij[i,j] / SL ) ) * SW;

s.t. COMPARISON_C {i in I,j in K}:
CYij[i,j] >=  ( floor(CXij[i,j] / SL) - floor(0.5 * CXij[i,j] / SL) + floor( -0.25 * CXij[i,j] / SL ) ) * SW;

s.t. COMPARISON_D {i in I,j in K}:
DYij[i,j] >=  ( floor(DXij[i,j] / SL) - floor(0.5 * DXij[i,j] / SL) + floor( -0.25 * DXij[i,j] / SL ) ) * SW;


#################################  O E issues

s.t. eq_PPoutsideABCD {ii in {1..2}, i in I, j in K}:
abs((AXij[i,j] - PPP[ii,1])*(BYij[i,j] - PPP[ii,2]) - (AYij[i,j] - PPP[ii,2])*(BXij[i,j] - PPP[ii,1])) * 0.5 + abs((BXij[i,j] - PPP[ii,1])*(CYij[i,j] - PPP[ii,2]) - (BYij[i,j] - PPP[ii,2])*(CXij[i,j] - PPP[ii,1])) * 0.5 + abs((CXij[i,j] - PPP[ii,1])*(DYij[i,j] - PPP[ii,2]) - (CYij[i,j] - PPP[ii,2])*(DXij[i,j] - PPP[ii,1])) * 0.5 + abs((DXij[i,j] - PPP[ii,1])*(AYij[i,j] - PPP[ii,2]) - (DYij[i,j] - PPP[ii,2])*(AXij[i,j] - PPP[ii,1])) * 0.5 >= (l+n+m)*2*b + 0.01;



################################# Ceiling
s.t. COMPARISON_A_22 {i in I,j in K}:
AYij[i,j] <= CL;

s.t. COMPARISON_B_22 {i in I,j in K}:
BYij[i,j] <= CL;

s.t. COMPARISON_C_22 {i in I,j in K}:
CYij[i,j] <= CL;

s.t. COMPARISON_D_22 {i in I,j in K}:
DYij[i,j] <= CL;


data;
param: dljtauk :=
0  1  -4.13939
0  2   1.73939
0  3  -3
1  1    3.22474
1  2   -3.56784
1  3   5.53197
2  1   1.16784
2  2    0.775255
2  3  -7.53197
3  1  -0.253197
3  2   1.0532
3  3    5;

param: tauj :=
	0		0
	1		0.1550510257216822
	2		0.6449489742783178
	3		1.0;

param: omega:=
	1		 3.76403062700467e-1 
	2		 5.12485826188421e-1
	3		 1.11111111111111e-1;
	
	
param: PPP :=

1	1	0
1	2	0
2	1	6.75
2	2	0;