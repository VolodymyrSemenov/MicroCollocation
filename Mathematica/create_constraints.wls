#!/usr/bin/env wolframscript
(* ::Package:: *)

numPoint=9;
tstep=x1/(numPoint-1);

deq[x1_,x2_, x3_,x4_, x5_,Va_]:={x2,
x1*x4^2-g*Sin[x3]-Subscript[k, 0]/m*(x1-l0)-b/m*x2,x4,
(1+(R^2 J^2)/(m*x1))^-1*((-2*x2*x4)/x1-(g*Cos[x3])/x1-(R^2*c*x4)/(m*x1^2)+(kt*x5*R)/(m*x1^2)),
Va/La-(x5*Ra)/La-(R*kb*x4)/La}

cpoint[xl_,xr_,dxl_,dxr_,h_]:=(xl+xr)/2+h/8*(dxl-dxr)

xC[vxl_,vxr_,vdxl_,vdxr_,vh_]:=
{cpoint[vxl[[1]],vxr[[1]],vdxl[[1]],vdxr[[1]],vh],
cpoint[vxl[[2]],vxr[[2]],vdxl[[2]],vdxr[[2]],vh],
cpoint[vxl[[3]],vxr[[3]],vdxl[[3]],vdxr[[3]],vh],
cpoint[vxl[[4]],vxr[[4]],vdxl[[4]],vdxr[[4]],vh],
cpoint[vxl[[5]],vxr[[5]],vdxl[[5]],vdxr[[5]],vh]}

colpoint[cxl_,cxr_, cxc_,cdxl_,cdxr_, cdxc_,ch_]:=(cxl-cxr)+ch/6*(cdxl+4*cdxc+cdxr)

colP[vxl_,vxr_,vxc_,vdxl_,vdxr_,vdxc_,vh_] :=
{colpoint[vxl[[1]],vxr[[1]],vxc[[1]],vdxl[[1]],vdxr[[1]],vdxc[[1]],vh],
colpoint[vxl[[2]],vxr[[2]],vxc[[2]],vdxl[[2]],vdxr[[2]],vdxc[[2]], vh],
colpoint[vxl[[3]],vxr[[3]],vxc[[3]],vdxl[[3]],vdxr[[3]],vdxc[[3]], vh],
colpoint[vxl[[4]],vxr[[4]],vxc[[4]],vdxl[[4]],vdxr[[4]],vdxc[[4]], vh],
colpoint[vxl[[5]],vxr[[5]],vxc[[5]],vdxl[[5]],vdxr[[5]],vdxc[[5]], vh]
}

hContot[gxL_,gxR_]:=(
xdotl=deq[gxL[[1]],gxL[[2]],gxL[[3]],gxL[[4]],gxL[[5]], gxL[[6]]];
xdotr=deq[gxR[[1]],gxR[[2]],gxR[[3]],gxR[[4]],gxR[[5]], gxR[[6]]];

xCent=xC[gxL,gxR,xdotl,xdotr,tstep];
uC=(gxL[[6]]+gxR[[6]])/2;

xdotC= deq[xCent[[1]],xCent[[2]],xCent[[3]],xCent[[4]],xCent[[5]], uC];
clear[deq, xC, cpoint];

colP[gxL,gxR,xCent,xdotl,xdotr,xdotC,tstep])

lagrange[fx_]:= (D[fx, {{x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, x11, x12, x13, x14, x15, x16, x17, x18, x19, x20, x21, x22, x23, x24, x25, x26, x27, x28, x29, x30, x31, x32, x33, x34, x35, x36, x37, x38, x39, x40, x41, x42, x43, x44, x45, x46, x47, x48, x49, x50, x51, x52, x53, x54, x55}}])
hess[fxx_] :=
(D[fxx, {{x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, x11, x12, x13, x14, x15, x16, x17, x18, x19, x20, x21, x22, x23, x24, x25, x26, x27, x28, x29, x30, x31, x32, x33, x34, x35, x36, x37, x38, x39, x40, x41, x42, x43, x44, x45, x46, x47, x48, x49, x50, x51, x52, x53, x54, x55}},{{x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, x11, x12, x13, x14, x15, x16, x17, x18, x19, x20, x21, x22, x23, x24, x25, x26, x27, x28, x29, x30, x31, x32, x33, x34, x35, x36, x37, x38, x39, x40, x41, x42, x43, x44, x45, x46, x47, x48, x49, x50, x51, x52, x53, x54, x55}}])

dvar1={x11,x20,x29,x38,x47,x2};
dvar2={x12,x21,x30,x39,x48,x3};
dvar3={x13,x22,x31,x40,x49,x4};
dvar4={x14,x23,x32,x41,x50,x5};
dvar5={x15,x24,x33,x42,x51,x6};
dvar6={x16,x25,x34,x43,x52,x7};
dvar7={x17,x26,x35,x44,x53,x8};
dvar8={x18,x27,x36,x45,x54,x9};
dvar9={x19,x28,x37,x46,x55,x10};

h1=hContot[dvar1,dvar2];
h2=hContot[dvar2,dvar3];
h3=hContot[dvar3,dvar4];
h4=hContot[dvar4,dvar5];
h5=hContot[dvar5,dvar6];
h6=hContot[dvar6,dvar7];
h7=hContot[dvar7,dvar8];
h8=hContot[dvar8,dvar9];

scriptPath = FileNameJoin[Drop[FileNameSplit[ $InputFileName ],-1]]
SetDirectory[scriptPath]

Export["h1_1.txt",h1[[1]]]
Export["h1_2.txt",h1[[2]]]
Export["h1_3.txt",h1[[3]]]
Export["h1_4.txt",h1[[4]]]
Export["h1_5.txt",h1[[5]]]
Export["h2_1.txt",h2[[1]]]
Export["h2_2.txt",h2[[2]]]
Export["h2_3.txt",h2[[3]]]
Export["h2_4.txt",h2[[4]]]
Export["h2_5.txt",h2[[5]]]
Export["h3_1.txt",h3[[1]]]
Export["h3_2.txt",h3[[2]]]
Export["h3_3.txt",h3[[3]]]
Export["h3_4.txt",h3[[4]]]
Export["h3_5.txt",h3[[5]]]
Export["h4_1.txt",h4[[1]]]
Export["h4_2.txt",h4[[2]]]
Export["h4_3.txt",h4[[3]]]
Export["h4_4.txt",h4[[4]]]
Export["h4_5.txt",h4[[5]]]
Export["h5_1.txt",h5[[1]]]
Export["h5_2.txt",h5[[2]]]
Export["h5_3.txt",h5[[3]]]
Export["h5_4.txt",h5[[4]]]
Export["h5_5.txt",h5[[5]]]
Export["h6_1.txt",h6[[1]]]
Export["h6_2.txt",h6[[2]]]
Export["h6_3.txt",h6[[3]]]
Export["h6_4.txt",h6[[4]]]
Export["h6_5.txt",h6[[5]]]
Export["h7_1.txt",h7[[1]]]
Export["h7_2.txt",h7[[2]]]
Export["h7_3.txt",h7[[3]]]
Export["h7_4.txt",h7[[4]]]
Export["h7_5.txt",h7[[5]]]
Export["h8_1.txt",h8[[1]]]
Export["h8_2.txt",h8[[2]]]
Export["h8_3.txt",h8[[3]]]
Export["h8_4.txt",h8[[4]]]
Export["h8_5.txt",h8[[5]]]
Export["lag_h1_1.txt",lagrange[h1[[1]]]]
Export["lag_h1_2.txt",lagrange[h1[[2]]]]
Export["lag_h1_3.txt",lagrange[h1[[3]]]]
Export["lag_h1_4.txt",lagrange[h1[[4]]]]
Export["lag_h1_5.txt",lagrange[h1[[5]]]]
Export["lag_h2_1.txt",lagrange[h2[[1]]]]
Export["lag_h2_2.txt",lagrange[h2[[2]]]]
Export["lag_h2_3.txt",lagrange[h2[[3]]]]
Export["lag_h2_4.txt",lagrange[h2[[4]]]]
Export["lag_h2_5.txt",lagrange[h2[[5]]]]
Export["lag_h3_1.txt",lagrange[h3[[1]]]]
Export["lag_h3_2.txt",lagrange[h3[[2]]]]
Export["lag_h3_3.txt",lagrange[h3[[3]]]]
Export["lag_h3_4.txt",lagrange[h3[[4]]]]
Export["lag_h3_5.txt",lagrange[h3[[5]]]]
Export["lag_h4_1.txt",lagrange[h4[[1]]]]
Export["lag_h4_2.txt",lagrange[h4[[2]]]]
Export["lag_h4_3.txt",lagrange[h4[[3]]]]
Export["lag_h4_4.txt",lagrange[h4[[4]]]]
Export["lag_h4_5.txt",lagrange[h4[[5]]]]
Export["lag_h5_1.txt",lagrange[h5[[1]]]]
Export["lag_h5_2.txt",lagrange[h5[[2]]]]
Export["lag_h5_3.txt",lagrange[h5[[3]]]]
Export["lag_h5_4.txt",lagrange[h5[[4]]]]
Export["lag_h5_5.txt",lagrange[h5[[5]]]]
Export["lag_h6_1.txt",lagrange[h6[[1]]]]
Export["lag_h6_2.txt",lagrange[h6[[2]]]]
Export["lag_h6_3.txt",lagrange[h6[[3]]]]
Export["lag_h6_4.txt",lagrange[h6[[4]]]]
Export["lag_h6_5.txt",lagrange[h6[[5]]]]
Export["lag_h7_1.txt",lagrange[h7[[1]]]]
Export["lag_h7_2.txt",lagrange[h7[[2]]]]
Export["lag_h7_3.txt",lagrange[h7[[3]]]]
Export["lag_h7_4.txt",lagrange[h7[[4]]]]
Export["lag_h7_5.txt",lagrange[h7[[5]]]]
Export["lag_h8_1.txt",lagrange[h8[[1]]]]
Export["lag_h8_2.txt",lagrange[h8[[2]]]]
Export["lag_h8_3.txt",lagrange[h8[[3]]]]
Export["lag_h8_4.txt",lagrange[h8[[4]]]]
Export["lag_h8_5.txt",lagrange[h8[[5]]]]
Export["hess_h1_1.txt",hess[h1[[1]]]]
Export["hess_h1_2.txt",hess[h1[[2]]]]
Export["hess_h1_3.txt",hess[h1[[3]]]]
Export["hess_h1_4.txt",hess[h1[[4]]]]
Export["hess_h1_5.txt",hess[h1[[5]]]]
Export["hess_h2_1.txt",hess[h2[[1]]]]
Export["hess_h2_2.txt",hess[h2[[2]]]]
Export["hess_h2_3.txt",hess[h2[[3]]]]
Export["hess_h2_4.txt",hess[h2[[4]]]]
Export["hess_h2_5.txt",hess[h2[[5]]]]
Export["hess_h3_1.txt",hess[h3[[1]]]]
Export["hess_h3_2.txt",hess[h3[[2]]]]
Export["hess_h3_3.txt",hess[h3[[3]]]]
Export["hess_h3_4.txt",hess[h3[[4]]]]
Export["hess_h3_5.txt",hess[h3[[5]]]]
Export["hess_h4_1.txt",hess[h4[[1]]]]
Export["hess_h4_2.txt",hess[h4[[2]]]]
Export["hess_h4_3.txt",hess[h4[[3]]]]
Export["hess_h4_4.txt",hess[h4[[4]]]]
Export["hess_h4_5.txt",hess[h4[[5]]]]
Export["hess_h5_1.txt",hess[h5[[1]]]]
Export["hess_h5_2.txt",hess[h5[[2]]]]
Export["hess_h5_3.txt",hess[h5[[3]]]]
Export["hess_h5_4.txt",hess[h5[[4]]]]
Export["hess_h5_5.txt",hess[h5[[5]]]]
Export["hess_h6_1.txt",hess[h6[[1]]]]
Export["hess_h6_2.txt",hess[h6[[2]]]]
Export["hess_h6_3.txt",hess[h6[[3]]]]
Export["hess_h6_4.txt",hess[h6[[4]]]]
Export["hess_h6_5.txt",hess[h6[[5]]]]
Export["hess_h7_1.txt",hess[h7[[1]]]]
Export["hess_h7_2.txt",hess[h7[[2]]]]
Export["hess_h7_3.txt",hess[h7[[3]]]]
Export["hess_h7_4.txt",hess[h7[[4]]]]
Export["hess_h7_5.txt",hess[h7[[5]]]]
Export["hess_h8_1.txt",hess[h8[[1]]]]
Export["hess_h8_2.txt",hess[h8[[2]]]]
Export["hess_h8_3.txt",hess[h8[[3]]]]
Export["hess_h8_4.txt",hess[h8[[4]]]]
Export["hess_h8_5.txt",hess[h8[[5]]]]

