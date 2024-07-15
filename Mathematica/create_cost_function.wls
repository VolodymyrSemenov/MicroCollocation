#!/usr/bin/env wolframscript
(* ::Package:: *)

costF=-10 Subscript[x, 28]Sin[Subscript[x, 37]]-10Subscript[x, 19]*Subscript[x, 37]Cos[Subscript[x, 37]]+ Subscript[x, 28 ]Cos[Subscript[x, 37]]-Subscript[x, 19]*Subscript[x, 46]Sin[Subscript[x, 37]]+Subscript[x, 1]/(14*Subscript[R, a])(\!\(
\*SubsuperscriptBox[\(x\), \(2\), \(2\)] + \(2 
\*SubsuperscriptBox[\(x\), \(3\), \(2\)]\) + \ \(2 
\*SubsuperscriptBox[\(x\), \(4\), \(2\)]\) + \(2 
\*SubsuperscriptBox[\(x\), \(5\), \(2\)]\) + \(2 
\*SubsuperscriptBox[\(x\), \(6\), \(2\)]\) + \(2 
\*SubsuperscriptBox[\(x\), \(7\), \(2\)]\) + \(2 
\*SubsuperscriptBox[\(x\), \(8\), \(2\)]\) + \(2 
\*SubsuperscriptBox[\(x\), \(9\), \(2\)]\) + 
\*SubsuperscriptBox[\(x\), \(10\), \(2\)]\))

scriptPath = FileNameJoin[Drop[FileNameSplit[ $InputFileName ],-1]]
SetDirectory[scriptPath]

Export["cost_function.txt", costF]
Export["lag_cost_function.txt",D[costF, {{Subscript[x, 1], Subscript[x, 2], Subscript[x, 3], Subscript[x, 4],Subscript[x, 5], Subscript[x, 6], Subscript[x, 7], Subscript[x, 8],Subscript[x, 9], Subscript[x, 10],Subscript[x, 11], Subscript[x, 12], Subscript[x, 13], Subscript[x, 14], Subscript[x, 15],Subscript[x, 16], Subscript[x, 17], Subscript[x, 18], Subscript[x, 19],Subscript[x, 20], Subscript[x, 21],Subscript[x, 22], Subscript[x, 23], Subscript[x, 24], Subscript[x, 25], Subscript[x, 26],Subscript[x, 27], Subscript[x, 28], Subscript[x, 29], Subscript[x, 30],Subscript[x, 31], Subscript[x, 32],Subscript[x, 33], Subscript[x, 34], Subscript[x, 35], Subscript[x, 36], Subscript[x, 37],Subscript[x, 38], Subscript[x, 39], Subscript[x, 40], Subscript[x, 41],Subscript[x, 42], Subscript[x, 43],Subscript[x, 44], Subscript[x, 45], Subscript[x, 46], Subscript[x, 47], Subscript[x, 48],Subscript[x, 49], Subscript[x, 50], Subscript[x, 51], Subscript[x, 52],Subscript[x, 53], Subscript[x, 54],Subscript[x, 55]}}]]
Export["hess_cost_function.txt",D[costF, {{Subscript[x, 1], Subscript[x, 2], Subscript[x, 3], Subscript[x, 4],Subscript[x, 5], Subscript[x, 6], Subscript[x, 7], Subscript[x, 8],Subscript[x, 9], Subscript[x, 10],Subscript[x, 11], Subscript[x, 12], Subscript[x, 13], Subscript[x, 14], Subscript[x, 15],Subscript[x, 16], Subscript[x, 17], Subscript[x, 18], Subscript[x, 19],Subscript[x, 20], Subscript[x, 21],Subscript[x, 22], Subscript[x, 23], Subscript[x, 24], Subscript[x, 25], Subscript[x, 26],Subscript[x, 27], Subscript[x, 28], Subscript[x, 29], Subscript[x, 30],Subscript[x, 31], Subscript[x, 32],Subscript[x, 33], Subscript[x, 34], Subscript[x, 35], Subscript[x, 36], Subscript[x, 37],Subscript[x, 38], Subscript[x, 39], Subscript[x, 40], Subscript[x, 41],Subscript[x, 42], Subscript[x, 43],Subscript[x, 44], Subscript[x, 45], Subscript[x, 46], Subscript[x, 47], Subscript[x, 48],Subscript[x, 49], Subscript[x, 50], Subscript[x, 51], Subscript[x, 52],Subscript[x, 53], Subscript[x, 54],Subscript[x, 55]}},{{Subscript[x, 1], Subscript[x, 2], Subscript[x, 3], Subscript[x, 4],Subscript[x, 5], Subscript[x, 6], Subscript[x, 7], Subscript[x, 8],Subscript[x, 9], Subscript[x, 10],Subscript[x, 11], Subscript[x, 12], Subscript[x, 13], Subscript[x, 14], Subscript[x, 15],Subscript[x, 16], Subscript[x, 17], Subscript[x, 18], Subscript[x, 19],Subscript[x, 20], Subscript[x, 21],Subscript[x, 22], Subscript[x, 23], Subscript[x, 24], Subscript[x, 25], Subscript[x, 26],Subscript[x, 27], Subscript[x, 28], Subscript[x, 29], Subscript[x, 30],Subscript[x, 31], Subscript[x, 32],Subscript[x, 33], Subscript[x, 34], Subscript[x, 35], Subscript[x, 36], Subscript[x, 37],Subscript[x, 38], Subscript[x, 39], Subscript[x, 40], Subscript[x, 41],Subscript[x, 42], Subscript[x, 43],Subscript[x, 44], Subscript[x, 45], Subscript[x, 46], Subscript[x, 47], Subscript[x, 48],Subscript[x, 49], Subscript[x, 50], Subscript[x, 51], Subscript[x, 52],Subscript[x, 53], Subscript[x, 54],Subscript[x, 55]}}]]
