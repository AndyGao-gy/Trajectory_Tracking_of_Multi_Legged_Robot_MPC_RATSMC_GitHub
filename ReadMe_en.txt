
#% Brief descriptions of the supplementary materials/project codes:

1. The supplementary materials correspond to the regular paper entitled "Trajectory Tracking of Multi-Legged Robot Based on Model Predictive and Sliding Mode Control". The authors of this paper are Yong Gao, Wu Wei, Xinmei Wang, Dongliang Wang, Yanjie Li, and Qiuda Yu.

2. All program codes are run in MATLAB. The version of MATLAB we use is MATLAB_R2020a.

3. The two folders correspond to two simulation scenarios: directed curved trajectory tracking and undirected polyline trajectory tracking. The folder named "Curve_Tracking" corresponds to the former, and "Polyline_Tracking" corresponds to the latter.

4. The codes in each folder run in the following order:
	(1) Reference_Curve.m (or Reference_Polyline.m)
	(2) MPC_NTSMC_Curve.m, MPC_PID_Curve.m, MPC_RATSMC_Curve.m, MPC_SMC_Curve.m 
		(or MPC_NTSMC_Polyline.m, MPC_PID_Polyline.m, MPC_RATSMC_Polyline.m, MPC_SMC_Polyline.m)
	(3) Plot_Comparison_Curve.m (or Plot_Comparison_Polyline.m)

5. In each folder, a file named "FUNCTIONSET.m" stores all the self-defined MATLAB function sets/function classes.

6. The subfolders, namely "ReferenceTrajectory", "Comparison_Data", and "Fig_Store", are used to temporarily store the data associated with the reference trajectory, the test data of different algorithms, and figures in .eps format, respectively.

7. Copyright: AndyGao. Date: Sep 2021.