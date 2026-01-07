using MathNet.Numerics.Distributions;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Factorization;
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel;
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel;
using OSDC.DotnetLibraries.General.Common;
using System;
using System.Reflection;
using System.Reflection.Metadata;
using System.Security.Cryptography.X509Certificates;
using static NORCE.Drilling.Simulator4nDOF.Simulator.Utilities;

namespace NORCE.Drilling.Simulator4nDOF.Simulator
{
    public static class AccelerationCalculation
    {
       

        public static void PrepareAxialTorsional(AxialTorsionalModel model, State state, SimulationParameters parameters)
        {
            //Update waves            
            model.DownwardAxialWave = state.PipeAngularVelocity + parameters.Drillstring.TorsionalWaveSpeed * state.PipeShearStrain;
            model.UpwardTorsionalWave = state.PipeAngularVelocity - parameters.Drillstring.TorsionalWaveSpeed * state.PipeShearStrain; // Upward traveling wave, torsional
            model.DownwardAxialWave = state.PipeAxialVelocity + parameters.Drillstring.AxialWaveSpeed * state.PipeAxialStrain; // Downward traveling wave, axial
            model.UpwardAxialWave = state.PipeAxialVelocity - parameters.Drillstring.AxialWaveSpeed * state.PipeAxialStrain; // Upward traveling wave, axial
        }
        public static void PreprareLateral(LateralModel model, State state, SimulationParameters parameters)
        {   
            Vector<double> elementWiseProduct = parameters.Drillstring.YoungModuli.PointwiseMultiply(parameters.Drillstring.PipeArea);
            model.ScalingMatrix = Vector<double>.Build.Dense(parameters.LumpedCells.DistributedToLumpedRatio, 1).ToColumnMatrix();
            Matrix<double> dragMatrix = model.ScalingMatrix * elementWiseProduct.ToRowMatrix();
            dragMatrix = state.PipeAxialStrain.PointwiseMultiply(dragMatrix);
            Vector<double> drag_flattened = ToVector(dragMatrix.ToColumnMajorArray());
            Vector<double> drag = LinearInterpolate(parameters.DistributedCells.x, drag_flattened, parameters.LumpedCells.ElementLength);
            Vector<double> phiVec_dote = ExtendVectorStart(0, parameters.Trajectory.phiVec_dot);
            Vector<double> thetaVec_dote = ExtendVectorStart(0, parameters.Trajectory.thetaVec_dot);
            Vector<double> thetaVece = ExtendVectorStart(0, parameters.Trajectory.thetaVec);
            Vector<double> trapezoidalsIntegration = CummulativeTrapezoidal(parameters.LumpedCells.ElementLength, Reverse(parameters.Buoyancy.dsigma_dx));
            model.Tension = Reverse(trapezoidalsIntegration) + parameters.Buoyancy.axialBuoyancyForceChangeOfDiameters - drag;
            Vector<double> fN_softstring = (Square((model.Tension + parameters.Buoyancy.normalBuoyancyForceChangeOfDiameters).PointwiseMultiply(thetaVec_dote) - parameters.Buoyancy.Wb.PointwiseMultiply(thetaVece.PointwiseSin())) +
                                            Square((model.Tension + parameters.Buoyancy.normalBuoyancyForceChangeOfDiameters).PointwiseMultiply(phiVec_dote).PointwiseMultiply(thetaVece.PointwiseSin()))).PointwiseSqrt();
            Vector<double> I_fN_softstring = Utilities.CummulativeTrapezoidal(parameters.LumpedCells.ElementLength, fN_softstring);
            model.SoftStringNormalForce = Diff(I_fN_softstring); // [N] Lumped normal force per element assuming soft - string model(not used in 4nDOF model)

            Vector<double> AiExtended = ExtendVectorStart(parameters.Drillstring.InnerArea[0], parameters.Drillstring.InnerArea);
            Vector<double> AoExtended = ExtendVectorStart(parameters.Drillstring.OuterArea[0], parameters.Drillstring.OuterArea);
            
            /*Vector<double> F_comp = AiExtended.PointwiseMultiply(parameters.Buoyancy.stringPressure - parameters.Buoyancy.hydrostaticStringPressure) * (1 - 2 * parameters.Drillstring.PoissonRatio)
                                    - AoExtended.PointwiseMultiply(parameters.Buoyancy.annularPressure - parameters.Buoyancy.hydrostaticAnnularPressure) * (1 - 2 * parameters.Drillstring.PoissonRatio)
                                  - Tension;
            Tension = -F_comp;*/
            
            model.Tension += (1 - 2 * parameters.Drillstring.PoissonRatio) * 
                (  
                    AoExtended.PointwiseMultiply(parameters.Buoyancy.annularPressure - parameters.Buoyancy.hydrostaticAnnularPressure) 
                    - AiExtended.PointwiseMultiply(parameters.Buoyancy.stringPressure - parameters.Buoyancy.hydrostaticStringPressure)
                );   

            Vector<double> bendingStiffness = ExtendVectorStart(parameters.Drillstring.BendingStiffness[0], parameters.Drillstring.BendingStiffness); 
            model.BendingStiffness = - (bendingStiffness - Math.Pow(Math.PI, 2) * model.Tension / (2 * parameters.Drillstring.PipeLengthForBending)).PointwiseMaximum(0.0);                                            
            model.PolarMomentTimesShearModuli = parameters.Drillstring.PipePolarMoment.PointwiseMultiply(parameters.Drillstring.ShearModuli); // Element-wise multiplication
            Matrix<double> TorqueMatrix = state.PipeShearStrain.PointwiseMultiply(model.ScalingMatrix * model.PolarMomentTimesShearModuli.ToRowMatrix()); // 5x136 matrix
            Vector<double> torqueFlattened = ToVector(TorqueMatrix.ToColumnMajorArray());
            Vector<double> torque = LinearInterpolate(parameters.DistributedCells.x, torqueFlattened, parameters.LumpedCells.ElementLength);
            // Normal force components in Frenet-Serret coordinate system
            Vector<double> binormal = ExtendVectorStart(parameters.Trajectory.bz[0], parameters.Trajectory.bz);
            Vector<double> normal = ExtendVectorStart(parameters.Trajectory.nz[0], parameters.Trajectory.nz);
            Vector<double> curvatureExtended = ExtendVectorStart(parameters.Trajectory.curvature[0], parameters.Trajectory.curvature);
            Vector<double> curvature_dotExtended = ExtendVectorStart(parameters.Trajectory.curvature_dot[0], parameters.Trajectory.curvature_dot);
            Vector<double> curvature_ddotExtended = ExtendVectorStart(parameters.Trajectory.curvature_ddot[0], parameters.Trajectory.curvature_ddot);
            Vector<double> youngModulus = ExtendVectorStart(parameters.Drillstring.YoungModuli[0], parameters.Drillstring.YoungModuli);
            Vector<double> momentOfInertia = ExtendVectorStart(parameters.Drillstring.PipeInertia[0], parameters.Drillstring.PipeInertia);
            Vector<double> torsionExtended = ExtendVectorStart(parameters.Trajectory.torsion[0], parameters.Trajectory.torsion);
            Vector<double> torsion_dotExtended = ExtendVectorStart(parameters.Trajectory.torsion_dot[0], parameters.Trajectory.torsion_dot);
            Vector<double> diffTorqueExtended = ExtendVectorStart(0, Diff(torque) / parameters.LumpedCells.DistanceBetweenElements);

            Vector<double> fB =
                parameters.Buoyancy.Wb.PointwiseMultiply(binormal) +
                curvatureExtended.PointwiseMultiply(diffTorqueExtended) +
                curvature_dotExtended.PointwiseMultiply(torque) -
                2 * youngModulus.PointwiseMultiply(momentOfInertia).PointwiseMultiply(curvature_dotExtended).PointwiseMultiply(torsionExtended) -
                youngModulus.PointwiseMultiply(momentOfInertia).PointwiseMultiply(curvatureExtended).PointwiseMultiply(torsion_dotExtended);

            // Compute pre-stressed forces
            Vector<double> I_fB = CummulativeTrapezoidal(parameters.LumpedCells.ElementLength, fB);

            Vector<double> fN =
                curvatureExtended.PointwiseMultiply(model.Tension + parameters.Buoyancy.normalBuoyancyForceChangeOfDiameters) +
                parameters.Buoyancy.Wb.PointwiseMultiply(normal) -
                youngModulus.PointwiseMultiply(momentOfInertia).PointwiseMultiply(curvature_ddotExtended) +
                youngModulus.PointwiseMultiply(momentOfInertia).PointwiseMultiply(curvatureExtended).PointwiseMultiply(Square(torsionExtended)) -
                curvatureExtended.PointwiseMultiply(torsionExtended).PointwiseMultiply(torque);

            Vector<double> I_fN = CummulativeTrapezoidal(parameters.LumpedCells.ElementLength, fN);
            model.PreStressNormalForce = Diff(I_fB);
            model.PreStressBinormalForce = Diff(I_fN);

            Vector<double> expression =
                (parameters.Trajectory.hy.PointwiseMultiply(parameters.Trajectory.nz) - (parameters.Trajectory.hz.PointwiseMultiply(parameters.Trajectory.ny))).PointwiseMultiply(parameters.Trajectory.tx) +
                (parameters.Trajectory.hz.PointwiseMultiply(parameters.Trajectory.nx) - (parameters.Trajectory.hx.PointwiseMultiply(parameters.Trajectory.nz))).PointwiseMultiply(parameters.Trajectory.ty) +
                (parameters.Trajectory.hx.PointwiseMultiply(parameters.Trajectory.ny) - (parameters.Trajectory.hy.PointwiseMultiply(parameters.Trajectory.nx))).PointwiseMultiply(parameters.Trajectory.tz);

            //Vector<double> signToolFace = expression.Map(x => (double)Math.Sign(x));

            Vector<double> signToolFace = Vector<double>.Build.Dense(expression.Count);
            Vector<double> dotProduct = Vector<double>.Build.Dense(expression.Count);
            for (int i = 0; i < expression.Count; i++)
            {
                signToolFace[i] = Math.Sign(expression[i]);
                dotProduct[i] = parameters.Trajectory.hx[i] * parameters.Trajectory.nx[i] +
                                parameters.Trajectory.hy[i] * parameters.Trajectory.ny[i] +
                                parameters.Trajectory.hz[i] * parameters.Trajectory.nz[i];
                dotProduct[i] = Math.Max(-1, dotProduct[i]);
                dotProduct[i] = Math.Min(1, dotProduct[i]);
            }

            // Compute the toolface angle
            model.ToolFaceAngle = dotProduct.PointwiseAcos().PointwiseMultiply(signToolFace);
            


        }
        public static void AxialTorsionalSystem(AxialTorsionalModel torsionalModel, Input simulationInput, Configuration configuration, State state, SimulationParameters parameters)
        {                    
            double axialVelocityLeft;
            double axialVelocityRight;
            double torsionalVelocityLeft;
            double torsionalVelocityRight;        
            for (int i = 0; i < state.AxialVelocity.Count; i++)
            {
                axialVelocityLeft = (i == 0) ? simulationInput.CalculateSurfaceAxialVelocity : state.AxialVelocity[i - 1];
                torsionalVelocityLeft = (i == 0) ? state.TopDriveAngularVelocity : state.AngularVelocity[i - 1];
                axialVelocityRight = state.AxialVelocity[i];
                torsionalVelocityRight = state.AngularVelocity[i];
                // Left boundaries   
                torsionalModel.DownwardTorsionalWaveLeftBoundary[i] = - torsionalModel.UpwardTorsionalWave[0, i] + 2 * torsionalVelocityLeft;
                torsionalModel.UpwardTorsionalWaveLeftBoundary[i]   = - torsionalModel.UpwardAxialWave[0, i] + 2 * axialVelocityLeft;
                // Right boundaries
                torsionalModel.DownwardAxialWaveRightBoundary[i] = - torsionalModel.DownwardTorsionalWave[parameters.LumpedCells.DistributedToLumpedRatio - 1, i] + 2 * torsionalVelocityRight;
                torsionalModel.UpwardAxialWaveRightBoundary[i] = - torsionalModel.DownwardAxialWave[parameters.LumpedCells.DistributedToLumpedRatio - 1, i] + 2 * axialVelocityRight;                        
            }
            // Left boundaries
            //torsionalModel.DownwardTorsionalWaveLeftBoundary = -torsionalModel.UpwardTorsionalWave.Row(0) + 2 * torsionalModel.RotationalVelocity.SubVector(0, torsionalModel.RotationalVelocity.Count - 1);
            //torsionalModel.UpwardTorsionalWaveLeftBoundary = -torsionalModel.UpwardAxialWave.Row(0) + 2 * torsionalModel.AxialVelocity.SubVector(0, torsionalModel.AxialVelocity.Count - 1);
            //// Right boundaries
            //torsionalModel.DownwardAxialWaveRightBoundary = -torsionalModel.DownwardTorsionalWave.Row(parameters.LumpedCells.DistributedToLumpedRatio - 1) + 2 * torsionalModel.RotationalVelocity.SubVector(1, torsionalModel.RotationalVelocity.Count - 1);
            //torsionalModel.UpwardAxialWaveRightBoundary = -torsionalModel.DownwardAxialWave.Row(parameters.LumpedCells.DistributedToLumpedRatio - 1) + 2 * torsionalModel.AxialVelocity.SubVector(1, torsionalModel.AxialVelocity.Count - 1);

            state.BitVelocity = 0.5 * (torsionalModel.DownwardAxialWave[parameters.LumpedCells.DistributedToLumpedRatio - 1, torsionalModel.DownwardAxialWave.ColumnCount - 1] + torsionalModel.UpwardAxialWave[parameters.LumpedCells.DistributedToLumpedRatio - 1, torsionalModel.UpwardAxialWave.ColumnCount - 1]);
            
            
            double angularVelocityBottom;
            if (!configuration.UseMudMotor)
                angularVelocityBottom = 0.5 * (torsionalModel.DownwardTorsionalWave[parameters.LumpedCells.DistributedToLumpedRatio - 1, torsionalModel.DownwardTorsionalWave.ColumnCount - 1] + torsionalModel.UpwardTorsionalWave[parameters.LumpedCells.DistributedToLumpedRatio - 1, torsionalModel.UpwardTorsionalWave.ColumnCount - 1]);
            else
                angularVelocityBottom = state.MudRotorAngularVelocity;

            double[] bitForces = parameters.BitRock.
                CalculateInteractionForce(state, angularVelocityBottom, torsionalModel.DownwardTorsionalWave, parameters);
            torsionalModel.TorqueOnBit = bitForces[0];
            torsionalModel.WeightOnBit = bitForces[1];    
            // manage the bit sticking off bottom condition
            if (!state.onBottom)
            {
                double omega_ = state.AngularVelocity[state.AngularVelocity.Count - 1];
                if (simulationInput.StickingBoolean)
                {
                    int lastIndex = parameters.Drillstring.ShearModuli.Count - 1;
                    Vector<double> TorsionalAcceleration = torsionalModel.UpwardTorsionalWave.Row(parameters.LumpedCells.DistributedToLumpedRatio - 1);
                    Vector<double> AxialAcceleration = torsionalModel.UpwardAxialWave.Row(parameters.LumpedCells.DistributedToLumpedRatio - 1);
                    torsionalModel.TorqueOnBit = parameters.Drillstring.PipePolarMoment[lastIndex] * parameters.Drillstring.ShearModuli[lastIndex] / parameters.Drillstring.TorsionalWaveSpeed * TorsionalAcceleration[TorsionalAcceleration.Count - 1];
                    torsionalModel.WeightOnBit = parameters.Drillstring.PipeArea[lastIndex] * parameters.Drillstring.YoungModuli[lastIndex] / parameters.Drillstring.AxialWaveSpeed * AxialAcceleration[TorsionalAcceleration.Count - 1];
                }
                else
                {
                    double normalForce_ = simulationInput.BottomExtraNormalForce;
                    if (normalForce_ > 0)
                    {
                        double ro_ = parameters.Drillstring.OuterRadius[parameters.Drillstring.OuterRadius.Count - 1];
                        double Fs_ = normalForce_ * 1.0; //Static force
                        double Fc_ = normalForce_ * 0.5; //Kinematic force
                        double va_ = state.AxialVelocity[state.AxialVelocity.Count - 1]; //Axial velocity
                        double v_ = Math.Sqrt(va_ * va_ + omega_ * omega_ * ro_ * ro_); //Tangential velocity
                        if (Math.Abs(v_) < 1e-6)
                        {
                            torsionalModel.TorqueOnBit = 0;
                            torsionalModel.WeightOnBit = 0;

                        }
                        else
                        {
                            //Commented unnecessary regularization
                            double Ff_ = (Fc_ + (Fs_ - Fc_) * Math.Exp(-va_ / parameters.Friction.v_c)) * v_ / Math.Sqrt(v_ * v_);// + 0.001 * 0.001);                        
                            torsionalModel.TorqueOnBit = Ff_ * (ro_ * ro_ * omega_) / Math.Sqrt(va_ * va_ + ro_ * ro_ * omega_ * omega_);
                            torsionalModel.WeightOnBit = Ff_ * va_ / Math.Sqrt(va_ * va_ + ro_ * ro_ * omega_ * omega_);
                        }
                    }
                    else
                    {
                        torsionalModel.TorqueOnBit = 0;
                        torsionalModel.WeightOnBit = 0;
                    }
                }
            }   
            torsionalModel.DownwardTorsionalWaveStackedWithLeftBoundary = torsionalModel.DownwardTorsionalWaveLeftBoundary.ToRowMatrix().Stack(torsionalModel.DownwardTorsionalWave);
            torsionalModel.UpwardTorsionalWaveStackedWithLeftBoundary = torsionalModel.UpwardTorsionalWave.Stack(torsionalModel.DownwardAxialWaveRightBoundary.ToRowMatrix());               
            torsionalModel.DownwardAxialWaveStackedWithRightBoundary = torsionalModel.UpwardTorsionalWaveLeftBoundary.ToRowMatrix().Stack(torsionalModel.DownwardAxialWave);
            torsionalModel.UpwardAxialWaveStackedWithRightBoundary = torsionalModel.UpwardAxialWave.Stack(torsionalModel.UpwardAxialWaveRightBoundary.ToRowMatrix());          
        }        
        
        public static void LateralSystem(LateralModel lateralModel, AxialTorsionalModel axialTorsionalModel, Input simulationInput, Configuration configuration, State state, SimulationParameters parameters)
        {
            // Compute torque from the top drive on the first distributed element
            lateralModel.TauTD = (parameters.Drillstring.PipePolarMoment[0] * parameters.Drillstring.ShearModuli[0]) / (2 * parameters.Drillstring.TorsionalWaveSpeed) * (axialTorsionalModel.DownwardTorsionalWave[0, 0] - axialTorsionalModel.UpwardTorsionalWave[0, 0]);
            // Compute torque for every other lumped element apart from top drive - CAN BE MOVED OUTSIDE THE TIMESTEP
            Vector<double> factor = parameters.Drillstring.PipePolarMoment.PointwiseMultiply(parameters.Drillstring.ShearModuli) / (2.0 * parameters.Drillstring.TorsionalWaveSpeed);
            // Extract row pPL from aa and bb, then subtract
            Vector<double> rowDiff = axialTorsionalModel.DownwardTorsionalWave.Row(parameters.LumpedCells.DistributedToLumpedRatio - 1) - axialTorsionalModel.UpwardTorsionalWave.Row(parameters.LumpedCells.DistributedToLumpedRatio - 1);
            // Element-wise multiplication of factor with row difference
            lateralModel.TauM = factor.PointwiseMultiply(rowDiff);
            
            Vector<double> factor2 = parameters.Drillstring.PipeArea.PointwiseMultiply(parameters.Drillstring.YoungModuli) / (2.0 * parameters.Drillstring.AxialWaveSpeed);
            Vector<double> rowDiff2 = axialTorsionalModel.DownwardAxialWave.Row(parameters.LumpedCells.DistributedToLumpedRatio - 1) - axialTorsionalModel.UpwardAxialWave.Row(parameters.LumpedCells.DistributedToLumpedRatio - 1);
            lateralModel.ForceM = factor2.PointwiseMultiply(rowDiff2);
        
            for (int i = 1; i < lateralModel.TorqueDistribution.Count; i++)
            {
                // Populate Torque Distributions
                lateralModel.TorqueDistribution[i - 1] = parameters.Drillstring.PipePolarMoment[i] * 
                    parameters.Drillstring.ShearModuli[i] * 
                    (
                        axialTorsionalModel.DownwardTorsionalWave.Row(0)[i] -
                        axialTorsionalModel.UpwardTorsionalWave.Row(0)[i]
                    ) / (2 * parameters.Drillstring.TorsionalWaveSpeed);
                lateralModel.ForceDistribution[i - 1] = parameters.Drillstring.PipeArea[i] * 
                    parameters.Drillstring.YoungModuli[i] * 
                    (
                        axialTorsionalModel.DownwardAxialWave.Row(0)[i] -
                        axialTorsionalModel.UpwardAxialWave.Row(0)[i]
                    ) / (2 * parameters.Drillstring.AxialWaveSpeed);
            }
            //Populate the last element
            lateralModel.ForceDistribution[lateralModel.ForceDistribution.Count - 1] = axialTorsionalModel.WeightOnBit;
            if (!configuration.UseMudMotor)
            {
                lateralModel.MudTorque = 0;
                lateralModel.TorqueDistribution[lateralModel.TorqueDistribution.Count - 1] = axialTorsionalModel.TorqueOnBit;               
            }
            else
            {
                double speedRatio = (state.MudRotorAngularVelocity - state.MudStatorAngularVelocity) / parameters.MudMotor.omega0_motor;              
                if (speedRatio <= 1)
                    lateralModel.MudTorque = parameters.MudMotor.T_max_motor * Math.Pow(1 - speedRatio, 1.0 / parameters.MudMotor.alpha_motor);
                else
                    lateralModel.MudTorque = -parameters.MudMotor.P0_motor * parameters.MudMotor.V_motor * (speedRatio - 1);
                lateralModel.TorqueDistribution[lateralModel.TorqueDistribution.Count - 1] = lateralModel.MudTorque;               
            }
            //Calculate forces
            for (int i = 0; i < state.XDisplacement.Count; i++)
            {
                #region Polar Coordinates Conversion
                // Get the radial displacement 
                double radialDisplacement = Math.Sqrt(state.XDisplacement[i] * state.XDisplacement[i] + state.YDisplacement[i] * state.YDisplacement[i]);   
                // Calculate the whirl angle 
                double whirlAngle = Math.Atan2(state.YDisplacement[i], state.XDisplacement[i]);
                // Pre-calculate whirl angle sin and cos to avoid multiple calculations
                double cosWhirlAngle = whirlAngle == 0.0 ? 1.0 : state.XDisplacement[i]/radialDisplacement; // Math.Cos(whirlAngle);
                double sinWhirlAngle = whirlAngle == 0.0 ? 0.0 : state.YDisplacement[i]/radialDisplacement; // Math.Sin(whirlAngle);
                // Calculate the radial velocity
                double radialVelocity = state.XVelocity[i] * cosWhirlAngle 
                    + state.YVelocity[i] * sinWhirlAngle;
                // Calculate whirl velocity
                double whirlVelocity = radialDisplacement == 0.0 ? 0.0 : (state.YVelocity[i] * state.XDisplacement[i] - state.XVelocity[i] * state.YDisplacement[i])/(radialDisplacement * radialDisplacement);
                #endregion
                #region Collision calculation
                // Check if there is collision or not and store the Heaveside Step Function
                double HeavesideStep = radialDisplacement >= parameters.Wellbore.DrillStringClearance[i] ? 1.0 : 0.0;                                    
                // Calculate normal force
                double normalCollisionForce =  HeavesideStep * 
                    (
                        parameters.Wellbore.WallStiffness * (radialDisplacement - parameters.Wellbore.DrillStringClearance[i]) 
                       + parameters.Wellbore.WallDamping * radialVelocity
                    );  
                                        
                if (lateralModel.NormalCollisionForce.Count > 1 && i == lateralModel.NormalCollisionForce.Count - 2)
                    normalCollisionForce += simulationInput.ForceToInduceBitWhirl;    
                #endregion
                #region Elastic Force Calculation
                // If it is the first element, get the pinned boundary condition
                double XiMinus1 = (i == 0) ? 0.0 : state.XDisplacement[i - 1];
                double YiMinus1 = (i == 0) ? 0.0 : state.YDisplacement[i - 1];
                // If it is the last element, get the pinned boundary condition
                double XiPlus1 = (i == state.XDisplacement.Count - 1) ? 0.0 : state.XDisplacement[i + 1];
                double YiPlus1 = (i == state.XDisplacement.Count - 1) ? 0.0 : state.YDisplacement[i + 1];
                // Same with the stiffness
                double kMinus1 = (i == 0) ? 1E8 : lateralModel.BendingStiffness[i - 1];
                double kPlus1 = (i == state.XDisplacement.Count - 1) ? 1E8 : lateralModel.BendingStiffness[i + 1];
                double ElasticForceX = kMinus1 * XiMinus1  - (kMinus1 + kPlus1) * state.XDisplacement[i] + kPlus1 * XiPlus1;
                double ElasticForceY = kMinus1 * YiMinus1  - (kMinus1 + kPlus1) * state.YDisplacement[i] + kPlus1 * YiPlus1;
                double PreStressForceX = lateralModel.PreStressNormalForce[i] * Math.Sin(lateralModel.ToolFaceAngle[i]) + 
                                lateralModel.PreStressBinormalForce[i] * Math.Cos(lateralModel.ToolFaceAngle[i]) ;
                double PreStressForceY = lateralModel.PreStressNormalForce[i] * Math.Cos(lateralModel.ToolFaceAngle[i]) - 
                                lateralModel.PreStressBinormalForce[i] * Math.Sin(lateralModel.ToolFaceAngle[i]) ;                                    
                #endregion
                #region Fluid Force Calculation
                //Extracts angular speed depending on if it is a sleeve or not
                bool hasSleeve = state.SleeveToLumpedIndex[i] != -1;
                //If it is not used properly, it should crash the code.
                int sleeveIndex = hasSleeve ?  state.SleeveToLumpedIndex[i] : -1;
                //Select between sleeve and non-sleeve nodes
                double rotationSpeed = hasSleeve ? state.SleeveAngularVelocity[sleeveIndex] : state.AngularVelocity[i];                            
                double rotationSpeedSquared = rotationSpeed * rotationSpeed;
                double fluidDampingCoefficient = parameters.Wellbore.FluidDampingCoefficient / parameters.Drillstring.FluidAddedMass[i];                    
                double fluidForceX = - parameters.Drillstring.FluidAddedMass[i] * 
                                    (
                                        fluidDampingCoefficient * state.XVelocity[i]
                                        - 0.25 * rotationSpeedSquared * state.XDisplacement[i]
                                        + rotationSpeed * state.YVelocity[i]
                                        + 0.5 * fluidDampingCoefficient * rotationSpeed * state.YDisplacement[i]
                                    );
                double fluidForceY = - parameters.Drillstring.FluidAddedMass[i] * 
                                    (
                                        fluidDampingCoefficient * state.YVelocity[i]
                                        - 0.25 * rotationSpeedSquared * state.YDisplacement[i]
                                        - rotationSpeed * state.XVelocity[i]
                                        - 0.5 * fluidDampingCoefficient * rotationSpeed * state.XDisplacement[i]
                                    );
                #endregion
                // Sleeve braking force
                double sleeveBrakeForce = hasSleeve ? parameters.Drillstring.SleeveTorsionalDamping[sleeveIndex] * (whirlVelocity - state.SleeveAngularVelocity[sleeveIndex]) : 0;
                #region Unbalance Forces
                // lateral forces due to mass imbalance
                // The imbalance force comes from the assumption that the pipe element center of mass i slocated at a distance from its geometric center, 
                // which causes a lateral force and a torque as the pipe is displaced                
                double rotationAngle = hasSleeve ? state.SleeveAngularDisplacement[sleeveIndex] : state.AngularDisplacement[i];                                        
                double rotationAcceleration = hasSleeve ? state.SleeveAngularAcceleration[sleeveIndex] : state.AngularAcceleration[i];                                        
                double unbalanceForceX = parameters.Drillstring.EccentricMass[i] * parameters.Drillstring.Eccentricity[i]*
                    (
                        rotationSpeedSquared * Math.Cos(rotationAngle)
                        + state.AngularAcceleration[i] * Math.Sin(rotationAngle)
                    );
                double unbalanceForceY = parameters.Drillstring.EccentricMass[i] * parameters.Drillstring.Eccentricity[i]* 
                    (
                        rotationSpeedSquared * Math.Sin(rotationAngle)
                        - rotationAcceleration * Math.Cos(rotationAngle)
                    );                                                            
                #endregion                                        
                #region Coulomb Friction
                // Axial velocity
                double axialVelocity = state.AxialVelocity[i];
                // "Masks" sleeve or non-sleeve variables if hasSleeve = true
                double outerRadius = hasSleeve ? parameters.Drillstring.SleeveOuterRadius : parameters.Drillstring.OuterRadius[i];
                double innerRadius = hasSleeve ? parameters.Drillstring.SleeveInnerRadius : parameters.Drillstring.OuterRadius[i];  
                double inertia = hasSleeve ? parameters.Drillstring.SleeveMassMomentOfInertia : parameters.Drillstring.LumpedElementMomentOfInertia[i];
                double tangentialVelocity = whirlVelocity * radialDisplacement + rotationSpeed * outerRadius;
                double axialStaticForce = 1.0 / parameters.InnerLoopTimeStep * state.AxialVelocity[i] * parameters.Drillstring.LumpedElementMass[i] 
                        + lateralModel.ForceM[i] - lateralModel.ForceDistribution[i] - parameters.Drillstring.CalculatedAxialDamping * state.AxialVelocity[i];                    
                //UNUSED                    
                double sumForcesX = ElasticForceX + PreStressForceX + fluidForceX + unbalanceForceX;
                double sumForcesY = ElasticForceY + PreStressForceY + fluidForceY + unbalanceForceY;
                double[] totalForce = new double[3] { sumForcesX, sumForcesY, axialStaticForce }; 
                double[] normalVector = new double[3] { cosWhirlAngle, sinWhirlAngle, 0 };                                        
                // ----------------------------- Needs to be corrected! ---------------------------- 
                // The tangential velocity must be the difference of the total velocity from the normal velocity.
                // The total velocity is calculated by [dx, dy, dz] + omega x R.
                // The normal velocity is the projection of the total on the direction of the collision [cos(whirlAngle); sin(whirlAngle); 0]                                                                                                    
                double term1 = (
                                -2 * (parameters.Drillstring.LumpedElementMass[i] 
                                + parameters.Drillstring.FluidAddedMass[i]) 
                                - inertia / (outerRadius * outerRadius) 
                                + parameters.Drillstring.EccentricMass[i] 
                                * parameters.Drillstring.Eccentricity[i] 
                                / outerRadius * Math.Cos(state.AngularDisplacement[i] - whirlAngle)
                            ) 
                            * radialVelocity * whirlVelocity;                   
                double term2 = outerRadius * (ElasticForceX + PreStressForceX + fluidForceX) * sinWhirlAngle;
                double term3 = - outerRadius * (ElasticForceY + PreStressForceY + fluidForceY) * cosWhirlAngle;                                        
                double term4 = - (
                                      lateralModel.TauM[i] 
                                    - lateralModel.TorqueDistribution[i]
                                    - parameters.Drillstring.CalculatedTorsionalDamping * rotationSpeed
                                ) / outerRadius;
                double term5 = parameters.Drillstring.EccentricMass[i] 
                                * parameters.Drillstring.Eccentricity[i] 
                                * rotationSpeed 
                                * rotationSpeed 
                                * Math.Sin(rotationAngle - whirlAngle);
                double term6 = hasSleeve ? sleeveBrakeForce * innerRadius : 0.0;
                double denominator = parameters.Drillstring.LumpedElementMass[i] 
                                    + parameters.Drillstring.FluidAddedMass[i] 
                                    + inertia / (outerRadius * outerRadius) 
                                    - (
                                        parameters.Drillstring.EccentricMass[i] 
                                        * parameters.Drillstring.Eccentricity[i] 
                                        / parameters.Drillstring.OuterRadius[i] 
                                        * Math.Cos(rotationAngle - whirlAngle)
                                    );
                double thetaDotNoSlip = (term1 + term2 + term3 + term4 + term5 + term6) / denominator;
                double phiDdotNoSlip = 0.0;      
                if (!hasSleeve)
                {                        
                    double denominator_ = parameters.Drillstring.LumpedElementMass[i] 
                                    + parameters.Drillstring.FluidAddedMass[i] 
                                    + inertia / (outerRadius * outerRadius) 
                                    - (parameters.Drillstring.EccentricMass[i] * parameters.Drillstring.Eccentricity[i]
                                     / outerRadius * Math.Cos(rotationAngle - whirlAngle));
                    double term1_ = (-2 * (parameters.Drillstring.LumpedElementMass[i] + parameters.Drillstring.FluidAddedMass[i]) 
                                        - inertia / (outerRadius * outerRadius) 
                                        + parameters.Drillstring.EccentricMass[i] * parameters.Drillstring.Eccentricity[i] 
                                        / outerRadius 
                                        * Math.Cos(rotationAngle - whirlAngle)) 
                                        * radialVelocity 
                                        * whirlVelocity;
                    double term2_ = - (ElasticForceX + PreStressForceX + fluidForceX) * sinWhirlAngle;
                    double term3_ =   (ElasticForceY + PreStressForceY + fluidForceY) * cosWhirlAngle;
                    double term4_ = - (
                                        lateralModel.TauM[i] 
                                        - lateralModel.TorqueDistribution[i]
                                        - parameters.Drillstring.CalculatedTorsionalDamping * rotationSpeed)
                                        / outerRadius;
                    double term5_ = parameters.Drillstring.EccentricMass[i] 
                                    * parameters.Drillstring.Eccentricity[i]
                                    * state.AngularVelocity[i] 
                                    * state.AngularVelocity[i] * Math.Sin(rotationAngle - whirlAngle);              
                    phiDdotNoSlip = (term1_ + term2_ + term3_ + term4_ + term5_)/denominator_;
                }
                else
                {                        
                    double denominator_ = parameters.Drillstring.LumpedElementMass[i]
                                         + parameters.Drillstring.FluidAddedMass[i] 
                                         + inertia / (outerRadius*outerRadius) 
                                         - parameters.Drillstring.EccentricMass[i] 
                                         * parameters.Drillstring.Eccentricity[i] / outerRadius 
                                         * Math.Cos(rotationAngle - whirlAngle);
                    double term1_ = (
                                    - 2 * (parameters.Drillstring.LumpedElementMass[i] + parameters.Drillstring.FluidAddedMass[i]) 
                                    - inertia / (outerRadius * outerRadius) 
                                    + parameters.Drillstring.EccentricMass[i] * parameters.Drillstring.Eccentricity[i] 
                                    / outerRadius * Math.Cos(rotationAngle - whirlAngle))
                                    * radialVelocity 
                                    * whirlVelocity;
                    double term2_ = - (ElasticForceX + PreStressForceX + fluidForceX) * sinWhirlAngle;
                    double term3_ =   (ElasticForceY + PreStressForceY + fluidForceY) * cosWhirlAngle;
                    double term4_ = - innerRadius / outerRadius * sleeveBrakeForce;
                    double term5_ = parameters.Drillstring.EccentricMass[i] 
                                    * parameters.Drillstring.Eccentricity[i]
                                    * rotationSpeedSquared * Math.Sin(rotationAngle - whirlAngle);
                    phiDdotNoSlip = (term1 + term2 + term3 + term4 + term5) / denominator;                                                            
                }
                //Tangetial force
                double tangentialStaticForce = inertia/outerRadius * radialVelocity * whirlVelocity
                    + radialDisplacement * phiDdotNoSlip 
                    + (
                        lateralModel.TauM[i] - lateralModel.TorqueDistribution[i] 
                        - parameters.Drillstring.CalculatedTorsionalDamping * whirlVelocity
                    )/outerRadius;                  
                double velocityMagnitude = Math.Sqrt(axialVelocity * axialVelocity  + tangentialVelocity * tangentialVelocity) + Constants.eps;
                double staticFrictionForceCalculated = Math.Sqrt(axialStaticForce * axialStaticForce + tangentialStaticForce * tangentialStaticForce) + Constants.eps;
                double staticFrictionForceCriteria = normalCollisionForce * parameters.Friction.mu_s[i];
                //Check for slip
                if (state.slip_condition[i] == 0)
                    // If the previous state was a not a slip condition, re-evaluate by comparing forces.  
                    state.slip_condition[i] = staticFrictionForceCalculated > staticFrictionForceCriteria ? 1:0;
                else
                    // If the previous state was a slip condition  
                    state.slip_condition[i] = velocityMagnitude < parameters.Friction.v_c ? 0 : state.slip_condition[i];
                //Masking system for no slip
                double invertedSlipCondition = 1.0 - state.slip_condition[i];
                double coulombForce;
                double axialCoulombFrictionForce;
                double tangentialCoulombFrictionForce;
                if (state.slip_condition[i] == 0)
                {
                    coulombForce = Math.Max(Math.Min(staticFrictionForceCalculated, staticFrictionForceCriteria), - staticFrictionForceCriteria);
                    axialCoulombFrictionForce = coulombForce * axialStaticForce / staticFrictionForceCalculated;
                    tangentialCoulombFrictionForce = coulombForce * tangentialStaticForce / staticFrictionForceCalculated;
                }
                else
                {
                    coulombForce = normalCollisionForce * (parameters.Friction.mu_k[i] + (parameters.Friction.mu_s[i] - parameters.Friction.mu_k[i]) * Math.Exp( - parameters.Friction.stribeck *  velocityMagnitude));
                    axialCoulombFrictionForce = coulombForce * axialVelocity / velocityMagnitude;
                    tangentialCoulombFrictionForce = coulombForce * tangentialVelocity / velocityMagnitude;                        
                }
                if (hasSleeve)
                {
                    axialCoulombFrictionForce = axialCoulombFrictionForce *  (1 - parameters.Drillstring.AxialFrictionReduction);
                    tangentialCoulombFrictionForce = Math.Sqrt(coulombForce * coulombForce - axialCoulombFrictionForce * axialCoulombFrictionForce) * Math.Sign (tangentialCoulombFrictionForce);
                }                
                if (i == parameters.Drillstring.IndexSensor)
                {
                    lateralModel.PhiDdotNoSlipSensor = phiDdotNoSlip;
                    lateralModel.ThetaDotNoSlipSensor = thetaDotNoSlip;
                }

                #endregion                   
                #region Accelerations
                //If there is a sleeve, no torque is actually used
                double frictionTorque = hasSleeve ? sleeveBrakeForce * outerRadius : tangentialCoulombFrictionForce * outerRadius;
                state.AngularAcceleration[i] = (
                                lateralModel.TauM[i]
                                 - lateralModel.TorqueDistribution[i] 
                                 - parameters.Drillstring.CalculatedTorsionalDamping * whirlVelocity
                                 - frictionTorque
                            )/inertia;
                // It needs to be zeroed before, as there might have changes in the sleeve position when the number of lumped parameters change 
                state.SleeveForces[i] = 0;
                if (hasSleeve)
                {
                    state.SleeveForces[i] = hasSleeve ? tangentialCoulombFrictionForce : 0;
                    // Why is there a TimeStep in here?                    
                    state.SleeveAngularAcceleration[sleeveIndex] = parameters.InnerLoopTimeStep * (sleeveBrakeForce * parameters.Drillstring.SleeveInnerRadius - parameters.Drillstring.SleeveOuterRadius * state.SleeveForces[i]) / parameters.Drillstring.SleeveMassMomentOfInertia;;
                }
                //Used for debugging purposes
                lateralModel.NormalCollisionForce[i] = normalCollisionForce;
                state.RadialDisplacement[i] = radialDisplacement;
                state.RadialVelocity[i] = radialVelocity;
                state.WhirlAngle[i] = whirlAngle;
                state.WhirlVelocity[i] = whirlVelocity;
                double ZAcceleration = (lateralModel.ForceM[i] - lateralModel.ForceDistribution[i] - parameters.Drillstring.CalculatedAxialDamping * state.AxialVelocity[i] - axialCoulombFrictionForce)/parameters.Drillstring.LumpedElementMass[i];
                double XAcceleration = (ElasticForceX + PreStressForceX + fluidForceX + unbalanceForceX - parameters.Drillstring.CalculateLateralDamping * state.XVelocity[i] - lateralModel.NormalCollisionForce[i] * cosWhirlAngle
                    + tangentialCoulombFrictionForce * sinWhirlAngle) / (parameters.Drillstring.LumpedElementMass[i] + parameters.Drillstring.FluidAddedMass[i]);
                double YAcceleration= (ElasticForceY + PreStressForceY + fluidForceY + unbalanceForceY - parameters.Drillstring.CalculateLateralDamping * state.YVelocity[i] - lateralModel.NormalCollisionForce[i] * sinWhirlAngle
                    - tangentialCoulombFrictionForce * cosWhirlAngle) / (parameters.Drillstring.LumpedElementMass[i] + parameters.Drillstring.FluidAddedMass[i]);
                state.AxialAcceleration[i] = ZAcceleration;
                state.XAcceleration[i] = XAcceleration;
                state.YAcceleration[i] = YAcceleration;                    
                #endregion
                #region  Debbugging Outputs
                    if (double.IsNaN(XAcceleration) || double.IsNaN(YAcceleration) || double.IsNaN(ZAcceleration) || double.IsNaN(state.AngularAcceleration[i]))
                    {
                        Console.WriteLine("NaN detected in lateral calculations at element " + i.ToString() +
                            " XAcc: " + XAcceleration.ToString() +
                            " YAcc: " + YAcceleration.ToString() +
                            " ZAcc: " + ZAcceleration.ToString() +
                            " AngularAcc: " + state.AngularAcceleration[i].ToString() +
                            " NormalForce: " + normalCollisionForce.ToString() +
                            " RadialDisp: " + radialDisplacement.ToString() +
                            " RadialVel: " + radialVelocity.ToString() +
                            " WhirlVel: " + whirlVelocity.ToString() +
                            " TangentialVel: " + tangentialVelocity.ToString() +
                            " AxialVel: " + axialVelocity.ToString() +
                            " SlipCondition: " + state.slip_condition[i].ToString()
                            );
                    }
                
                #endregion

            }             
            state.TopDriveAngularVelocity = state.TopDriveAngularVelocity + 1.0 / parameters.Wellbore.TopDriveInertia * parameters.InnerLoopTimeStep * (simulationInput.TopDriveTorque - lateralModel.TauTD);                
                                     
        }
    }
}