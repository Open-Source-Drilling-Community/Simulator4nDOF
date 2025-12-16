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
        {   Vector<double> elementWiseProduct = parameters.Drillstring.YoungModuli.PointwiseMultiply(parameters.Drillstring.PipeArea);
            model.ScalingMatrix = Vector<double>.Build.Dense(parameters.LumpedCells.PL, 1).ToColumnMatrix();
            Matrix<double> dragMatrix = model.ScalingMatrix * elementWiseProduct.ToRowMatrix();
            dragMatrix = state.PipeAxialStrain.PointwiseMultiply(dragMatrix);
            Vector<double> drag_flattened = ToVector(dragMatrix.ToColumnMajorArray());
            Vector<double> drag = LinearInterpolate(parameters.DistributedCells.x, drag_flattened, parameters.LumpedCells.xL);
            Vector<double> phiVec_dote = ExtendVectorStart(0, parameters.Trajectory.phiVec_dot);
            Vector<double> thetaVec_dote = ExtendVectorStart(0, parameters.Trajectory.thetaVec_dot);
            Vector<double> thetaVece = ExtendVectorStart(0, parameters.Trajectory.thetaVec);
            Vector<double> trapezoidalsIntegration = CummulativeTrapezoidal(parameters.LumpedCells.xL, Reverse(parameters.Buoyancy.dsigma_dx));
            model.Tension = Reverse(trapezoidalsIntegration) + parameters.Buoyancy.axialBuoyancyForceChangeOfDiameters - drag;
            Vector<double> fN_softstring = (Square((model.Tension + parameters.Buoyancy.normalBuoyancyForceChangeOfDiameters).PointwiseMultiply(thetaVec_dote) - parameters.Buoyancy.Wb.PointwiseMultiply(thetaVece.PointwiseSin())) +
                                            Square((model.Tension + parameters.Buoyancy.normalBuoyancyForceChangeOfDiameters).PointwiseMultiply(phiVec_dote).PointwiseMultiply(thetaVece.PointwiseSin()))).PointwiseSqrt();
            Vector<double> I_fN_softstring = Utilities.CummulativeTrapezoidal(parameters.LumpedCells.xL, fN_softstring);
            model.SoftStringNormalForce = Diff(I_fN_softstring); // [N] Lumped normal force per element assuming soft - string model(not used in 4nDOF model)

            Vector<double> AiExtended = ExtendVectorStart(parameters.Drillstring.InnerArea[0], parameters.Drillstring.InnerArea);
            Vector<double> AoExtended = ExtendVectorStart(parameters.Drillstring.OuterArea[0], parameters.Drillstring.OuterArea);
            
            /*Vector<double> F_comp = AiExtended.PointwiseMultiply(parameters.Buoyancy.stringPressure - parameters.Buoyancy.hydrostaticStringPressure) * (1 - 2 * parameters.Drillstring.PoissonRatio)
                                    - AoExtended.PointwiseMultiply(parameters.Buoyancy.annularPressure - parameters.Buoyancy.hydrostaticAnnularPressure) * (1 - 2 * parameters.Drillstring.PoissonRatio)
                                  - Tension;
            Tension = -F_comp;*/
            
            model.Tension +=  (1 - 2 * parameters.Drillstring.PoissonRatio) * 
                (  
                    AoExtended.PointwiseMultiply(parameters.Buoyancy.annularPressure - parameters.Buoyancy.hydrostaticAnnularPressure) 
                    - AiExtended.PointwiseMultiply(parameters.Buoyancy.stringPressure - parameters.Buoyancy.hydrostaticStringPressure)
                );   

            Vector<double> kbExtended = ExtendVectorStart(parameters.Drillstring.BendingStiffness[0], parameters.Drillstring.BendingStiffness); 
            model.BendingStiffness = - (kbExtended - Math.Pow(Math.PI, 2) * model.Tension / (2 * parameters.Drillstring.PipeLengthForBending)).PointwiseMaximum(0.0);                                            
            model.PolarMomentTimesShearModuli = parameters.Drillstring.PipePolarMoment.PointwiseMultiply(parameters.Drillstring.ShearModuli); // Element-wise multiplication
            Matrix<double> TorqueMatrix = state.PipeShearStrain.PointwiseMultiply(model.ScalingMatrix * model.PolarMomentTimesShearModuli.ToRowMatrix()); // 5x136 matrix
            Vector<double> torqueFlattened = ToVector(TorqueMatrix.ToColumnMajorArray());
            Vector<double> torque = LinearInterpolate(parameters.DistributedCells.x, torqueFlattened, parameters.LumpedCells.xL);
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
            Vector<double> diffTorqueExtended = ExtendVectorStart(0, Diff(torque) / parameters.LumpedCells.dxL);

            Vector<double> fB =
                parameters.Buoyancy.Wb.PointwiseMultiply(binormal) +
                curvatureExtended.PointwiseMultiply(diffTorqueExtended) +
                curvature_dotExtended.PointwiseMultiply(torque) -
                2 * youngModulus.PointwiseMultiply(momentOfInertia).PointwiseMultiply(curvature_dotExtended).PointwiseMultiply(torsionExtended) -
                youngModulus.PointwiseMultiply(momentOfInertia).PointwiseMultiply(curvatureExtended).PointwiseMultiply(torsion_dotExtended);

            // Compute pre-stressed forces
            Vector<double> I_fB = CummulativeTrapezoidal(parameters.LumpedCells.xL, fB);

            Vector<double> fN =
                curvatureExtended.PointwiseMultiply(model.Tension + parameters.Buoyancy.normalBuoyancyForceChangeOfDiameters) +
                parameters.Buoyancy.Wb.PointwiseMultiply(normal) -
                youngModulus.PointwiseMultiply(momentOfInertia).PointwiseMultiply(curvature_ddotExtended) +
                youngModulus.PointwiseMultiply(momentOfInertia).PointwiseMultiply(curvatureExtended).PointwiseMultiply(Square(torsionExtended)) -
                curvatureExtended.PointwiseMultiply(torsionExtended).PointwiseMultiply(torque);

            Vector<double> I_fN = CummulativeTrapezoidal(parameters.LumpedCells.xL, fN);
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
        public static void AxialTorsionalSystem(AxialTorsionalModel model, Input simulationInput, Configuration configuration, State state, SimulationParameters parameters)
        {
            
            //Create velocity vectors            
            model.OL_vec = ExtendVectorStart(state.TopDriveAngularVelocity, state.LumpedElementAngularVelocity);
            model.VL_vec = ExtendVectorStart(simulationInput.CalculateSurfaceAxialVelocity, state.LumpedElementAxialVelocity);
            // Left boundaries
            model.DownwardTorsionalWaveLeftBoundary = -model.UpwardTorsionalWave.Row(0) + 2 * model.OL_vec.SubVector(0, model.OL_vec.Count - 1);
            model.UpwardTorsionalWaveLeftBoundary = -model.UpwardAxialWave.Row(0) + 2 * model.VL_vec.SubVector(0, model.VL_vec.Count - 1);
            // Right boundaries
            model.DownwardAxialWaveRightBoundary = -model.DownwardTorsionalWave.Row(parameters.LumpedCells.PL - 1) + 2 * model.OL_vec.SubVector(1, model.OL_vec.Count - 1);
            model.UpwardAxialWaveRightBoundary = -model.DownwardAxialWave.Row(parameters.LumpedCells.PL - 1) + 2 * model.VL_vec.SubVector(1, model.VL_vec.Count - 1);

            state.BitVelocity = 0.5 * (model.DownwardAxialWave[parameters.LumpedCells.PL - 1, model.DownwardAxialWave.ColumnCount - 1] + model.UpwardAxialWave[parameters.LumpedCells.PL - 1, model.UpwardAxialWave.ColumnCount - 1]);
            
            
            double angularVelocityBottom;
            if (!configuration.UseMudMotor)
                angularVelocityBottom = 0.5 * (model.DownwardTorsionalWave[parameters.LumpedCells.PL - 1, model.DownwardTorsionalWave.ColumnCount - 1] + model.UpwardTorsionalWave[parameters.LumpedCells.PL - 1, model.UpwardTorsionalWave.ColumnCount - 1]);
            else
                angularVelocityBottom = state.MudRotorAngularVelocity;

            double[] bitForces = parameters.BitRock.
                CalculateInteractionForce(state, angularVelocityBottom, model.DownwardTorsionalWave, parameters);
            model.TorqueOnBit = bitForces[0];
            model.WeightOnBit = bitForces[1];    
            // manage the bit sticking off bottom condition
            if (!state.onBottom)
            {
                double omega_ = state.LumpedElementAngularVelocity[state.LumpedElementAngularVelocity.Count - 1];
                if (simulationInput.StickingBoolean)
                {
                    int lastIndex = parameters.Drillstring.ShearModuli.Count - 1;
                    Vector<double> TorsionalAcceleration = model.UpwardTorsionalWave.Row(parameters.LumpedCells.PL - 1);
                    Vector<double> AxialAcceleration = model.UpwardAxialWave.Row(parameters.LumpedCells.PL - 1);
                    model.TorqueOnBit = parameters.Drillstring.PipePolarMoment[lastIndex] * parameters.Drillstring.ShearModuli[lastIndex] / parameters.Drillstring.TorsionalWaveSpeed * TorsionalAcceleration[TorsionalAcceleration.Count - 1];
                    model.WeightOnBit = parameters.Drillstring.PipeArea[lastIndex] * parameters.Drillstring.YoungModuli[lastIndex] / parameters.Drillstring.AxialWaveSpeed * AxialAcceleration[TorsionalAcceleration.Count - 1];
                }
                else
                {
                    double normalForce_ = simulationInput.BottomExtraNormalForce;
                    if (normalForce_ > 0)
                    {
                        double ro_ = parameters.Drillstring.OuterRadius[parameters.Drillstring.OuterRadius.Count - 1];
                        double Fs_ = normalForce_ * 1.0; //Static force
                        double Fc_ = normalForce_ * 0.5; //Kinematic force
                        double va_ = state.LumpedElementAxialVelocity[state.LumpedElementAxialVelocity.Count - 1]; //Axial velocity
                        double v_ = Math.Sqrt(va_ * va_ + omega_ * omega_ * ro_ * ro_); //Tangential velocity
                        if (Math.Abs(v_) < 1e-6)
                        {
                            model.TorqueOnBit = 0;
                            model.WeightOnBit = 0;

                        }
                        else
                        {
                            //Commented unnecessary regularization
                            double Ff_ = (Fc_ + (Fs_ - Fc_) * Math.Exp(-va_ / parameters.Friction.v_c)) * v_ / Math.Sqrt(v_ * v_);// + 0.001 * 0.001);                        
                            model.TorqueOnBit = Ff_ * (ro_ * ro_ * omega_) / Math.Sqrt(va_ * va_ + ro_ * ro_ * omega_ * omega_);
                            model.WeightOnBit = Ff_ * va_ / Math.Sqrt(va_ * va_ + ro_ * ro_ * omega_ * omega_);
                        }
                    }
                    else
                    {
                        model.TorqueOnBit = 0;
                        model.WeightOnBit = 0;
                    }
                }
            }   
            model.DownwardTorsionalWaveStackedWithLeftBoundary = model.DownwardTorsionalWaveLeftBoundary.ToRowMatrix().Stack(model.DownwardTorsionalWave);
            model.UpwardTorsionalWaveStackedWithLeftBoundary = model.UpwardTorsionalWave.Stack(model.DownwardAxialWaveRightBoundary.ToRowMatrix());               
            model.DownwardAxialWaveStackedWithRightBoundary = model.UpwardTorsionalWaveLeftBoundary.ToRowMatrix().Stack(model.DownwardAxialWave);
            model.UpwardAxialWaveStackedWithRightBoundary = model.UpwardAxialWave.Stack(model.UpwardAxialWaveRightBoundary.ToRowMatrix());          
        }        

    }
}