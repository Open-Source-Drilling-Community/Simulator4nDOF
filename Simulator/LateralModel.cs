using MathNet.Numerics.Distributions;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Factorization;
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel;
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel;
using OSDC.DotnetLibraries.General.Common;
using System;
using System.Reflection;
using System.Reflection.Metadata;
using static NORCE.Drilling.Simulator4nDOF.Simulator.Utilities;

namespace NORCE.Drilling.Simulator4nDOF.Simulator
{
    public class LateralModel
    {
       
        public Vector<double> NormalCollisionForce;        
        public Vector<double> SoftStringNormalForce;
        public Vector<double> Tension;
        public Vector<double> BendingStiffness;
        public Vector<double> PolarMomentTimesShearModuli;
        public Vector<double> PreStressNormalForce;
        public Vector<double> PreStressBinormalForce;
                        
        public Matrix<double> ScalingMatrix;
        public Vector<double> ToolFaceAngle;

        public double MudTorque;        
    
        public Vector<double> BendingMomentX;
        public Vector<double> BendingMomentY;

        public double PhiDdotNoSlipSensor;
        public double ThetaDotNoSlipSensor;
        
        public LateralModel(SimulationParameters simulationParameters, State state)
        {

            NormalCollisionForce = Vector<double>.Build.Dense(state.XDisplacement.Count);;        
            SoftStringNormalForce = Vector<double>.Build.Dense(state.XDisplacement.Count);
            Tension = Vector<double>.Build.Dense(state.XDisplacement.Count);
            BendingStiffness = Vector<double>.Build.Dense(state.XDisplacement.Count);
            PolarMomentTimesShearModuli = Vector<double>.Build.Dense(state.XDisplacement.Count);
            PreStressNormalForce = Vector<double>.Build.Dense(state.XDisplacement.Count);
            PreStressBinormalForce = Vector<double>.Build.Dense(state.XDisplacement.Count);
            ScalingMatrix = Vector<double>.Build.Dense(simulationParameters.LumpedCells.DistributedToLumpedRatio, 1).ToColumnMatrix();
            ToolFaceAngle = Vector<double>.Build.Dense(state.XDisplacement.Count);
            MudTorque = 0;        
            BendingMomentX = Vector<double>.Build.Dense(state.XDisplacement.Count);
            BendingMomentY = Vector<double>.Build.Dense(state.YDisplacement.Count);
            PhiDdotNoSlipSensor = 0;
            ThetaDotNoSlipSensor = 0;
        
            /*
            Vector<double> elementWiseProduct = simulationParameters.Drillstring.YoungModuli.PointwiseMultiply(simulationParameters.Drillstring.PipeArea);
            ScalingMatrix = Vector<double>.Build.Dense(simulationParameters.LumpedCells.DistributedToLumpedRatio, 1).ToColumnMatrix();
            Matrix<double> dragMatrix = ScalingMatrix * elementWiseProduct.ToRowMatrix();
            dragMatrix = state.PipeAxialStrain.PointwiseMultiply(dragMatrix);
            Vector<double> drag_flattened = ToVector(dragMatrix.ToColumnMajorArray());
            Vector<double> drag = LinearInterpolate(simulationParameters.DistributedCells.x, drag_flattened, simulationParameters.LumpedCells.ElementLength);
            Vector<double> phiVec_dote = ExtendVectorStart(0, simulationParameters.Trajectory.phiVec_dot);
            Vector<double> thetaVec_dote = ExtendVectorStart(0, simulationParameters.Trajectory.thetaVec_dot);
            Vector<double> thetaVece = ExtendVectorStart(0, simulationParameters.Trajectory.thetaVec);
            Vector<double> trapezoidalsIntegration = CummulativeTrapezoidal(simulationParameters.LumpedCells.ElementLength, Reverse(simulationParameters.Buoyancy.dsigma_dx));
            Tension = Reverse(trapezoidalsIntegration) + simulationParameters.Buoyancy.axialBuoyancyForceChangeOfDiameters - drag;
            Vector<double> fN_softstring = (Square((Tension + simulationParameters.Buoyancy.normalBuoyancyForceChangeOfDiameters).PointwiseMultiply(thetaVec_dote) - simulationParameters.Buoyancy.Wb.PointwiseMultiply(thetaVece.PointwiseSin())) +
                                            Square((Tension + simulationParameters.Buoyancy.normalBuoyancyForceChangeOfDiameters).PointwiseMultiply(phiVec_dote).PointwiseMultiply(thetaVece.PointwiseSin()))).PointwiseSqrt();
            Vector<double> I_fN_softstring = Utilities.CummulativeTrapezoidal(simulationParameters.LumpedCells.ElementLength, fN_softstring);
            SoftStringNormalForce = Diff(I_fN_softstring); // [N] Lumped normal force per element assuming soft - string model(not used in 4nDOF model)

            Vector<double> AiExtended = ExtendVectorStart(simulationParameters.Drillstring.InnerArea[0], simulationParameters.Drillstring.InnerArea);
            Vector<double> AoExtended = ExtendVectorStart(simulationParameters.Drillstring.OuterArea[0], simulationParameters.Drillstring.OuterArea);
               

            BendingMomentX = Vector<double>.Build.Dense(state.XDisplacement.Count);
            BendingMomentY = Vector<double>.Build.Dense(state.YDisplacement.Count);
    
            
            Tension +=  (1 - 2 * simulationParameters.Drillstring.PoissonRatio) * 
                (  
                    AoExtended.PointwiseMultiply(simulationParameters.Buoyancy.annularPressure - simulationParameters.Buoyancy.hydrostaticAnnularPressure) 
                    - AiExtended.PointwiseMultiply(simulationParameters.Buoyancy.stringPressure - simulationParameters.Buoyancy.hydrostaticStringPressure)
                );   

            Vector<double> kbExtended = ExtendVectorStart(simulationParameters.Drillstring.BendingStiffness[0], simulationParameters.Drillstring.BendingStiffness); 
            BendingStiffness = - (kbExtended - Math.Pow(Math.PI, 2) * Tension / (2 * simulationParameters.Drillstring.PipeLengthForBending)).PointwiseMaximum(0.0);                                            
            PolarMomentTimesShearModuli = simulationParameters.Drillstring.PipePolarMoment.PointwiseMultiply(simulationParameters.Drillstring.ShearModuli); // Element-wise multiplication
            Matrix<double> TorqueMatrix = state.PipeShearStrain.PointwiseMultiply(ScalingMatrix * PolarMomentTimesShearModuli.ToRowMatrix()); // 5x136 matrix
            Vector<double> torqueFlattened = ToVector(TorqueMatrix.ToColumnMajorArray());
            Vector<double> torque = LinearInterpolate(simulationParameters.DistributedCells.x, torqueFlattened, simulationParameters.LumpedCells.ElementLength);
            // Normal force components in Frenet-Serret coordinate system
            Vector<double> binormal = ExtendVectorStart(simulationParameters.Trajectory.bz[0], simulationParameters.Trajectory.bz);
            Vector<double> normal = ExtendVectorStart(simulationParameters.Trajectory.nz[0], simulationParameters.Trajectory.nz);
            Vector<double> curvatureExtended = ExtendVectorStart(simulationParameters.Trajectory.Curvature[0], simulationParameters.Trajectory.Curvature);
            Vector<double> curvature_dotExtended = ExtendVectorStart(simulationParameters.Trajectory.CurvatureDerivative[0], simulationParameters.Trajectory.CurvatureDerivative);
            Vector<double> curvature_ddotExtended = ExtendVectorStart(simulationParameters.Trajectory.CurvatureSecondDerivative[0], simulationParameters.Trajectory.CurvatureSecondDerivative);
            Vector<double> youngModulus = ExtendVectorStart(simulationParameters.Drillstring.YoungModuli[0], simulationParameters.Drillstring.YoungModuli);
            Vector<double> momentOfInertia = ExtendVectorStart(simulationParameters.Drillstring.PipeInertia[0], simulationParameters.Drillstring.PipeInertia);
            Vector<double> torsionExtended = ExtendVectorStart(simulationParameters.Trajectory.Torsion[0], simulationParameters.Trajectory.Torsion);
            Vector<double> torsion_dotExtended = ExtendVectorStart(simulationParameters.Trajectory.TorsionDerivative[0], simulationParameters.Trajectory.TorsionDerivative);
            Vector<double> diffTorqueExtended = ExtendVectorStart(0, Diff(torque) / simulationParameters.LumpedCells.DistanceBetweenElements);

            Vector<double> fB =
                simulationParameters.Buoyancy.Wb.PointwiseMultiply(binormal) +
                curvatureExtended.PointwiseMultiply(diffTorqueExtended) +
                curvature_dotExtended.PointwiseMultiply(torque) -
                2 * youngModulus.PointwiseMultiply(momentOfInertia).PointwiseMultiply(curvature_dotExtended).PointwiseMultiply(torsionExtended) -
                youngModulus.PointwiseMultiply(momentOfInertia).PointwiseMultiply(curvatureExtended).PointwiseMultiply(torsion_dotExtended);

            // Compute pre-stressed forces
            Vector<double> I_fB = CummulativeTrapezoidal(simulationParameters.LumpedCells.ElementLength, fB);

            Vector<double> fN =
                curvatureExtended.PointwiseMultiply(Tension + simulationParameters.Buoyancy.normalBuoyancyForceChangeOfDiameters) +
                simulationParameters.Buoyancy.Wb.PointwiseMultiply(normal) -
                youngModulus.PointwiseMultiply(momentOfInertia).PointwiseMultiply(curvature_ddotExtended) +
                youngModulus.PointwiseMultiply(momentOfInertia).PointwiseMultiply(curvatureExtended).PointwiseMultiply(Square(torsionExtended)) -
                curvatureExtended.PointwiseMultiply(torsionExtended).PointwiseMultiply(torque);

            Vector<double> I_fN = CummulativeTrapezoidal(simulationParameters.LumpedCells.ElementLength, fN);
            PreStressNormalForce = Diff(I_fB);
            PreStressBinormalForce = Diff(I_fN);

            Vector<double> expression =
                (simulationParameters.Trajectory.hy.PointwiseMultiply(simulationParameters.Trajectory.nz) - (simulationParameters.Trajectory.hz.PointwiseMultiply(simulationParameters.Trajectory.ny))).PointwiseMultiply(simulationParameters.Trajectory.tx) +
                (simulationParameters.Trajectory.hz.PointwiseMultiply(simulationParameters.Trajectory.nx) - (simulationParameters.Trajectory.hx.PointwiseMultiply(simulationParameters.Trajectory.nz))).PointwiseMultiply(simulationParameters.Trajectory.ty) +
                (simulationParameters.Trajectory.hx.PointwiseMultiply(simulationParameters.Trajectory.ny) - (simulationParameters.Trajectory.hy.PointwiseMultiply(simulationParameters.Trajectory.nx))).PointwiseMultiply(simulationParameters.Trajectory.tz);

            //Vector<double> signToolFace = expression.Map(x => (double)Math.Sign(x));

            Vector<double> signToolFace = Vector<double>.Build.Dense(expression.Count);
            Vector<double> dotProduct = Vector<double>.Build.Dense(expression.Count);
            for (int i = 0; i < expression.Count; i++)
            {
                signToolFace[i] = Math.Sign(expression[i]);
                dotProduct[i] = simulationParameters.Trajectory.hx[i] * simulationParameters.Trajectory.nx[i] +
                                simulationParameters.Trajectory.hy[i] * simulationParameters.Trajectory.ny[i] +
                                simulationParameters.Trajectory.hz[i] * simulationParameters.Trajectory.nz[i];
                dotProduct[i] = Math.Max(-1, dotProduct[i]);
                dotProduct[i] = Math.Min(1, dotProduct[i]);
            }

            // Compute the toolface angle
            ToolFaceAngle = dotProduct.PointwiseAcos().PointwiseMultiply(signToolFace);
            MudTorque = 0.0;
            //ForceDistribution = Vector<double>.Build.Dense(simulationParameters.Drillstring.PipeArea.Count); 
            //TorqueDistribution = Vector<double>.Build.Dense(simulationParameters.Drillstring.PipePolarMoment.Count);
            //TauM = Vector<double>.Build.Dense(simulationParameters.Drillstring.ShearModuli.Count);
            //ForceM = Vector<double>.Build.Dense(simulationParameters.Drillstring.YoungModuli.Count);            
            NormalCollisionForce = Vector<double>.Build.Dense(state.XDisplacement.Count);*/
        }

        public void UpdateBendingMoments(State state, SimulationParameters simulationParameters)
        {
            double XiMinus1, YiMinus1, XiPlus1, YiPlus1;
            double invElementLengthSquared = 1.0 / (simulationParameters.LumpedCells.ElementLength * simulationParameters.LumpedCells.ElementLength);
            for (int i = 1; i < state.XDisplacement.Count - 1; i++)
            {
                XiMinus1 = (i == 0) ? 0.0 : state.XDisplacement[i - 1];
                YiMinus1 = (i == 0) ? 0.0 : state.YDisplacement[i - 1];
                XiPlus1 = (i == state.XDisplacement.Count - 1) ? 0.0 : state.XDisplacement[i + 1];
                YiPlus1 = (i == state.YDisplacement.Count - 1) ? 0.0 : state.YDisplacement[i + 1];
                //Calcualte the bending moments using the central difference scheme
                BendingMomentX[i] = simulationParameters.Drillstring.YoungModuli[i] * simulationParameters.Drillstring.PipeInertia[i] *
                    (XiPlus1 - 2 * state.XDisplacement[i] + XiMinus1) * invElementLengthSquared; // Bending moment x-component
                BendingMomentY[i] = simulationParameters.Drillstring.YoungModuli[i] * simulationParameters.Drillstring.PipeInertia[i] *
                    (YiPlus1 - 2 * state.YDisplacement[i] + YiMinus1) * invElementLengthSquared; //
            }
        }
    }
}