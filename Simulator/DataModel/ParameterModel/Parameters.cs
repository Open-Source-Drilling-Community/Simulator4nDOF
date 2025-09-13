using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel;
using MathNet.Numerics;
using MathNet.Numerics.LinearAlgebra;
using static NORCE.Drilling.Simulator4nDOF.Simulator.Utilities;

namespace NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel
{
    public class Parameters
    {
        public Fluid fluid;
        public BitRock br;
        public MudMotor mm;
        public Trajectory t;
        public LumpedCells lc;
        public DistributedCells dc;
        public Wellbore w;
        public Drillstring ds;
        public Buoyancy b;
        public Friction f;

        public Parameters(DataModel.Configuration c)
        {
            fluid = new Fluid(c.FluidDensity);
            lc = new LumpedCells(c.BitDepth, c.LengthBetweenLumpedElements);
            ds = new Drillstring(lc, 
                                 fluid, 
                                 c.DrillStringSourceType,
                                 c.DrillstringFile, 
                                 c.DrillString,
                                 c.DrillStringOpenLab,
                                 c.BitDepth, 
                                 c.BitRadius, 
                                 c.SensorDistanceFromBit, 
                                 c.SleeveDistancesFromBit,
                                 c.SleeveDamping,
                                 c.DrillPipeLumpedElementLength,
                                 c.TorsionalDamping,
                                 c.AxialDamping,
                                 c.LateralDamping);
            w = new Wellbore(ds,
                             lc,
                             c.BoreHoleSizes,
                             c.TopDriveMomentOfInertia,
                             c.FluidDamping);
            t = new Trajectory(lc, c.TrajectoryFile);
            b = new Buoyancy(lc, t, ds, c.StringPressureFile, c.AnnulusPressureFile, c.UseBuoyancyFactor);


            mm = new MudMotor();
            dc = new DistributedCells(lc, ds, c.SurfaceRPM);
            br = new BitRock(dc, ds, c.RockStrengthEpsilon, c.BitWearLength, c.BitRockFrictionCoeff, c.PdcBladeNo);
            f = new Friction(lc, c.CoulombStaticFriction, c.CoulombKineticFriction, c.Stribeck);
        }

        public void AddNewLumpedElement()
        {
            double Pro = Math.Min(ds.ro[0], ds.ro[1]);                          // [m] Drill pipe outer radius
            double Pri = Math.Max(ds.ri[0], ds.ri[1]);                          // [m] Drill pipe inner radius
            double Pro_tj = Math.Max(ds.ro[0], ds.ro[1]);                       // [m] Tool joint outer radius
            double Pri_tj = Math.Min(ds.ri[0], ds.ri[1]);                       // [m] Tool joint inner radius
            double Jp = Math.PI / 2.0 * (Math.Pow(Pro, 4) - Math.Pow(Pri, 4));  // [m ^ 4] Drill string polar moment of inertia
            double Ap = Math.PI * (Math.Pow(Pro, 2) - Math.Pow(Pri, 2));        // [m ^ 2] Drill string cross sectional area
            double Ip = Math.PI / 4.0 * (Math.Pow(Pro, 4) - Math.Pow(Pri, 4));  // [m ^ 4] Drill string moment of inertia
            double Atj = Math.PI * (Math.Pow(Pro_tj, 2) - Math.Pow(Pri_tj, 2)); // [m ^ 2] Tool joint cross sectional area
            double Ao = Math.PI * Math.Pow(Pro, 2);                             // [m ^ 2] Drill pipe outer surface area
            double Ai = Math.PI * Math.Pow(Pri, 2);                             // [m ^ 2] Drill pipe inner surface area
            double Atjo = Math.PI * Math.Pow(Pro_tj, 2);                        // [m ^ 2] Tool joint outer surface area
            double Atji = Math.PI * Math.Pow(Pri_tj, 2);                        // [m ^ 2] Tool joint inner surface area
            double ecc_percent = Math.Max(ds.e[0], ds.e[1]) / Math.Max(ds.ro[0], ds.ro[1]); // eccentricity percent relative to total radius
            double mass_imbalance_percent = ds.M_e[0] / ds.M_L[0];

            // if current top element is a drill pipe, add a tool joint, and vice - versa
            if (ds.ro[0] == Pro)
            {
                // Copy the original array elements to the new array, starting at index 1
                ds.ro = ExtendVectorStart(Pro_tj, ds.ro);
                ds.ri = ExtendVectorStart(Pri_tj, ds.ri);
                ds.e = ExtendVectorStart(Pro_tj * ecc_percent, ds.e);
                ds.M_L = ExtendVectorStart(ds.rho * ds.l_L[0] * Ap + ds.rho * ds.l_tj * Atj, ds.M_L);
            }
            else
            {
                ds.ro = ExtendVectorStart(Pro, ds.ro);
                ds.ri = ExtendVectorStart(Pri, ds.ri);
                ds.e = ExtendVectorStart(0, ds.e);
                ds.M_L = ExtendVectorStart(ds.rho * ds.l_L[0] * Ap, ds.M_L);
            }

            ds.J = ExtendVectorStart(Jp, ds.J);
            ds.A = ExtendVectorStart(Ap, ds.A);
            ds.I = ExtendVectorStart(Ip, ds.I);
            ds.Ao = ExtendVectorStart(Ao, ds.Ao);
            ds.Ai = ExtendVectorStart(Ai, ds.Ai);
            ds.Atjo = ExtendVectorStart(Atjo, ds.Atjo);
            ds.Atji = ExtendVectorStart(Atji, ds.Atji);
            ds.E = ExtendVectorStart(ds.E[0], ds.E);
            ds.G = ExtendVectorStart(ds.G[0], ds.G);
            ds.weightCorr = ExtendVectorStart(ds.weightCorr[0], ds.weightCorr);
            ds.l_L = ExtendVectorStart(ds.l_L[0], ds.l_L);
            ds.I_L = ExtendVectorStart(ds.rho * ds.l_L[0] * Jp, ds.I_L);
            f.mu_s = ExtendVectorStart(f.mu_s[0], f.mu_s);
            f.mu_k = ExtendVectorStart(f.mu_k[0], f.mu_k);
            ds.kb = ExtendVectorStart(ds.kb[0], ds.kb);
            ds.Mf = ExtendVectorStart(Math.PI * fluid.rhoMud * (Math.Pow(ds.ri[0], 2) + ds.C_am * Math.Pow(ds.ro[0], 2)) * ds.l_L[0] / 2.0, ds.Mf);
            ds.M_e = ExtendVectorStart(mass_imbalance_percent * ds.M_L[0], ds.M_e);

            // Update spatial variables
            int Pt_old = lc.PL * lc.NL;
            int NL_old = lc.NL;

            // Generate the range from 0 to dc.x[0] with step size dx
            List<double> range = new List<double>();
            for (double value = 0; value <= dc.x[0]; value += dc.dx)
            {
                range.Add(value);
            }
            dc.x = Vector<double>.Build.Dense(range.Concat(dc.x).ToArray());
            lc.xL = ExtendVectorStart(0, lc.xL); // lumped section
            lc.NL = lc.NL + 1;
            if (ds.iS.Count > 0)
            {
                // Increment each element in ds.iS by 1
                for (int i = 0; i < ds.iS.Count; i++)
                {
                    ds.iS[i] += 1;
                }
            }
        }
    }
}