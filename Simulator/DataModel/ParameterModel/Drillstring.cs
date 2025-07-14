using MathNet.Numerics.LinearAlgebra;
using NORCE.Drilling.Simulator4nDOF.ModelShared;
using NORCE.Drilling.Simulator4nDOF.Simulator;
using System.Globalization;
using static NORCE.Drilling.Simulator4nDOF.Simulator.Utilities;

namespace NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel
{
    public class Drillstring
    {
        public double Rb = .2159 / 2;                   // [m] Bit radius(used in both Detournay and MSE model)

        // Material Properties 
        private readonly double Ep = 200e9;             // [Pa] Pipe Young's modulus
        private readonly double Ec = 200e9;             // [Pa] Collar Young's modulus
        private readonly double Gp = 79e9;              // [Pa] Pipe shear modulus
        private readonly double Gc = 79e9;              // [Pa] Collar shear modulus
        public readonly double rho = 7850;              // [kg / m3] Density of steel
        public readonly double nu = 0.28;               // Poisson ratio of steel

        public readonly double l_dp = 1.3;             // [m] Drill pipe lumped element length 
        private readonly double l_BHA = 1.5;            // [m] BHA Lumped element length
        public double l_tj = 0.5;                       // [m] Default tool joint length

        public Vector<double> ro;                       // [m] Outer radius vector
        public Vector<double> ri;                       // [m] Inner radius vector
        public Vector<double> e;                        // [m] Eccentricity vector
        public Vector<double> Ao;                       // [m^2] Drillpipe outer area vector
        public Vector<double> Ai;                       // [m^2] Drillpipe inner area vector
        public Vector<double> Atjo;                     // [m^2] Tool joint outer area vector
        public Vector<double> Atji;                     // [m^2] Tool joint inner area vector

        // Inertial peroperties                         
        public Vector<double> J;                        // [m^4] Drillpipe polar moment of inertia vector
        public Vector<double> A;                        // [m^2] Drillpipe cross sectional area vector
        public Vector<double> I;                        // [m^4] Drillpipe moment of inertia vector

        public Vector<double> weightCorr;               // [-] Linear weight correction factor vector
        public Vector<double> E;                        // [Pa] Young's modulus vector
        public Vector<double> G;                        // [Pa] Shear modulus vector

        public Vector<double> I_L;                      // [kg.m^2] Lumped element mass moment of inertia
        public Vector<double> M_L;                      // [kg] Lumped element mass
        public Vector<double> l_L;                      // [kg.m^2] Lumped element mass moment of inertia
                                                        
        public readonly double C_am = 1.7;              // Added fluid mass coefficient
        public readonly double mass_imbalance_percent = .05; //percent (fraction?) of mass imbalance compared to total mass -  todo rename?

        // Sleeves - to be configured
        public Vector<double> sleeveDistancesFromBit;  // Example data

        public double r_Si = 0.1651 / 2;                // [m] Sleeve inner radius
        public double r_So = 0.1905 / 2 + 0.005;        // [m] Sleeve outer radius
        public double l_S = 1.0;                        // [m] Sleeve length
        public double k_ec_factor = 2.0e2;              // [N.s/rad] Sleeve torsional damping coefficient
        public double axialFrictionReduction = 0.99;    // Axial friction reduction factor due to spur wheels

        // Sleeves - calculated
        public int NS;                                  // Total number of sleeves
        public Vector<double> iS;                       // Index of sleeves in lumped nodes
        public double I_S;                              // [kg.m^2] Sleeve mass moment of inertia
        public Vector<double> k_ec;                     // [N.s/rad] Sleeve torsional damping coefficient

        // Calculated - todo move?
        public double c_t;                              // [m/s] Torsional wave velocity
        public double c_a;

        public double kt_factor = 1.0;                  //[N.m.s/rad] Torsional damping 
        public double ka_factor = 1.0;                  //[N.s/m] Axial damping
        public double kl_factor = 0.0;                  //[N.s/m] Lateral damping

        // Structureal damping - calculated
        public double kt;
        public double ka;
        public double kl;

        public double Lb;                               // [m] Length of pipe used in bending stiffness calculation
        public Vector<double> kb;                       // [N/m] Bending stiffness
        public Vector<double> Mf;                       // [kg] Added fluid mass
        public Vector<double> M_e;                      // [kg] Eccentric mass

        public double zeta_p;                           // Characteristic drill pipe impedance

        //IMU sensors -  configure
        public double sensorDistanceFromBit = 63;       // [m] axial distance(relative to bit depth) of an IMU measuring downhole RPM and accelerations
        public readonly double sensorMisalignmentPolarAngle = 0 * Math.PI / 180; // polar angle of the accelerometer misalignment[rad]
        public readonly double sensorMisalignmentAzimuthAngle = 0 * Math.PI / 180; // azimuth angle of the accelerometer misalignment[rad]
        public readonly Vector<double> sensorDisplacementsInLocalFrame = Vector<double>.Build.Dense(3, 0); // radial, tangential and axial displacements in the local frame of the accelerometer

        //IMU sensors -  to be calculated
        public int idx_sensor;
        public double sensorRadialDistance; // [m] radial distance from centerline of the tool to the accelerometer

        public Drillstring(LumpedCells lc, 
                           Fluid fluid, 
                           string drillstringFile, 
                           DrillString drillString,
                           double MD, 
                           double Rb, 
                           double sensorDistanceFromBit, 
                           Vector<double> sleeveDistancesFromBit, 
                           double k_ec_factor, 
                           double l_dp,
                           double kt_factor,
                           double ka_factor,
                           double kl_factor)
        {
            this.l_dp = l_dp;
            this.sensorDistanceFromBit = sensorDistanceFromBit;
            this.sleeveDistancesFromBit = sleeveDistancesFromBit;
            this.k_ec_factor = k_ec_factor;
            this.kt_factor = kt_factor;
            this.ka_factor = ka_factor;
            this.kl_factor = kl_factor;
            this.Rb = Rb;

            List<string> component_type = new List<string>();
            List<double> component_length = new List<double>();
            List<double> connection_id = new List<double>();
            List<double> connection_od = new List<double>();
            List<double> connection_length = new List<double>();
            List<double> id = new List<double>();
            List<double> od = new List<double>();
            List<double> linear_weight = new List<double>();

            bool readFromMS = true;
            double scaleFactor = readFromMS ? Constants.in2m : 1.0;

            if (readFromMS)
            {
                foreach (var section in drillString.DrillStringSectionList)
                {
                    var compCount = section.Count;
                    // If same component in a section then simplify and insert one component with length adjusted
                    var repetitions = section.SectionComponentList.Count == 1 ? 1 : compCount;

                    for (int i = 0; i < repetitions; i++)
                    {
                        foreach (var comp in section.SectionComponentList)
                        {
                            var type = comp.Type;
                            var length = comp.Length * (section.SectionComponentList.Count == 1 ? compCount : 1);
                            var mass = 0.0;
                            var connectionID = double.MaxValue;
                            var connectionOD = double.MinValue;
                            var connectionLength = double.MaxValue;

                            foreach (var part in comp.PartList)
                            {
                                mass += part.Mass;
                                connectionID = Math.Min(connectionID, part.InnerDiameter);
                                connectionOD = Math.Max(connectionOD, part.OuterDiameter);
                                connectionLength = Math.Min(connectionLength, part.TotalLength);
                            }

                            var partList = comp.PartList.ToList();
                            var iD = (partList.Count == 3) ? partList[1].InnerDiameter : connectionID;
                            var oD = (partList.Count == 3) ? partList[1].OuterDiameter : connectionOD;

                            var adjustedType = type switch
                            {
                                DrillStringComponentTypes.DrillPipe => "Drillpipe",
                                DrillStringComponentTypes.HeavyWeightDrillPipe => "HW drillpipe",
                                _ => type.ToString()
                            };

                            component_type.Add(adjustedType);
                            component_length.Add(length);
                            connection_id.Add(connectionID);
                            connection_od.Add(connectionOD);
                            connection_length.Add(connectionLength);
                            id.Add(iD);
                            od.Add(oD);
                            linear_weight.Add(mass / length);
                        }
                    }
                }

                component_type.Reverse();
                component_length.Reverse();
                connection_id.Reverse();
                connection_od.Reverse();
                connection_length.Reverse();
                id.Reverse();
                od.Reverse();
                linear_weight.Reverse();
            }
            else
            {
                List<string[]> data = new List<string[]>();
                using (StreamReader sr = new StreamReader(drillstringFile))
                {
                    string line;
                    // Skip header line
                    sr.ReadLine();
                    while ((line = sr.ReadLine()) != null)
                    {
                        string[] values = line.Split(';');
                        data.Add(values);
                    }
                }

                foreach (var row in data)
                {
                    component_type.Add(row[7]);
                }

                foreach (var row in data)
                {
                    var str = row[10];
                    if (str == "N/A")
                        component_length.Add(double.NaN);
                    else
                        component_length.Add(Convert.ToDouble(str, CultureInfo.InvariantCulture));
                }

                foreach (var row in data)
                {
                    var str = row[0];
                    if (str == "N/A")
                        connection_id.Add(double.NaN);
                    else
                        connection_id.Add(Convert.ToDouble(str, CultureInfo.InvariantCulture));
                }

                foreach (var row in data)
                {
                    var str = row[5];
                    if (str == "N/A")
                        connection_od.Add(double.NaN);
                    else
                        connection_od.Add(Convert.ToDouble(str, CultureInfo.InvariantCulture));
                }

                foreach (var row in data)
                {
                    var str = row[4];
                    if (str == "N/A")
                        connection_length.Add(double.NaN);
                    else
                        connection_length.Add(Convert.ToDouble(str, CultureInfo.InvariantCulture));
                }

                foreach (var row in data)
                {
                    var str = row[12];
                    if (str == "N/A")
                        id.Add(double.NaN);
                    else
                        id.Add(Convert.ToDouble(str, CultureInfo.InvariantCulture));
                }

                foreach (var row in data)
                {
                    var str = row[6];
                    if (str == "N/A")
                        od.Add(double.NaN);
                    else
                        od.Add(Convert.ToDouble(str, CultureInfo.InvariantCulture));
                }
                foreach (var row in data)
                {
                    linear_weight.Add(Convert.ToDouble(row[3], CultureInfo.InvariantCulture));
                }
            }

            int n = component_type.Count();
            List<double> LcBha = new List<double>();            // [m] BHA component length vector
            List<double> ODBha = new List<double>();            // [m] BHA component OD vector
            List<double> IDBha = new List<double>();            // [m] BHA component ID vector
            List<double> LinearWeightBha = new List<double>();  // [kg/m] BHA component linear weight vector
            List<int> isStab = new List<int>();                 // array indicating which BHA components are stabilizers (1 = stabilizer, 0 = no stabilizer)
            List<double> LDp = new List<double>();              // [m] Drillpipe length vector (including heavy weight drillpipe)
            List<double> ODDp = new List<double>();             // [m] Drillpipe OD vector
            List<double> IDDp = new List<double>();             // [m] Drillpipe ID vector
            List<double> ODDp_tj = new List<double>();          // [m] Drillpipe tool joint OD vector
            List<double> IDDp_tj = new List<double>();          // [m] Drillpipe tool joint ID vector
            List<double> LinearWeightDp = new List<double>();   // [kg/m] Drillpipe linear weight vector

            int idx_bhaEnd = 0;

            // Loop through each component
            for (int i = 0; i < n; i++)
            {
                if (component_type[i] == "HW drillpipe" || component_type[i] == "Drillpipe")
                {
                    // Exit loop if a heavy-weight drillpipe or regular drillpipe is found
                    idx_bhaEnd = i - 1;
                    break;
                }

                // Insert elements at the beginning to maintain order (MATLAB prepends using [newVal, array])
                LcBha.Insert(0, component_length[i]);
                ODBha.Insert(0, od[i] * Constants.in2m);
                IDBha.Insert(0, id[i] * Constants.in2m);
                LinearWeightBha.Insert(0, linear_weight[i]);

                if (component_type[i] == "Stabilizer")
                {
                    isStab.Insert(0, 1);
                }
                else
                {
                    isStab.Insert(0, 0);
                }
            }

            for (int i = idx_bhaEnd; i < n; i++) // go through the remaining components to get the heavy weight drillpipe (+jars), drillpipe and tool joint dimensions
            {
                if (component_type[i] == "HW drillpipe" || component_type[i] == "Jar" || component_type[i] == "Drillpipe")
                {
                    LDp.Insert(0, component_length[i]);
                    ODDp.Insert(0, od[i] * Constants.in2m);
                    IDDp.Insert(0, id[i] * Constants.in2m);
                    ODDp_tj.Insert(0, connection_od[i] * Constants.in2m);
                    IDDp_tj.Insert(0, connection_id[i] * Constants.in2m);
                    LinearWeightDp.Insert(0, linear_weight[i]);
                    l_tj = connection_length[i];
                }
            }

            // Stabilizers
            double nStab = isStab.Sum(); // Number of stabilizers
            List<double> LStab = new List<double>();
            List<double> roStab = new List<double>();
            List<double> riStab = new List<double>();
            List<double> AStab = new List<double>();
            List<double> JStab = new List<double>();
            List<double> IStab = new List<double>();

            if (nStab > 0)
            {
                List<int> idxStab = isStab.Select((value, index) => new { value, index })
                                          .Where(x => x.value != 0)
                                          .Select(x => x.index)
                                          .ToList();

                LStab = idxStab.Select(i => LcBha[i]).ToList();         // [m] Stabilizer length
                roStab = idxStab.Select(i => ODBha[i] / 2).ToList();    // [m] Stabilizer outer radius
                riStab = idxStab.Select(i => IDBha[i] / 2).ToList();    // [m] Stabilizer inner radius
                AStab = roStab.Zip(riStab, (ro, ri) => Math.PI * (Math.Pow(ro, 2) - Math.Pow(ri, 2))).ToList();  // [m^2] Cross sectional area for stabilizer                                                                                                              // Compute JStab (Polar moment of inertia)
                JStab = roStab.Zip(riStab, (ro, ri) => Math.PI / 2 * (Math.Pow(ro, 4) - Math.Pow(ri, 4))).ToList();
                IStab = roStab.Zip(riStab, (ro, ri) => Math.PI / 4 * (Math.Pow(ro, 4) - Math.Pow(ri, 4))).ToList();
            }

            List<double> Lc = new List<double>();
            List<double> Cro = new List<double>();
            List<double> Cri = new List<double>();
            List<double> Ac = new List<double>();
            List<double> Jc = new List<double>();
            List<double> Ic = new List<double>();
            List<double> Lwc = new List<double>();

            double Lc_sum = 0, Cro_sum = 0, Cri_sum = 0, Ac_sum = 0, Jc_sum = 0, Ic_sum = 0, Lwc_sum = 0;

            for (int i = 0; i < LcBha.Count; i++)
            {
                if (isStab[i] == 0)
                {
                    Lc_sum += LcBha[i];
                    Cro_sum += LcBha[i] * ODBha[i] / 2;
                    Cri_sum += LcBha[i] * IDBha[i] / 2;
                    Ac_sum += Math.PI * LcBha[i] * (Math.Pow(ODBha[i] / 2, 2) - Math.Pow(IDBha[i] / 2, 2));
                    Jc_sum += Math.PI / 2 * LcBha[i] * (Math.Pow(ODBha[i] / 2, 4) - Math.Pow(IDBha[i] / 2, 4));
                    Ic_sum += Math.PI / 4 * LcBha[i] * (Math.Pow(ODBha[i] / 2, 4) - Math.Pow(IDBha[i] / 2, 4));
                    Lwc_sum += LcBha[i] * LinearWeightBha[i];
                }
                else if (i > 0)
                {
                    Lc.Add(Lc_sum);
                    Cro.Add(Cro_sum / Lc_sum);
                    Cri.Add(Cri_sum / Lc_sum);
                    Ac.Add(Ac_sum / Lc_sum);
                    Jc.Add(Jc_sum / Lc_sum);
                    Ic.Add(Ic_sum / Lc_sum);
                    Lwc.Add(Lwc_sum / Lc_sum);
                    Lc_sum = 0; Cro_sum = 0; Cri_sum = 0; Ac_sum = 0; Jc_sum = 0; Ic_sum = 0; Lwc_sum = 0;
                }
            }

            if (Lc_sum > 0)
            {
                Lc.Add(Lc_sum);
                Cro.Add(Cro_sum / Lc_sum);
                Cri.Add(Cri_sum / Lc_sum);
                Ac.Add(Ac_sum / Lc_sum);
                Jc.Add(Jc_sum / Lc_sum);
                Ic.Add(Ic_sum / Lc_sum);
                Lwc.Add(Lwc_sum / Lc_sum);
            }

            double Lavg = lc.L / lc.NL;

            List<int> Nc = Lc.Select(l => Math.Max((int)Math.Floor(l / Lavg), 1)).ToList(); // number of elements in BHA section, excluding stabilizers
            List<int> Np = LDp.Select(l => (int)Math.Round(l / Lavg, MidpointRounding.AwayFromZero)).ToList(); // number of elements in drillpipe section (including heavy weight drillpipe)

            if (nStab + Nc.Sum() + Np.Sum() > lc.NL)
            {
                if (Np.Count > 1)
                {
                    for (int i = Np.Count - 1; i >= 0; i--)
                    {
                        if (nStab + Nc.Sum() + Np.Skip(i).Sum() > lc.NL)
                        {
                            Np[i] = lc.NL - ((int)nStab + Nc.Sum() + Np.Skip(i + 1).Sum());
                            for (int w = 0; w < i; w++)
                            {
                                Np[w] = 0;
                            }
                            break;
                        }
                    }
                }
                else
                {
                    Np[0] = lc.NL - ((int)nStab + Nc.Sum());
                }
            }

            double sleeveThreshold = (nStab + Nc.Sum()) * Lavg;
            // Create a boolean mask for elements below the threshold
            var sleeves_belowTopOfBha = sleeveDistancesFromBit
                .Select(x => x < sleeveThreshold)
                .ToArray();

            // Filter sleeveDistancesFromBit to keep elements where sleeves_belowTopOfBha is false
            var filteredSleeveDistancesFromBit = sleeveDistancesFromBit
                .EnumerateIndexed()
                .Where(x => !sleeves_belowTopOfBha[x.Item1])
                .Select(x => x.Item2)
                .ToArray();
            sleeveDistancesFromBit = Vector<double>.Build.DenseOfArray(filteredSleeveDistancesFromBit);//// ignore sleeves which would go below the top of the BHA

            NS = sleeveDistancesFromBit.Count;  // Total number of sleeves
            iS = Vector<double>.Build.Dense(NS);//new int[p.NS];

            for (int i = 0; i < NS; i++)
            {
                iS[i] = Array.FindIndex(lc.xL.ToArray(), x => x > MD - lc.dxL - sleeveDistancesFromBit[i] && x <= MD - sleeveDistancesFromBit[i]); // Index of sleeves in lumped nodes
            }
            iS = Reverse(iS);

            I_S = rho * l_S * Math.PI / 2 * (Math.Pow(r_So, 4) - Math.Pow(r_Si, 4)); //  [kg.m^2] Sleeve mass moment of inertia

            Vector<double> vectorOfOnes = Vector<double>.Build.Dense(iS.Count(), 1.0);
            k_ec = k_ec_factor * vectorOfOnes; // [N.s/rad] Sleeve torsional damping coefficient

            // Define nodes with tool joints; if discretization is every 10 m or more, we use tool joint diameters
            // everywhere there is a drillpipe / heavy weight drillpipe element unless there is a sleeve at that node
            List<int> idx_nonzeroPipeElements = Np.Select((value, index) => new { value, index })
                                                  .Where(x => x.value > 0)
                                                  .Select(x => x.index)
                                                  .ToList();
            List<int> isToolJoint = new List<int>(new int[lc.NL]);
            int idx_previousToolJoint = 0; // index of previous tool joint
            isToolJoint[idx_previousToolJoint] = 1;

            for (int i = idx_previousToolJoint + 1; i < Np.Sum(); i++)
            {
                if (lc.xL[i] - lc.xL[idx_previousToolJoint] >= 10 && !iS.Contains(i))
                {
                    isToolJoint[i] = 1;
                    idx_previousToolJoint = i;
                }
                else
                {
                    isToolJoint[i] = 0;
                }
            }

            double ecc_percent = 0.05;                  // eccentricity percent relative to total radius

            List<double> roList = new List<double>();   // [m] Outer radius vector
            List<double> riList = new List<double>();   // [m] Inner radius vector
            List<double> eList = new List<double>();    // [m] Eccentricity vector
            List<double> AoList = new List<double>();   // [m^2] Drillpipe outer area vector
            List<double> AiList = new List<double>();   // [m^2] Drillpipe inner area vector
            List<double> AtjoList = new List<double>(); // [m^2] Tool joint outer area vector
            List<double> AtjiList = new List<double>(); // [m^2] Tool joint inner area vector

            List<double> JList = new List<double>();    // [m^4] Drillpipe polar moment of inertia vector
            List<double> AList = new List<double>();    // [m^2] Drillpipe cross sectional area vector
            List<double> IList = new List<double>();    // [m^4] Drillpipe moment of inertia vector

            List<double> weightCorrList = new List<double>(); // [-] Linear weight correction factor vector
            List<double> EList = new List<double>();    // [Pa] Young's modulus vector
            List<double> GList = new List<double>();    // [Pa] Shear modulus vector

            List<double> I_LList = new List<double>();  // [kg.m^2] Lumped element mass moment of inertia
            List<double> M_LList = new List<double>();  // [kg] Lumped element mass

            int j = idx_nonzeroPipeElements[0];
            for (int i = 0; i < Np.Sum(); i++)
            {
                if (i + 1 - Np.Take(j).Sum() > Np[j])
                {
                    j++;
                }
                if (isToolJoint[i] == 1)
                {
                    roList.Add(ODDp_tj[j] / 2);
                    riList.Add(IDDp_tj[j] / 2);
                    eList.Add(ODDp_tj[j] / 2 * ecc_percent);
                }
                else
                {
                    roList.Add(ODDp[j] / 2);
                    riList.Add(IDDp[j] / 2);
                    eList.Add(0);
                }
            }

            for (int i = idx_nonzeroPipeElements[0]; i < Np.Count; i++)
            {
                JList.AddRange(Enumerable.Repeat(Math.PI / 2 * (Math.Pow(ODDp[i] / 2, 4) - Math.Pow(IDDp[i] / 2, 4)), Np[i]));
                AList.AddRange(Enumerable.Repeat(Math.PI * (Math.Pow(ODDp[i] / 2, 2) - Math.Pow(IDDp[i] / 2, 2)), Np[i]));
                IList.AddRange(Enumerable.Repeat(Math.PI / 4 * (Math.Pow(ODDp[i] / 2, 4) - Math.Pow(IDDp[i] / 2, 4)), Np[i]));
                AoList.AddRange(Enumerable.Repeat(Math.PI * Math.Pow(ODDp[i] / 2, 2), Np[i]));
                AiList.AddRange(Enumerable.Repeat(Math.PI * Math.Pow(IDDp[i] / 2, 2), Np[i]));
                AtjoList.AddRange(Enumerable.Repeat(Math.PI * Math.Pow(ODDp_tj[i] / 2, 2), Np[i]));
                AtjiList.AddRange(Enumerable.Repeat(Math.PI * Math.Pow(IDDp_tj[i] / 2, 2), Np[i]));
                weightCorrList.AddRange(Enumerable.Repeat(LinearWeightDp[i], Np[i]));
                EList.AddRange(Enumerable.Repeat(Ep, Np[i]));
                GList.AddRange(Enumerable.Repeat(Gp, Np[i]));
            }

            foreach (int i in iS)
            {
                AtjoList[i] = Math.PI * Math.Pow(r_So, 2);
                AtjiList[i] = Math.PI * Math.Pow(r_Si, 2);
            }

            for (int i = 0; i < nStab; i++)
            {
                riList.AddRange(Enumerable.Repeat(Cri[i], Nc[i]).Concat(new[] { riStab[i] }));
                roList.AddRange(Enumerable.Repeat(Cro[i], Nc[i]).Concat(new[] { roStab[i] }));
                eList.AddRange(Enumerable.Repeat(Cro[i] * ecc_percent, Nc[i]).Concat(new[] { roStab[i] * ecc_percent }));
                JList.AddRange(Enumerable.Repeat(Jc[i], Nc[i]).Concat(new[] { JStab[i] }));
                AList.AddRange(Enumerable.Repeat(Ac[i], Nc[i]).Concat(new[] { AStab[i] }));
                IList.AddRange(Enumerable.Repeat(Ic[i], Nc[i]).Concat(new[] { IStab[i] }));
                AoList.AddRange(Enumerable.Repeat(Math.PI * Math.Pow(Cro[i], 2), Nc[i]).Concat(new[] { Math.PI * Math.Pow(roStab[i], 2) }));
                AiList.AddRange(Enumerable.Repeat(Math.PI * Math.Pow(Cri[i], 2), Nc[i]).Concat(new[] { Math.PI * Math.Pow(riStab[i], 2) }));
                AtjoList.AddRange(Enumerable.Repeat(Math.PI * Math.Pow(Cro[i], 2), Nc[i]).Concat(new[] { Math.PI * Math.Pow(roStab[i], 2) }));
                AtjiList.AddRange(Enumerable.Repeat(Math.PI * Math.Pow(Cri[i], 2), Nc[i]).Concat(new[] { Math.PI * Math.Pow(riStab[i], 2) }));
                weightCorrList.AddRange(Enumerable.Repeat(Lwc[i], Nc[i]).Concat(new[] { AStab[i] * rho }));
            }

            if (Nc.Count > nStab)
            {
                riList.AddRange(Enumerable.Repeat(Cri.Last(), Nc.Last()));
                roList.AddRange(Enumerable.Repeat(Cro.Last(), Nc.Last()));
                eList.AddRange(Enumerable.Repeat(Cro.Last() * ecc_percent, Nc.Last()));
                JList.AddRange(Enumerable.Repeat(Jc.Last(), Nc.Last()));
                AList.AddRange(Enumerable.Repeat(Ac.Last(), Nc.Last()));
                IList.AddRange(Enumerable.Repeat(Ic.Last(), Nc.Last()));
                AoList.AddRange(Enumerable.Repeat(Math.PI * Math.Pow(Cro.Last(), 2), Nc.Last()));
                AiList.AddRange(Enumerable.Repeat(Math.PI * Math.Pow(Cri.Last(), 2), Nc.Last()));
                AtjoList.AddRange(Enumerable.Repeat(Math.PI * Math.Pow(Cro.Last(), 2), Nc.Last()));
                AtjiList.AddRange(Enumerable.Repeat(Math.PI * Math.Pow(Cri.Last(), 2), Nc.Last()));
                weightCorrList.AddRange(Enumerable.Repeat(Lwc.Last(), Nc.Last()));
            }

            EList.AddRange(Enumerable.Repeat(Ec, Nc.Sum() + (int)nStab));
            GList.AddRange(Enumerable.Repeat(Gc, Nc.Sum() + (int)nStab));

            List<double> Atj = AtjoList.Zip(AtjiList, (atjo, atji) => atjo - atji).ToList();
            weightCorrList = weightCorrList.Prepend(weightCorrList.First()).Zip(AList.Prepend(AList.First()), (wc, a) => wc / a / rho).ToList();
            zeta_p = JList.First() * Math.Sqrt(Gp * rho); // Characteristic drill pipe impedance

            List<double> l_L_List = Enumerable.Repeat(l_dp, Np.Sum()).ToList();
            for (int i = 0; i < nStab; i++)
            {

                l_L_List.AddRange(Enumerable.Repeat(l_BHA, Nc[i]));
                l_L_List.Add(LStab[i]);

            }
            if (Nc.Count > nStab)
            {
                l_L_List.AddRange(Enumerable.Repeat(l_BHA, Nc.Last()));
            }
            l_L = ToVector(l_L_List);

            I_LList = l_L_List.Zip(JList, (l, j) => rho * l * j).ToList();  // [kg.m^2] Lumped element mass moment of inertia
            M_LList = l_L_List.Zip(AList, (l, a) => rho * l * a).ToList();  // [kg] Lumped element mass
            Vector<double> M_tj = rho * l_tj * ToVector(Atj);               // [kg] Tool joint lumped mass
            Vector<double> M_S = rho * l_S * ToVector(Atj);                 // [kg] Sleeve lumped mass

            for (int i = 0; i < lc.NL; i++)
            {
                if (isToolJoint[i] == 1 && Math.Abs(Atj[i] - AList[i]) > 1e-3)
                {
                    M_LList[i] += M_tj[i];
                }
                if (iS.Contains(i))
                {
                    M_LList[i] += M_S[i];
                }
            }

            ro = Vector<double>.Build.DenseOfArray(roList.ToArray());     // [m] Outer radius vector
            ri = Vector<double>.Build.DenseOfArray(riList.ToArray());     // [m] Inner radius vector
            e = Vector<double>.Build.DenseOfArray(eList.ToArray()); ;     // [m] Eccentricity vector
            Ao = Vector<double>.Build.DenseOfArray(AoList.ToArray()); ;   // [m^2] Drillpipe outer area vector
            Ai = Vector<double>.Build.DenseOfArray(AiList.ToArray()); ;   // [m^2] Drillpipe inner area vector
            Atjo = Vector<double>.Build.DenseOfArray(AtjoList.ToArray()); // [m^2] Tool joint outer area vector
            Atji = Vector<double>.Build.DenseOfArray(AtjiList.ToArray()); // [m^2] Tool joint inner area vector

            J = Vector<double>.Build.DenseOfArray(JList.ToArray());       // [m^4] Drillpipe polar moment of inertia vector
            A = Vector<double>.Build.DenseOfArray(AList.ToArray());       // [m^2] Drillpipe cross sectional area vector
            I = Vector<double>.Build.DenseOfArray(IList.ToArray());       // [m^4] Drillpipe moment of inertia vector

            weightCorr = Vector<double>.Build.DenseOfArray(weightCorrList.ToArray()); // [-] Linear weight correction factor vector
            E = Vector<double>.Build.DenseOfArray(EList.ToArray());       // [Pa] Young's modulus vector
            G = Vector<double>.Build.DenseOfArray(GList.ToArray());       // [Pa] Shear modulus vector

            I_L = Vector<double>.Build.DenseOfArray(I_LList.ToArray());   // [kg.m^2] Lumped element mass moment of inertia
            M_L = Vector<double>.Build.DenseOfArray(M_LList.ToArray());   // [kg] Lumped element mass


            // Wave velocities
            c_t = Math.Sqrt(G.Average() / rho);  //[m/s] Torsional wave velocity
            c_a = Math.Sqrt(E.Average() / rho);  //[m/s] Axial wave velocity

            // Structural damping coefficients
            kt = kt_factor * I_L.Average(); //[N.m.s/rad] Torsional damping 
            ka = ka_factor * M_L.Average(); //[N.s/m] Axial damping 
            kl = kl_factor * M_L.Average(); //[N.s/m] Lateral damping

            Lb = lc.dxL; //[m] Length of pipe used in bending stiffness calculation 
            kb = Math.Pow(Math.PI, 4) / (2 * Math.Pow(Lb, 3)) * E.PointwiseMultiply(I);// [N/m] Bending stiffness
            Mf = 0.5 * Math.PI * fluid.rhoMud * (ri.PointwisePower(2) + C_am * ro.PointwisePower(2)).PointwiseMultiply(l_L); //[kg] Added fluid mass

            M_e = mass_imbalance_percent * M_L; // [kg] Eccentric mass

            var dxL = lc.dxL;
            idx_sensor = (int)lc.xL
            .Select((value, index) => new { Value = value, Index = index })
            .Where(x => x.Value > MD - sensorDistanceFromBit - dxL && x.Value <= MD - sensorDistanceFromBit)
            .Select(x => (double)x.Index) // Convert index to double
            .First();

            if (iS.Contains(idx_sensor))
                sensorRadialDistance = 0.5 * (r_Si + r_So);
            else
            {
                int adjustedIndex = idx_sensor; // If idx_sensor is already zero-based, no adjustment is needed
                sensorRadialDistance = 0.5 * (ri[adjustedIndex] + ro[adjustedIndex]);
            }

        }
    }
}
