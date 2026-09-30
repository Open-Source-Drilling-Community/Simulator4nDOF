using Microsoft.Extensions.Logging;
using NORCE.Drilling.Simulator4nDOF.Model;
using NORCE.Drilling.Simulator4nDOF.ModelShared;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;

namespace NORCE.Drilling.Simulator4nDOF.Service.Managers
{
    internal sealed class ResolvedContextualData
    {
        public required DrillString DrillString { get; init; }
        public required DrillingFluidDescription DrillingFluidDescription { get; init; }
        public Trajectory? Trajectory { get; init; }
        public SurveyRun? SurveyRun { get; init; }
        public required Rig Rig { get; init; }
        public GeothermalProperties? GeothermalProperties { get; init; }
        public required double FluidDensity { get; init; }
        public required double BitRadius { get; init; }
        public CasingSection? CasingSection { get; init; }
    }

    internal static class ContextualDataResolver
    {
        public static async Task<ResolvedContextualData> ResolveAsync(Simulation simulation)
        {
            ArgumentNullException.ThrowIfNull(simulation);

            var contextualData = simulation.ContextualData;
            ArgumentNullException.ThrowIfNull(contextualData);

            var drillString = await LoadRequiredAsync(
                contextualData.DrillStringID,
                "drill string",
                id => APIUtils.ClientDrillString.GetDrillStringByIdAsync(id));
            var drillingFluidDescription = await LoadRequiredAsync(
                contextualData.DrillingFluidDescriptionID,
                "drilling fluid description",
                id => APIUtils.ClientDrillingFluid.GetDrillingFluidDescriptionByIdAsync(id));
           
            var (trajectory, surveyRun) = await ResolveTrajectorySourceAsync(contextualData);

            var rig = await LoadRigAsync(simulation);

            var wellBoreArchitecture = await LoadOptionalAsync(
                contextualData.WellBoreArchitectureID,
                id => APIUtils.ClientWellBoreArchitecture.GetWellBoreArchitectureByIdAsync(id));

            var geothermalProperties = await LoadInterpolatedGeothermalProperties(contextualData.GeothermalPropertiesID, simulation.Config);
        
            var casingSection = ResolveCasingSection(wellBoreArchitecture, contextualData.CasingID);
            var fluidDensity = ResolveFluidDensity(drillingFluidDescription, contextualData.Temperature, contextualData.SurfacePipePressure);
            var bitRadius = ResolveBitRadius(drillString);

            return new ResolvedContextualData
            {
                DrillString = drillString,
                DrillingFluidDescription = drillingFluidDescription,
                Trajectory = trajectory,
                SurveyRun = surveyRun,
                Rig = rig,
                CasingSection = casingSection,
                FluidDensity = fluidDensity,
                BitRadius = bitRadius,
                GeothermalProperties = geothermalProperties
            };
        }

        private static async Task<Rig> LoadRigAsync(Simulation simulation)
        {
            if (simulation.ContextualData?.RigID is Guid rigId && rigId != Guid.Empty)
            {
                return await LoadRequiredAsync(
                    rigId,
                    "rig",
                    id => APIUtils.ClientRig.GetRigByIdAsync(id));
            }

            if (simulation.WellBoreID == null || simulation.WellBoreID == Guid.Empty)
            {
                throw new Exception("RigID is missing and the simulation has no WellBoreID for rig tracking.");
            }

            var wellBore = await LoadRequiredAsync(
                simulation.WellBoreID,
                "wellbore",
                id => APIUtils.ClientWellBore.GetWellBoreByIdAsync(id));

            if (wellBore.RigJobs is not null)
            {
                RigJob? latestJob = wellBore.RigJobs.OrderBy(job => job.StartDate).LastOrDefault();
                if (latestJob is null)
                    throw new Exception($"Wellbore '{simulation.WellBoreID}' has an authoritative empty rig-job history.");
                return await LoadRequiredAsync(
                    latestJob.RigID,
                    "rig",
                    id => APIUtils.ClientRig.GetRigByIdAsync(id));
            }

#pragma warning disable CS0612
            if (wellBore.RigID is Guid legacyRigId && legacyRigId != Guid.Empty)
            {
                return await LoadRequiredAsync(
                    legacyRigId,
                    "rig",
                    id => APIUtils.ClientRig.GetRigByIdAsync(id));
            }
#pragma warning restore CS0612

            if (wellBore.WellID == null || wellBore.WellID == Guid.Empty)
            {
                throw new Exception($"Wellbore '{simulation.WellBoreID}' has no WellID, so the rig cannot be tracked.");
            }

            var well = await LoadRequiredAsync(
                wellBore.WellID,
                "well",
                id => APIUtils.ClientWell.GetWellByIdAsync(id));
            if (well.ClusterID == null || well.ClusterID == Guid.Empty)
            {
                throw new Exception($"Well '{wellBore.WellID}' has no ClusterID, so the rig cannot be tracked.");
            }

            var cluster = await LoadRequiredAsync(
                well.ClusterID,
                "cluster",
                id => APIUtils.ClientCluster.GetClusterByIdAsync(id));
            if (cluster.RigID == null || cluster.RigID == Guid.Empty)
            {
                throw new Exception($"Cluster '{well.ClusterID}' has no RigID, so the rig cannot be tracked.");
            }

            return await LoadRequiredAsync(
                cluster.RigID,
                "rig",
                id => APIUtils.ClientRig.GetRigByIdAsync(id));
        }

        private static async Task<T> LoadRequiredAsync<T>(Guid? id, string label, Func<Guid, Task<T>> loader) where T : class
        {
            if (id == null || id == Guid.Empty)
            {
                throw new Exception($"A {label} ID is required in contextual data.");
            }
            T? value = null;
            try
            {
                value = await loader(id.Value);
                if (value == null)
                {
                    throw new Exception($"{label} '{id}' could not be loaded from its microservice.");
                }
            }
            catch (Exception ex)
            {
                throw new Exception(ex.ToString());
            }
            return value;
        }

        private static async Task<T?> LoadOptionalAsync<T>(Guid? id, Func<Guid, Task<T>> loader) where T : class
        {
            if (id == null || id == Guid.Empty)
            {
                return null;
            }

            return await loader(id.Value);
        }
        private static readonly TimeSpan TrajectoryCalculationPollInterval = TimeSpan.FromMilliseconds(500);
        private static readonly TimeSpan TrajectoryCalculationTimeout = TimeSpan.FromMinutes(2);

        /// <summary>
        /// A trajectory is always required. If a survey run of that trajectory is selected, it takes priority.
        /// Exactly one of the returned values is not null.
        /// </summary>
        private static async Task<(Trajectory? Trajectory, SurveyRun? SurveyRun)> ResolveTrajectorySourceAsync(ContextualData contextualData)
        {
            var calculationType = contextualData.TrajectoryCalculationType;
            var trajectory = await LoadRequiredAsync(
                contextualData.TrajectoryID,
                "trajectory",
                trajectoryID => APIUtils.ClientTrajectory.GetTrajectoryByIdAsync(trajectoryID, includeCalculatedStations: true));

            if (contextualData.SurveyRunID is Guid surveyRunID && surveyRunID != Guid.Empty)
            {
                if (trajectory.SurveyRunSectionList == null || !trajectory.SurveyRunSectionList.Any(section => section.SurveyRunID == surveyRunID))
                {
                    throw new Exception($"Survey run '{surveyRunID}' does not belong to trajectory '{contextualData.TrajectoryID}'.");
                }
                return (null, await LoadCalculatedSurveyRunAsync(surveyRunID, calculationType));
            }

            return (await LoadCalculatedTrajectoryAsync(trajectory, calculationType), null);
        }

        private static async Task<Trajectory> LoadCalculatedTrajectoryAsync(Trajectory trajectory, TrajectoryCalculationType calculationType)
        {
            if (HasStations(trajectory.SurveyStationList) && trajectory.CalculationType == calculationType)
            {
                return trajectory;
            }

            try
            {
                var surveyStations = await ConcatenateSurveyRunStationsAsync(trajectory, calculationType);

                Guid calculatedID = Guid.NewGuid();
                Trajectory calculatedTrajectory = new Trajectory
                {
                    MetaInfo = new MetaInfo { ID = calculatedID },
                    Name = "Calculated for simulation",
                    Description = trajectory.Description,
                    CreationDate = DateTimeOffset.UtcNow,
                    LastModificationDate = DateTimeOffset.UtcNow,
                    FieldID = trajectory.FieldID,
                    ClusterID = trajectory.ClusterID,
                    WellID = trajectory.WellID,
                    WellBoreID = trajectory.WellBoreID,
                    TrajectoryType = trajectory.TrajectoryType,
                    SurveyRunSectionList = trajectory.SurveyRunSectionList,
                    SurveyStationList = surveyStations,
                    TieInPoint = trajectory.TieInPoint ?? surveyStations[0],
                    CalculationType = calculationType,
                    MDStep = trajectory.MDStep
                };
                // Send calculation request
                await APIUtils.ClientTrajectory.PostTrajectoryAsync(calculatedTrajectory);
                // Load calculated trajectory
                calculatedTrajectory = await WaitForCalculationAsync(
                    () => APIUtils.ClientTrajectory.GetTrajectoryByIdAsync(calculatedID, includeCalculatedStations: true),
                    result => result.CalculationState,
                    result => result.CalculationMessage,
                    result => HasStations(result.SurveyStationList),
                    $"trajectory '{calculatedID}'");
                // Delete temporary trajectory from database
                if (calculatedTrajectory.LastModificationDate != null)
                {
                    await APIUtils.ClientTrajectory.DeleteTrajectoryByIdAsync(calculatedID, calculatedTrajectory.LastModificationDate.Value);
                }
                return calculatedTrajectory;
            }
            catch (Exception ex)
            {
                throw new Exception($"Failed to build calculated trajectory: {ex.Message}", ex);
            }
        }

        private static async Task<SurveyRun> LoadCalculatedSurveyRunAsync(Guid surveyRunID, TrajectoryCalculationType calculationType)
        {
            var surveyRun = await LoadRequiredAsync(
                surveyRunID,
                "survey run",
                id => APIUtils.ClientTrajectory.GetSurveyRunByIdAsync(id, includeCalculatedStations: true));
            if (HasStations(surveyRun.SurveyStationList) && surveyRun.CalculationType == calculationType)
            {
                return surveyRun;
            }

            try
            {
                var surveyRunWithMeasurements = await LoadRequiredAsync(
                    surveyRunID,
                    "survey run",
                    id => APIUtils.ClientTrajectory.GetSurveyRunByIdAsync(id, includeMeasurements: true));
                if (surveyRunWithMeasurements.SurveyMeasurementList == null || surveyRunWithMeasurements.SurveyMeasurementList.Count == 0)
                {
                    throw new Exception($"Survey run '{surveyRunID}' has neither survey stations calculated with {calculationType} nor survey measurements to calculate them from.");
                }

                Guid calculatedID = Guid.NewGuid();
                SurveyRun calculatedSurveyRun = new SurveyRun
                {
                    MetaInfo = new MetaInfo { ID = calculatedID },
                    Name = "Calculated for simulation",
                    Description = surveyRunWithMeasurements.Description,
                    CreationDate = DateTimeOffset.UtcNow,
                    LastModificationDate = DateTimeOffset.UtcNow,
                    FieldID = surveyRunWithMeasurements.FieldID,
                    ClusterID = surveyRunWithMeasurements.ClusterID,
                    WellID = surveyRunWithMeasurements.WellID,
                    WellBoreID = surveyRunWithMeasurements.WellBoreID,
                    SurveyInstrumentID = surveyRunWithMeasurements.SurveyInstrumentID,
                    SurveyRunType = surveyRunWithMeasurements.SurveyRunType,
                    ParentSurveyRunID = surveyRunWithMeasurements.ParentSurveyRunID,
                    TieInPoint = surveyRunWithMeasurements.TieInPoint,
                    SurveyMeasurementList = surveyRunWithMeasurements.SurveyMeasurementList,
                    SurveyStationList = null,
                    CalculationType = calculationType
                };
                // Send calculation request
                await APIUtils.ClientTrajectory.PostSurveyRunAsync(calculatedSurveyRun);
                // Load calculated survey run
                calculatedSurveyRun = await WaitForCalculationAsync(
                    () => APIUtils.ClientTrajectory.GetSurveyRunByIdAsync(calculatedID, includeCalculatedStations: true),
                    result => result.CalculationState,
                    result => result.CalculationMessage,
                    result => HasStations(result.SurveyStationList),
                    $"survey run '{calculatedID}'");
                // Delete temporary survey run from database
                if (calculatedSurveyRun.LastModificationDate != null)
                {
                    await APIUtils.ClientTrajectory.DeleteSurveyRunByIdAsync(calculatedID, calculatedSurveyRun.LastModificationDate.Value);
                }
                return calculatedSurveyRun;
            }
            catch (Exception ex)
            {
                throw new Exception($"Failed to build calculated survey run: {ex.Message}", ex);
            }
        }

        private static async Task<List<SurveyStation>> ConcatenateSurveyRunStationsAsync(Trajectory trajectory, TrajectoryCalculationType calculationType)
        {
            if (trajectory.SurveyRunSectionList == null || trajectory.SurveyRunSectionList.Count == 0)
            {
                throw new Exception($"Trajectory '{trajectory.MetaInfo?.ID}' has no survey run sections to calculate its survey stations from.");
            }

            var sections = trajectory.SurveyRunSectionList.OrderBy(section => section.StartAbscissa).ToList();
            var surveyStations = new List<SurveyStation>();
            for (int i = 0; i < sections.Count; i++)
            {
                // Each survey run is recalculated if needed, so that all stations use the requested calculation type
                var surveyRun = await LoadCalculatedSurveyRunAsync(sections[i].SurveyRunID, calculationType);

                double startAbscissa = sections[i].StartAbscissa;
                double endAbscissa = i + 1 < sections.Count ? sections[i + 1].StartAbscissa : double.PositiveInfinity;
                var sectionStations = surveyRun.SurveyStationList
                    .Where(station => GetStationAbscissa(station) != null)
                    .Where(station => GetStationAbscissa(station) >= startAbscissa && GetStationAbscissa(station) < endAbscissa)
                    .OrderBy(station => GetStationAbscissa(station));
                foreach (var station in sectionStations)
                {
                    // Skip duplicated stations at section boundaries
                    if (surveyStations.Count > 0 && Math.Abs(GetStationAbscissa(surveyStations[^1])!.Value - GetStationAbscissa(station)!.Value) < 1e-6)
                    {
                        continue;
                    }
                    surveyStations.Add(station);
                }
            }

            if (surveyStations.Count == 0)
            {
                throw new Exception($"No survey stations could be collected from the survey runs of trajectory '{trajectory.MetaInfo?.ID}'.");
            }
            return surveyStations;
        }

        private static bool HasStations(ICollection<SurveyStation>? stations)
        {
            return stations != null && stations.Count > 0;
        }

        private static double? GetStationAbscissa(SurveyStation station)
        {
            return station.Abscissa ?? station.MD;
        }

        private static async Task<T> WaitForCalculationAsync<T>(
            Func<Task<T>> reload,
            Func<T, CalculationState> state,
            Func<T, string?> message,
            Func<T, bool> hasStations,
            string label) where T : class
        {
            DateTime deadline = DateTime.UtcNow + TrajectoryCalculationTimeout;
            while (true)
            {
                var result = await reload();
                if (result == null)
                {
                    throw new Exception($"Calculated {label} could not be loaded from its microservice.");
                }
                var currentState = state(result);
                if (currentState == CalculationState.Failed)
                {
                    throw new Exception($"Calculation of {label} failed: {message(result)}");
                }
                if (currentState == CalculationState.Completed)
                {
                    if (!hasStations(result))
                    {
                        throw new Exception($"Calculated {label} has no survey stations.");
                    }
                    return result;
                }
                if (DateTime.UtcNow > deadline)
                {
                    throw new TimeoutException($"Calculation of {label} did not complete within {TrajectoryCalculationTimeout.TotalSeconds} s (state: {currentState}).");
                }
                await Task.Delay(TrajectoryCalculationPollInterval);
            }
        }

        private static async Task<GeothermalProperties?> LoadInterpolatedGeothermalProperties(Guid? id, Config? config) 
        {
            if (id == null || id == Guid.Empty)
            {
                return null;
            }
            GeothermalProperties? geothermalProperties = await APIUtils.ClientGeothermalProperties.GetGeothermalPropertiesByIdAsync(id.Value); 
            if (geothermalProperties != null)
            {
                //If the data is a RawData type, it means it was not completed.
                if (geothermalProperties.TableType == TableType.RawData)
                {
                    try
                    {
                        double defaultStep = config == null ? 30.0 : config.LengthBetweenLumpedElements ;
                        Guid completedID = Guid.NewGuid();
                        MetaInfo metaInfo = new MetaInfo
                        {
                            ID = completedID
                        };
                        GeothermalPropertiesCompletionOrder completedGeothermal = new GeothermalPropertiesCompletionOrder
                        {
                            MetaInfo = metaInfo,  
                            Name = "Completed for simulation",
                            CreationDate = DateTimeOffset.UtcNow,
                            LastModificationDate = DateTimeOffset.UtcNow,
                            ReferenceGeothermalProperties = geothermalProperties,
                            InterpolationStep = defaultStep
                        };                 
                        // Send interpolation order
                        await APIUtils.ClientGeothermalProperties.PostGeothermalPropertiesCompletionOrderAsync(completedGeothermal);
                        // Load interpolation order
                        completedGeothermal = await APIUtils.ClientGeothermalProperties.GetGeothermalPropertiesCompletionOrderByIdAsync(completedID);
                        // Delete old interpolation from database
                        await APIUtils.ClientGeothermalProperties.DeleteGeothermalPropertiesByIdAsync(completedGeothermal.CompletedGeothermalProperties.MetaInfo.ID);
                        return completedGeothermal.CompletedGeothermalProperties;
                    }   
                    catch 
                    {
                        throw new Exception ("Failed to fetch interpolated geothermal data!");
                    }
                }            
            }
            return geothermalProperties;
        }

        private static CasingSection? ResolveCasingSection(WellBoreArchitecture? wellBoreArchitecture, int? casingID)
        {
            if (wellBoreArchitecture?.CasingSections == null || casingID == null)
            {
                return null;
            }
            var casingSections = wellBoreArchitecture.CasingSections.ToList();
            return casingSections[(int) casingID];
        }

        private static double ResolveFluidDensity(DrillingFluidDescription drillingFluidDescription, double temperature, double surfacePipePressure)
        {
            var meanDensity = drillingFluidDescription.FluidMassDensity?.GaussianValue?.Mean;
            var fluidPvtParameters = ResolveFluidPvtParameters(drillingFluidDescription);
            if (fluidPvtParameters == null)
            {
                return meanDensity ?? 0.0;
            }

            var a0 = fluidPvtParameters.A0?.GaussianValue?.Mean;
            var b0 = fluidPvtParameters.B0?.GaussianValue?.Mean;
            var c0 = fluidPvtParameters.C0?.GaussianValue?.Mean;
            var d0 = fluidPvtParameters.D0?.GaussianValue?.Mean;
            var e0 = fluidPvtParameters.E0?.GaussianValue?.Mean;
            var f0 = fluidPvtParameters.F0?.GaussianValue?.Mean;
            if (a0 == null || b0 == null || c0 == null || d0 == null || e0 == null || f0 == null)
            {
                return meanDensity ?? 0.0;
            }

            return (double)a0
                + (double)b0 * temperature
                + (double)c0 * surfacePipePressure
                + (double)d0 * surfacePipePressure * temperature
                + (double)e0 * surfacePipePressure * surfacePipePressure
                + (double)f0 * surfacePipePressure * surfacePipePressure * temperature;
        }

        private static FluidPVTParameters? ResolveFluidPvtParameters(DrillingFluidDescription drillingFluidDescription)
        {
            if (drillingFluidDescription.FluidPVTParameters != null)
            {
                return drillingFluidDescription.FluidPVTParameters;
            }

            var composition = drillingFluidDescription.DrillingFluidComposition;
            var brineProperties = composition?.BrineProperies;
            var baseOilProperties = composition?.BaseOilProperies;
            if (brineProperties?.PVTParameters == null || baseOilProperties?.PVTParameters == null)
            {
                return null;
            }

            return CalculateFluidPvtParameters(
                brineProperties.PVTParameters,
                baseOilProperties.PVTParameters,
                brineProperties.MassFraction?.GaussianValue?.Mean,
                brineProperties.MassDensity?.GaussianValue?.Mean,
                baseOilProperties.MassFraction?.GaussianValue?.Mean,
                baseOilProperties.MassDensity?.GaussianValue?.Mean);
        }

        private static FluidPVTParameters? CalculateFluidPvtParameters(
            BrinePVTParameters brinePvtParameters,
            BaseOilPVTParameters baseOilPvtParameters,
            double? brineMassFraction,
            double? brineMass,
            double? baseOilMassFraction,
            double? baseOilMass)
        {
            if (
                baseOilPvtParameters.A0?.GaussianValue?.Mean == null ||
                baseOilPvtParameters.B0?.GaussianValue?.Mean == null ||
                baseOilPvtParameters.C0?.GaussianValue?.Mean == null ||
                baseOilPvtParameters.D0?.GaussianValue?.Mean == null ||
                baseOilPvtParameters.E0?.GaussianValue?.Mean == null ||
                baseOilPvtParameters.F0?.GaussianValue?.Mean == null ||
                brinePvtParameters.S0?.GaussianValue?.Mean == null ||
                brinePvtParameters.S1?.GaussianValue?.Mean == null ||
                brinePvtParameters.S2?.GaussianValue?.Mean == null ||
                brinePvtParameters.S3?.GaussianValue?.Mean == null ||
                brinePvtParameters.Bw?.GaussianValue?.Mean == null ||
                brinePvtParameters.Cw?.GaussianValue?.Mean == null ||
                brinePvtParameters.Dw?.GaussianValue?.Mean == null ||
                brinePvtParameters.Ew?.GaussianValue?.Mean == null ||
                brinePvtParameters.Fw?.GaussianValue?.Mean == null ||
                brineMassFraction == null ||
                brineMass == null ||
                baseOilMassFraction == null ||
                baseOilMass == null)
            {
                return null;
            }

            double baseOilVolume = (double)baseOilMass / (double)baseOilMassFraction;
            double brineVolume = (double)brineMass / (double)brineMassFraction;
            double totalVolume = baseOilVolume + brineVolume;
            if (totalVolume <= 0)
            {
                return null;
            }

            double aBaseOil = (double)baseOilPvtParameters.A0.GaussianValue.Mean;
            double bBaseOil = (double)baseOilPvtParameters.B0.GaussianValue.Mean;
            double cBaseOil = (double)baseOilPvtParameters.C0.GaussianValue.Mean;
            double dBaseOil = (double)baseOilPvtParameters.D0.GaussianValue.Mean;
            double eBaseOil = (double)baseOilPvtParameters.E0.GaussianValue.Mean;
            double fBaseOil = (double)baseOilPvtParameters.F0.GaussianValue.Mean;

            double aBrine = (double)(
                brinePvtParameters.S0.GaussianValue.Mean
                + brinePvtParameters.S1.GaussianValue.Mean * brineMassFraction
                + brinePvtParameters.S2.GaussianValue.Mean * brineMassFraction * brineMassFraction
                + brinePvtParameters.S3.GaussianValue.Mean * brineMassFraction * brineMassFraction * brineMassFraction);
            double bBrine = (double)brinePvtParameters.Bw.GaussianValue.Mean;
            double cBrine = (double)brinePvtParameters.Cw.GaussianValue.Mean;
            double dBrine = (double)brinePvtParameters.Dw.GaussianValue.Mean;
            double eBrine = (double)brinePvtParameters.Ew.GaussianValue.Mean;
            double fBrine = (double)brinePvtParameters.Fw.GaussianValue.Mean;

            return new FluidPVTParameters
            {
                A0 = new GaussianDrillingProperty { GaussianValue = new GaussianDistribution { Mean = (aBaseOil * baseOilVolume + aBrine * brineVolume) / totalVolume } },
                B0 = new GaussianDrillingProperty { GaussianValue = new GaussianDistribution { Mean = (bBaseOil * baseOilVolume + bBrine * brineVolume) / totalVolume } },
                C0 = new GaussianDrillingProperty { GaussianValue = new GaussianDistribution { Mean = (cBaseOil * baseOilVolume + cBrine * brineVolume) / totalVolume } },
                D0 = new GaussianDrillingProperty { GaussianValue = new GaussianDistribution { Mean = (dBaseOil * baseOilVolume + dBrine * brineVolume) / totalVolume } },
                E0 = new GaussianDrillingProperty { GaussianValue = new GaussianDistribution { Mean = (eBaseOil * baseOilVolume + eBrine * brineVolume) / totalVolume } },
                F0 = new GaussianDrillingProperty { GaussianValue = new GaussianDistribution { Mean = (fBaseOil * baseOilVolume + fBrine * brineVolume) / totalVolume } },
            };
        }

        private static double ResolveBitRadius(DrillString drillString)
        {
            var bitOuterDiameter = drillString.DrillStringSectionList?
                .SelectMany(section => section.SectionComponentList ?? Enumerable.Empty<DrillStringComponent>())
                .Where(component => component.Type == DrillStringComponentTypes.Bit)
                .SelectMany(component => component.PartList ?? Enumerable.Empty<DrillStringComponentPart>())
                .Select(part => Math.Max(part.OuterDiameter, part.OuterDiameterState2 ?? part.OuterDiameter))
                .DefaultIfEmpty(0.0)
                .Max() ?? 0.0;

            return bitOuterDiameter > 0.0 ? bitOuterDiameter / 2.0 : 0.0;
        }
    }
}
