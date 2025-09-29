namespace WebApp.Components
{
    public enum PanelState
    {
        MainVisible,
        NewSimulationVisible,
        EditSimulationVisible
    }
    public class MainPanelLogic
    {
        public bool AutoUpdate { get; set; } = false;
        public PanelState PanelState { get; set; } = PanelState.MainVisible;
        public Guid SimulationID { get; set; } = Guid.Empty;
    }
}
