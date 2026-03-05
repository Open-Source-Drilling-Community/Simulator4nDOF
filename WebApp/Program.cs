using MudBlazor;
using MudBlazor.Services;

var builder = WebApplication.CreateBuilder(args);

// Add services to the container.
builder.Services.AddRazorPages();
builder.Services.AddServerSideBlazor();
builder.Services.AddMudServices(config =>
{
    config.SnackbarConfiguration.PositionClass = Defaults.Classes.Position.BottomLeft;
    config.SnackbarConfiguration.PreventDuplicates = false;
    config.SnackbarConfiguration.NewestOnTop = false;
    config.SnackbarConfiguration.ShowCloseIcon = true;
    config.SnackbarConfiguration.VisibleStateDuration = 5000;
    config.SnackbarConfiguration.HideTransitionDuration = 500;
    config.SnackbarConfiguration.ShowTransitionDuration = 500;
    config.SnackbarConfiguration.SnackbarVariant = Variant.Filled;
});

var app = builder.Build();

app.UseForwardedHeaders();
// This needs to match with what is defined in "charts/<helm-chart-name>/templates/values.yaml ingress.Path
app.UsePathBase("/Simulator4nDOF/webapp");

if (!String.IsNullOrEmpty(builder.Configuration["Simulator4nDOFHostURL"]))
    NORCE.Drilling.Simulator4nDOF.WebApp.Configuration.Simulator4nDOFHostURL = builder.Configuration["Simulator4nDOFHostURL"];
if (!String.IsNullOrEmpty(builder.Configuration["WellBoreHostURL"]))
    NORCE.Drilling.Simulator4nDOF.WebApp.Configuration.WellBoreHostURL = builder.Configuration["WellBoreHostURL"];
if (!String.IsNullOrEmpty(builder.Configuration["FieldHostURL"]))
    NORCE.Drilling.Simulator4nDOF.WebApp.Configuration.FieldHostURL = builder.Configuration["FieldHostURL"];
if (!String.IsNullOrEmpty(builder.Configuration["ClusterHostURL"]))
    NORCE.Drilling.Simulator4nDOF.WebApp.Configuration.ClusterHostURL = builder.Configuration["ClusterHostURL"];
if (!String.IsNullOrEmpty(builder.Configuration["WellHostURL"]))
    NORCE.Drilling.Simulator4nDOF.WebApp.Configuration.WellHostURL = builder.Configuration["WellHostURL"];
if (!String.IsNullOrEmpty(builder.Configuration["UnitConversionHostURL"]))
    NORCE.Drilling.Simulator4nDOF.WebApp.Configuration.UnitConversionHostURL = builder.Configuration["UnitConversionHostURL"];
if (!String.IsNullOrEmpty(builder.Configuration["DrillStringHostURL"]))
    NORCE.Drilling.Simulator4nDOF.WebApp.Configuration.DrillStringHostURL = builder.Configuration["DrillStringHostURL"];
if (!String.IsNullOrEmpty(builder.Configuration["TrajectoryHostURL"]))
    NORCE.Drilling.Simulator4nDOF.WebApp.Configuration.TrajectoryHostURL = builder.Configuration["TrajectoryHostURL"];
if (!String.IsNullOrEmpty(builder.Configuration["WellBoreArchitectureHostURL"]))
    NORCE.Drilling.Simulator4nDOF.WebApp.Configuration.WellBoreArchitectureHostURL = builder.Configuration["WellBoreArchitectureHostURL"];
if (!String.IsNullOrEmpty(builder.Configuration["DrillingFluidHostURL"]))
    NORCE.Drilling.Simulator4nDOF.WebApp.Configuration.DrillingFluidHostURL = builder.Configuration["DrillingFluidHostURL"];



// Configure the HTTP request pipeline.
if (!app.Environment.IsDevelopment())
{
    app.UseExceptionHandler("/Error");
    // The default HSTS value is 30 days. You may want to change this for production scenarios, see https://aka.ms/aspnetcore-hsts.
    app.UseHsts();
}

app.UseHttpsRedirection();

app.UseStaticFiles();

app.UseRouting();

app.MapBlazorHub();
app.MapFallbackToPage("/_Host");

app.Run();
