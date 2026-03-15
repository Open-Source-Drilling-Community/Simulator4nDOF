FROM mcr.microsoft.com/dotnet/aspnet:8.0 AS base
WORKDIR /app
EXPOSE 8080
EXPOSE 443

FROM mcr.microsoft.com/dotnet/sdk:8.0 AS build

WORKDIR /src
COPY ["Service/Service.csproj", "Service/"]
RUN dotnet restore "Service/Service.csproj"

COPY . .
WORKDIR "/src/Service"
RUN dotnet build "Service.csproj" -c Release -o /app

FROM build AS publish
RUN dotnet publish "Service.csproj" -c Release -o /app

FROM base AS final
WORKDIR /app
VOLUME /home
COPY --from=publish /app .
ENTRYPOINT ["dotnet", "Service.dll"]
