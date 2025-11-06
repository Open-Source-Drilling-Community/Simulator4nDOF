using System.Text.Json.Nodes;

namespace NORCE.Drilling.Simulator4nDOF.Service.Mcp;

public sealed record McpHandshake(
    string ProtocolVersion,
    string? ClientName,
    string? ClientVersion,
    string? SessionId,
    JsonObject? Capabilities);
