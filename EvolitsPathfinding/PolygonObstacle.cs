using System.Numerics;

namespace EvolitsPathfinding;

/// <summary>
/// Simple polygon implementation of a pathfinding obstacle.
/// </summary>
public class PolygonObstacle : IObstacle
{
    public Vector2 Position { get; set; }
    public IList<Vector2> Corners { get; set; }

    public PolygonObstacle(Vector2 position, IList<Vector2> corners)
    {
        Position = position;
        Corners = corners;
    }
}