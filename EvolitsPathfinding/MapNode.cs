using System.Numerics;

namespace EvolitsPathfinding;

public class MapNode
{
    /// <summary>
    /// Position of the node on the cartesian plane.
    /// </summary>
    public Vector2 Position { get; private set; }
    
    /// <summary>
    /// Index of the node in the IPathfindingMap's outer list.
    /// </summary>
    public int IndexX { get; private set; }
    
    /// <summary>
    /// Index of the node in the IPathfindingMap's inner list.
    /// </summary>
    public int IndexY { get; private set; }

    /// <summary>
    /// Whether the node is traversable terrain.
    /// </summary>
    public bool IsObstacle { get; set; }

    public float Cost { get; set; }

    public float DistanceFromStart { get; set; }

    public float Heuristic { get; set; }

    public MapNode? Parent { get; set; }

    public bool IsPath { get; set; }

    public MapNode(Vector2 position, int indexX, int indexY)
    {
        Position = position;
        IndexX = indexX;
        IndexY = indexY;
        IsObstacle = false;
    }
}