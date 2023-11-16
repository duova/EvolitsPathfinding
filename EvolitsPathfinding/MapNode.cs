using System.Numerics;

namespace EvolitsPathfinding;

public class MapNode
{
    /// <summary>
    /// Position of the Node on the IMap.
    /// </summary>
    public Vector2 Position { get; private set; }

    /// <summary>
    /// Whether the INode is traversable terrain.
    /// </summary>
    public bool IsObstacle { get; set; }

    /// <summary>
    /// The nodes that surround this node with zero index being north (positive y) and going clockwise.
    /// </summary>
    public MapNode[] SurroundingNodes { get; set; } = new MapNode[8];

    public MapNode(Vector2 position)
    {
        Position = position;
        IsObstacle = false;
    }
}