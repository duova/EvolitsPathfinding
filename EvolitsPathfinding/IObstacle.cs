using System.Numerics;

namespace EvolitsPathfinding;

/// <summary>
/// An obstacle that can be baked into an IPathfindingMap with a IMapBaker for pathfinding.
/// </summary>
public interface IObstacle
{
    /// <summary>
    /// The position of the obstacle on a cartesian plane.
    /// </summary>
    Vector2 Position { get; protected set; }

    /// <summary>
    /// The positions of the corners of the polygon area <i>in order</i>.
    /// The last position is connected to the first position.
    /// </summary>
    IList<Vector2> Corners { get; protected set; }
}