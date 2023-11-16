using System.Numerics;

namespace EvolitsPathfinding;

/// <summary>
/// A map with IObstacles baked in with the IMapBaker to produce routes for a navigating object of a certain radius.
/// </summary>
public interface IPathfindingMap
{
    /// <summary>
    /// IMapBaker used for this map.
    /// </summary>
    IMapBaker Baker { get; protected set; }

    /// <summary>
    /// Coordinates of the most bottom-left point. Or the point with the lowest X and Y.
    /// </summary>
    Vector2 BottomLeftBoundary { get; protected set; }

    /// <summary>
    /// Coordinates of the most upper-right point. Or the point with the highest X and Y.
    /// </summary>
    Vector2 UpperRightBoundary { get; protected set; }

    /// <summary>
    /// The radius of the object navigating through a path calculated with this map.
    /// This is baked into the map for faster calculations.
    /// </summary>
    float NavigatorRadius { get; protected set; }

    /// <summary>
    /// INodes that make up the map. Ordered from left to right, then top to bottom.
    /// </summary>
    IReadOnlyList<MapNode> Nodes { get; }
    
    /// <summary>
    /// Number of IObstacles baked into the map.
    /// </summary>
    int ObstaclesBaked { get; protected set; }

    /// <summary>
    /// Calculates a path for this IPathfindingMap.
    /// </summary>
    /// <param name="startPoint">Starting point of the path.</param>
    /// <param name="endPoint">Ending point of the path.</param>
    /// <returns>List of points on the path in order.</returns>
    IList<Vector2> GetPath(Vector2 startPoint, Vector2 endPoint);

    /// <summary>
    /// Bakes an IObstacle into this IPathfindingMap with the current IMapBaker.
    /// </summary>
    /// <param name="obstacle">IObstacle to bake.</param>
    /// <returns>Whether baking succeeded.</returns>
    bool BakeObstacle(IObstacle obstacle);
    
    string Serialize();

    string Deserialize();
}