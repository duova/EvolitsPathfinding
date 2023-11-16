namespace EvolitsPathfinding;

/// <summary>
/// An object in an IPathfindingMap used to bake an IObstacle into the IPathfindingMap.
/// </summary>
public interface IMapBaker
{
    /// <summary>
    /// Bakes the IObstacle into the IPathfindingMap.
    /// </summary>
    /// <param name="obstacle">IObstacle to bake into the IPathfindingMap.</param>
    /// <param name="pathfindingMap">IPathfindingMap to bake into.</param>
    /// <returns>Whether the IObstacle was baked. If false the IObstacle was out of bounds of the map.</returns>
    bool BakeObstacle(IObstacle obstacle, IPathfindingMap pathfindingMap);
}