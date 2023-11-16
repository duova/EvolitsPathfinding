using System.Numerics;

namespace EvolitsPathfinding;

public struct Line
{
    public Vector2 PointA;
    public Vector2 PointB;

    public Line(Vector2 pointA, Vector2 pointB)
    {
        PointA = pointA;
        PointB = pointB;
    }
}

/// <summary>
/// Default implementation of an IMapBaker.
/// </summary>
public class MapBaker : IMapBaker
{
    public bool BakeObstacle(IObstacle obstacle, IPathfindingMap pathfindingMap)
    {
        //Get the bounding box of the obstacle to eliminate nodes that are definitely not within the area.
        
        Vector2 upperRightBoundingCorner;
        Vector2 bottomLeftBoundingCorner;
        upperRightBoundingCorner.X = float.MaxValue;
        upperRightBoundingCorner.Y = float.MaxValue;
        bottomLeftBoundingCorner.X = float.MinValue;
        bottomLeftBoundingCorner.Y = float.MinValue;

        foreach (var point in obstacle.Corners)
        {
            if (point.X > upperRightBoundingCorner.X)
            {
                upperRightBoundingCorner.X = point.X;
            }
            if (point.Y > upperRightBoundingCorner.Y)
            {
                upperRightBoundingCorner.Y = point.Y;
            }
            if (point.X < bottomLeftBoundingCorner.X)
            {
                bottomLeftBoundingCorner.X = point.X;
            }
            if (point.Y < bottomLeftBoundingCorner.Y)
            {
                bottomLeftBoundingCorner.Y = point.Y;
            }
        }

        //Check if bounding box is within the map.
        if (!IsWithinBoundingBoxInclusive(bottomLeftBoundingCorner, pathfindingMap.UpperRightBoundary,
                pathfindingMap.BottomLeftBoundary)) return false;
        if (!IsWithinBoundingBoxInclusive(upperRightBoundingCorner, pathfindingMap.UpperRightBoundary,
                pathfindingMap.BottomLeftBoundary)) return false;
        
        //Get nodes that are within the bounding box.
        var nodesWithinBounds = new List<MapNode>();
        foreach (var node in pathfindingMap.Nodes)
        {
            if (IsWithinBoundingBoxInclusive(node.Position, upperRightBoundingCorner, bottomLeftBoundingCorner))
            {
                nodesWithinBounds.Add(node);
            }
        }
        
        //In order to bake a map for a certain navigating object radius, we need to essentially trace the lines made
        //by the polygon points with a circle of the radius and make all nodes that it contacts non-traversable.
        //This way we find the area where the center point of the navigating object cannot traverse, which is easier to
        //do pathfinding calculations with.
        //We need to iterate through all the nodes in the bounding box to check and set each of them.
        var navRadiusSquared = pathfindingMap.NavigatorRadius * pathfindingMap.NavigatorRadius;
        //Save data about the line made by the points for reuse.
        List<Line> lines = new();
        for (var i = 0; i < obstacle.Corners.Count; i++)
        {
            var current = obstacle.Corners[i];
            var next = i + 1 < obstacle.Corners.Count ? obstacle.Corners[i + 1] : obstacle.Corners[0];
            lines.Add(new Line(current, next));
        }
        foreach (var node in nodesWithinBounds)
        {
            //First we know the node is in the polygon area if it is within the radius of any of the points.
            foreach (var point in obstacle.Corners)
            {
                if ((node.Position - point).LengthSquared() > navRadiusSquared) continue;
                node.IsObstacle = true;
                break;
            }
            
            //Then for the closest line on the polygon, we check if it is within the radius distance from the closest line.
            var closestLine = lines.MinBy(line => (line.PointA - node.Position).LengthSquared() + (line.PointB - node.Position).LengthSquared());
            //Check distance and make non-traversable if close enough.
            if (GetSquaredDistanceOfPointFromLine(closestLine, node.Position) <= navRadiusSquared)
            {
                node.IsObstacle = true;
            }
        }

        return true;
    }

    private bool IsWithinBoundingBoxInclusive(Vector2 position, Vector2 boxTopRight, Vector2 boxBottomLeft)
    {
        if (position.X > boxTopRight.X) return false;
        if (position.Y > boxTopRight.Y) return false;
        if (position.X < boxBottomLeft.X) return false;
        if (position.Y < boxBottomLeft.Y) return false;
        return true;
    }

    private float GetSquaredDistanceOfPointFromLine(Line line, Vector2 point)
    {
        
    }
}