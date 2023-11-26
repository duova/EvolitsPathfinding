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
        //We translate the corner coordinates of the obstacle by the position to get corner coordinates relative to the map.

        var cornersInMapSpace = obstacle.Corners.Select(corner => corner + obstacle.Position).ToList();
        
        //Get the bounding box of the obstacle to eliminate nodes that are definitely not within the area.
        
        Vector2 upperRightBoundingCorner;
        Vector2 bottomLeftBoundingCorner;
        upperRightBoundingCorner.X = float.MinValue;
        upperRightBoundingCorner.Y = float.MinValue;
        bottomLeftBoundingCorner.X = float.MaxValue;
        bottomLeftBoundingCorner.Y = float.MaxValue;

        foreach (var point in cornersInMapSpace)
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
        
        //Extend by the navigator radius as we bake the navigator radius into the map instead of calculating during pathfinding.
        upperRightBoundingCorner += new Vector2(pathfindingMap.NavigatorRadius, pathfindingMap.NavigatorRadius);
        bottomLeftBoundingCorner -= new Vector2(pathfindingMap.NavigatorRadius, pathfindingMap.NavigatorRadius);

        //Check if bounding box is within the map.
        if (!IsWithinBoundingBoxInclusive(bottomLeftBoundingCorner, pathfindingMap.UpperRightBoundary,
                pathfindingMap.BottomLeftBoundary)) return false;
        if (!IsWithinBoundingBoxInclusive(upperRightBoundingCorner, pathfindingMap.UpperRightBoundary,
                pathfindingMap.BottomLeftBoundary)) return false;
        
        //Get nodes that are within the bounding box.
        var nodesWithinBounds = new List<MapNode>();
        foreach (var node in pathfindingMap.Nodes.SelectMany(list => list))
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
        for (var i = 0; i < cornersInMapSpace.Count; i++)
        {
            var current = cornersInMapSpace[i];
            var next = i + 1 < cornersInMapSpace.Count ? cornersInMapSpace[i + 1] : cornersInMapSpace[0];
            lines.Add(new Line(current, next));
        }
        foreach (var node in nodesWithinBounds)
        {
            //First we know the node is in the polygon area if it is within the radius of any of the points.
            foreach (var point in cornersInMapSpace)
            {
                if ((node.Position - point).LengthSquared() > navRadiusSquared) continue;
                node.IsObstacle = true;
                break;
            }
            
            //Then for the each line on the polygon, we check if it is within the radius distance of the node.
            foreach (var line in lines)
            {
                //Check if node is on a tangent to the line.
                var lineVector = line.PointB - line.PointA;
                var tangentVector = new Vector2(-lineVector.Y, lineVector.X);
                var pointAVector = node.Position - line.PointA;
                var pointBVector = node.Position - line.PointB;
                if (CrossProduct(new Vector3(tangentVector, 0), new Vector3(pointAVector, 0)).Z > 0) continue;
                if (CrossProduct(new Vector3(tangentVector, 0), new Vector3(pointBVector, 0)).Z < 0) continue;
                //Check distance and make non-traversable if close enough.
                if (GetSquaredDistanceOfPointFromLine(line, node.Position) > navRadiusSquared) continue;
                node.IsObstacle = true;
                //Don't need to keep checking if it is already inside a line.
                break;
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
        var lineVector = line.PointB - line.PointA;
        var sideVector = point - line.PointA;
        var crossProduct = CrossProduct(new Vector3(lineVector, 0), new Vector3(sideVector, 0));
        //Cross product for area / base length of parallelogram.
        return crossProduct.LengthSquared() / lineVector.LengthSquared();
    }
    
    public static Vector3 CrossProduct(Vector3 v1, Vector3 v2)
    {
        var x = v1.Y * v2.Z - v2.Y * v1.Z;
        var y = (v1.X * v2.Z - v2.X * v1.Z) * -1;
        var z = v1.X * v2.Y - v2.X * v1.Y;

        return new Vector3(x, y, z);
    }
}