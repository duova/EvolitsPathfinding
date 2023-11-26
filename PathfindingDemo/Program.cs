using System.Numerics;
using EvolitsPathfinding;

//Create a map to use for pathfinding.
IPathfindingMap pathfindingMap = new PathfindingMap(new MapBaker(), new Vector2(0f, 0f), new Vector2(25f, 25f), 0.5f, 5);

//Create obstacles that we intend to place in the map.
Vector2[] obstacleACorners = {new(0f, 0f), new(-2f, 4f), new(0f, 7f), new(-6f, 5f), new(4f, -4f), new(7f, 2f)};
Vector2[] obstacleBCorners = {new(0f, 0f), new(7f, 4f), new(-1f, 9f), new(-4f, 5f), new(0f, 7f), new(4f, 4f)};
IObstacle obstacleA = new PolygonObstacle(new Vector2(15f, 13f), obstacleACorners);
IObstacle obstacleB = new PolygonObstacle(new Vector2(9f, 4f), obstacleBCorners);

//Bake obstacles into map.
pathfindingMap.BakeObstacle(obstacleA);
pathfindingMap.BakeObstacle(obstacleB);

//Find path.
var path = pathfindingMap.GetPath(new Vector2(4f, 3f), new Vector2(16f, 17f));

//Print baked map with path. Note that this is rotated.
foreach (var yNodeList in pathfindingMap.Nodes.Reverse())
{
    var toPrint = "";
    foreach (var node in yNodeList.Reverse())
    {
        if (node.IsPath)
        {
            toPrint += "0 ";
        }
        else if (node.IsObstacle)
        {
            toPrint += "X ";
        }
        else
        {
            toPrint += "  ";
        }
    }
    Console.WriteLine(toPrint);
}