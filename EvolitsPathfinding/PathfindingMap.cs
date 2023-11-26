using System.Numerics;

namespace EvolitsPathfinding;

/// <summary>
/// Default implementation of a IPathfindingMap.
/// </summary>
public class PathfindingMap : IPathfindingMap
{
    public IMapBaker Baker { get; set; }
    
    public Vector2 BottomLeftBoundary { get; set; }
    
    public Vector2 UpperRightBoundary { get; set; }
    
    public float NavigatorRadius { get; set; }

    public IReadOnlyList<IReadOnlyList<MapNode>> Nodes => _nodes;

    private List<List<MapNode>> _nodes = new();
    
    public int ObstaclesBaked { get; set; }

    private readonly int _resolution;
    
    private static readonly float RootTwo = MathF.Sqrt(2f);

    /// <summary>
    /// Constructs a map that can be used to produce paths for a navigating agent of a certain radius.
    /// IObstacles can be baked in to represent obstacles the navigator must navigate around.
    /// </summary>
    /// <param name="baker">IMapBaker used to bake IObstacles into the map.</param>
    /// <param name="origin">Bottom-left coordinate of the map.</param>
    /// <param name="size">Size of the map in the same unit as the coordinates.</param>
    /// <param name="navigatorRadius">Radius of the navigating agent in the same unit as coordinates.</param>
    /// <param name="resolution">Number of nodes per unit in one dimension. For example, a resolution of 5 will have 25 nodes in a square unit.</param>
    public PathfindingMap(IMapBaker baker, Vector2 origin, Vector2 size, float navigatorRadius, int resolution)
    {
        Baker = baker;
        BottomLeftBoundary = origin;
        UpperRightBoundary = origin + size;
        NavigatorRadius = navigatorRadius;
        ObstaclesBaked = 0;
        InitializeNodes(resolution);
        _resolution = resolution;
    }
    
    public IEnumerable<Vector2> GetPath(Vector2 startPoint, Vector2 endPoint)
    {
        List<MapNode> openList = new();
        List<MapNode> closedList = new();
        var halfInverseResolution = 1f / _resolution * 2f;
        
        //Reset nodes.
        foreach (var node in _nodes.SelectMany(list => list))
        {
            node.Cost = 0;
            node.DistanceFromStart = 0;
            node.Heuristic = 0;
            node.IsPath = false;
        }
        
        //Find starting node.
        MapNode? startingNode = null;
        for (var x = 0; x < _nodes.Count; x++)
        {
            if (_nodes[x].First().Position.X > startPoint.X + halfInverseResolution) continue;
            if (_nodes[x].First().Position.X < startPoint.X - halfInverseResolution) continue;
            for (var y = 0; y < _nodes[x].Count; y++)
            {
                if (_nodes[x][y].Position.Y > startPoint.Y + halfInverseResolution) continue;
                if (_nodes[x][y].Position.Y < startPoint.Y - halfInverseResolution) continue;
                startingNode = _nodes[x][y];
                break;
            }

            if (startingNode != null) break;
        }
        
        //Find target node.
        MapNode? targetNode = null;
        for (var x = 0; x < _nodes.Count; x++)
        {
            if (_nodes[x].First().Position.X > endPoint.X + halfInverseResolution) continue;
            if (_nodes[x].First().Position.X < endPoint.X - halfInverseResolution) continue;
            for (var y = 0; y < _nodes[x].Count; y++)
            {
                if (_nodes[x][y].Position.Y > endPoint.Y + halfInverseResolution) continue;
                if (_nodes[x][y].Position.Y < endPoint.Y - halfInverseResolution) continue;
                targetNode = _nodes[x][y];
                break;
            }

            if (targetNode != null) break;
        }

        if (startingNode == null || targetNode == null || startingNode == targetNode || startingNode.IsObstacle || targetNode.IsObstacle)
        {
            //Start or target is outside of map, in an obstacle, or are the same node.
            return new List<Vector2>();
        }
        
        //Calculate variables for starting node.
        openList.Add(startingNode);
        startingNode.DistanceFromStart = 0;
        startingNode.Heuristic = new Vector2(targetNode.IndexX - startingNode.IndexX,
            targetNode.IndexY - startingNode.IndexY).LengthSquared();
        startingNode.Cost = startingNode.Heuristic;

        while (openList.Count > 0)
        {
            //We will go from the node with the lowest cost, so we can switch that to the closed list as we're
            //going to be done with it after.
            var currentNode = openList.OrderBy(node => node.Cost).First();
            if (currentNode == targetNode) break;
            openList.Remove(currentNode);
            closedList.Add(currentNode);

            //Calculate costs for adjacent nodes.
            var adjacentNodes = GetAdjacentNodes(currentNode.IndexX, currentNode.IndexY);
            for (var i = 0; i < adjacentNodes.Length; i++)
            {
                var adjNode = adjacentNodes[i];
                if (adjNode == null) continue;
                if (adjNode.IsObstacle) continue;
                if (closedList.Contains(adjNode)) continue;
                if (!openList.Contains(adjNode))
                {
                    //Add to open list and calculate costs.
                    openList.Add(adjNode);
                    adjNode.Parent = currentNode;
                    //Use root 2 for diagonal distances.
                    adjNode.DistanceFromStart = currentNode.DistanceFromStart + (i % 2 == 1 ? RootTwo : 1);
                    adjNode.Heuristic = new Vector2(targetNode.IndexX - adjNode.IndexX,
                        targetNode.IndexY - adjNode.IndexY).LengthSquared();
                    adjNode.Cost = adjNode.DistanceFromStart + adjNode.Heuristic;
                }
                else
                {
                    //Reroute if an adjacent open node is easier to reach through the current node.
                    if (adjNode.DistanceFromStart <= currentNode.DistanceFromStart) continue;
                    adjNode.Parent = currentNode;
                    adjNode.DistanceFromStart = currentNode.DistanceFromStart + (i % 2 == 1 ? RootTwo : 1);
                    adjNode.Cost = adjNode.DistanceFromStart + adjNode.Heuristic;
                }
            }
        }

        //Backtrack to get route.
        var retVal = new List<Vector2>();
        if (!openList.Contains(targetNode)) return retVal;
        var iteratedNode = targetNode;
        while (iteratedNode != startingNode && iteratedNode != null)
        {
            //We only add path vectors that are contacting obstacles as they are the points that matter to the path.
            if (iteratedNode == targetNode || GetAdjacentNodes(iteratedNode.IndexX, iteratedNode.IndexY).Any(node => node is { IsObstacle: true }))
            {
                retVal.Add(iteratedNode.Position);
                iteratedNode.IsPath = true;
            }

            iteratedNode = iteratedNode.Parent;
        }
        retVal.Add(startingNode.Position);
        startingNode.IsPath = true;
        retVal.Reverse();
        return retVal;
    }

    public bool BakeObstacle(IObstacle obstacle)
    {
        if (!Baker.BakeObstacle(obstacle, this)) return false;
        ObstaclesBaked++;
        return true;
    }

    private void InitializeNodes(int resolution)
    {
        if (resolution <= 0)
            throw new Exception("Tried to create pathfinding map with a resolution of less or equal to 0.");
        var inverseResolution = 1f / resolution;
        //Create nodes based on the resolution.
        var indexX = 0;
        for (var x = BottomLeftBoundary.X; x < UpperRightBoundary.X; x += inverseResolution)
        {
            _nodes.Add(new List<MapNode>());
            var indexY = 0;
            for (var y = BottomLeftBoundary.Y; y < UpperRightBoundary.Y; y += inverseResolution)
            {
                _nodes.Last().Add(new MapNode(new Vector2(x, y), indexX, indexY));
                indexY++;
            }
            indexX++;
        }
    }
    
    /// <summary>
    /// Gets the 8 nodes adjacent to the list index specified. Starts at north (0, 1) and continues clockwise.
    /// </summary>
    /// <param name="x">X or outer loop index of the node.</param>
    /// <param name="y">Y or inner loop index of the node.</param>
    /// <returns></returns>
    private MapNode?[] GetAdjacentNodes(int x, int y)
    {
        if (x < 0 || y < 0 || x >= _nodes.Count || y >= _nodes.First().Count) throw new Exception("Tried to get adjacent nodes of a node that doesn't exist.");
        var retVal = new MapNode?[8];
        var topEdgeExists = y + 1 < _nodes.First().Count;
        var leftEdgeExists = x > 0;
        var rightEdgeExists = x + 1 < _nodes.Count;
        var bottomEdgeExists = y > 0;
        retVal[0] = topEdgeExists ? _nodes[x][y + 1] : null;
        retVal[1] = topEdgeExists && rightEdgeExists ? _nodes[x + 1][y + 1] : null;
        retVal[2] = rightEdgeExists ? _nodes[x + 1][y] : null;
        retVal[3] = bottomEdgeExists && rightEdgeExists ? _nodes[x + 1][y - 1] : null;
        retVal[4] = bottomEdgeExists ? _nodes[x][y - 1] : null;
        retVal[5] = bottomEdgeExists && leftEdgeExists ? _nodes[x - 1][y - 1] : null;
        retVal[6] = leftEdgeExists ? _nodes[x - 1][y] : null;
        retVal[7] = topEdgeExists && leftEdgeExists ? _nodes[x - 1][y + 1] : null;
        return retVal;
    }
}