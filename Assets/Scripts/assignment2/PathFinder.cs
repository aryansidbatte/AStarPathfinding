using UnityEngine;
using System.Collections.Generic;

// Internal class for A* entries
private class AStarEntry
{
    public GraphNode node;
    public float g;              // cost from start to this entry
    public float f;              // g + heuristic
    public Vector3 position;     // last waypoint (start center or wall midpoint)
    public AStarEntry parent;    // previous entry in chain
    public Wall cameThrough;     // wall crossed to reach this node

    public AStarEntry(GraphNode node, float g, float f, Vector3 position, AStarEntry parent)
    {
        this.node = node;
        this.g = g;
        this.f = f;
        this.position = position;
        this.parent = parent;
        this.cameThrough = cameThrough;
    }
}


public class PathFinder : MonoBehaviour
{
    private int inf = -1;


    private float getCost(GraphNode node, GraphNode start, GraphNode destination, Vector3 target)
    {
        Vector3 dirToTarget = target - node.GetCenter();
        Vector3 dirToStart = start.GetCenter() - node.GetCenter();
        return dirToTarget.Normalize() + dirToStart.Normalize()
    }




    // Assignment 2: Implement AStar
    //
    // DO NOT CHANGE THIS SIGNATURE (parameter types + return type)
    // AStar will be given the start node, destination node and the target position, and should return 
    // a path as a list of positions the agent has to traverse to reach its destination, as well as the
    // number of nodes that were expanded to find this path
    // The last entry of the path will be the target position, and you can also use it to calculate the heuristic
    // value of nodes you add to your search frontier; the number of expanded nodes tells us if your search was
    // efficient
    //
    // Take a look at StandaloneTests.cs for some test cases
    public static (List<Vector3>, int) AStar(GraphNode start, GraphNode destination, Vector3 target)
    {
        // Implement A* here
        List<Vector3> path = new List<Vector3>() { target };
        GraphNode parent = null;
        List<AStarEntry> frontier = new List<AStarEntry>() { start.GetID() };
        HashSet<AStarEntry> closed = new HashSet<AStarEntry>();
        Dictionary<AStarEntry,float> costs = new Dictionary<AStarEntry,float>();
        int expansions;

        // make first entry
        Vector3 startPos = start.GetCenter()
        float h0 = (target.GetCenter() - start.GetCenter()).Normalize();
        AStarEntry startEntry = new AStarEntry(start, 0f, h0, startPos, null);
        costs[start.GetID()] = 0f;

        while (frontier.Count > 0)
        {
            AStarEntry current = frontier[0];
            frontier.RemoveAt(0);

            if (frontier[0] == destination.GetID())
            {
                return (ReconstructPath(current, target), expansions);
            }

            closed.Add(current)
            expansions++;

            List<GraphNeighbors> neighbors = current.node.GetNeighbors();
            for (int i = 0; i < neighbors.Count; i++){
                GraphNode nextNode = neighbors[i].GetNode();
                int nextID = nextNode.GetID();
                // this line is supposed to look for next's ID in 
                // closed, but closed now stores AStarEntry instead of
                // IDs (int)
                // i don't think we can make a new AStarEntry 
                // for nextNode at this step because we haven't calculated
                // the f and g values yet
                // it seems like we should go back to having closed
                // store IDs and just find a way to map back from IDs
                // to AStarEntry for line 78
                if (closed.Contains())
            }
        }
        

         else 
        {

        }

        // return path and number of nodes expanded
        return (path, 0);

    }

    private static List<Vector3> ReconstructPath(AStarEntry endEntry, Vector target)
    {
    List<Vector3> path = new List<Vector3>();
    AStarEntry e = endEntry;
    while (e.parent != null)
    {
        path.Add(e.position);
        e = e.parent;
    }
    path.Reverse();
    path.Add(target);
    return path;
    }

    public Graph graph;
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        EventBus.OnTarget += PathFind;
        EventBus.OnSetGraph += SetGraph;
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    public void SetGraph(Graph g)
    {
        graph = g;
    }

    // entry point
    public void PathFind(Vector3 target)
    {
        if (graph == null) return;

        // find start and destination nodes in graph
        GraphNode start = null;
        GraphNode destination = null;
        foreach (var n in graph.all_nodes)
        {
            if (Util.PointInPolygon(transform.position, n.GetPolygon()))
            {
                start = n;
            }
            if (Util.PointInPolygon(target, n.GetPolygon()))
            {
                destination = n;
            }
        }
        if (destination != null)
        {
            // only find path if destination is inside graph
            EventBus.ShowTarget(target);
            (List<Vector3> path, int expanded) = PathFinder.AStar(start, destination, target);

            Debug.Log("found path of length " + path.Count + " expanded " + expanded + " nodes, out of: " + graph.all_nodes.Count);
            EventBus.SetPath(path);
        }
    }

    

 
}
