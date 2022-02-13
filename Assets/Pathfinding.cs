/* 
 Yasmine Najd
 References: Sebastian Lague youtube channel and github code "https://github.com/SebLague/Pathfinding"
 */


using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using Debug = UnityEngine.Debug;
using System.Runtime.InteropServices;

public class Pathfinding : MonoBehaviour
{

    public Transform seeker, target;
    public Transform seekerBFS, targetBFS;
    public Transform seekerDFS, targetDFS;
    public Transform seekerUCS, targetUCS;
    Grid grid;
    public int memoryUCS = 0;
    public int memoryBFS = 0;
    public int memoryDFS = 0;
    public int memory = 0;
    public Stopwatch timerA = new Stopwatch();
    public Stopwatch timerUCS = new Stopwatch();
    public Stopwatch timerBFS = new Stopwatch();
    public Stopwatch timerDFS = new Stopwatch();
    void Awake()
    {
        grid = GetComponent<Grid>();
    }

    void Update()
    {
        FindPath(seeker.position, target.position);
        BFSFindPath(seekerBFS.position, targetBFS.position);
        DFSFindPath(seekerDFS.position, targetDFS.position);
        UCSFindPath(seekerUCS.position, targetUCS.position);
    }

    void Start()
    { 
        FindPath(seeker.position, target.position);
        Debug.Log("A* runtime:" + timerA.Elapsed.ToString() + "The memory used to find the path:" + memory);
        BFSFindPath(seekerBFS.position, targetBFS.position);
        Debug.Log("BFS runtime:" + timerBFS.Elapsed.ToString() + "The memory used to find the path:" + memoryBFS);
        DFSFindPath(seekerDFS.position, targetDFS.position);
        Debug.Log("DFS runtime:" + timerDFS.Elapsed.ToString() + "The memory used to find the path:" + memoryDFS);
        UCSFindPath(seekerUCS.position, targetUCS.position);
        Debug.Log("UCS runtime:" + timerUCS.Elapsed.ToString() + "The memory used to find the path:" + memoryUCS);

    }

    void FindPath(Vector3 startPos, Vector3 targetPos)
    {
        Node startNode = grid.NodeFromWorldPoint(startPos);
        Node targetNode = grid.NodeFromWorldPoint(targetPos);

        List<Node> openSet = new List<Node>();
        HashSet<Node> closedSet = new HashSet<Node>();
        timerA.Start();
        openSet.Add(startNode);

        while (openSet.Count > 0)
        {
            Node node = openSet[0];
            for (int i = 1; i < openSet.Count; i++)
            {
                if (openSet[i].fCost < node.fCost || openSet[i].fCost == node.fCost)
                {
                    if (openSet[i].hCost < node.hCost)
                        node = openSet[i];
                }
            }

            openSet.Remove(node);
            closedSet.Add(node);

            if (node == targetNode)
            {
                RetracePath(startNode, targetNode, ref memory);
                timerA.Stop();
                return;
            }

            foreach (Node neighbour in grid.GetNeighbours(node))
            {
                if (!neighbour.walkable || closedSet.Contains(neighbour))
                {
                    continue;
                }

                int newCostToNeighbour = node.gCost + GetDistance(node, neighbour);
                if (newCostToNeighbour < neighbour.gCost || !openSet.Contains(neighbour))
                {
                    neighbour.gCost = newCostToNeighbour;
                    neighbour.hCost = GetDistance(neighbour, targetNode);
                    neighbour.parent = node;

                    if (!openSet.Contains(neighbour))
                        openSet.Add(neighbour);
                }
            }
        }
        timerA.Stop();
    }

    void BFSFindPath(Vector3 startPos, Vector3 targetPos)
    {
        Node startNode = grid.NodeFromWorldPoint(startPos);
        Node targetNode = grid.NodeFromWorldPoint(targetPos);

        Queue<Node> queue = new Queue<Node>();
        HashSet<Node> Explored = new HashSet<Node>();
        Explored.Add(startNode);
        timerBFS.Start();

        queue.Enqueue(startNode);

        while (queue.Count != 0)
        {
            Node currentNode = queue.Dequeue();
            if (currentNode == targetNode)
            {
                RetracePath(startNode, targetNode, ref memoryBFS);
                timerBFS.Stop();
                return;
            }
            Explored.Add(currentNode);
            foreach (Node neighbour in grid.GetNeighbours(currentNode))
            {
                if (!neighbour.walkable || Explored.Contains(neighbour))
                {
                    continue;
                }
                if (neighbour.walkable || !queue.Contains(neighbour))
                {
                    Explored.Add(neighbour);
                    neighbour.parent = currentNode;
                    queue.Enqueue(neighbour);
                }
            }

        }
        timerBFS.Stop();

    }

    void DFSFindPath(Vector3 startPos, Vector3 targetPos)
    {
        Node startNode = grid.NodeFromWorldPoint(startPos);
        Node targetNode = grid.NodeFromWorldPoint(targetPos);

        Stack<Node> stack = new Stack<Node>();
        HashSet<Node> Explored = new HashSet<Node>();
        timerDFS.Start();
        stack.Push(startNode);

        while (stack.Count != 0)
        {
            Node currentNode = stack.Pop();

            if (currentNode == targetNode)
            {
                RetracePath(startNode, targetNode, ref memoryDFS);
                timerDFS.Stop();
                return;
            }
            Explored.Add(currentNode);
            foreach (Node neighbour in grid.GetNeighbours(currentNode))
            {
                if (!neighbour.walkable || Explored.Contains(neighbour))
                {
                    continue;
                }
                if (neighbour.walkable || !stack.Contains(neighbour))
                {
                    neighbour.parent = currentNode;
                    stack.Push(neighbour);
                }
            } 
        }
        timerDFS.Stop();
    }

    void UCSFindPath(Vector3 startPos, Vector3 targetPos)
    {
        Node startNode = grid.NodeFromWorldPoint(startPos);
        Node targetNode = grid.NodeFromWorldPoint(targetPos);

        List<Node> openSet = new List<Node>();
        HashSet<Node> closedSet = new HashSet<Node>();
        timerUCS.Start();
        openSet.Add(startNode);

        while (openSet.Count > 0)
        {
            Node node = openSet[0];
            for (int i = 1; i < openSet.Count; i++)
            {
                if (openSet[i].gCost < node.gCost || openSet[i].gCost == node.gCost)
                {
                    node = openSet[i];
                }
            }

            openSet.Remove(node);
            closedSet.Add(node);

            if (node == targetNode)
            {
                RetracePath(startNode, targetNode, ref memoryUCS);
                timerUCS.Stop();
                return;
            }

            foreach (Node neighbour in grid.GetNeighbours(node))
            {
                if (!neighbour.walkable || closedSet.Contains(neighbour))
                {
                    continue;
                }

                int CostNeighbour = node.gCost + GetDistance(node, neighbour);
                if (CostNeighbour < neighbour.gCost || !openSet.Contains(neighbour))
                {
                    neighbour.gCost = CostNeighbour;
                    neighbour.parent = node;

                    if (!openSet.Contains(neighbour))
                        openSet.Add(neighbour);
                }
            }
        }
        timerUCS.Stop();
    }

    void RetracePath(Node startNode, Node endNode, ref int totalMemory)
    {
        List<Node> path = new List<Node>();
        Node currentNode = endNode;

        while (currentNode != startNode)
        {
            totalMemory++;
            path.Add(currentNode);
            currentNode = currentNode.parent;
        }
        path.Reverse();

        grid.path = path;

    }

    int GetDistance(Node nodeA, Node nodeB)
    {
        int dstX = Mathf.Abs(nodeA.gridX - nodeB.gridX);
        int dstY = Mathf.Abs(nodeA.gridY - nodeB.gridY);

        if (dstX > dstY)
            return 14 * dstY + 10 * (dstX - dstY);
        return 14 * dstX + 10 * (dstY - dstX);
    }
}