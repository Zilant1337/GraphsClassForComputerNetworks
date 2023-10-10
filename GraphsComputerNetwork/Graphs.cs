using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace GraphsComputerNetwork
{
    internal class Graph
    {
        public class Vertex
        {
            private float xCoordinate;
            private float yCoordinate;
            private string name;
            private float dataPassthroughModifier;

            public Vertex(float xCoordinate, float yCoordinate, float dataPassthroughModifier = 1, string name = "")
            {
                this.dataPassthroughModifier = dataPassthroughModifier;
                this.xCoordinate = xCoordinate;
                this.yCoordinate = yCoordinate;
                this.name = name;
            }
            public float GetXCoordinate()
            {
                return xCoordinate;
            }
            public float GetYCoordinate()
            {
                return yCoordinate;
            }
            public void SetXCoordinate(float xCoordinate)
            {
                this.xCoordinate = xCoordinate;
            }
            public void SetTCoordinate(float yCoordinate)
            {
                this.yCoordinate = yCoordinate;
            }
            public void SetCoordinates(int xCoordinate, int yCoordinate)
            {
                this.xCoordinate = xCoordinate;
                this.yCoordinate = yCoordinate;
            }

            public string GetName()
            {
                return name;
            }
            public void SetName(string name)
            {
                this.name = name;
            }
            public void SetDataPassthroughModifier(float dataPassthroughModifier)
            {
                this.dataPassthroughModifier = dataPassthroughModifier;
            }
            public float GetDataPassthroughModifier()
            {
                return dataPassthroughModifier;
            }
            public double GetDistance(Vertex otherVertex)
            {
                return Math.Sqrt(Math.Pow((otherVertex.GetXCoordinate() - this.GetXCoordinate()), 2) + Math.Pow((otherVertex.GetYCoordinate() - this.GetYCoordinate()), 2));
            }

        }
        public class Edge
        {
            private float maxLoad;
            private float currentLoad;
            private Vertex startVertex;
            private Vertex endVertex;
            
            private bool isDirected;

            public Edge(bool isDirected, Vertex startVertex, Vertex endVertex, float maxLoad, float currentLoad=0)
            {
                this.isDirected = isDirected;
                this.startVertex = startVertex;
                this.endVertex = endVertex;
                this.currentLoad = currentLoad;
                this.maxLoad = maxLoad;
            }
            public Vertex GetStartVertex()
            {
                return startVertex;
            }
            public Vertex GetEndVertex()
            {
                return endVertex;
            }
            public void SetStartVertexNumber(Vertex startVertex)
            {
                this.startVertex = startVertex;
            }
            public void SetEndVertexNumber(Vertex endVertex)
            {
                this.endVertex = endVertex;
            }
            public void SetMaxLoad(float maxLoad)
            {
                this.maxLoad = maxLoad;
            }
            public float GetMaxLoad()
            {
                return maxLoad;
            }
            public void AddLoad(float addedLoad)
            {
                currentLoad += addedLoad;
            }
            public float GetCurrentLoad()
            {
                return currentLoad;
            }
            public bool GetDirection()
            {
                return isDirected;
            }
        }

        private List<Vertex> vertices;
        private List<Edge> edges;
        private double[][] adjacencyMatrix;
        private double[][] loadMatrix;
        private double[][] fastestPathsMatrix;
        private string name;
        public Graph() {
            vertices = new List<Vertex>();
            edges = new List<Edge>();
        }
        public Graph(List<Vertex> vertices, List<Edge> edges= null, string name = "")
        {
            this.vertices = vertices;
            this.edges = edges;
            this.name = name;
            adjacencyMatrix = new double[vertices.Count()][];
            for (int i = 0; i < vertices.Count(); i++)
            {
                fastestPathsMatrix[i] = new double[edges.Count()];
                adjacencyMatrix[i] = new double[vertices.Count()];
                for (int j = 0; j < vertices.Count(); j++)
                {
                    adjacencyMatrix[i][j] = 0;
                }
                
            }
            foreach (Edge i in edges)
            {
                adjacencyMatrix[vertices.IndexOf(i.GetStartVertex())][vertices.IndexOf(i.GetEndVertex())] = 1;
                if (!i.GetDirection())
                {
                    adjacencyMatrix[vertices.IndexOf(i.GetEndVertex())][vertices.IndexOf(i.GetStartVertex())] =1;
                }
            }
            

            this.name = name;
        }
        public void AddEdge(Vertex Vertex1, Vertex Vertex2, float maxLoad, bool isDirected,float currentLoad=0)
        {
            edges.Append(new Edge(isDirected, Vertex1, Vertex2,maxLoad,currentLoad));
        }
        public double[][] GetAdjacencyMatrix()
        {
            return adjacencyMatrix;
        }
        public double[] GetAdjecencyMatrix(int numberOfPoint)
        {
            return adjacencyMatrix[numberOfPoint];
        }
        public void AddVertex(Vertex newVertex)
        {
            vertices.Append(newVertex);
            ReformAdjacencyMatrix();
            GenerateFastestRoutes();
        }
        public List<Vertex> GetVertices()
        {
            return vertices;
        }
        public Vertex GetVertices(int number)
        {
            return vertices[number];
        }
        public List<Edge> GetEdges()
        {
            return edges;
        }
        public Edge GetEdges(int number)
        {
            return edges[number];
        }
        public void SetVertexCoordinates(int PointNumber, int x, int y)
        {
            vertices[PointNumber].SetCoordinates(x, y);
        }
        public IEnumerable<Vertex> GetAdjacentVertices(int vertexNumber)
        {
            if (vertexNumber < 0 || vertexNumber >= this.vertices.Count()) throw new ArgumentOutOfRangeException("Cannot access vertex");

            List<Vertex> adjacentVertices = new List<Vertex>();
            for (int i = 0; i < this.vertices.Count(); i++)
            {
                if (this.adjacencyMatrix[vertexNumber][i] > 0)
                    adjacentVertices.Add(vertices[i]);
            }
            return adjacentVertices;
        }
        public string GetName()
        {
            return name;
        }
        public void SetName(string name)
        {
            this.name = name;
        }
        
        public double GetShortestPath(Vertex vert1, Vertex vert2)
        {
            return fastestPathsMatrix[vertices.IndexOf(vert1)][vertices.IndexOf(vert2)];
        }
        public void GenerateFastestRoutes()
        {
            for (int i = 0; i < edges.Count(); i++)
            {
                fastestPathsMatrix[i] = new double[edges.Count()];
                bool[] sptSet=new bool[edges.Count()];
                for(int j = 0; j < edges.Count(); j++)
                {
                    fastestPathsMatrix[i][j]=double.MaxValue;
                    sptSet[j]=false;
                }
                fastestPathsMatrix[i][i]=0;
                for(int count = 0; count < edges.Count() - 1; count++)
                {
                    int u = MinDistanceIndex(fastestPathsMatrix[i], sptSet,edges.Count());
                    sptSet[u] = true;
                    for (int v = 0; v < edges.Count(); v++)
                    {
                        if (!sptSet[v] && adjacencyMatrix[u][v] != 0 && fastestPathsMatrix[i][u]
                            != double.MaxValue && fastestPathsMatrix[i][u]
                            + adjacencyMatrix[u][v]
                            < fastestPathsMatrix[i][v])
                        {
                            fastestPathsMatrix[i][v] = fastestPathsMatrix[i][u]
                            + adjacencyMatrix[u][v];
                        }
                    }
                }

            }
            return;
        }
        private int MinDistanceIndex(double[] distance, bool[]sptSet,int V)
        {
            double min =double.MaxValue;
            int minIndex = -1;
            for (int v = 0; v < V; v++)
            {
                if (!sptSet[v] && distance[v] <= min)
                {
                    min = distance[v];
                    minIndex = v;
                }
            }
            return minIndex;
        }
        public Edge GetEdge(Vertex vert1,Vertex vert2)
        {
            foreach (Edge edge in edges)
            {
                if ((!edge.GetDirection()&&(edge.GetStartVertex()==vert1&&edge.GetEndVertex()==vert2
                    || edge.GetStartVertex() == vert2 && edge.GetEndVertex() == vert1))
                    ||(edge.GetDirection()&&edge.GetStartVertex()==vert1&&edge.GetEndVertex()==vert2))
                {
                    return edge;
                }
            }
            return null;
        }
        public void RemoveVertex(Vertex vert)
        {
            vertices.Remove(vert);
            foreach(Vertex i in vertices)
            {
                if (GetEdge(vert, i)!=null)
                {
                    edges.Remove(GetEdge(vert, i));
                    if (GetEdge(i, vert) != null)
                    {
                        edges.Remove(GetEdge(i,vert));
                    }
                }
            }
            ReformAdjacencyMatrix();
            GenerateFastestRoutes();

        }
        public void RemoveEdge(Edge edge)
        {
            edges.Remove(edge);
            ReformAdjacencyMatrix();
            GenerateFastestRoutes();
        }
        public void RemoveEdge(Vertex vert1, Vertex vert2)
        {
            edges.Remove(GetEdge(vert1, vert2));
            ReformAdjacencyMatrix();
            GenerateFastestRoutes();
        }
        private void ReformAdjacencyMatrix()
        {
            adjacencyMatrix = new double[vertices.Count()][];
            for (int i = 0; i < vertices.Count(); i++)
            {
                adjacencyMatrix[i] = new double[vertices.Count()];
                for (int j = 0; j < vertices.Count(); j++)
                {
                    adjacencyMatrix[i][j] = 0;
                }

            }
            foreach (Edge i in edges)
            {
                adjacencyMatrix[vertices.IndexOf(i.GetStartVertex())][vertices.IndexOf(i.GetEndVertex())] = 1;
                if (!i.GetDirection())
                {
                    adjacencyMatrix[vertices.IndexOf(i.GetEndVertex())][vertices.IndexOf(i.GetStartVertex())] = 1;
                }
            }
        }
    }
}
