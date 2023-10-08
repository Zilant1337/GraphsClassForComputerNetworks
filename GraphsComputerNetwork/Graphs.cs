using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace GraphsComputerNetwork
{
    internal class Graphs
    {
        public class Vertex
        {
            private float xCoordinate;
            private float yCoordinate;
            private string name;
            private float dataPassthroughModifier;
            private int number;

            public Vertex(float xCoordinate, float yCoordinate, float dataPassthroughModifier = 1, string name = "", int number = -1)
            {
                this.dataPassthroughModifier = dataPassthroughModifier;
                this.xCoordinate = xCoordinate;
                this.yCoordinate = yCoordinate;
                this.name = name;
                this.number = number;
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
            public int GetNumber()
            {
                return number;
            }
            public void SetNumber(int number)
            {
                this.number = number;
            }
            public double GetDistance(Vertex otherVertex)
            {
                return Math.Sqrt(Math.Pow((otherVertex.GetXCoordinate() - this.GetXCoordinate()), 2) + Math.Pow((otherVertex.GetYCoordinate() - this.GetYCoordinate()), 2));
            }

        }
        public class Edge
        {
            private double length;
            private float modifier;
            private Vertex startVertex;
            private Vertex endVertex;
            private int number;
            private bool isDirected;

            public Edge(bool isDirected, Vertex startVertex, Vertex endVertex, int number, double length = 0, float modifier = 1)
            {
                this.isDirected = isDirected;
                this.startVertex = startVertex;
                this.endVertex = endVertex;
                this.number = number;
                this.length = length;
            }
            public double GetLength()
            {
                return length;
            }
            public void SetLength(float length)
            {
                this.length = length;
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
            public void SetModifier(float modifier)
            {
                this.modifier = modifier;
            }
            public float GetModifier()
            {
                return modifier;
            }
            public bool GetDirection()
            {
                return isDirected;
            }
        }

        private Vertex[] vertices;
        private Edge[] edges;
        private double[][] adjacencyMatrix;
        private double[][] fastestPathsMatrix;
        private string name;
        public Graphs() {
            vertices = new Vertex[0];
            edges = new Edge[0];
        }
        public Graphs(Vertex[] vertices, Edge[] edges= null, string name = "")
        {
            this.vertices = vertices;
            this.edges = edges;
            this.name = name;
            adjacencyMatrix = new double[vertices.Length][];
            for (int i = 0; i < vertices.Length; i++)
            {
                fastestPathsMatrix[i] = new double[edges.Length];
                adjacencyMatrix[i] = new double[vertices.Length];
                for (int j = 0; j < vertices.Length; j++)
                {
                    adjacencyMatrix[i][j] = 0;
                }
                
            }
            foreach (Edge i in edges)
            {
                adjacencyMatrix[i.GetStartVertex().GetNumber()][i.GetEndVertex().GetNumber()] = i.GetModifier();
                if (!i.GetDirection())
                {
                    adjacencyMatrix[i.GetEndVertex().GetNumber()][i.GetStartVertex().GetNumber()] = i.GetModifier();
                }
            }
            

            this.name = name;
        }
        public void AddEdge(Vertex Vertex1, Vertex Vertex2, float modifier, bool isDirected)
        {
            edges.Append(new Edge(isDirected, Vertex1, Vertex2, edges.Length, Vertex1.GetDistance(Vertex2), modifier));
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
            newVertex.SetNumber(vertices.Length);
            vertices.Append(newVertex);
        }
        public Vertex[] GetVertices()
        {
            return vertices;
        }
        public Vertex GetVertices(int number)
        {
            return vertices[number];
        }
        public Edge[] GetEdges()
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
            if (vertexNumber < 0 || vertexNumber >= this.vertices.Length) throw new ArgumentOutOfRangeException("Cannot access vertex");

            List<Vertex> adjacentVertices = new List<Vertex>();
            for (int i = 0; i < this.vertices.Length; i++)
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
            return fastestPathsMatrix[vert1.GetNumber()][vert2.GetNumber()];
        }
        public void GenerateFastestRoutes()
        {
            for (int i = 0; i < edges.Length; i++)
            {
                fastestPathsMatrix[i] = new double[edges.Length];
                bool[] sptSet=new bool[edges.Length];
                for(int j = 0; j < edges.Length; j++)
                {
                    fastestPathsMatrix[i][j]=double.MaxValue;
                    sptSet[j]=false;
                }
                fastestPathsMatrix[i][i]=0;
                for(int count = 0; count < edges.Length - 1; count++)
                {
                    int u = MinDistance(fastestPathsMatrix[i], sptSet,edges.Length);
                    sptSet[u] = true;
                    for (int v = 0; v < edges.Length; v++)
                    {
                        if (!sptSet[v] && adjacencyMatrix[u][v] != 0 && fastestPathsMatrix[i][u]
                            != double.MaxValue && fastestPathsMatrix[i][u]
                            + GetEdge(vertices[u], vertices[v]).GetLength() * 
                            GetEdge(vertices[u], vertices[v]).GetModifier() * 
                            vertices[u].GetDataPassthroughModifier() * 
                            vertices[v].GetDataPassthroughModifier() 
                            < fastestPathsMatrix[i][v])
                        {
                            fastestPathsMatrix[i][v] = fastestPathsMatrix[i][u]
                            + GetEdge(vertices[u], vertices[v]).GetLength() *
                            GetEdge(vertices[u], vertices[v]).GetModifier() *
                            vertices[u].GetDataPassthroughModifier() *
                            vertices[v].GetDataPassthroughModifier();
                        }
                    }
                }

            }
            return;
        }
        private int MinDistance(double[] distance, bool[]sptSet,int V)
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
    }
}
