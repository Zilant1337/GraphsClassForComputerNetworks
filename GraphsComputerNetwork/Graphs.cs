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
        private float[][] adjacencyMatrix;
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
            adjacencyMatrix = new float[vertices.Length][];
            for (int i = 0; i < vertices.Length; i++)
            {
                adjacencyMatrix[i] = new float[vertices.Length];
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
        public float[][] GetAdjacencyMatrix()
        {
            return adjacencyMatrix;
        }
        public float[] GetAdjecencyMatrix(int numberOfPoint)
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
    }
}
