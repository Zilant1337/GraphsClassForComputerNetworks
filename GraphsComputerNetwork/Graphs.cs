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

            public Vertex(float xCoordinate, float yCoordinate, float dataPassthroughModifier=1,string name="")
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
            public float GetYCoordinate ()
            {
                return yCoordinate;
            }
            public void SetXCoordinate(float xCoordinate)
            {
                this.xCoordinate=xCoordinate;
            }
            public void SetTCoordinate(float yCoordinate)
            {
                this.yCoordinate=yCoordinate;
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
                this.dataPassthroughModifier= dataPassthroughModifier;
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
                return Math.Sqrt(Math.Pow((otherVertex.GetXCoordinate()-this.GetXCoordinate()),2)+Math.Pow((otherVertex.GetYCoordinate() - this.GetYCoordinate()), 2));
            }

        }
        public class Branch
        {
            private double length;
            private float modifier;
            private int startVertexNumber;
            private int endVertexNumber;
            private int number;
            private bool isDirected;

            public Branch(bool isDirected, int startVertexNumber, int endVertexNumber, int number, double length = 0,float modifier=1)
            {
                this.isDirected = isDirected;
                this.startVertexNumber = startVertexNumber;
                this.endVertexNumber = endVertexNumber;
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
            public int GetStartVertexNumber()
            {
                return startVertexNumber;
            }
            public int GetEndVertexNumber()
            {
                return endVertexNumber;
            }
            public void SetStartVertexNumber(int startVertexNumber)
            {
                this.startVertexNumber = startVertexNumber;
            }
            public void SetEndVertexNumber(int endVertexNumber)
            {
                this.endVertexNumber = endVertexNumber;
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
        private Branch[] branches;
        private float[][] adjacencyMatrix;
        public Graphs() {
            vertices = new Vertex[0];
            branches = new Branch[0];
        }
        public Graphs(Vertex[] vertices, Branch[] branches)
        {
            this.vertices = vertices;
            this.branches = branches;
            //Дописать алгоритм заполнения матрицы расстояний
            adjacencyMatrix = new float[vertices.Length][];
            for (int i = 0; i < vertices.Length; i++)
            {
                adjacencyMatrix[i]= new float[vertices.Length];
                for (int j = 0; j < vertices.Length; j++)
                {
                    adjacencyMatrix[i][j] = 0;
                }
            }
            foreach(Branch i in branches)
            {
                adjacencyMatrix[i.GetStartVertexNumber()][i.GetEndVertexNumber()] = i.GetModifier();
                if (!i.GetDirection())
                {
                    adjacencyMatrix[i.GetEndVertexNumber()][i.GetStartVertexNumber()] = i.GetModifier();
                }
            }
        }
        public void AddBranch(Vertex Vertex1, Vertex Vertex2, float modifier,bool isDirected)
        {
            branches.Append(new Branch(isDirected,Vertex1.GetNumber(),Vertex2.GetNumber(),branches.Length,Vertex1.GetDistance(Vertex2),modifier));
        }
        public float[][] GetDistanceMatrix()
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
        public Branch[] GetBranches()
        {
            return branches;
        }
        public Branch GetBranches(int number)
        {
            return branches[number];               
        }
        public void SetVertexCoordinates(int PointNumber, int x, int y)
        {
            vertices[PointNumber].SetCoordinates(x, y);
        }
        public IEnumerable<int> GetAdjacentVertices(int vertexNumber)
        {
            if (vertexNumber < 0 || vertexNumber >= this.vertices.Length) throw new ArgumentOutOfRangeException("Cannot access vertex");

            List<int> adjacentVertices = new List<int>();
            for (int i = 0; i < this.vertices.Length; i++)
            {
                if (this.adjacencyMatrix[vertexNumber][i] > 0)
                    adjacentVertices.Add(i);
            }
            return adjacentVertices;
        }
    }
}
