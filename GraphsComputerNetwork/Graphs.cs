using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace GraphsComputerNetwork
{
    internal class Graphs
    {
        public class Point
        {
            private int[] coordinates;
            private string name;
            public Point(int[]coordinates)
            {
                if (coordinates.Length != 2)
                {
                    throw new ArgumentException("Wrong array size");
                }
                this.coordinates = coordinates;
                name = string.Empty;
            }
            public Point(int x, int y)
            {
                coordinates = new int[2] {x,y};
                name = string.Empty;
            }
            public int[] GetCoordinates()
            {
                return coordinates;
            }
            public int GetCoordinates (int number)
            {
                return coordinates[number];
            }
            public int GetX()
            {
                return coordinates[0];
            }
            public int GetY()
            {
                return coordinates[1];
            }
            public void SetCoordinates(int[]x)
            {
                if (coordinates.Length != 2)
                {
                    throw new ArgumentException("Wrong array size");
                }
                coordinates = x;
            }
            public void SetCoordinates(int x, int y)
            {
                coordinates[0]= x;
                coordinates[1]= y;
            }
            public void SetX(int x)
            {
                coordinates[0] = x;
            }
            public void SetY(int y)
            {
                coordinates[1]= y;
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
        public class Edge
        {
            private float length;
            public float GetLength()
            {
                return length;
            }
            public void SetLength(float length)
            {
                this.length = length;
            }
        }
        private Point[] points;
        private Edge[] edges;
        private float[][] distanceMatrix;
        public Graphs() {
            points = new Point[0];
            edges = new Edge[0];
        }
        public Graphs(Point[] points, Edge[] edges)
        {
            this.points = points;
            this.edges = edges;
            //Дописать алгоритм заполнения матрицы расстояний
        }
        public void AddEdge(Point Point1, Point Point2)
        {

        }
        public float[][] GetDistanceMatrix()
        {
            return distanceMatrix;
        }
        public float[] GetDistanceMatrix(int numberOfPoint)
        {
            return distanceMatrix[numberOfPoint];
        }
        
        public Point[] GetPoints()
        {
            return points;
        }
        public Point GetPoint(int number)
        {
            return points[number];
        }
        public Edge[] GetEdges()
        {
            return edges;
        }
        public Edge GetEdges(int number)
        {
            return edges[number];               
        }
        public void SetPointCoordinates(int PointNumber, int[]coordinates)
        {
            points[PointNumber].SetCoordinates(coordinates);
        }

        public void AddPoint(Point newPoint)
        {
            points.Append(newPoint);
        }
        public void AddEdge(Edge newEdge)
        {
            edges.Append(newEdge);
        }
    }
}
