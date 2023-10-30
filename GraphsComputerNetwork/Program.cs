using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Data;
using System.Data.SqlClient;
using System.IO;

namespace GraphsComputerNetwork
{
    internal class Program
    {
        static void Main(string[] args)
        {
            string connectionString = "Data Source=(LocalDB)\\MSSQLLocalDB;AttachDbFilename=" + Directory.GetParent(Environment.CurrentDirectory).Parent.Parent.FullName + @"\data\dbTest.mdf; Integrated Security=True;";
            SqlConnection connection = new SqlConnection(connectionString);
            connection.Open();
        }
    }
}
