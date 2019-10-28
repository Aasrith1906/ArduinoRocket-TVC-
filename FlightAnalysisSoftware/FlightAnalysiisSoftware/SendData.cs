using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Data;
using System.Data.Sql;
using System.Data.SqlClient;
using System.IO;

namespace FlightAnalysiisSoftware
{
    class SendData
    {
        private static string FileName{get;set;}

        public static SqlConnection con = new SqlConnection();
        public static SqlCommand cmd = new SqlCommand();
        public SendData(string file_name)
        {
            FileName = file_name;
        }

        private FileStream OpenFile()
        {
            FileStream stream = new FileStream(FileName, FileMode.Open);

            return stream;
        }

        public string ReadData()
        {
            string data = File.ReadAllText(FileName);
            /*byte[] buffer = { } ;

            

            FileStream stream = OpenFile();
            
            while(stream.CanRead)
            {
                stream.Read(buffer, 0, 1);
            }

            data = Encoding.ASCII.GetString(buffer);

            */    
            return data;
            
        }

    }
}
