using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.IO;
using System.Data;
using System.Data.Sql;
using System.Data.SqlClient;

namespace FlightAnalysiisSoftware
{
    class Data
    {  
        
        private string file_path { get; set; }

        private static SqlConnection connection = new SqlConnection();
       
        private static SqlCommand cmd = new SqlCommand();


        public Data(string File_Path)
        {
            this.file_path = File_Path;
        }

        public int SendData()
        {
            return 1;
        }

        public int OpenFileRead()
        {
            
            return 1;
        }
    }
}
