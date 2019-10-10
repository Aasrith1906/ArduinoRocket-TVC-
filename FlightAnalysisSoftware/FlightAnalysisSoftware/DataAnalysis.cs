/*
 * this is a class which contains all the methods which can be used to analyse the flight data recorded from the sensors
 * requires the txt file from the sd card
 * shows the data in the form of graphs
 * alt(alt - base_alt)-time
 * acceleration(x,y,z)-time 
 * pressure-time
 * temp-time




using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.IO;



namespace FlightAnalysisSoftware
{
    class DataAnalysis
    {

        private string FileName;
        public DataAnalysis(string filename)
        {
            filename = this.FileName;
        }
        

    }
}
