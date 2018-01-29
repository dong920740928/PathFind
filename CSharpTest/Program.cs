using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using UX;
using UX.Game;

namespace CSharpTest
{
   // using UX.PathFindNS2 = UX.PathFindNS;
    class Program
    {



        static void Main(string[] args)
        {

            //            UXLog.SetSyncLog();
            //            UXLog.Start(LogTargets.File | LogTargets.Syslog);
            //            //UXLog.Start(LogTargets.All);
            //            UXLog.SetDefaultName("PathFindTest");

            //            debug();
            //            var h = HeightMap.GetHeightMap("Scene_Arena_001.hmap");
            //            var s = Stopwatch.StartNew();
            //            var a = h.GetHeight(2.980612f, 8.627451f);
            //
            //            Console.WriteLine(s.Elapsed); 
            debug();
        }

        static void debug()
        {

           
            UX.PathFindNS.PathFinder p = UX.PathFindNS.PathFinderNav.CreatePathFinder(@"scene_public005_tiangongge02.nav");

            


            UXVector3 start = new UXVector3(128.576f, 2f, 75.037f);
            UXVector3 end = new UXVector3(126.596f, 3f, 74.377f);

            uint tag = 0xffffffff;
            uint outtag;
//            int area;
//            var ss = p.GetAreaIfWalkable(new UXVector3(-805f, 0, 37f), tag, out area);
            List<UXVector3> path  = p.FindPath(start, end, 0.5f, tag, out outtag);
            
          
            p.Dispose();
        }
        static void test1()
        {

//            UXStopWatch a = new UXStopWatch();
//            a.Start();
//
//            
//            //UX.PathFindNS2.PathFinder p = UX.PathFindNS2.PathFinder.CreatePathFinder("Z:\\Nav2\\Scene_Activity_006.nav");
//            UX.PathFindNS.PathFinder p = UX.PathFindNS.PathFinder.CreatePathFinder(@"Scene_Raid_002.nav");
//            a.Stop();
//            Console.WriteLine(a.Result);
//
//            UXVector3 start = new UXVector3(173.386f, 2f, 128.522f);
//            UXVector3 end = new UXVector3(192.761f, 3f, 178.300f);
//
//            uint tag = 0xffffffff;
//            uint outtag;
//            UXStopWatch b = new UXStopWatch();
//            List<UXVector3> path = new List<UXVector3>();
//            b.Start();
//            for (int i = 0; i < 10000; i++)
//            {
//                path = p.GetMoveDisPath(start, 180f, 15f, tag);
//                path = p.FindPath(start, end, tag, out outtag, false);
//            }
//            b.Stop();
//            Console.Write(b.Result);
//            Console.WriteLine("GetMoveDisPath result:");
//            foreach (var v in path)
//            {
//                Console.WriteLine(v);
//            }
//            p.Dispose();
        }
        static void test2()
        {
            //UXStopWatch a = new UXStopWatch();
            //a.Start();
            //UX.PathFindNS1.PathFinder p = UX.PathFindNS1.PathFinder.CreatePathFinder("..\\ServerRes\\Scene_Activity_006.nav");
            //a.Stop();
            //Console.WriteLine(a.Result);

            //UXVector3 start = new UXVector3(173.386f, 2f, 128.522f);
            //UXVector3 end = new UXVector3(192.761f, 3f, 178.300f);
            //uint tag = 0xffffffff;
            //uint outtag;
            //UXStopWatch b = new UXStopWatch();
            //List<UXVector3> path = new List<UXVector3>();
            //b.Start();
            //for (int i = 0; i < 100; i++)
            //{
            //    path = p.GetMoveDisPath(start, 180f, 15f, tag);
            //    path = p.FindPath(start, end, tag, out outtag, false);
            //}
            //b.Stop();
            //Console.Write(b.Result);
            //Console.WriteLine("GetMoveDisPath result:");
            //foreach (var v in path)
            //{
            //    Console.WriteLine(v);
            //}
        }
    }
}
