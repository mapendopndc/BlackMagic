using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;

namespace BlackMagic
{
    public class BlackMagicComponent : GH_Component
    {

        private BoidSimulation myBoidSimulation;

        public BlackMagicComponent()
          : base(
                "Boids",
                "Boids",
                "Living creatures in constant interaction with each other",
                "Black Magic", "Creatures")
        {
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddBooleanParameter("Reset", "Reset", "Reset Simulation", GH_ParamAccess.item);
            pManager.AddPointParameter("Starting Positions", "StartPos", "Initial boid locations", GH_ParamAccess.list);
            pManager.AddNumberParameter("Separation FOV", "sFOV", "Collision avoidance field of view", GH_ParamAccess.item);
            pManager.AddNumberParameter("Separation Strength", "sMag", "Collision avoidance magnitude", GH_ParamAccess.item);
            pManager.AddNumberParameter("Alignment FOV", "aFOV", "Velocity matching field of view", GH_ParamAccess.item);
            pManager.AddNumberParameter("Alignment Strength", "aMag", "Velocity matching magnitude", GH_ParamAccess.item);
            pManager.AddNumberParameter("Cohesion FOV", "cFOV", "Flock centering field of view", GH_ParamAccess.item);
            pManager.AddNumberParameter("Cohesion Strength", "cMag", "Flock centering magnitude", GH_ParamAccess.item);
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddPointParameter("Positions", "Positions", "Updated boid positions", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            bool iReset = false;
            List<Point3d> iStartPos = new List<Point3d>();
            double iSepFOV = 0;
            double iSepMag = 0;
            double iAlFOV = 0;
            double iAlMag = 0;
            double iCoFOV = 0;
            double iCoMag = 0;

            DA.GetData("Reset", ref iReset);
            DA.GetDataList("Starting Positions", iStartPos);
            DA.GetData("Separation FOV", ref iSepFOV);
            DA.GetData("Separation Strength", ref iSepMag);
            DA.GetData("Alignment FOV", ref iAlFOV);
            DA.GetData("Alignment Strength", ref iAlMag);
            DA.GetData("Cohesion FOV", ref iCoFOV);
            DA.GetData("Cohesion Strength", ref iCoMag);

            if (iReset || myBoidSimulation == null)
                myBoidSimulation = new BoidSimulation(iStartPos);

            myBoidSimulation.separationFOV = iSepFOV;
            myBoidSimulation.separationMag = iSepMag;
            myBoidSimulation.alignmentFOV = iAlFOV;
            myBoidSimulation.alignmentMag = iAlMag;
            myBoidSimulation.cohesionFOV = iCoFOV;
            myBoidSimulation.cohesionMag = iCoMag;

            myBoidSimulation.Update();

            DA.SetDataList("Positions", myBoidSimulation.BoidPts);
        }

        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                return null;
            }
        }

        public override Guid ComponentGuid
        {
            get { return new Guid("7eb55393-94c9-4bfe-be03-d1e0a28f26a5"); }
        }
    }
}
