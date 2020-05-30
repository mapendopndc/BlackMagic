using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;

namespace BlackMagic
{
    /* 
     * Boid simulation implementation based on Craig Reynolds' "Flocks, Herds, and Schools; A Distributed Behavioral Model"
     * 
     * Simulates the aggregate motion of a flock of birds through three primary behavioral mechanisms:
     *   1. Collision Avoidance - Separating from nearby flock mates as to avoid any imminent collisions
     *   2. Velocity Matching - Attempting to match speed and direction of nearby flock mates
     *   3. Flock Centering - Tendency to move towards geometric centroid of flock
     */

    class BoidSimulation
    {
        public double separationFOV;
        public double separationMag;
        public double cohesionFOV;
        public double cohesionMag;
        public double alignmentFOV;
        public double alignmentMag;

        private List<Point3d> boidPts;
        private List<Vector3d> boidVls;
        private List<Vector3d> weightedAccelerations;


        public BoidSimulation(List<Point3d> startPos)
        {
            boidPts = startPos;
            boidVls = new List<Vector3d>();
            for (int i = 0; i < boidPts.Count; i++)
                boidVls.Add(Vector3d.Zero);
        }

        public List<Point3d> BoidPts
        {
            get { return boidPts; }
        }

        public void Update()
        {
            // Initializing accelerations
            weightedAccelerations = new List<Vector3d>();

            for (int i = 0; i < boidPts.Count; i++)
                weightedAccelerations.Add(Vector3d.Zero);

            // Computing weighted acceleration vector based on flock behavior mechanisms outlined by Craig Reynolds
            Separate();
            Align();
            Cohere();

            // Move boids according to their weighted accelerations
            for (int i = 0; i < boidPts.Count; i++)
            {
                boidPts[i] += weightedAccelerations[i];
                boidVls[i] = weightedAccelerations[i];
            }
                
            
        }


        /* Performs collision avoidance on all boid particles in simulation
         * 
         * Naive algorithm for collision avoidance achieves goal in O(N^2) time complexity
         * 
         * This implementation uses R-tree class from Rhino's library in order to retrieve a list
         * of boids within a specified radius in time complexity nearing O(M * logM n), where nodes
         * have maximum of M entries
         * 
         * Calculates distances from all boids to their closest counterparts
         * Adds vector proportional to their proximity to the weighted acceleration
         * 
         */

        public void Separate()
        {
            // R-Tree initialization
            RTree rTree = new RTree();

            for (int i = 0; i < boidPts.Count; i++)
                rTree.Insert(boidPts[i], i);

            // Array of integer lists to store indexes of the boids within field of view for all boids in the simulation
            List<int>[] collisionIndices = new List<int>[boidPts.Count];

            for (int i = 0; i < boidPts.Count; i++)
                collisionIndices[i] = new List<int>();

            // Spherical range search of R-Tree based on separation field of view
            for (int i = 0; i < boidPts.Count; i++)
                rTree.Search(
                    new Sphere(boidPts[i], separationFOV),
                    (sender, args) => { if (i < args.Id) collisionIndices[i].Add(args.Id); });

            for (int i = 0; i < collisionIndices.Length; i++)
            {
                foreach (int j in collisionIndices[i])
                {
                    Vector3d acceleration = boidPts[j] - boidPts[i];
                    double distance = acceleration.Length;
                    if (distance > separationFOV) continue;

                    // Scale acceleration to make it inversely proportional to distance
                    acceleration *= 0.5 * (distance - separationFOV) / distance;
                    
                    weightedAccelerations[i] += acceleration * separationMag;
                    weightedAccelerations[j] -= acceleration * separationMag;
                }
            }

        }

         /* Performs velocity matching on all boid particles in simulation
         * 
         * Implemented using R-Trees for fast range search
         * 
         * Averages the velocities of neighboring boids and adds it to the weighted acceleration 
         * 
         */

        public void Align()
        {
            // R-Tree initialization
            RTree rTree = new RTree();

            for (int i = 0; i < boidPts.Count; i++)
                rTree.Insert(boidPts[i], i);

            // Array of integer lists to store indexes of the boids within field of view for all boids in the simulation
            List<int>[] collisionIndices = new List<int>[boidPts.Count];

            for (int i = 0; i < boidPts.Count; i++)
                collisionIndices[i] = new List<int>();

            // Spherical range search of R-Tree based on alignment field of view
            for (int i = 0; i < boidPts.Count; i++)
                rTree.Search(
                    new Sphere(boidPts[i], alignmentFOV),
                    (sender, args) => { if (i < args.Id) collisionIndices[i].Add(args.Id); });

            for (int i = 0; i < collisionIndices.Length; i++)
            {
                Vector3d acceleration = Vector3d.Zero;
                foreach (int j in collisionIndices[i])
                    acceleration += boidVls[j];
                acceleration -= Vector3d.Zero;
                acceleration /= (collisionIndices[i].Count == 0) ? 1 : collisionIndices[i].Count;
                weightedAccelerations[i] += acceleration * alignmentMag;
            }
        }

        /* Performs velocity matching on all boid particles in simulation
        * 
        * Implemented using R-Trees for fast range search
        * 
        * Computes centroid of neighboring boids and updates the weighted acceleration accordingly
        * 
        */

        public void Cohere()
        {
            // R-Tree initialization
            RTree rTree = new RTree();

            for (int i = 0; i < boidPts.Count; i++)
                rTree.Insert(boidPts[i], i);

            // Array of integer lists to store indexes of the boids within field of view for all boids in the simulation
            List<int>[] collisionIndices = new List<int>[boidPts.Count];

            for (int i = 0; i < boidPts.Count; i++)
                collisionIndices[i] = new List<int>();

            // Spherical range search of R-Tree based on cohesion field of view
            for (int i = 0; i < boidPts.Count; i++)
                rTree.Search(
                    new Sphere(boidPts[i], cohesionFOV),
                    (sender, args) => { if (i < args.Id) collisionIndices[i].Add(args.Id); });

            for (int i = 0; i < collisionIndices.Length; i++)
            {
                
                Point3d centroid = Point3d.Origin;
                foreach (int j in collisionIndices[i])
                    centroid += boidPts[j];

                centroid /= (collisionIndices[i].Count == 0) ? 1 : collisionIndices[i].Count;
                Vector3d acceleration = Vector3d.Zero;
                acceleration = boidPts[i] - centroid;
                weightedAccelerations[i] -= acceleration * cohesionMag;
            }
        }
    }
}
