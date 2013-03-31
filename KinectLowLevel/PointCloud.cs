using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using MathNet.Numerics.LinearAlgebra.Double;
using MathNet.Numerics.LinearAlgebra.Double.Factorization;
using System.Windows.Media;
using KdTree;
using Microsoft.Kinect;

namespace KinectLowLevel
{
    class CloudPoint
    {
        public Int32 color { get; set; }
        public Vector location { get; set; }
        public Vector normal { get; set; }

        public CloudPoint(Vector location, Int32 color, Vector normal)
        {
            this.color = color;
            this.location = location;
            this.normal = normal;
        }
    }

    class PointCloud
    {
        /// <summary>
        /// KD tree of points.
        /// </summary>
        KdTree<CloudPoint, double> points;

        // we want someone else to handle the alignment because that is shitty
        // these arrays need to be the same size or else correspondence is weird.
        public PointCloud(short[] depthArray, int depthWidth, int depthHeight,
                          byte[] colorArray, int colorWidth, int colorHeight)
        {
            
            // TODO take in color format and switch on it
            if (depthArray.Length != (colorArray.Length/4))
                throw new ArgumentException("color and depth array lengths must match");

            // go through arrays and pack into CloudPoints
            List<CloudPoint> cloudPoints = new List<CloudPoint>();

            for (int i = 0; i < depthArray.Length; i++)
            {
                // pushing these into math coordinates
                int x = (i + 1) % depthWidth;
                int y = (i + 1) / depthWidth;
                int z = depthArray[i];
                double[] location = new double[3];
                location[0] = x;
                location[1] = y;
                location[2] = z;

                DenseVector v = new DenseVector(location);

                // we can't calculate surface normals until it's all read in
                cloudPoints.Add(new CloudPoint(v, 
                                                colorArray[i] + 
                                                (colorArray[i+1] << 8)+
                                                (colorArray[i+2] << 16)+
                                                (colorArray[i+3] << 24), 
                                                null));
            }


            points = KdTree<CloudPoint, double>.Construct(3, cloudPoints, (x) => x.location);

            // express normals, this is also hella slow
            foreach (CloudPoint p in points)
            {
                // THE NUMBER FOUR IS HARDCODED IN
                // slow
                C5.IPriorityQueue<CloudPoint> neighborhood = points.FindNearestNNeighbors(p.location, 4);

                DenseVector avg =
                    neighborhood.Aggregate(new DenseVector(3), (sum, val) => sum + (DenseVector)val.location) / 4.0;

                // express covariance matrix
                DenseMatrix cov = 
                    neighborhood.Aggregate(new DenseMatrix(3), 
                                              (sum, val) => sum + 
                                                  (DenseMatrix)((val.location - avg).ToColumnMatrix() * 
                                                                (val.location - avg).ToRowMatrix())) * (1/4.0);

                // since the COV is symmetric the SVD will find its smallest eigval eigvec
                Svd covSVD = cov.Svd(true);
                DenseVector sing = (DenseVector)covSVD.S();
                DenseMatrix unitary = (DenseMatrix)covSVD.VT();

                DenseVector eigV = (DenseVector)unitary.Column(sing.MinimumIndex());
                // if the normal is facing away from the viewpoint, flip it
                // optical center viewpoint
                DenseVector viewP = new DenseVector(new double[] { depthWidth / 2, depthHeight / 2, 0 });
                if (eigV.DotProduct(viewP - eigV) < 0)
                    eigV = eigV * -1;

                eigV = eigV / eigV.Norm(3);

                p.normal = eigV;
            }
        }


        const float close = 0.1f;
        // this will be the slowest possible thing you can do.
        // we can kind of assume the closest points are in correspondance because the translation should be v small 
        public bool getCholeskyICP(PointCloud other, DenseMatrix guessR, DenseVector guessT, int iterations,
                                    out DenseMatrix R, out DenseVector T)
        {
            R = guessR;
            T = guessT;

            for (int i = 0; i < iterations; i++)
            {
                var matchedPoints = new List<Tuple<CloudPoint, CloudPoint>>();
                // find correspondance and prune bad matches, v slow
                foreach (CloudPoint o in other.points)
                {
                    CloudPoint m = this.points.FindNearestNeighbor(R * o.location + T);

                    if ((m.location - R * o.location + T).Norm(3) < close)
                    {
                        matchedPoints.Add(new Tuple<CloudPoint, CloudPoint>(m, o));
                    }
                }

                // these both accumulate
                DenseMatrix cov = new DenseMatrix(6);
                DenseVector b = new DenseVector(6);

                // incremental transform energy minimization
                // loosely follow along in http://www.cs.princeton.edu/~smr/papers/icpstability.pdf
                foreach (Tuple<CloudPoint, CloudPoint> t in matchedPoints)
                {
                    var m = t.Item1;
                    var o = t.Item2;

                    // we are moving o to m
                    var n = m.normal;
                    var c = cross3(o.location, n);

                    // don't know if this works
                    DenseVector cAugN = (DenseVector) c.Concat(n);

                    // the cov is formed by taking the outer product of [c n] with itself
                    cov = cov + (DenseMatrix)cAugN.OuterProduct(cAugN);

                    // the dot product commutes with multiplication
                    // evaluate the dot, then do a Hadamard product
                    var diffDot = (o.location - m.location).DotProduct(n);

                    var bCur = (new DenseVector(6, diffDot)).PointwiseMultiply(cAugN);

                    b = b + (DenseVector)bCur;
                } // finished accumulating cov and b

                // solve cov * x = b with a cholesky factorization
                DenseCholesky dCholesky = new DenseCholesky(cov);
                var x = dCholesky.Solve(b);

                R = new DenseMatrix(new double[3, 3]{ {1,  -x[2],  x[1]},
                                                     {x[2], 1,   -x[0]},
                                                     {-x[1],x[0], 1   }});

                T = new DenseVector(x.SubVector(3, 3));

            } // did enough iterations

            return true;
        } 

        // jesus somehow math.net doesn't have a cross product
        public static Vector cross3(Vector a, Vector b)
        {
            DenseVector result = new DenseVector(3);

            result[0] = a[1] * b[2] - a[2] * b[1];
            result[1] = -(a[0] * b[2] - a[2] * b[2]);
            result[2] = a[0] * b[1] - a[1] * b[0];

            return result;
        }
    }
}
