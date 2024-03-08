//'Hare: Accelerated Multi-Resolution Ray Tracing (GPL)
//'
//'Copyright (c) 2008 - 2024, Arthur van der Harten			
//'This program is free software; you can redistribute it and/or modify
//'it under the terms of the GNU General Public License as published 
//'by the Free Software Foundation; either version 3 of the License, or
//'(at your option) any later version.
//'This program is distributed in the hope that it will be useful,
//'but WITHOUT ANY WARRANTY; without even the implied warranty of
//'MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//'GNU General Public License for more details.
//'
//'You should have received a copy of the GNU General Public 
//'License along with this program; if not, write to the Free Software
//'Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

using System;
using System.Collections.Generic;
using System.Linq;

namespace Hare
{
    namespace Geometry
    {
        /// <summary>
        /// The parent class of all Polygons...
        /// </summary>
        public abstract class Polygon
        {
            //protected internal 
            public Vector Normal;
            public Vertex[] Points;
            public List<Edge> Edges = new List<Edge>();
            public Point Centroid;
            protected internal int VertexCount;
            protected double d;
            protected internal bool IsDegenerate = false;
            protected internal bool IsConvex;
            //To which topology does this polygon belong?
            protected internal int top_index;
            protected int Poly_index;
            protected double Inv_Dot_Normal;
            protected int Plane_index;//Optional
            //A vector transformation coordinate system
            public Vector diffx;
            public Vector diffy;
            public Vector diffz;

            /// <summary>
            /// Finds the intersection point of a ray with a double sided polygon.
            /// </summary>
            /// <param name="R">The ray to be tested against the polygon.</param>
            /// <param name="P">The Vertex points of the Polygon.</param>
            /// <param name="I">The </param>
            /// <returns></returns>
            public abstract bool Intersect(Ray R, Point[] P, out Point Xpt, out double u, out double v, out double t, out int polyid);
            public abstract Point GetRandomPoint(double Rndx, double Rndy, double Side);

            public abstract double closestpoint(Point p);

            public Point triclosestpoint(int a, int b, int c, Point p)
            {
                // Check if P in vertex region outside A
                Vector ab = Points[b] - Points[a];
                Vector ac = Points[c] - Points[a];
                Vector ap = p - Points[a];
                double d1 = Hare_math.Dot(ab, ap);
                double d2 = Hare_math.Dot(ac, ap);
                if (d1 <= 0.0f && d2 <= 0.0f) return Points[a]; // barycentric coordinates (1,0,0)

                // Check if P in vertex region outside B
                Vector bp = p - Points[b];
                double d3 = Hare_math.Dot(ab, bp);
                double d4 = Hare_math.Dot(ac, bp);
                if (d3 >= 0.0f && d4 <= d3) return Points[b]; // barycentric coordinates (0,1,0)

                // Check if P in edge region of AB, if so return projection of P onto AB
                double vc = d1 * d4 - d3 * d2;
                if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f)
                {
                    double v1 = d1 / (d1 - d3);
                    return Points[a] + v1 * ab; // barycentric coordinates (1-v,v,0)
                }

                // Check if P in vertex region outside C
                Vector cp = p - Points[c];
                double d5 = Hare_math.Dot(ab, cp);
                double d6 = Hare_math.Dot(ac, cp);
                if (d6 >= 0.0f && d5 <= d6) return Points[c]; // barycentric coordinates (0,0,1)

                // Check if P in edge region of AC, if so return projection of P onto AC
                double vb = d5 * d2 - d1 * d6;
                if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f)
                {
                    double w1 = d2 / (d2 - d6);
                    return Points[a] + w1 * ac; // barycentric coordinates (1-w,0,w)
                }

                // Check if P in edge region of BC, if so return projection of P onto BC
                double va = d3 * d6 - d5 * d4;
                if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f)
                {
                    double w2 = (d4 - d3) / ((d4 - d3) + (d5 - d6));
                    return Points[b] + w2 * (Points[c] - Points[b]); // barycentric coordinates (0,1-w,w)
                }

                // P inside face region. Compute Q through its barycentric coordinates (u,v,w)
                double denom = 1.0f / (va + vb + vc);
                double vv = vb * denom;
                double ww = vc * denom;
                return Points[a] + ab * vv + ac * ww; // = u*a + v*b + w*c, u = va * denom = 1.0f - v - w

            }

            public double SqDistanceToEdges(Point p)
            {
                double d = double.PositiveInfinity;

                for (int i = 0; i < Points.Length; i++)
                {
                    int mod = Points.Length - 1;
                    Vector edge = Points[(i + 1) % mod] - Points[i];
                    Vector peb = p - Points[(i + 1) % mod];
                    Vector pea = p - Points[i];
                    double e = Hare_math.Dot(pea, edge);
                    if (e <= 0)
                    {
                        d = Math.Min(Hare_math.Dot(pea, pea), d);
                        continue;
                    }
                    double f = Hare_math.Dot(edge, edge);
                    if (e >= f)
                    {
                        d = Math.Min(Hare_math.Dot(peb, peb), d);
                        continue;
                    }
                    else
                    {
                        d = Math.Min(Hare_math.Dot(pea, pea) - e * e / f, d);
                        continue;
                    }
                }
                return d;
            }


            public Polygon(ref List<Vertex> Vertices, int index, int id, int Plane_ID = -1, List<Edge> edges = null)
            {
                Plane_index = Plane_ID;
                if (edges != null) Edges = edges;

                top_index = index;

                Centroid = new Hare.Geometry.Point();
                for (int i = 0; i < Vertices.Count; i++) Centroid += Vertices[i];
                Centroid /= Vertices.Count;

                for (int j = 2; j < Vertices.Count; j++)
                {
                    Normal = Hare_math.Cross(Vertices[1] - Vertices[0], Vertices[j] - Vertices[0]);
                    if (!Normal.IsZeroVector()) break;
                }

                Points = new Vertex[Vertices.Count];
                for (int p = 0; p < Vertices.Count; p++)
                {
                    Points[p] = Vertices[p];
                }

                Normal.Normalize();

                diffz = new Vector(Normal.dx, Normal.dy, Normal.dz);
                diffx = new Vector(0, 0, 1);
                double proj = Math.Abs(Hare_math.Dot(diffz, diffx));
                if (0.99 < proj && 1.01 > proj) diffx = new Vector(1, 0, 0);
                diffy = new Vector(diffz.dx, diffz.dy, diffz.dz);
                diffy = Hare_math.Cross(diffy, diffx);
                diffx = Hare_math.Cross(diffy, diffz);
                diffx.Normalize();
                diffy.Normalize();
                diffz.Normalize();

                d = Hare_math.Dot(Normal.dx, Normal.dy, Normal.dz, Points[0].x, Points[0].y, Points[0].z);

                VertexCount = Points.Length;
                Inv_Dot_Normal = 1 / (Hare_math.Dot(Normal, Normal));
                if (VertexCount < 3)
                {
                    IsDegenerate = true;
                }
                Poly_index = id;
                IsConvex = Convexity();
            }

            public int  VertextCT
            {
                get
                {
                   return VertexCount;
                }
            }

            public int Plane_ID
            {
                get
                {
                    return Plane_index;
                }
                set 
                {
                    Plane_index = value;
                }
            }

            public int Poly_ID
            {
                get
                {
                    return this.Poly_index;
                }
            }

            public void reverse()
            {
                Array.Reverse(Points);
            }

            public struct Point2d
            {
                public double x;
                public double y;
                public Point2d(double x_in, double y_in)
                {
                    x = x_in;
                    y = y_in;
                }
            }

            /// <summary>
            /// Given a directed line pq, determine whether qr turns CW or CCW.
            /// </summary>
            /// <param name="p">First vertex...</param>
            /// <param name="q">Second vertex...</param>
            /// <param name="r">Third vertex...</param>
            /// <returns>integer indicating the direction of the polygon...</returns>
            protected int WhichSide2d(Point2d p, Point2d q, Point2d r)
            {
                // Given a directed line pq, determine whether qr turns CW or CCW.
                double result;
                result = (p.x - q.x) * (q.y - r.y) - (p.y - q.y) * (q.x - r.x);
                if ((result < 0)) return -1;
                // q lies to the left  (qr turns CW).
                if ((result > 0)) return 1;
                // q lies to the right (qr turns CCW).
                return 0;
                // q lies on the line from p to r.
            }

            /// <summary>
            /// Lexicographic comparison of p and q.
            /// </summary>
            /// <param name="p">Point p...</param>
            /// <param name="q">Point q...</param>
            /// <returns>returns whether </returns>
            protected int Compare(Point2d p, Point2d q)
            {
                if ((p.x < q.x)) return -1;
                //p is less than q.	
                if ((p.x > q.x)) return 1;
                //p is greater than q.
                if ((p.y < q.y)) return -1;
                //p is less than q.	
                if ((p.y > q.y)) return 1;
                //p is greater than q.
                return 0;
                //p is equal to q.
            }

            /// <summary>
            /// Determines whether or not a polygon is convex.
            /// </summary>
            /// <param name="Vertices">An array of the vertices of the polygon.</param>
            /// <returns>True if convex, false if concave or degenerate.</returns>
            private bool Convexity()
            {
                int curDir;
                int thisDir;
                int thisSign;
                int angleSign = 0;
                int dirChanges = 0;
                int q1;
                int q2;
                int q3;
                List<Point2d> P2ds = new List<Point2d>();

                foreach (Point p in Points)
                {
                    P2ds.Add(new Point2d(p.x, p.y));
                }

                q1 = 0;
                q2 = 1;
                q3 = 1;
                curDir = Compare(P2ds[q1], P2ds[q2]);
                while (q3 < Points.Length - 1)
                {
                    q3 += 1;

                    thisDir = Compare(P2ds[q2], P2ds[q3]);
                    if (thisDir == -curDir)
                    {
                        dirChanges += 1;
                    }
                    curDir = thisDir;
                    thisSign = WhichSide2d(P2ds[q1], P2ds[q2], P2ds[q3]);
                    if (thisSign != 0)
                    {
                        if (angleSign == -thisSign)
                        {
                            return false;
                        }
                        angleSign = thisSign;
                    }
                    q1 = q2;
                    q2 = q3;
                }

                if (Compare(P2ds[q2], P2ds[0]) != 0)
                {
                    q3 = 0;
                    thisDir = Compare(P2ds[q2], P2ds[q3]);
                    if (thisDir == -curDir)
                    {
                        dirChanges += 1;
                    }
                    curDir = thisDir;
                    thisSign = WhichSide2d(P2ds[q1], P2ds[q2], P2ds[q3]);
                    if (thisSign != 0)
                    {
                        if (angleSign == -thisSign)
                        {
                            return false;
                        }
                        angleSign = thisSign;
                    }
                    q1 = q2;
                    q2 = q3;
                }
                q3 = 1;
                thisDir = Compare(P2ds[q2], P2ds[q3]);
                if (thisDir == -curDir)
                {
                    dirChanges += 1;
                }
                curDir = thisDir;
                thisSign = WhichSide2d(P2ds[q1], P2ds[q2], P2ds[q3]);
                if (thisSign != 0)
                {
                    if (angleSign == -thisSign)
                    {
                        return false;
                    }
                    angleSign = thisSign;
                }
                q1 = q2;
                q2 = q3;

                if (dirChanges > 2) return false;
                return true;
            }

            /// <summary>
            /// Ray-Triangle intersection algorithm, based on the algorithm published by Tomas Akenine-Möller, May 2000
            /// </summary>
            /// <param name="orig">Ray origin point</param>
            /// <param name="dir">Ray direction vector</param>
            /// <param name="vert0">First triangle vertex</param>
            /// <param name="vert1">Second triangle vertex</param>
            /// <param name="vert2">Third triangle vertex</param>
            /// <param name="t">t-value along ray where an intersection point has been found (if any).</param>
            /// <param name="u">u-value on triangle where an intersection point has been found (if any).</param>
            /// <param name="v">v-value on triangle where an intersection point has been found (if any).</param>
            /// <returns>True if an intersection was found, false if not.</returns>
            protected bool RayXtri(Ray R, Point vert0, Point vert1, Point vert2, ref double t, ref double u, ref double v)
            {

                /* find vectors for two edges sharing vert0 */
                Vector edge1 = vert1 - vert0;
                Vector edge2 = vert2 - vert0;

                /* begin calculating determinant - also used to calculate U parameter */
                Vector pvec = Hare_math.Cross(R.dx, R.dy, R.dz, edge2.dx, edge2.dy, edge2.dz);

                /* if determinant is near zero, ray lies in plane of triangle */
                double det = Hare_math.Dot(edge1, pvec);

                /* calculate distance from vert0 to ray origin */
                double tvecx = R.x - vert0.x;
                double tvecy = R.y - vert0.y;
                double tvecz = R.z - vert0.z;
                double invdet = 1.0 / det;

                Vector qvec = Hare_math.Cross(tvecx, tvecy, tvecz, edge1.dx, edge1.dy, edge1.dz);

                if (det > 0.000001)
                {
                    u = Hare_math.Dot(tvecx, tvecy, tvecz, pvec.dx, pvec.dy, pvec.dz);
                    if (u < 0.0 || u > det)
                        return false;

                    /* calculate V parameter and test bounds */
                    v = Hare_math.Dot(R.dx, R.dy, R.dz, qvec.dx, qvec.dy, qvec.dz);
                    if (v < 0.0 || u + v > det)
                        return false;
                }
                else if (det < -0.000001)
                {
                    /* calculate U parameter and test bounds */
                    u = Hare_math.Dot(tvecx, tvecy, tvecz, pvec.dx, pvec.dy, pvec.dz);
                    if (u > 0.0 || u < det)
                        return false;
                    /* calculate V parameter and test bounds */
                    v = Hare_math.Dot(R.dx, R.dy, R.dz, qvec.dx, qvec.dy, qvec.dz);
                    if (v > 0.0 || u + v < det)
                        return false;
                }
                else return false;  /* ray is parallell to the plane of the triangle */

                t = Hare_math.Dot(edge2, qvec) * invdet;
                u *= invdet;
                v *= invdet;

                return true;
            }

            /// <summary>
            /// Ray-Triangle intersection algorithm, based on the algorithm published by Tomas Akenine-Möller, May 2000
            /// </summary>
            /// <param name="orig">Ray origin point</param>
            /// <param name="dir">Ray direction vector</param>
            /// <param name="vert0">First triangle vertex</param>
            /// <param name="vert1">Second triangle vertex</param>
            /// <param name="vert2">Third triangle vertex</param>
            /// <param name="t">t-value along ray where an intersection point has been found (if any).</param>
            /// <param name="u">u-value on triangle where an intersection point has been found (if any).</param>
            /// <param name="v">v-value on triangle where an intersection point has been found (if any).</param>
            /// <returns>True if an intersection was found, false if not.</returns>
            protected bool RayXtri(Point orig, Vector dir, Point vert0, Point vert1, Point vert2, ref double t, ref double u, ref double v)
            {

                /* find vectors for two edges sharing vert0 */
                Vector edge1 = vert1 - vert0;
                Vector edge2 = vert2 - vert0;

                /* begin calculating determinant - also used to calculate U parameter */
                Vector pvec = Hare_math.Cross(dir, edge2);

                /* if determinant is near zero, ray lies in plane of triangle */
                double det = Hare_math.Dot(edge1, pvec);

                /* calculate distance from vert0 to ray origin */
                Vector tvec = orig - vert0;
                double invdet = 1.0 / det;

                Vector qvec = Hare_math.Cross(tvec, edge1);

                if (det > 0.000001)
                {
                    u = Hare_math.Dot(tvec, pvec);
                    if (u < 0.0 || u > det)
                        return false;

                    /* calculate V parameter and test bounds */
                    v = Hare_math.Dot(dir, qvec);
                    if (v < 0.0 || u + v > det)
                        return false;
                }
                else if (det < -0.000001)
                {
                    /* calculate U parameter and test bounds */
                    u = Hare_math.Dot(tvec, pvec);
                    if (u > 0.0 || u < det)
                        return false;
                    /* calculate V parameter and test bounds */
                    v = Hare_math.Dot(dir, qvec);
                    if (v > 0.0 || u + v < det)
                        return false;
                }
                else return false;  /* ray is parallell to the plane of the triangle */

                t = Hare_math.Dot(edge2, qvec) * invdet;
                u *= invdet;
                v *= invdet;

                return true;
            }
            
            /// <summary>
            /// Finds the distance between a specified point and the closest point on the plane of a polygon.
            /// </summary>
            /// <param name="q">The point to compare to the plane of the polygon.</param>
            /// <returns>The distance between a point and the plane of the polygon.</returns>
            public double DistToPlane(Point q) 
            {
                return (Hare_math.Dot(Normal.dx, Normal.dy, Normal.dz, q.x, q.y, q.z) - d) * Inv_Dot_Normal;
            }

            /// <summary>
            /// Returns a boolean indicating which side of a polygon a ray is located on.
            /// </summary>
            /// <param name="Dir">A vector indicating the direction of the ray.</param>
            /// <returns>True if the ray is on the correct side of the polygon to indicate an intersection point.</returns>
            protected bool Ray_Side(Vector Dir)
            {
                double n = Hare_math.Dot(Dir, Normal);
                if (n < 0) { return false; }
                return true;
            }

            /// <summary>
            /// Returns a boolean indicating which side of a polygon a ray is located on.
            /// </summary>
            /// <param name="Dir">A vector indicating the direction of the ray.</param>
            /// <returns>True if the ray is on the correct side of the polygon to indicate an intersection point.</returns>
            protected bool Ray_Side(double dx, double dy, double dz)
            {
                double n = Hare_math.Dot(dx, dy, dz, Normal.dx, Normal.dy, Normal.dz);
                if (n < 0) { return false; }
                return true;
            }

            /// <summary>
            /// Finds the closest point on the plane of a polygon from a specified point.
            /// </summary>
            /// <param name="q">The point to be compared.</param>
            /// <returns>The closest point on the plane.</returns>
            public Point ClosestPtPointPlane(Point q)
            {
                double t = Hare_math.Dot(Normal.dx, Normal.dy, Normal.dz, q.x, q.y, q.z) - d;
                return q - (t * Normal);
            }
        }

        /// <summary>
        /// A triangle derived from the Polygon class.
        /// </summary>
        public class Triangle : Polygon
        {
            public Triangle(ref List<Vertex> Vertices, int index, int id, int PlaneID)
                : base(ref Vertices, index, id, PlaneID)
            {
                if (VertexCount != 3) throw new ApplicationException("Faulty Triangle");
            }
            
            public Triangle(ref List<Vertex> Vertices, int index, int id)
                : base(ref Vertices, index, id)
            {
                if (VertexCount != 3) throw new ApplicationException("Faulty Triangle");
            }

            public override bool Intersect(Ray R, Point[] P, out Point Xpt, out double u, out double v, out double t, out int polyid)
            {
                u = 0; v = 0; t = 0;
                bool Intersects;
                if (Ray_Side(R.dx, R.dy, R.dz))
                {
                    Intersects = RayXtri(R, P[0], P[1], P[2], ref t, ref u, ref v);
                }
                else
                {
                    Intersects = RayXtri(R, P[2], P[1], P[0], ref t, ref u, ref v);
                }

                if (Intersects)
                {
                    //I = new X_Event(R.origin + R.direction * t, u, v, t, this.Poly_index);
                    Xpt = new Point(R.x + R.dx * t, R.y + R.dy * t, R.z + R.dz * t);
                    polyid = this.Poly_index;
                    return true;
                }
                else
                {
                    Xpt = default(Point);
                    polyid = this.Poly_index;
                    return false;
                }
            }

            public override Point GetRandomPoint(double urand, double vrand, double wrand)
            {
                double tmp = Math.Sqrt(urand);
                double u = 1 - tmp;
                double v = vrand * tmp;
                return Points[0] + u * (Points[1] - Points[0]) + v * (Points[2] - Points[0]);
            }

            public override double closestpoint(Point p)
            {
                Point p1 = triclosestpoint(0,1,2,p);
                Vector p1p = p1 - p;
                return p1p.dx*p1p.dx + p1p.dy*p1p.dy + p1p.dz*p1p.dz;
            }
        }

        /// <summary>
        /// A quadrilateral derived from the Polygon class.
        /// </summary>
        public class Quadrilateral : Polygon
        {
            private double[] area_Fraction = new double[2];

            public Quadrilateral(ref List<Vertex> Vertices, int index, int id, int PlaneID)
                : base(ref Vertices, index, id, PlaneID)
            {
                if (VertexCount != 3) throw new ApplicationException("Faulty Triangle");
            }

            public Quadrilateral(ref List<Vertex> Vertices, int index, int id)
                : base(ref Vertices, index, id)
            {
                if (VertexCount != 4) throw new ApplicationException("Faulty Quadrilateral");

                area_Fraction[0] = 0.5 *Hare_math.Cross((Points[1] - Points[0]),(Points[2] - Points[0])).Length();
                area_Fraction[1] = 0.5 * Hare_math.Cross((Points[2] - Points[3]),(Points[1] - Points[3])).Length();
                double Area = area_Fraction[0] + area_Fraction[1];
                area_Fraction[0] /= Area;
                area_Fraction[1] /= Area;
            }

            public override bool Intersect(Ray R, Point[] P, out Point Xpt, out double u, out double v, out double t, out int polyid)
            {
                u = 0; v = 0; t = 0;
                if (Ray_Side(R.dx, R.dy, R.dz))
                {
                    if (RayXtri(R, P[0], P[1], P[2], ref t, ref u, ref v))
                    {
                        //I = new X_Event(R.origin + R.direction * t, u, v, t, this.Poly_index);
                        Xpt = new Point( R.x + R.dx * t, R.y + R.dy * t, R.z + R.dz * t) ;
                        polyid = this.Poly_index;
                        return true;
                    }
                    else if (RayXtri(R, P[2], P[3], P[0], ref t, ref u, ref v))
                    {
                        //I = new X_Event(R.origin + R.direction * t, u, v, t, this.Poly_index);
                        Xpt = new Point(R.x + R.dx * t, R.y + R.dy * t, R.z + R.dz * t);
                        polyid = this.Poly_index;
                        return true;
                    }
                    else
                    {
                        //I = new X_Event(this.Poly_index);
                        Xpt = default(Point);
                        polyid = this.Poly_index;
                        return false;
                    }
                }
                else
                {
                    if (RayXtri(R, P[2], P[1], P[0], ref t, ref u, ref v))
                    {
                        //I = new X_Event(R.origin + R.direction * t, u, v, t, this.Poly_index);
                        Xpt = new Point(R.x + R.dx * t, R.y + R.dy * t, R.z + R.dz * t);
                        polyid = this.Poly_index;
                        return true;
                    }
                    else if (RayXtri(R, P[0], P[3], P[2], ref t, ref u, ref v))
                    {
                        //I = new X_Event(R.origin + R.direction * t, u, v, t, this.Poly_index);
                        Xpt = new Point(R.x + R.dx * t, R.y + R.dy * t, R.z + R.dz * t);
                        polyid = this.Poly_index;
                        return true;
                    }
                    else
                    {
                        //I = new X_Event(this.Poly_index);
                        Xpt = default(Point);
                        polyid = this.Poly_index;
                        return false;
                    }
                }
            }

            public override double closestpoint(Point p)
            {
                Point p1 = triclosestpoint(0, 1, 2, p);
                Vector p1p = p1 - p;
                double p1ps = p1p.dx * p1p.dx + p1p.dy * p1p.dy + p1p.dz * p1p.dz;
                if(p1ps < 0.01) return p1ps;

                Point p2 = triclosestpoint(0, 3, 2, p);
                Vector p2p = p2 - p;
                double p2ps = p2p.dx * p2p.dx + p2p.dy * p2p.dy + p2p.dz * p2p.dz;
                return Math.Min(p1ps, p2ps);
            }


            public override Point GetRandomPoint(double urand, double vrand, double Side)
            {
                //double u = urand;
                //double v = (1.0 - u) * vrand;
                //double w = 1 - u - v;

                double tmp = Math.Sqrt(urand);
                double u = 1 - tmp;
                double v = vrand*tmp;
                //double w = 1 - x - y;

                //if (Side < area_Fraction[0]) return (u * Points[0] + v * Points[1] + w * Points[2]);
                //else return (u * Points[0] + v * Points[2] + w * Points[3]);

                if (Side < area_Fraction[0])
                {
                    return Points[0] + u*(Points[1] - Points[0]) + v*(Points[2] - Points[0]);
                }

                else
                {
                    return Points[0] + u*(Points[2] - Points[0]) + v*(Points[3] - Points[0]);
                }
            }
        }

        /// <summary>
        /// An unfinished class designed to handle polygons of higher numbers of sides...
        /// Care to finish it?
        /// </summary>
        public class Arb_Poly : Polygon
        {
            public Arb_Poly(ref List<Vertex> Vertices, int[] Ps, int index, int id) : base(ref Vertices, index, id) { }

            public override bool Intersect(Ray R, Point[] P, out Point Xpt, out double u, out double v, out double t, out int polyid)
            {
                Xpt = default(Point);
                u = 0;
                v = 0;
                t = 0;
                polyid = Poly_index;
                return true;
            }

            //public bool pnpoly(Point P)
            //{
            //    int i, j;
            //    bool c = false;
            //    for (i = 0, j = VertexCount - 1; i < VertexCount; j = i++)
            //    {
            //        if ((((Vertex(i).y <= P.y) && (P.y < Vertex(j).y)) || 
            //            ((Vertex(j).y <= P.y) && (P.y < Vertex(i).y))) &&
            //            (P.x < (Vertex(j).x - Vertex(i).x) * (P.y - Vertex(i).y) / (Vertex(j).y - Vertex(i).y) + Vertex(i).x))
            //            c = !c;
            //    }
            //    return c;
            //}

            public override double closestpoint(Point p)
            {
                throw new NotImplementedException();
            }

            public override Point GetRandomPoint(double urand, double vrand, double Side)
            {
                return new Point();
            }
        }
    }    
}