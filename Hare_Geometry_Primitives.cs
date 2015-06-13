//'Hare: Accelerated Multi-Resolution Ray Tracing (GPL) by Arthur van der Harten
//'
//'Copyright (c) 2008, 2009, Arthur van der Harten			
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
namespace Hare
{
    namespace Geometry
    {
        /// <summary>
        /// Simple vector type variable, complete with certain common operators...
        /// </summary>
        public class Vector : Point
        {
            public Vector(double x_in, double y_in, double z_in)
                : base(x_in, y_in, z_in)
            {
            }

            public Vector()
                : base()
            {
            }

            public void Normalize()
            {
                double factor = x * x + y * y + z * z;
                if (factor == 0) return;
                factor = Math.Sqrt(factor);
                x /= factor;
                y /= factor;
                z /= factor;
            }

            public double Length()
            {
                return Math.Sqrt(x * x + y * y + z * z);
            }

            public static Point operator +(Vector P, Point Q)
            {
                return new Point(P.x + Q.x, P.y + Q.y, P.z + Q.z);
            }

            public static Vector operator +(Vector P, Vector Q)
            {
                return new Vector(P.x + Q.x, P.y + Q.y, P.z + Q.z);
            }

            public static Vector operator -(Vector P, Point Q)
            {
                return new Vector(P.x - Q.x, P.y - Q.y, P.z - Q.z);
            }

            public static Vector operator *(Vector P, double Q)
            {
                return new Vector(P.x * Q, P.y * Q, P.z * Q);
            }

            public static Vector operator *(double Q, Vector P)
            {
                return new Vector(P.x * Q, P.y * Q, P.z * Q);
            }

            public static Vector operator /(Vector P, double Q)
            {
                return new Vector(P.x / Q, P.y / Q, P.z / Q);
            }
        }

        /// <summary>
        /// Simple point type variable, complete with certain common operators...
        /// </summary>
        public class Point //: IDisposable
        {
            public double x;
            public double y;
            public double z;

            int code;

            public Point(double x_in, double y_in, double z_in)
            {
                x = x_in;
                y = y_in;
                z = z_in;

                code = (int)((((497 + x*1000) * 71 + y*1000) * 71) + z*1000);
            }

            public Point()
            {
            }

            public static Point operator +(Point P, Point Q)
            {
                return new Point(P.x + Q.x, P.y + Q.y, P.z + Q.z);
            }

            public static Vector operator -(Point P, Point Q)
            {
                return new Vector(P.x - Q.x, P.y - Q.y, P.z - Q.z);
            }

            public static Point operator *(Point P, double Q)
            {
                return new Point(P.x * Q, P.y * Q, P.z * Q);
            }

            public static Point operator *(double Q, Point P)
            {
                return new Point(P.x * Q, P.y * Q, P.z * Q);
            }

            public static Point operator /(Point P, double Q)
            {
                return new Point(P.x / Q, P.y / Q, P.z / Q);
            }

            /// <summary>
            /// Access x(0), y(1), z(2) variables by integers. (ie. for use with integer based loops)
            /// </summary>
            /// <param name="i"> Integer indicating which variable to return. x(0) : y(1) : z(2) </param>
            /// <returns> The double value of the variable. </returns>
            public double byint(int i)
            {
                switch (i)
                {
                    case 0:
                        return x;
                    case 1:
                        return y;
                    case 2:
                        return z;
                }
                throw new Exception("Misrepresented Point-Dimension Index");
            }

            /// <summary>
            /// Checks for a vector of infinitesimal length.
            /// </summary>
            /// <returns> True or False... </returns>
            public bool IsZeroVector()
            {
                if ((x * x + y * y + z * z) < double.Epsilon) return true;
                return false;
            }

            /// <summary>
            /// Rounds the variables of the point to a given precision... (Not adviseable...)
            /// </summary>
            /// <param name="Precision">The number of siginificant digits to round to...</param>
            public void Round(int Precision)
            {
                x = Math.Round(x, Precision);
                y = Math.Round(y, Precision);
                z = Math.Round(z, Precision);
            }

            public void Hash2(Topology.MS_AABB Modelspace, out ulong bucket, out ulong pos)
            {
                double Xoff = x - Modelspace.Min.x, Yoff = y - Modelspace.Min.y, Zoff = z - Modelspace.Min.z;

                UInt64 xlocus = (UInt64)Math.Floor(Xoff);
                UInt64 ylocus = (UInt64)Math.Floor(Yoff);
                UInt64 zlocus = (UInt64)Math.Floor(Zoff);

                bucket = (ulong)Modelspace.XYTot * zlocus + (ulong)Modelspace.ydim * xlocus + ylocus;

                UInt64 xpos = (UInt64)((Xoff - xlocus) * 1000), ypos = (UInt64)((Yoff - ylocus) * 1000), zpos = (UInt64)((Zoff - zlocus) * 1000);

                pos = 1000000 * zpos + 1000 * xpos + ypos;
            }
        }
        /// <summary>
        /// A ray class storing all relevant data to accurately cast a ray.
        /// </summary>
        public class Ray
        {
            public Ray(Point o, Vector d, int Thread_ID, int ID_IN)
            {
                Ray_ID = ID_IN;
                origin = o;
                direction = d;
                ThreadID = Thread_ID;
                a = 0;
            }
            public int ThreadID;
            public Point origin;
            public Vector direction;
            public int a;
            public int Ray_ID;
        }

        /// <summary>
        /// A structure containing all data about the intersection resulting from a "shoot" operation in a spatial partition class.
        /// </summary>
        public class X_Event
        {
            public readonly double u;
            public readonly double v;
            public double t;
            public readonly bool Hit;
            public readonly Point X_Point;
            public readonly int Poly_id;
            /// <summary>
            /// Used to represent either an empty intersection event, or a no-intersection event...
            /// </summary>
            public X_Event(int Poly_index)
            {
                X_Point = default(Point);
                t = 0;
                Hit = false;
                Poly_id = Poly_index;
            }

            public X_Event() 
            {
                u = 0;
                v = 0;
                t = 0;
                Hit = false;
                X_Point = default(Point);
                Poly_id = -1;
            }

            /// <summary>
            /// Used to store information about a discovered intersection event...
            /// </summary>
            /// <param name="P"> The intersection point. </param>
            /// <param name="u_in"> U coordinate of the intersection. </param>
            /// <param name="v_in"> V coordinate of the intersection.</param>
            /// <param name="t_in"> The distance along the ray in units related to the ray's length. If raylength is 1, this is the actual travel distance of the ray in model units. </param>
            /// <param name="Poly_index"> The index of the polygon upon which the intersection point may be found. </param>
            public X_Event(Point P, double u_in, double v_in, double t_in, int Poly_index)
            {
                u = u_in;
                v = v_in;
                t = t_in;
                Hit = true;
                X_Point = P;
                Poly_id = Poly_index;
            }
        }

        public class plane 
        {
            double a, b, c, d;
            int code;

            public plane(Polygon p)
            {
                a = p.Normal.x;
                b = p.Normal.y;
                c = p.Normal.z;
                d = -(a * p.Points[0].x + b * p.Points[0].y + c * p.Points[0].z);

                //a = (float)(p.Points[0].y * (p.Points[1].z - p.Points[2].z) + p.Points[2].y * (p.Points[2].z - p.Points[0].z) + p.Points[2].y * (p.Points[0].z - p.Points[1].z));
                //b = (float)(p.Points[0].z * (p.Points[1].x - p.Points[2].x) + p.Points[1].z * (p.Points[2].x - p.Points[0].x) + p.Points[2].z * (p.Points[0].x - p.Points[1].x));
                //c = (float)(p.Points[0].x * (p.Points[1].y - p.Points[2].y) + p.Points[1].x * (p.Points[2].y - p.Points[0].y) + p.Points[2].x * (p.Points[0].y - p.Points[1].y));
                //d = (float)-(p.Points[0].x * (p.Points[1].y * p.Points[2].z - p.Points[2].y * p.Points[1].z) + p.Points[1].x * (p.Points[2].y * p.Points[0].z - p.Points[0].y * p.Points[2].z) + p.Points[2].x * (p.Points[0].y * p.Points[1].z - p.Points[1].y * p.Points[0].z));

                if (d < 0)
                {
                    a *= -1;
                    b *= -1;
                    c *= -1;
                    d *= -1;
                }

                code = (int)(((Math.Round(d,3).GetHashCode() * 7 + Math.Round(a,3).GetHashCode()) * 11 + Math.Round(b,3).GetHashCode()) * 17 + Math.Round(c,3).GetHashCode());
            }

            public override int GetHashCode()
            {
                return code;
            }
        }
    }
}