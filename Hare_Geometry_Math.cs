//'Hare: Accelerated Multi-Resolution Ray Tracing (GPL)
//'
//'Copyright (c) 2008 - 2025, Open Research in Acoustical Science and Education, Inc. - a 501(c)3 nonprofit			
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
using System.Numerics;

namespace Hare
    {
    namespace Geometry
    {
        /// <summary>
        /// A shared class containing several useful geometrical mathematical functions.
        /// </summary>
        public static class Hare_math
        {
            /// <summary>
            /// The dot product of two vectors.
            /// </summary>
            /// <param name="a"> Point or vector A... </param>
            /// <param name="b"> Point or vector B... </param>
            /// <returns> A double value from 0 to 1. Two identical vectors produce a value of 1. Two perpendicular vectors produce a value of 0. Two exact opposite vectors produce a value of -1.</returns>
            public static double Dot(Point a, Point b)
            {
                return (a.x * b.x) + (a.y * b.y) + (a.z * b.z);
            }
            public static double Dot(Vector a, Vector b)
            {
                return (a.dx * b.dx) + (a.dy * b.dy) + (a.dz * b.dz);
            }
            public static double Dot(double ax, double ay, double az, double bx, double by, double bz)
            {
                return (ax * bx) + (ay * by) + (az * bz);
            }

            /// <summary>
            /// The cross product of 2 vectors.
            /// </summary>
            /// <param name="a"> Point or vector A... </param>
            /// <param name="b"> Point or vector B... </param>
            /// <returns> A vector perpendicular to the input vectors. </returns>
            public static Vector Cross(Point a, Point b)
            {
                return new Vector(a.y * b.z - a.z * b.y, -(a.x * b.z - a.z * b.x), a.x * b.y - a.y * b.x);
            }
            public static Vector Cross(Point a, Vector b)
            {
                return new Vector(a.y * b.dz - a.z * b.dy, -(a.x * b.dz - a.z * b.dx), a.x * b.dy - a.y * b.dx);
            }
            public static Vector Cross(Vector a, Vector b)
            {
                return new Vector(a.dy * b.dz - a.dz * b.dy, -(a.dx * b.dz - a.dz * b.dx), a.dx * b.dy - a.dy * b.dx);
            }
            public static Vector Cross(double ax, double ay, double az, double bx, double by, double bz)
            {
                return new Vector(ay * bz - az * by, -(ax * bz - az * bx), ax * by - ay * bx);
            }
            public static void Cross(double ax, double ay, double az, double bx, double by, double bz, out double dx, out double dy, out double dz)
            {
                dx = ay * bz - az * by; dy = -(ax * bz - az * bx); dz = ax * by - ay * bx;
            }


            /// <summary>
            /// Dot(A, Cross(B, C))
            /// </summary>
            /// <param name="A">Point or vector A...</param>
            /// <param name="B">Point or vector B...</param>
            /// <param name="C">Point or vector C...</param>
            /// <returns></returns>
            public static double ScalarTriple(Vector A, Vector B, Vector C)
            {
                return Dot(A, Cross(B, C));
            }

            public static double distance(double x1, double y1, double z1, double x2, double y2, double z2)
            {
                double dx = x2 - x1, dy = y2 - y1, dz = z2 - z1;
                return Math.Sqrt(dx * dx + dy * dy + dz * dz);
            }

            public static void Normalize(ref double dx, ref double dy, ref double dz)
            {
                double factor = dx * dx + dy * dy + dz * dz;
                if (factor == 0) return;
                factor = Math.Sqrt(factor);
                dx /= factor;
                dy /= factor;
                dz /= factor;
            }
        }

        /// <summary>
        /// This is a collection of tools used to check geometry for defects before creating a topology. 
        /// They are provided outside of the topology class in order to allow the programmer to maintain 
        /// an accurate list of object properties. If the corrections were implemented within the topology 
        /// class, this would be difficult, as while the programmer's object properties were already built, 
        /// the geometry system in Hare would change.
        /// </summary>

        public static class Corrective_Tools
        {
           /// <summary>
           /// Determines whether or not all triangular subdivisions of a convex polygon can be considered coplanar.
           /// </summary>
           /// <param name="P"></param>
           /// <returns></returns>
            public static bool IsCoPlanar(Point[] P)
            {
                if (P.Length > 3) 
                {
                    Vector First_Tri_CP = Hare_math.Cross(P[1] - P[0], P[2] - P[0]);
                    First_Tri_CP.Normalize();
                    for (int j = 2, k = 3; k < P.Length; j++, k++)
                    {
                        Vector V = Hare_math.Cross(P[j] - P[0], P[k] - P[0]);
                        double x = Hare_math.Dot(First_Tri_CP, V);
                        if (x < 1) return false;
                    }
                }
                return true;
            }
        }
    }
}
