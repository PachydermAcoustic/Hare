//'Hare: Accelerated Multi-Resolution Ray Tracing (GPL)
//'
//'Copyright (c) 2008 - 2015, Arthur van der Harten			
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
                return new Vector(a.y * b.z - a.z * b.y, -(a.x * b.z - a.z * b.x), a.x * b.y - a.y * b.x);
            }
            public static Vector Cross(Vector a, Vector b)
            {
                return new Vector(a.y * b.z - a.z * b.y, -(a.x * b.z - a.z * b.x), a.x * b.y - a.y * b.x);
            }

            /// <summary>
            /// Dot(A, Cross(B, C))
            /// </summary>
            /// <param name="A">Point or vector A...</param>
            /// <param name="B">Point or vector B...</param>
            /// <param name="C">Point or vector C...</param>
            /// <returns></returns>
            public static double ScalarTriple(Point A, Point B, Point C)
            {
                return Dot(A, Cross(B, C));
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

        //    //public static Vector Normalize(Vector V)
        //    //{
        //    //    V.Normalize();
        //    //    return V;
        //    //}
        }
    }
}
