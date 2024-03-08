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
namespace Hare
{
    namespace Geometry
    {
        public partial class AABB
        {
            /// Translated from c code from Tomas Akenine-Möller's version. 
            /// Information on the original can be found below:
            /// 
            /********************************************************/
            /* AABB-triangle overlap test code                      */
            /* by Tomas Akenine-Möller                              */
            /* Function: int triBoxOverlap(float boxcenter[3],      */
            /*          float halfwidth[3],float triverts[3][3]); */
            /* History:                                             */
            /*   2001-03-05: released the code in its first version */
            /*   2001-06-18: changed the order of the tests, faster */
            /*                                                      */
            /* Acknowledgement: Many thanks to Pierre Terdiman for  */
            /* suggestions and discussions on how to optimize code. */
            /* Thanks to David Hunt for finding a ">="-bug!         */
            /********************************************************/
            
            private void FINDMINMAX(double x0, double x1, double x2, ref double min, ref double max)
            {
                min = x0;
                max = x0;
                if (x1 < min) min = x1;
                if (x1 > max) max = x1;
                if (x2 < min) min = x2;
                if (x2 > max) max = x2;
            }

            private bool planeBoxOverlap(Vector normal, Point vert, Point maxbox)
            {
                Point vmin = new Point(0,0,0), vmax = new Point(0,0,0);
                double v;

                v = vert.x;
                if (normal.dx > 0.0f)
                {
                    vmin.x = -maxbox.x - v;
                    vmax.x = maxbox.x - v;
                }
                else
                {
                    vmin.x = maxbox.x - v;
                    vmax.x = -maxbox.x - v;
                }

                v = vert.y;
                if (normal.dy > 0.0f)
                {
                    vmin.y = -maxbox.y - v;
                    vmax.y = maxbox.y - v;
                }
                else
                {
                    vmin.y = maxbox.y - v;
                    vmax.y = -maxbox.y - v;
                }

                v = vert.z;
                if (normal.dz > 0.0f)
                {
                    vmin.z = -maxbox.z - v;
                    vmax.z = maxbox.z - v;
                }
                else
                {
                    vmin.z = maxbox.z - v;
                    vmax.z = -maxbox.z - v;
                }

                if (Hare_math.Dot(normal.dx, normal.dy, normal.dz, vmin.x, vmin.y, vmin.z) > 0.0f) return false;
                if (Hare_math.Dot(normal.dx, normal.dy, normal.dz, vmax.x, vmax.y, vmax.z) >= 0.0f) return true;
                return false;
            }

            Point v0, v1, v2;
            double min, max, rad, p0, p1, p2;

            /*======================== X-tests ========================*/
            private bool AXISTEST_X01(double a, double b, double fa, double fb)
            {
                p0 = a * v0.y - b * v0.z;
                p2 = a * v2.y - b * v2.z;
                if (p0 < p2) { min = p0; max = p2; } else { min = p2; max = p0; }
                rad = fa * halfwidth.y + fb * halfwidth.z;
                if (min > rad || max < -rad) return false;
                return true;
            }

            private bool AXISTEST_X2(double a, double b, double fa, double fb)
            {
                p0 = a * v0.y - b * v0.z;
                p1 = a * v1.y - b * v1.z;
                if (p0 < p1) { min = p0; max = p1; } else { min = p1; max = p0; }
                rad = fa * halfwidth.y + fb * halfwidth.z;
                if (min > rad || max < -rad) return false;
                return true;
            }

            /*======================== Y-tests ========================*/
            private bool AXISTEST_Y02(double a, double b, double fa, double fb)
            {
                p0 = -a * v0.x + b * v0.z;
                p2 = -a * v2.x + b * v2.z;
                if (p0 < p2) { min = p0; max = p2; } else { min = p2; max = p0; }
                rad = fa * halfwidth.x + fb * halfwidth.z;
                if (min > rad || max < -rad) return false;
                return true;
            }

            private bool AXISTEST_Y1(double a, double b, double fa, double fb)
            {
                p0 = -a * v0.x + b * v0.z;
                p1 = -a * v1.x + b * v1.z;
                if (p0 < p1) { min = p0; max = p1; } else { min = p1; max = p0; }
                rad = fa * halfwidth.x + fb * halfwidth.z;
                if (min > rad || max < -rad) return false;
                return true;
            }

            /*======================== Z-tests ========================*/

            private bool AXISTEST_Z12(double a, double b, double fa, double fb)
            {
                p1 = a * v1.x - b * v1.y;
                p2 = a * v2.x - b * v2.y;
                if (p2 < p1) { min = p2; max = p1; } else { min = p1; max = p2; }
                rad = fa * halfwidth.x + fb * halfwidth.y;
                if (min > rad || max < -rad) return false;
                return true;
            }

            private bool AXISTEST_Z0(double a, double b, double fa, double fb)
            {
                p0 = a * v0.x - b * v0.y;
                p1 = a * v1.x - b * v1.y;
                if (p0 < p1) { min = p0; max = p1; } else { min = p1; max = p0; }
                rad = fa * halfwidth.x + fb * halfwidth.y;
                if (min > rad || max < -rad) return false;
                return true;
            }


            public bool PolyBoxOverlap(Point[] P)
            {
                Point[][] triangles = new Point[P.Length-2][];
                
                for(int q = 0, j = 1, k = 2; k < P.Length; q++, j++, k++)
                { 
                    triangles[q] = new Point[]{ P[0], P[j], P[k] };
                }

                foreach (Point[] triverts in triangles)
                {

                    /*    use separating axis theorem to test overlap between triangle and box */
                    /*    need to test for overlap in these directions: */
                    /*    1) the {x,y,z}-directions (actually, since we use the AABB of the triangle */
                    /*       we do not even need to test these) */
                    /*    2) normal of the triangle */
                    /*    3) crossproduct(edge from tri, {x,y,z}-directin) */
                    /*       this gives 3x3=9 more tests */

                    v0 = new Point(0,0,0);
                    v1 = new Point(0,0,0);
                    v2 = new Point(0,0,0);

                    //   float axis[3];
                    double fex, fey, fez;		// -NJMP- "d" local variable removed
                    Vector normal, e0, e1, e2;

                    /* This is the fastest branch on Sun */
                    /* move everything so that the boxcenter is in (0,0,0) */
                    v0.x = triverts[0].x - Center.x;
                    v0.y = triverts[0].y - Center.y;
                    v0.z = triverts[0].z - Center.z;
                    v1.x = triverts[1].x - Center.x;
                    v1.y = triverts[1].y - Center.y;
                    v1.z = triverts[1].z - Center.z;
                    v2.x = triverts[2].x - Center.x;
                    v2.y = triverts[2].y - Center.y;
                    v2.z = triverts[2].z - Center.z;

                    /* compute triangle edges */
                    e0 = v1 - v0;     /* tri edge 0 */
                    e1 = v2 - v1;     /* tri edge 1 */
                    e2 = v0 - v2;     /* tri edge 2 */

                    /* Bullet 3:  */
                    /*  test the 9 tests first (this was faster) */
                    fex = Math.Abs(e0.dx);
                    fey = Math.Abs(e0.dy);
                    fez = Math.Abs(e0.dz);
                    if (!AXISTEST_X01(e0.dz, e0.dy, fez, fey)) continue;
                    if (!AXISTEST_Y02(e0.dz, e0.dx, fez, fex)) continue;
                    if (!AXISTEST_Z12(e0.dy, e0.dx, fey, fex)) continue;

                    fex = Math.Abs(e1.dx);
                    fey = Math.Abs(e1.dy);
                    fez = Math.Abs(e1.dz);
                    if (!AXISTEST_X01(e1.dz, e1.dy, fez, fey)) continue;
                    if (!AXISTEST_Y02(e1.dz, e1.dx, fez, fex)) continue;
                    if (!AXISTEST_Z0(e1.dy, e1.dx, fey, fex)) continue;

                    fex = Math.Abs(e2.dx);
                    fey = Math.Abs(e2.dy);
                    fez = Math.Abs(e2.dz);
                    if (!AXISTEST_X2(e2.dz, e2.dy, fez, fey)) continue;
                    if (!AXISTEST_Y1(e2.dz, e2.dx, fez, fex)) continue;
                    if (!AXISTEST_Z12(e2.dy, e2.dx, fey, fex)) continue;

                    /* Bullet 1: */
                    /*  first test overlap in the {x,y,z}-directions */
                    /*  find min, max of the triangle each direction, and test for overlap in */
                    /*  that direction -- this is equivalent to testing a minimal AABB around */
                    /*  the triangle against the AABB */

                    /* test in X-direction */
                    FINDMINMAX(v0.x, v1.x, v2.x, ref min, ref max);
                    if (min > halfwidth.x || max < -halfwidth.x) continue;

                    /* test in Y-direction */
                    FINDMINMAX(v0.y, v1.y, v2.y, ref min, ref max);
                    if (min > halfwidth.y || max < -halfwidth.y) continue;

                    /* test in Z-direction */
                    FINDMINMAX(v0.z, v1.z, v2.z, ref min, ref max);
                    if (min > halfwidth.z || max < -halfwidth.z) continue;

                    /* Bullet 2: */
                    /*  test if the box intersects the plane of the triangle */
                    /*  compute plane equation of triangle: normal*x+d=0 */

                    normal = Hare_math.Cross(e0, e1);
                    if (!planeBoxOverlap(normal, v0, halfwidth)) continue;	// -NJMP-
                    return true;   /* box and triangle overlaps */
                }
                return false;
            }


            // Given point p, return the point q on or in AABB b, that is closest to p

            public Point ClosestPt(Point p)
            {
                // For each coordinate axis, if the point coordinate value is
                // outside box, clamp it to the box, else keep it as is

                Point q = new Point();

                double v = p.x;
                if (v < Min.x) v = Min.x; // v = max(v, b.min[i])
                if (v > Max.x) v = Max.x; // v = min(v, b.max[i])
                q.x = v;

                v = p.y;
                if (v < Min.y) v = Min.y; // v = max(v, b.min[i])
                if (v > Max.y) v = Max.y; // v = min(v, b.max[i])
                q.y = v;

                v = p.z;
                if (v < Min.z) v = Min.z; // v = max(v, b.min[i])
                if (v > Max.z) v = Max.z; // v = min(v, b.max[i])
                q.z = v;

                return q;
            }
        }
    }
}