//'Hare: Accelerated Multi-Resolution Ray Tracing (GPL)
//'
//'Copyright (c) 2008 - 2025, Arthur van der Harten			
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
        /// An Axis-aligned Bounding Box class.
        /// </summary>
        public partial class AABB
        {
            protected internal Point Max;
            protected internal Point Min;
            protected Point[] Bounds;
            protected internal Point Center;
            protected internal Vector Width;
            protected internal Point Inv_Width;
            protected internal Point halfwidth;

            /// <summary>
            /// Bounding Box Constructor.
            /// </summary>
            /// <param name="Min_in">The low point of the box.</param>
            /// <param name="Max_in">The high point of the box.</param>
            public AABB(double Minx, double Miny, double Minz , double Maxx, double Maxy, double Maxz)
            {
                Min = new Point(Minx, Miny, Minz);
                Max = new Point(Maxx ,Maxy, Maxz);
                Bounds = new Point[2];
                Bounds[0] = Min;
                Bounds[1] = Max;
                Center = (Max + Min) / 2;
                Width = Max - Min;
                Inv_Width = new Point(1 / Width.dx, 1 / Width.dy, 1 / Width.dz);
                halfwidth = new Point(Width.dx, Width.dy, Width.dz) / 2;
            }

            /// <summary>
            /// Bounding Box Constructor.
            /// </summary>
            /// <param name="Min_in">The low point of the box.</param>
            /// <param name="Max_in">The high point of the box.</param>
            public AABB(Point Min_in, Point Max_in)
            {
                Max = Max_in;
                Min = Min_in;
                Bounds = new Point[2];
                Bounds[0] = Min;
                Bounds[1] = Max;
                Center = (Max + Min) / 2;
                Width = Max - Min;
                Inv_Width = new Point(1 / Width.dx, 1 / Width.dy,1 / Width.dz);
                halfwidth = new Point (Width.dx, Width.dy, Width.dz) / 2;
            }

            /// <summary>
            /// Determines whether or not a specified point is inside this Bounding Box.
            /// </summary>
            /// <param name="P">The Point.</param>
            /// <returns>True if inside, false if not.</returns>
            public bool IsPointInBox(double x, double y, double z) 
            {
                if (x < Min.x) return false;
                if (y < Min.y) return false;
                if (z < Min.z) return false;
                if (x > Max.x) return false;
                if (y > Max.y) return false;
                if (z > Max.z) return false;
                return true;
            }

            public bool Intersect(Ray R, ref double tmin, ref Point P)
            {
                tmin = 0;          // set to -FLT_MAX to get first hit on line
                double tmax = double.MaxValue; // set to max distance ray can travel (for segment)

                // For all three slabs
                if (System.Math.Abs(R.dx) < double.Epsilon)
                {
                    // Ray is parallel to slab. No hit if origin not within slab
                    if (R.x < Min.x || R.x > Max.x) return false;
                }
                else
                {
                    // Compute intersection t value of ray with near and far plane of slab
                    double ood = (1 / R.dx);
                    double t1 = (Min.x - R.x) * ood;
                    double t2 = (Max.x - R.x) * ood;
                    // Make t1 be intersection with near plane, t2 with far plane
                    if (t1 > t2)
                    {
                        double tswap = t1;
                        t1 = t2;
                        t2 = tswap;    
                    };
                    // Compute the intersection of slab intersections intervals
                    tmin = System.Math.Max(tmin, t1);
                    tmax = System.Math.Min(tmax, t2);
                    // Exit with no collision as soon as slab intersection becomes empty
                    if (tmin > tmax) return false;
                }

                if (System.Math.Abs(R.dy) < double.Epsilon)
                {
                    // Ray is parallel to slab. No hit if origin not within slab
                    if (R.y < Min.y || R.y > Max.y) return false;
                }
                else
                {
                    // Compute intersection t value of ray with near and far plane of slab
                    double ood = (1 / R.dy);
                    double t1 = (Min.y - R.y) * ood;
                    double t2 = (Max.y - R.y) * ood;
                    // Make t1 be intersection with near plane, t2 with far plane
                    if (t1 > t2)
                    {
                        double tswap = t1;
                        t1 = t2;
                        t2 = tswap;
                    };
                    // Compute the intersection of slab intersections intervals
                    tmin = System.Math.Max(tmin, t1);
                    tmax = System.Math.Min(tmax, t2);
                    // Exit with no collision as soon as slab intersection becomes empty
                    if (tmin > tmax) return false;
                }

                if (System.Math.Abs(R.dz) < double.Epsilon)
                {
                    // Ray is parallel to slab. No hit if origin not within slab
                    if (R.z < Min.z || R.z > Max.z) return false;
                }
                else
                {
                    // Compute intersection t value of ray with near and far plane of slab
                    double ood = (1 / R.dz);
                    double t1 = (Min.z - R.z) * ood;
                    double t2 = (Max.z - R.z) * ood;
                    // Make t1 be intersection with near plane, t2 with far plane
                    if (t1 > t2)
                    {
                        double tswap = t1;
                        t1 = t2;
                        t2 = tswap;
                    };
                    // Compute the intersection of slab intersections intervals
                    tmin = System.Math.Max(tmin, t1);
                    tmax = System.Math.Min(tmax, t2);
                    // Exit with no collision as soon as slab intersection becomes empty
                    if (tmin > tmax) return false;
                }

                // Ray intersects all 3 slabs. Return point (q) and intersection t value (tmin) 
                P = new Point(R.x + R.dx * tmin, R.y + R.dy * tmin, R.z + R.dz * tmin);
                
                return true;
            }

            public bool Intersect(ref Ray R, ref double tmin)
            {
                tmin = 0;          // set to -FLT_MAX to get first hit on line
                double tmax = double.MaxValue; // set to max distance ray can travel (for segment)

                // For all three slabs
                if (System.Math.Abs(R.dx) < double.Epsilon)
                {
                    // Ray is parallel to slab. No hit if origin not within slab
                    if (R.x < Min.x || R.x > Max.x) return false;
                }
                else
                {
                    // Compute intersection t value of ray with near and far plane of slab
                    double ood = (1 / R.dx);
                    double t1 = (Min.x - R.x) * ood;
                    double t2 = (Max.x - R.x) * ood;
                    // Make t1 be intersection with near plane, t2 with far plane
                    if (t1 > t2)
                    {
                        double tswap = t1;
                        t1 = t2;
                        t2 = tswap;
                    };
                    // Compute the intersection of slab intersections intervals
                    tmin = System.Math.Max(tmin, t1);
                    tmax = System.Math.Min(tmax, t2);
                    // Exit with no collision as soon as slab intersection becomes empty
                    if (tmin > tmax) return false;
                }

                if (System.Math.Abs(R.dy) < double.Epsilon)
                {
                    // Ray is parallel to slab. No hit if origin not within slab
                    if (R.y < Min.y || R.y > Max.y) return false;
                }
                else
                {
                    // Compute intersection t value of ray with near and far plane of slab
                    double ood = (1 / R.dy);
                    double t1 = (Min.y - R.y) * ood;
                    double t2 = (Max.y - R.y) * ood;
                    // Make t1 be intersection with near plane, t2 with far plane
                    if (t1 > t2)
                    {
                        double tswap = t1;
                        t1 = t2;
                        t2 = tswap;
                    };
                    // Compute the intersection of slab intersections intervals
                    tmin = System.Math.Max(tmin, t1);
                    tmax = System.Math.Min(tmax, t2);
                    // Exit with no collision as soon as slab intersection becomes empty
                    if (tmin > tmax) return false;
                }

                if (System.Math.Abs(R.dz) < double.Epsilon)
                {
                    // Ray is parallel to slab. No hit if origin not within slab
                    if (R.z < Min.z || R.z > Max.z) return false;
                }
                else
                {
                    // Compute intersection t value of ray with near and far plane of slab
                    double ood = (1 / R.dz);
                    double t1 = (Min.z - R.z) * ood;
                    double t2 = (Max.z - R.z) * ood;
                    // Make t1 be intersection with near plane, t2 with far plane
                    if (t1 > t2)
                    {
                        double tswap = t1;
                        t1 = t2;
                        t2 = tswap;
                    };
                    // Compute the intersection of slab intersections intervals
                    tmin = System.Math.Max(tmin, t1);
                    tmax = System.Math.Min(tmax, t2);
                    // Exit with no collision as soon as slab intersection becomes empty
                    if (tmin > tmax) return false;
                }

                // Ray intersects all 3 slabs. Return point (q) and intersection t value (tmin) 
                R.x = R.x + R.dx * tmin;
                R.y = R.y + R.dy * tmin;
                R.z = R.z + R.dz * tmin;

                return true;
            }

            /// <summary>
            /// The low corner of the box.
            /// </summary>
            public Point Min_PT
            {
                get 
                {
                    return Min;
                }    
            }

            /// <summary>
            /// The high corner of the box.
            /// </summary>
            public Point Max_PT
            {
                get 
                {
                    return Max;
                }
            }

            public double X_Length()
            {
                return Max.x - Min.x;
            }

            public double Y_Length()
            {
                return Max.y - Min.y;
            }

            public double Z_Length()
            {
                return Max.z - Min.z;
            }

            public bool Poly_Overlap_Area(Polygon Pts, out double Area)
            {
                System.Collections.Generic.List<Point> XPts = new System.Collections.Generic.List<Point>();
                int ct = 0;
                for (int i = 0; i < Pts.VertextCT; i++)
                {
                    if (this.IsPointInBox(Pts.Points[i].x, Pts.Points[i].y, Pts.Points[i].z))
                    {
                        ct++;
                        XPts.Add(Pts.Points[i]);
                    }
                }

                if (ct == Pts.VertextCT)
                {
                    if (ct == 3) Area = Hare_math.Cross(Pts.Points[1] - Pts.Points[0], Pts.Points[2] - Pts.Points[0]).Length() / 2;
                    else Area = Hare_math.Cross((Pts.Points[2] - Pts.Points[0]), (Pts.Points[3] - Pts.Points[1])).Length() / 2;
                    return true;
                }

                for (int i = 0; i < Pts.VertextCT; i++)
                {
                    int j = (i+1) % Pts.VertextCT; 
                    double tmin = 0;
                    Point X = new Point();
                    this.Intersect(new Ray(Pts.Points[i], Pts.Points[j] - Pts.Points[i], 0, 0), ref tmin, ref X);
                    if (tmin < 1) XPts.Add(X);
                }

                Point I;
                double u; double v; double t; int id;

                for (int i = 0; i < 12; i++)
                {
                    if (Pts.Intersect(Edge(i), Pts.Points, out I, out u, out v, out t, out id)) XPts.Add( I );
                }

                if (XPts.Count == 0) 
                {
                    Area = 0;
                    return false;
                }
                
                //Sort XPts by polar angle.

                Point Center = new Point();
                foreach (Point Pt in XPts) Center += Pt;
                Center /= XPts.Count;

                System.Collections.SortedList PtSort = new System.Collections.SortedList();
                if (Pts.Normal.dx == 1)
                {
                    foreach (Point Pt in XPts)
                    {
                        PtSort.Add(System.Math.Atan2(Pt.y - Center.y, Pt.z - Center.z), Pt);
                    }
                }
                else if (Pts.Normal.dy == 1)
                {
                    foreach (Point Pt in XPts)
                    {
                        PtSort.Add(System.Math.Atan2(Pt.x - Center.x, Pt.z - Center.z), Pt);
                    }
                }
                else
                {
                    foreach (Point Pt in XPts)
                    {
                        PtSort.Add(System.Math.Atan2(Pt.x - Center.x, Pt.y - Center.y), Pt);
                    }
                }

                Area = 0;

                for (int i = 0; i < PtSort.Count - 2; i++)
                {
                    Area += Hare_math.Cross((Point)PtSort[i + 1] - (Point)PtSort[i], (Point)PtSort[i + 2] - (Point)PtSort[i]).Length() / 2;
                }

                return true;
            }

            public Ray Edge(int i)
            {
                Point pt;
                switch (i)
                {
                    case 0:
                        return new Ray(Min_PT, new Vector(Max.x, Min.y, Min.z) - Min_PT, 0, 0);
                    case 1:
                        return new Ray(Min_PT, new Vector(Min.x, Max.y, Min.z) - Min_PT, 0, 0);
                    case 2:
                        return new Ray(Min_PT, new Vector(Min.x, Min.y, Max.z) - Min_PT, 0, 0);
                    
                    case 3:
                        pt = new Point(Min_PT.x, Min_PT.y, Max_PT.z);
                        return new Ray(pt, new Vector(Max.x, Min.y, Max.z) - pt, 0, 0);                    
                    case 4:
                        pt = new Point(Min_PT.x, Min_PT.y, Max_PT.z);
                        return new Ray(pt, new Vector(Min.x, Max.y, Max.z) - pt, 0, 0);
                    
                    case 5:
                        pt = new Point(Max_PT.x, Min_PT.y, Min_PT.z);
                        return new Ray(pt, new Vector(Max.x, Max.y, Min.z) - pt, 0, 0);
                    case 6:
                        pt = new Point(Max_PT.x, Min_PT.y, Min_PT.z);
                        return new Ray(pt, new Vector(Max.x, Min.y, Max.z) - pt, 0, 0);
                    
                    case 7:
                        pt = new Point(Min_PT.x, Max_PT.y, Min_PT.z);
                        return new Ray(pt, new Vector(Max.x, Max.y, Min.z) - pt, 0, 0);
                    case 8:
                        pt = new Point(Min_PT.x, Max_PT.y, Min_PT.z);
                        return new Ray(pt, new Vector(Min.x, Max.y, Max.z) - pt, 0, 0);

                    case 9:
                        return new Ray(Max_PT, new Vector(Max_PT.x - Max.x, Max_PT.y - Min.y, Max_PT.z - Max.z), 0, 0);
                    case 10:
                        return new Ray(Min_PT, new Vector(Max_PT.x - Max.x, Max_PT.x - Max.y, Max_PT.x - Min.z), 0, 0);
                    case 11:
                        return new Ray(Min_PT, new Vector(Max_PT.x - Max.x, Max_PT.x - Max.y, Max_PT.x - Min.z), 0, 0);
                    default:
                        return null;
                }
            }
        }
    }
}