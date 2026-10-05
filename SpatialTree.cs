using System;
using System.Collections.Generic;

namespace Hare.Geometry
{
    // Bounds never mutate the input ray. Closed intervals retain face/edge hits.
    internal struct PartitionBounds
    {
        internal double X0, Y0, Z0, X1, Y1, Z1;
        internal static PartitionBounds Empty => new PartitionBounds {
            X0 = double.PositiveInfinity, Y0 = double.PositiveInfinity, Z0 = double.PositiveInfinity,
            X1 = double.NegativeInfinity, Y1 = double.NegativeInfinity, Z1 = double.NegativeInfinity };
        internal double Min(int a) => a == 0 ? X0 : a == 1 ? Y0 : Z0;
        internal double Max(int a) => a == 0 ? X1 : a == 1 ? Y1 : Z1;
        internal void SetMin(int a, double v) { if (a == 0) X0 = v; else if (a == 1) Y0 = v; else Z0 = v; }
        internal void SetMax(int a, double v) { if (a == 0) X1 = v; else if (a == 1) Y1 = v; else Z1 = v; }
        internal void Include(PartitionBounds b)
        {
            X0 = Math.Min(X0, b.X0); Y0 = Math.Min(Y0, b.Y0); Z0 = Math.Min(Z0, b.Z0);
            X1 = Math.Max(X1, b.X1); Y1 = Math.Max(Y1, b.Y1); Z1 = Math.Max(Z1, b.Z1);
        }
        internal static PartitionBounds From(Point min, Point max) => new PartitionBounds {
            X0 = min.x, Y0 = min.y, Z0 = min.z, X1 = max.x, Y1 = max.y, Z1 = max.z };
        internal static PartitionBounds Polygon(Polygon polygon)
        {
            var b = Empty;
            foreach (Point p in polygon.Points)
            {
                if (!Finite(p.x) || !Finite(p.y) || !Finite(p.z))
                    throw new ArgumentException("Polygon coordinates must be finite.");
                b.Include(From(p, p));
            }
            // Conservative, scale-aware rounding allowance (not geometric voxelization).
            for (int a = 0; a < 3; a++)
            {
                double e = 1e-12 * Math.Max(1, Math.Max(Math.Abs(b.Min(a)), Math.Abs(b.Max(a))));
                b.SetMin(a, b.Min(a) - e); b.SetMax(a, b.Max(a) + e);
            }
            return b;
        }
        internal double Area
        {
            get { double x = X1 - X0, y = Y1 - Y0, z = Z1 - Z0; return 2 * (x*y + x*z + y*z); }
        }
        internal bool Overlaps(PartitionBounds b) =>
            X0 <= b.X1 && X1 >= b.X0 && Y0 <= b.Y1 && Y1 >= b.Y0 && Z0 <= b.Z1 && Z1 >= b.Z0;
        internal static bool Finite(double v) => !double.IsNaN(v) && !double.IsInfinity(v);
        internal static bool ValidRay(Ray r) => r != null && Finite(r.x) && Finite(r.y) && Finite(r.z) &&
            Finite(r.dx) && Finite(r.dy) && Finite(r.dz) && (r.dx != 0 || r.dy != 0 || r.dz != 0);
        private static bool Slab(double o, double d, double lo, double hi, ref double enter, ref double exit)
        {
            if (d == 0) return o >= lo && o <= hi;
            double t0 = (lo - o) / d, t1 = (hi - o) / d;
            if (t0 > t1) { double t = t0; t0 = t1; t1 = t; }
            enter = Math.Max(enter, t0); exit = Math.Min(exit, t1);
            return enter <= exit;
        }
        internal bool Intersect(Ray r, double limit, out double enter, out double exit)
        {
            enter = 0; exit = limit;
            return Slab(r.x, r.dx, X0, X1, ref enter, ref exit) &&
                Slab(r.y, r.dy, Y0, Y1, ref enter, ref exit) &&
                Slab(r.z, r.dz, Z0, Z1, ref enter, ref exit);
        }
    }

    internal struct PartitionVisit
    {
        internal int Node;
        internal double Enter;
        internal PartitionVisit(int node, double enter) { Node = node; Enter = enter; }
    }

    // One scratch instance per partition and executing thread. Caller ray IDs are irrelevant.
    internal sealed class PartitionScratch
    {
        internal uint[] Stamps = new uint[0];
        internal uint Generation;
        internal readonly List<PartitionVisit> Stack = new List<PartitionVisit>();
        internal int PolygonTests;
        internal void Begin(int count)
        {
            if (Stamps.Length < count) Stamps = new uint[count];
            if (Generation == uint.MaxValue) { Array.Clear(Stamps, 0, Stamps.Length); Generation = 0; }
            Generation++;
            Stack.Clear();
            PolygonTests = 0;
        }
        internal bool FirstTest(int id)
        {
            if (Stamps[id] == Generation) return false;
            Stamps[id] = Generation; PolygonTests++; return true;
        }
    }

    internal enum PartitionTreeKind { KD, Octree, BVH }

    internal sealed class PartitionTree
    {
        private sealed class Node
        {
            internal PartitionBounds Bounds;
            internal int[] Polygons;
            internal int[] Children;
        }
        private readonly Topology[] model;
        private readonly Node[][] nodes;
        private readonly PartitionTreeKind kind;
        private readonly int maxDepth, leafSize;
        private readonly System.Threading.ThreadLocal<PartitionScratch> scratch =
            new System.Threading.ThreadLocal<PartitionScratch>(() => new PartitionScratch());
        internal int LastPolygonTests => scratch.Value.PolygonTests;

        internal PartitionTree(Topology[] model, int maxDepth, int leafSize, PartitionTreeKind kind)
        {
            if (model == null) throw new ArgumentNullException(nameof(model));
            if (maxDepth < 0) throw new ArgumentOutOfRangeException(nameof(maxDepth));
            if (leafSize < 1) throw new ArgumentOutOfRangeException(nameof(leafSize));
            this.model = model; this.maxDepth = Math.Min(maxDepth, 64); this.leafSize = leafSize; this.kind = kind;
            nodes = new Node[model.Length][];
            for (int t = 0; t < model.Length; t++)
            {
                if (model[t] == null) throw new ArgumentException("Topology cannot be null.", nameof(model));
                var bounds = new PartitionBounds[model[t].Polygon_Count];
                var ids = new List<int>(bounds.Length);
                var box = PartitionBounds.Empty;
                for (int p = 0; p < bounds.Length; p++)
                {
                    bounds[p] = PartitionBounds.Polygon(model[t].Polys[p]);
                    box.Include(bounds[p]); ids.Add(p);
                }
                var list = new List<Node>();
                long remainingReferences = 7L * bounds.Length;
                if (ids.Count != 0) Build(list, bounds, ids, box, 0, ref remainingReferences);
                nodes[t] = list.ToArray();
            }
        }

        private static PartitionBounds Union(PartitionBounds[] bounds, List<int> ids)
        {
            var result = PartitionBounds.Empty;
            foreach (int id in ids) result.Include(bounds[id]);
            return result;
        }

        private int Build(List<Node> output, PartitionBounds[] bounds, List<int> ids,
            PartitionBounds box, int depth, ref long budget)
        {
            int index = output.Count;
            var node = new Node { Bounds = box, Polygons = ids.ToArray() };
            output.Add(node);
            if (ids.Count <= leafSize || depth >= maxDepth || !(box.Area > 0)) return index;
            List<int>[] groups = null;
            PartitionBounds[] boxes = null;
            if (kind == PartitionTreeKind.Octree)
            {
                groups = new List<int>[8]; boxes = new PartitionBounds[8];
                double cx = box.X0 + (box.X1 - box.X0) * .5;
                double cy = box.Y0 + (box.Y1 - box.Y0) * .5;
                double cz = box.Z0 + (box.Z1 - box.Z0) * .5;
                if (cx <= box.X0 || cx >= box.X1 || cy <= box.Y0 || cy >= box.Y1 || cz <= box.Z0 || cz >= box.Z1) return index;
                for (int c = 0; c < 8; c++)
                {
                    boxes[c] = new PartitionBounds {
                        X0 = (c & 4) == 0 ? box.X0 : cx, X1 = (c & 4) == 0 ? cx : box.X1,
                        Y0 = (c & 2) == 0 ? box.Y0 : cy, Y1 = (c & 2) == 0 ? cy : box.Y1,
                        Z0 = (c & 1) == 0 ? box.Z0 : cz, Z1 = (c & 1) == 0 ? cz : box.Z1 };
                    groups[c] = new List<int>();
                    foreach (int id in ids) if (boxes[c].Overlaps(bounds[id])) groups[c].Add(id);
                }
            }
            else
            {
                // Evaluate a small set of split planes using surface area and reference counts.
                double best = ids.Count;
                for (int axis = 0; axis < 3; axis++)
                {
                    for (int bin = 1; bin < 8; bin++)
                    {
                        double split = box.Min(axis) + (box.Max(axis) - box.Min(axis)) * bin / 8;
                        if (!(split > box.Min(axis) && split < box.Max(axis))) continue;
                        var left = new List<int>(); var right = new List<int>();
                        foreach (int id in ids)
                        {
                            if (kind == PartitionTreeKind.BVH)
                            {
                                double center = bounds[id].Min(axis) + (bounds[id].Max(axis) - bounds[id].Min(axis)) * .5;
                                (center < split ? left : right).Add(id);
                            }
                            else
                            {
                                if (bounds[id].Min(axis) <= split) left.Add(id);
                                if (bounds[id].Max(axis) >= split) right.Add(id);
                            }
                        }
                        if (left.Count == 0 || right.Count == 0 || left.Count == ids.Count || right.Count == ids.Count) continue;
                        var lb = box; var rb = box;
                        if (kind == PartitionTreeKind.BVH) { lb = Union(bounds, left); rb = Union(bounds, right); }
                        else { lb.SetMax(axis, split); rb.SetMin(axis, split); }
                        double cost = 1 + (lb.Area * left.Count + rb.Area * right.Count) / box.Area;
                        if (cost < best)
                        {
                            best = cost; groups = new[] { left, right }; boxes = new[] { lb, rb };
                        }
                    }
                }
            }
            if (groups == null) return index;
            int references = 0, nonempty = 0;
            double splitCost = 1;
            for (int c = 0; c < groups.Length; c++)
            {
                if (groups[c].Count == 0) continue;
                references += groups[c].Count; nonempty++;
                splitCost += boxes[c].Area / box.Area * groups[c].Count;
            }
            // Bound duplication and reject subdivisions whose estimated work does not improve.
            if (nonempty < 2 || splitCost >= ids.Count || references - ids.Count > budget) return index;
            budget -= references - ids.Count;
            var children = new List<int>(nonempty);
            for (int c = 0; c < groups.Length; c++)
                if (groups[c].Count != 0) children.Add(Build(output, bounds, groups[c], boxes[c], depth + 1, ref budget));
            node.Children = children.ToArray(); node.Polygons = null;
            return index;
        }

        internal bool Shoot(Ray ray, int topology, out X_Event result, int excluded1, int excluded2)
        {
            if ((uint)topology >= (uint)model.Length) throw new ArgumentOutOfRangeException(nameof(topology));
            result = new X_Event();
            var state = scratch.Value;
            state.Begin(model[topology].Polygon_Count);
            var tree = nodes[topology];
            double enter, exit;
            if (!PartitionBounds.ValidRay(ray) || tree.Length == 0 ||
                !tree[0].Bounds.Intersect(ray, double.PositiveInfinity, out enter, out exit)) return false;
            state.Stack.Add(new PartitionVisit(0, enter));
            double closest = double.PositiveInfinity;
            int bestId = -1;
            Point bestPoint = null; double bestU = 0, bestV = 0;
            while (state.Stack.Count > 0)
            {
                int last = state.Stack.Count - 1;
                var visit = state.Stack[last]; state.Stack.RemoveAt(last);
                if (visit.Enter > closest) continue;
                var node = tree[visit.Node];
                if (node.Children == null)
                {
                    foreach (int id in node.Polygons)
                    {
                        if (id == excluded1 || id == excluded2 || !state.FirstTest(id)) continue;
                        Point point; double u, v, t;
                        // Test the whole polygon once, retaining hits beyond this particular leaf.
                        if (model[topology].intersect(id, ray, out point, out u, out v, out t) &&
                            t > 1e-10 && (t < closest || (t == closest && id < bestId)))
                        { closest = t; bestId = id; bestPoint = point; bestU = u; bestV = v; }
                    }
                }
                else
                {
                    int start = state.Stack.Count;
                    foreach (int child in node.Children)
                    {
                        if (!tree[child].Bounds.Intersect(ray, closest, out enter, out exit)) continue;
                        // Descending entry distances: stack pops the nearest child first.
                        int pos = state.Stack.Count;
                        state.Stack.Add(new PartitionVisit(child, enter));
                        while (pos > start && state.Stack[pos - 1].Enter < enter)
                        { state.Stack[pos] = state.Stack[pos - 1]; pos--; }
                        state.Stack[pos] = new PartitionVisit(child, enter);
                    }
                }
            }
            if (bestId < 0) return false;
            result = new X_Event(bestPoint, bestU, bestV, closest, bestId);
            return true;
        }
    }
}

