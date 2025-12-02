using System;
using System.Collections.Generic;

namespace THREE
{
    public class MeshBVH
    {
        // Strategy constants
        public const int STRATEGY_CENTER = 0;
        public const int STRATEGY_SAH = 1;

        private struct PackedNode
        {
            public float MinX, MinY, MinZ;
            public float MaxX, MaxY, MaxZ;
            public int MetaA; // leaf: start offset | internal: right child node index
            public int MetaB; // leaf: count       | internal: split axis
            public bool IsLeaf;
            public int LeftChild;
            public int RightChild;
        }

        public class Options
        {
            public int Strategy { get; set; } = STRATEGY_CENTER;
            public int MaxDepth { get; set; } = 40;
            public int MaxLeafTris { get; set; } = 10;
            public bool UseSharedArrayBuffer { get; set; } = false;
            public bool SetBoundingBox { get; set; } = true;
            public Action<double>? OnProgress { get; set; } = null;
            public bool Indirect { get; set; } = false;
            public bool Verbose { get; set; } = true;
            public (int start, int end)? Range { get; set; } = null;
            public bool SkipGeneration { get; set; } = false;
        }

        public BufferGeometry Geometry { get; }

        private List<List<PackedNode>> _rootsNodes = new List<List<PackedNode>>();
        private List<int[]> _rootsTriangleOrder = new List<int[]>();

        private int[]? _indirectBuffer = null; // optional mapping array for indirect triangle resolution

        public bool Indirect => _indirectBuffer != null;
        public int RootCount => _rootsNodes.Count;

        public MeshBVH(BufferGeometry geometry, Options? options = null)
        {
            if (geometry == null) throw new ArgumentNullException(nameof(geometry));
            Geometry = geometry;
            options ??= new Options();

            if (!options.SkipGeneration)
            {
                BuildPackedTree(options);
                if (Geometry.BoundingBox == null && options.SetBoundingBox)
                {
                    Geometry.BoundingBox = GetBoundingBox(new Box3());
                }
            }
        }

        private void BuildPackedTree(Options options)
        {
            _rootsNodes.Clear();
            _rootsTriangleOrder.Clear();

            // Build per geometry group; if no groups, treat whole range as single root
            var groups = Geometry.Groups;
            if (groups == null || groups.Count == 0)
            {
                BuildRootRange(0, GetTriangleCount(out _), options);
            }
            else
            {
                foreach (var g in groups)
                {
                    int startTri = (int)g.Start / 3; // group.Start is vertex start? adapt if needed
                    int countTri = (int)g.Count / 3;
                    if (countTri <= 0) continue;
                    BuildRootRange(startTri, countTri, options);
                }
            }
        }

        private void BuildRootRange(int startTri, int triCount, Options options)
        {
            var nodes = new List<PackedNode>();
            int[] triOrder = new int[triCount];
            for (int i = 0; i < triCount; i++) triOrder[i] = startTri + i;
            BuildNodeRecursive(nodes, triOrder, 0, triCount, 0, options);
            _rootsNodes.Add(nodes);
            _rootsTriangleOrder.Add(triOrder);
        }

        private int GetTriangleCount(out bool isIndexed)
        {
            isIndexed = Geometry.Index != null;
            if (isIndexed) return Geometry.Index.count / 3;
            BufferAttribute<float> pos = null;
            if (Geometry.Attributes.ContainsKey("position")) pos = Geometry.Attributes["position"] as BufferAttribute<float>;
            return pos != null ? pos.count / 3 : 0;
        }

        private void BuildNodeRecursive(List<PackedNode> nodes, int[] triOrder, int start, int count, int depth, Options options)
        {
            // Compute bounds of this node
            var bounds = ComputeTrianglesBounds(triOrder, start, count);

            // Create node index
            int nodeIndex = nodes.Count;
            var node = new PackedNode
            {
                MinX = bounds.Min.X,
                MinY = bounds.Min.Y,
                MinZ = bounds.Min.Z,
                MaxX = bounds.Max.X,
                MaxY = bounds.Max.Y,
                MaxZ = bounds.Max.Z,
                MetaA = start,
                MetaB = count,
                IsLeaf = true,
                LeftChild = -1,
                RightChild = -1,
            };
            nodes.Add(node);

            // Leaf condition
            if (count <= options.MaxLeafTris || depth >= options.MaxDepth)
            {
                return;
            }

            // Choose split axis by longest extent
            Vector3 size = new Vector3(node.MaxX - node.MinX, node.MaxY - node.MinY, node.MaxZ - node.MinZ);
            int axis = 0;
            if (size.Y > size.X && size.Y >= size.Z) axis = 1; else if (size.Z > size.X && size.Z >= size.Y) axis = 2;

            // Partition triangles by centroid along axis at median center
            float[] centroids = new float[count];
            for (int i = 0; i < count; i++)
            {
                centroids[i] = TriangleCentroidComponent(triOrder[start + i], axis);
            }
            // Find median
            float median = FindMedian(centroids);

            int leftCount = 0;
            int rightCount = 0;
            // Stable partition triOrder[start..start+count)
            int[] temp = new int[count];
            for (int i = 0; i < count; i++)
            {
                int tri = triOrder[start + i];
                if (centroids[i] <= median) temp[leftCount++] = tri; else temp[count - (++rightCount)] = tri;
            }
            // If one side empty, split by count in half
            if (leftCount == 0 || rightCount == 0)
            {
                leftCount = count / 2;
                rightCount = count - leftCount;
                for (int i = 0; i < count; i++) temp[i] = triOrder[start + i];
            }
            // Copy back
            for (int i = 0; i < count; i++) triOrder[start + i] = temp[i];

            // Update current node to internal
            var updated = nodes[nodeIndex];
            updated.IsLeaf = false;
            updated.MetaB = axis; // split axis
            nodes[nodeIndex] = updated;

            // Build children
            int leftStart = start;
            int rightStart = start + leftCount;
            int leftNodeIndex = nodes.Count;
            BuildNodeRecursive(nodes, triOrder, leftStart, leftCount, depth + 1, options);
            int rightNodeIndex = nodes.Count;
            BuildNodeRecursive(nodes, triOrder, rightStart, rightCount, depth + 1, options);

            // Update child pointers and right child MetaA
            updated = nodes[nodeIndex];
            updated.LeftChild = leftNodeIndex;
            updated.RightChild = rightNodeIndex;
            updated.MetaA = rightNodeIndex; // store right child index similar to JS
            nodes[nodeIndex] = updated;
        }

        private Box3 ComputeTrianglesBounds(int[] triOrder, int start, int count)
        {
            var box = new Box3(); box.MakeEmpty();
            var index = Geometry.Index;
            BufferAttribute<float> pos = null;
            if (Geometry.Attributes.ContainsKey("position")) pos = Geometry.Attributes["position"] as BufferAttribute<float>;
            var posArr = pos != null ? pos.Array as float[] : null;
            for (int i = 0; i < count; i++)
            {
                int tri = triOrder[start + i];
                if (index != null)
                {
                    int i0 = index.GetX(tri * 3);
                    int i1 = index.GetX(tri * 3 + 1);
                    int i2 = index.GetX(tri * 3 + 2);
                    ExpandBoxWithIndex(box, posArr, i0);
                    ExpandBoxWithIndex(box, posArr, i1);
                    ExpandBoxWithIndex(box, posArr, i2);
                }
                else
                {
                    ExpandBoxWithIndex(box, posArr, tri * 3 + 0);
                    ExpandBoxWithIndex(box, posArr, tri * 3 + 1);
                    ExpandBoxWithIndex(box, posArr, tri * 3 + 2);
                }
            }
            return box;
        }

        private void ExpandBoxWithTriangle(Box3 box, int tri)
        {
            var index = Geometry.Index;
            BufferAttribute<float> pos = null;
            if (Geometry.Attributes.ContainsKey("position")) pos = Geometry.Attributes["position"] as BufferAttribute<float>;
            var arr = pos != null ? pos.Array as float[] : null;
            if (index != null)
            {
                int i0 = index.GetX(tri * 3); int i1 = index.GetX(tri * 3 + 1); int i2 = index.GetX(tri * 3 + 2);
                ExpandBoxWithIndex(box, arr, i0); ExpandBoxWithIndex(box, arr, i1); ExpandBoxWithIndex(box, arr, i2);
            }
            else
            {
                ExpandBoxWithIndex(box, arr, tri * 3 + 0);
                ExpandBoxWithIndex(box, arr, tri * 3 + 1);
                ExpandBoxWithIndex(box, arr, tri * 3 + 2);
            }
        }

        private void ExpandBoxWithIndex(Box3 box, float[] posArr, int vi)
        {
            int o = vi * 3;
            box.ExpandByPoint(new Vector3(posArr[o], posArr[o + 1], posArr[o + 2]));
        }

        private float TriangleCentroidComponent(int tri, int axis)
        {
            var index = Geometry.Index;
            BufferAttribute<float> pos = null;
            if (Geometry.Attributes.ContainsKey("position")) pos = Geometry.Attributes["position"] as BufferAttribute<float>;
            var posArr = pos != null ? pos.Array as float[] : null;
            float a, b, c;
            if (index != null)
            {
                int i0 = index.GetX(tri * 3);
                int i1 = index.GetX(tri * 3 + 1);
                int i2 = index.GetX(tri * 3 + 2);
                a = posArr[i0 * 3 + axis];
                b = posArr[i1 * 3 + axis];
                c = posArr[i2 * 3 + axis];
            }
            else
            {
                a = posArr[(tri * 3 + 0) * 3 + axis];
                b = posArr[(tri * 3 + 1) * 3 + axis];
                c = posArr[(tri * 3 + 2) * 3 + axis];
            }
            return (a + b + c) / 3f;
        }

        private float FindMedian(float[] values)
        {
            float[] copy = (float[])values.Clone(); System.Array.Sort(copy);
            int mid = copy.Length / 2; if (copy.Length % 2 == 1) return copy[mid];
            return (copy[mid - 1] + copy[mid]) * 0.5f;
        }

        public void Refit(int[]? nodeIndices = null)
        {
            for (int r = 0; r < _rootsNodes.Count; r++)
            {
                var nodes = _rootsNodes[r]; var triOrder = _rootsTriangleOrder[r];
                if (nodeIndices == null)
                {
                    for (int i = nodes.Count - 1; i >= 0; i--)
                    {
                        var n = nodes[i];
                        if (n.IsLeaf)
                        {
                            var b = ComputeTrianglesBounds(triOrder, n.MetaA, n.MetaB);
                            n.MinX = b.Min.X; n.MinY = b.Min.Y; n.MinZ = b.Min.Z;
                            n.MaxX = b.Max.X; n.MaxY = b.Max.Y; n.MaxZ = b.Max.Z; nodes[i] = n;
                        }
                        else
                        {
                            var lb = NodeBox(nodes[n.LeftChild]); var rb = NodeBox(nodes[n.RightChild]); var ub = lb.Union(rb);
                            n.MinX = ub.Min.X; n.MinY = ub.Min.Y; n.MinZ = ub.Min.Z;
                            n.MaxX = ub.Max.X; n.MaxY = ub.Max.Y; n.MaxZ = ub.Max.Z; nodes[i] = n;
                        }
                    }
                }
                else
                {
                    foreach (var idx in nodeIndices)
                    {
                        if (idx < 0 || idx >= nodes.Count) continue;
                        var n = nodes[idx]; if (n.IsLeaf)
                        {
                            var b = ComputeTrianglesBounds(triOrder, n.MetaA, n.MetaB);
                            n.MinX = b.Min.X; n.MinY = b.Min.Y; n.MinZ = b.Min.Z;
                            n.MaxX = b.Max.X; n.MaxY = b.Max.Y; n.MaxZ = b.Max.Z; nodes[idx] = n;
                        }
                    }
                }
            }
        }

        public void RefitTriangles(HashSet<int> triangleIndices)
        {
            // For each root update affected leaf nodes then propagate upward.
            for (int r = 0; r < _rootsNodes.Count; r++)
            {
                var nodes = _rootsNodes[r]; var order = _rootsTriangleOrder[r];
                // Update leaf nodes containing modified triangles
                for (int i = 0; i < nodes.Count; i++)
                {
                    var n = nodes[i]; if (!n.IsLeaf) continue;
                    bool affected = false;
                    for (int t = 0; t < n.MetaB; t++)
                    {
                        int triLocal = order[n.MetaA + t]; int triGlobal = ResolveTriangleIndex(triLocal);
                        if (triangleIndices.Contains(triGlobal)) { affected = true; break; }
                    }
                    if (affected)
                    {
                        var b = ComputeTrianglesBounds(order, n.MetaA, n.MetaB);
                        n.MinX = b.Min.X; n.MinY = b.Min.Y; n.MinZ = b.Min.Z;
                        n.MaxX = b.Max.X; n.MaxY = b.Max.Y; n.MaxZ = b.Max.Z; nodes[i] = n;
                    }
                }
                // Propagate bounds upward
                for (int i = nodes.Count - 1; i >= 0; i--)
                {
                    var n = nodes[i]; if (n.IsLeaf) continue;
                    var lb = NodeBox(nodes[n.LeftChild]); var rb = NodeBox(nodes[n.RightChild]); var ub = lb.Union(rb);
                    n.MinX = ub.Min.X; n.MinY = ub.Min.Y; n.MinZ = ub.Min.Z;
                    n.MaxX = ub.Max.X; n.MaxY = ub.Max.Y; n.MaxZ = ub.Max.Z; nodes[i] = n;
                }
            }
        }

        private Box3 NodeBox(PackedNode n)
        {
            var b = new Box3(); b.Min.Set(n.MinX, n.MinY, n.MinZ); b.Max.Set(n.MaxX, n.MaxY, n.MaxZ); return b;
        }

        public Box3 GetBoundingBox(Box3 target)
        {
            if (target == null) target = new Box3(); target.MakeEmpty();
            foreach (var nodes in _rootsNodes)
            {
                if (nodes.Count == 0) continue; target.Union(NodeBox(nodes[0]));
            }
            if (_rootsNodes.Count == 0) target.Union(ComputeGeometryBoundsAll());
            return target;
        }

        private Box3 ComputeGeometryBoundsAll()
        {
            var box = new Box3(); box.MakeEmpty(); BufferAttribute<float> pos = null;
            if (Geometry.Attributes.ContainsKey("position")) pos = Geometry.Attributes["position"] as BufferAttribute<float>;
            var arr = pos != null ? pos.Array as float[] : null; if (arr != null)
            { for (int i = 0; i < arr.Length; i += 3) box.ExpandByPoint(new Vector3(arr[i], arr[i + 1], arr[i + 2])); }
            return box;
        }

        public int[] GetTriangleOrder(int rootIndex) => rootIndex >=0 && rootIndex < _rootsTriangleOrder.Count ? _rootsTriangleOrder[rootIndex] : System.Array.Empty<int>();

        public void SetIndirectBuffer(int[] mapping, bool rebuild = false)
        {
            _indirectBuffer = mapping;
            if (rebuild)
            {
                // Rebuild entire BVH if triangle count changed
                int triCountOld = 0; foreach (var arr in _rootsTriangleOrder) triCountOld += arr.Length;
                int triCountNew = mapping.Length;
                if (triCountNew != triCountOld)
                {
                    // regenerate using current options defaults
                    BuildPackedTree(new Options());
                }
                else
                {
                    Refit();
                }
            }
        }

        public List<Intersection> Raycast(Ray ray, Material? material = null, double near = 0, double far = double.PositiveInfinity)
        {
            var hits = new List<Intersection>();
            int side = material != null ? material.Side : Constants.FrontSide;
            bool doubleSide = side == Constants.DoubleSide;
            bool backSide = side == Constants.BackSide;
            for (int r = 0; r < _rootsNodes.Count; r++)
            {
                var nodes = _rootsNodes[r]; var triOrder = _rootsTriangleOrder[r]; if (nodes.Count == 0) continue;
                var stack = new Stack<int>(); stack.Push(0);
                while (stack.Count > 0)
                {
                    int ni = stack.Pop(); var n = nodes[ni]; var box = NodeBox(n);
                    if (!ray.IntersectsBox(box)) continue;
                    if (n.IsLeaf)
                    {
                        TestLeafTriangles(ray, triOrder, n.MetaA, n.MetaB, hits, side, doubleSide, backSide, near, far, r);
                    }
                    else
                    {
                        stack.Push(n.RightChild); stack.Push(n.LeftChild);
                    }
                }
            }
            return hits;
        }

        public Intersection? RaycastFirst(Ray ray, Material? material = null, double near = 0, double far = double.PositiveInfinity)
        {
            Intersection? closest = null; var list = Raycast(ray, material, near, far);
            foreach (var h in list) if (closest == null || h.distance < closest.distance) closest = h;
            return closest;
        }

        private Vector3 ComputeBarycentric(Vector3 p, Vector3 a, Vector3 b, Vector3 c)
        {
            var v0 = b - a; var v1 = c - a; var v2 = p - a;
            float d00 = v0.Dot(v0); float d01 = v0.Dot(v1); float d11 = v1.Dot(v1); float d20 = v2.Dot(v0); float d21 = v2.Dot(v1);
            float denom = d00 * d11 - d01 * d01; if (denom == 0) return new Vector3(1,0,0);
            float v = (d11 * d20 - d01 * d21) / denom; float w = (d00 * d21 - d01 * d20) / denom; float u = 1f - v - w; return new Vector3(u,v,w);
        }

        private void TestLeafTriangles(Ray ray, int[] triOrder, int start, int count, List<Intersection> hits, int side, bool doubleSide, bool backSide, double near, double far, int rootIndex)
        {
            var index = Geometry.Index; BufferAttribute<float> pos = null;
            if (Geometry.Attributes.ContainsKey("position")) pos = Geometry.Attributes["position"] as BufferAttribute<float>;
            BufferAttribute<float> uvAttr = null; if (Geometry.Attributes.ContainsKey("uv")) uvAttr = Geometry.Attributes["uv"] as BufferAttribute<float>;
            var arr = pos != null ? pos.Array as float[] : null; var uvArr = uvAttr != null ? uvAttr.Array as float[] : null;
            for (int i = 0; i < count; i++)
            {
                int triLocal = triOrder[start + i]; int tri = ResolveTriangleIndex(triLocal);
                Vector3 a,b,c; Vector2 uva=null, uvb=null, uvc=null;
                if (index != null)
                {
                    int i0 = index.GetX(tri * 3); int i1 = index.GetX(tri * 3 + 1); int i2 = index.GetX(tri * 3 + 2);
                    a = new Vector3(arr[i0 * 3], arr[i0 * 3 + 1], arr[i0 * 3 + 2]);
                    b = new Vector3(arr[i1 * 3], arr[i1 * 3 + 1], arr[i1 * 3 + 2]);
                    c = new Vector3(arr[i2 * 3], arr[i2 * 3 + 1], arr[i2 * 3 + 2]);
                    if (uvArr != null)
                    {
                        uva = new Vector2(uvArr[i0 * 2], uvArr[i0 * 2 + 1]);
                        uvb = new Vector2(uvArr[i1 * 2], uvArr[i1 * 2 + 1]);
                        uvc = new Vector2(uvArr[i2 * 2], uvArr[i2 * 2 + 1]);
                    }
                }
                else
                {
                    int baseIdx = tri * 9;
                    a = new Vector3(arr[baseIdx], arr[baseIdx + 1], arr[baseIdx + 2]);
                    b = new Vector3(arr[baseIdx + 3], arr[baseIdx + 4], arr[baseIdx + 5]);
                    c = new Vector3(arr[baseIdx + 6], arr[baseIdx + 7], arr[baseIdx + 8]);
                    if (uvArr != null)
                    {
                        int uvBase = tri * 6;
                        uva = new Vector2(uvArr[uvBase], uvArr[uvBase + 1]);
                        uvb = new Vector2(uvArr[uvBase + 2], uvArr[uvBase + 3]);
                        uvc = new Vector2(uvArr[uvBase + 4], uvArr[uvBase + 5]);
                    }
                }
                Intersection? addHit(Vector3 hp)
                {
                    float dist = hp.DistanceTo(ray.origin); if (dist < near || dist > far) return null;
                    var bary = ComputeBarycentric(hp, a, b, c);
                    Vector2 uvInterp = null; if (uva != null)
                        uvInterp = new Vector2(uva.X * bary.X + uvb.X * bary.Y + uvc.X * bary.Z, uva.Y * bary.X + uvb.Y * bary.Y + uvc.Y * bary.Z);
                    return new Intersection { distance = dist, point = hp, faceIndex = tri, index = triLocal, rootIndex = rootIndex, barycentric = bary, uv = uvInterp };
                }
                Vector3 hitPoint = null;
                if (doubleSide)
                {
                    hitPoint = ray.IntersectTriangle(a, b, c, false);
                    if (hitPoint != null) { var h = addHit(hitPoint); if (h != null) hits.Add(h); continue; }
                    hitPoint = ray.IntersectTriangle(a, c, b, false);
                    if (hitPoint != null) { var h = addHit(hitPoint); if (h != null) hits.Add(h); }
                }
                else
                {
                    bool backfaceCulling = side == Constants.FrontSide; if (backSide) backfaceCulling = false;
                    hitPoint = ray.IntersectTriangle(a, b, c, backfaceCulling);
                    if (hitPoint != null) { var h = addHit(hitPoint); if (h != null) hits.Add(h); }
                }
            }
        }

        private int ResolveTriangleIndex(int tri) => Indirect && _indirectBuffer != null ? _indirectBuffer[tri] : tri;

        public bool IntersectsBox(Box3 box)
        { var bbox = Geometry.BoundingBox ?? GetBoundingBox(new Box3()); return bbox.IntersectsBox(box); }
        public bool IntersectsSphere(Sphere sphere)
        { var bbox = Geometry.BoundingBox ?? GetBoundingBox(new Box3()); return bbox.IntersectsSphere(sphere); }
        public double ClosestPointToPoint(Vector3 point, out Vector3 target) { target = point.Clone(); return 0.0; }

        public void Traverse(Func<int, bool, float[], int, int, bool> callback, int rootIndex = 0)
        {
            if (rootIndex < 0 || rootIndex >= _rootsNodes.Count) return; var nodes = _rootsNodes[rootIndex]; if (nodes.Count == 0) return;
            void TraverseNode(int idx, int depth)
            {
                var n = nodes[idx]; var bounds = new float[] { n.MinX, n.MinY, n.MinZ, n.MaxX, n.MaxY, n.MaxZ };
                if (n.IsLeaf) callback(depth, true, bounds, n.MetaA, n.MetaB);
                else
                {
                    bool stop = callback(depth, false, bounds, n.MetaB, 0);
                    if (!stop) { TraverseNode(n.LeftChild, depth + 1); TraverseNode(n.RightChild, depth + 1); }
                }
            }
            TraverseNode(0, 0);
        }
    }
}
