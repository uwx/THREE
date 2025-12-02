using System;
using System.Collections.Generic;
using System.Diagnostics;

namespace THREE
{
    // Root helper representing one BVH root (group) visualization
    public class MeshBVHRootHelper : LineSegments
    {
        public Material MaterialRef;
        public BufferGeometry GeometryRef; // kept for backward compat but mirrors this.Geometry
        public MeshBVH BVH;
        public int Depth = 10;
        public bool DisplayParents = false;
        public bool DisplayEdges = true;
        private int _groupIndex = 0;

        private static Box3 _box = new Box3();

        public MeshBVHRootHelper(MeshBVH bvh, Material material, int depth = 10, int groupIndex = 0) : base(new BufferGeometry(), material)
        {
            this.type = "MeshBVHRootHelper";
            this.Name = "MeshBVHRootHelper";
            MaterialRef = material;
            GeometryRef = (BufferGeometry)this.Geometry; // ensure reference
            BVH = bvh;
            Depth = depth;
            _groupIndex = groupIndex;
            DisplayEdges = true;
        }

        public void UpdateHelper()
        {
            var geometry = (BufferGeometry)this.Geometry; // use actual rendered geometry
            geometry.Index = null;
            geometry.Attributes.Clear();
            this.Visible = false;

            if (BVH == null)
                return;

            int targetDepth = Depth - 1;
            int boundsCount = 0;

            // First pass: count bounds to render
            BVH.Traverse((d, isLeaf, bounds, offset, count) =>
            {
                if (d >= targetDepth || isLeaf)
                {
                    boundsCount++;
                    return true; // stop descending
                }
                else if (DisplayParents)
                {
                    boundsCount++;
                }
                return false;
            }, _groupIndex);

            if (boundsCount == 0)
                return;

            // Second pass: fill position array
            int posIndex = 0;
            float[] positionArray = new float[boundsCount * 8 * 3];
            BVH.Traverse((d, isLeaf, bounds, offset, count) =>
            {
                bool terminate = d >= targetDepth || isLeaf;
                if (terminate || DisplayParents)
                {
                    // bounds: [minX,minY,minZ,maxX,maxY,maxZ]
                    _box.Min.Set(bounds[0], bounds[1], bounds[2]);
                    _box.Max.Set(bounds[3], bounds[4], bounds[5]);

                    for (int xi = -1; xi <= 1; xi += 2)
                    {
                        float xVal = xi < 0 ? _box.Min.X : _box.Max.X;
                        for (int yi = -1; yi <= 1; yi += 2)
                        {
                            float yVal = yi < 0 ? _box.Min.Y : _box.Max.Y;
                            for (int zi = -1; zi <= 1; zi += 2)
                            {
                                float zVal = zi < 0 ? _box.Min.Z : _box.Max.Z;
                                positionArray[posIndex++] = xVal;
                                positionArray[posIndex++] = yVal;
                                positionArray[posIndex++] = zVal;
                            }
                        }
                    }
                    return terminate; // stop descending if terminating condition
                }
                return false;
            }, _groupIndex);

            // Indices pattern
            int[] edgePattern;
            if (DisplayEdges)
            {
                edgePattern = new int[] {
                    // x axis
                    0,4, 1,5, 2,6, 3,7,
                    // y axis
                    0,2, 1,3, 4,6, 5,7,
                    // z axis
                    0,1, 2,3, 4,5, 6,7,
                };
            }
            else
            {
                edgePattern = new int[] {
                    // X-, X+
                    0,1,2, 2,1,3,
                    4,6,5, 6,7,5,
                    // Y-, Y+
                    1,4,5, 0,4,1,
                    2,3,6, 3,7,6,
                    // Z-, Z+
                    0,2,4, 2,6,4,
                    1,5,3, 3,5,7,
                };
            }

            int patternLen = edgePattern.Length;
            int[] indexArray = new int[patternLen * boundsCount];
            for (int i = 0; i < boundsCount; i++)
            {
                int posOffset = i * 8; // each box has 8 vertices
                int indexOffset = i * patternLen;
                for (int j = 0; j < patternLen; j++)
                {
                    indexArray[indexOffset + j] = posOffset + edgePattern[j];
                }
            }

            geometry.SetIndex(new BufferAttribute<int>(indexArray, 1, false));
            geometry.SetAttribute("position", new BufferAttribute<float>(positionArray, 3, false));
            this.Material = MaterialRef; // ensure material applied
            this.Visible = true;
        }

        public override void Raycast(Raycaster raycaster, List<Intersection> intersects)
        {
            // Helper is visualization only; no pick logic
        }
    }

    public class MeshBVHHelper : Object3D
    {
        public int Depth = 10;
        public Object3D MeshObject;
        public MeshBVH BVH;
        public bool DisplayParents = false;
        public bool DisplayEdges = true;
        public int ObjectIndex = 0;
        private List<MeshBVHRootHelper> _roots = new List<MeshBVHRootHelper>();

        public Material EdgeMaterial;
        public Material MeshMaterial;

        public THREE.Color Color
        {
            get => EdgeMaterial != null ? EdgeMaterial.Color ?? new THREE.Color(0,0,0) : new THREE.Color(0,0,0);
            set  {
                if (EdgeMaterial != null)
                    EdgeMaterial.Color = value;
            }
        }
        
        public float Opacity
        {
            get => EdgeMaterial != null ? EdgeMaterial.Opacity : 1f;
            set { if (EdgeMaterial != null) EdgeMaterial.Opacity = value; if (MeshMaterial != null) MeshMaterial.Opacity = value; }
        }

        public MeshBVHHelper(Object3D mesh = null, MeshBVH bvh = null, int depth = 10)
        {
            this.type = "MeshBVHHelper";
            this.Name = "MeshBVHHelper";
            MeshObject = mesh;
            BVH = bvh;
            Depth = depth;
            DisplayParents = false;
            DisplayEdges = true;
            ObjectIndex = 0;

            EdgeMaterial = new THREE.LineBasicMaterial
            {
                Transparent = true,
                Opacity = 0.3f,
                DepthWrite = false,
                Side = Constants.DoubleSide,
                Color = new Color(0x00FF88)
            };
            MeshMaterial = new THREE.MeshBasicMaterial
            {
                Transparent = true,
                Opacity = 0.3f,
                DepthWrite = false,
                Side = Constants.DoubleSide,
                Color = EdgeMaterial.Color
            };

            UpdateHelper();
        }

        public void UpdateHelper()
        {
            var mesh = MeshObject as Mesh;
            var bvh = BVH!=null ? BVH : ((BufferGeometry)MeshObject.Geometry).UserData["BoundsTree"] as MeshBVH;
            if (bvh == null && mesh != null && mesh.Geometry != null) { }

            int totalRoots = bvh != null ? bvh.RootCount : 0;

            while (_roots.Count > totalRoots)
            {
                var root = _roots[_roots.Count - 1];
                root.GeometryRef = null;
                _roots.RemoveAt(_roots.Count - 1);
                this.Remove(root);
            }

            for (int i = 0; i < totalRoots; i++)
            {
                if (i >= _roots.Count)
                {
                    var root = new MeshBVHRootHelper(bvh, EdgeMaterial, Depth, i);
                    this.Add(root);
                    _roots.Add(root);
                }
                var r = _roots[i];
                r.BVH = bvh;
                r.Depth = Depth;
                r.DisplayParents = DisplayParents;
                r.DisplayEdges = DisplayEdges;
                r.MaterialRef = DisplayEdges ? EdgeMaterial : MeshMaterial;
                r.UpdateHelper();
            }
        }

        public override void UpdateMatrixWorld(bool force)
        {
            var mesh = MeshObject as Mesh;
            var parent = this.Parent;
            if (mesh != null)
            {
                mesh.UpdateWorldMatrix(true, false);
                if (parent != null)
                {
                    this.Matrix.Copy(parent.MatrixWorld).Invert().Multiply(mesh.MatrixWorld);
                }
                else
                {
                    this.Matrix.Copy(mesh.MatrixWorld);
                }
                this.Matrix.Decompose(this.Position, this.Quaternion, this.Scale);
            }
            base.UpdateMatrixWorld(force);
        }

        public MeshBVHHelper Clone()
        {
            return new MeshBVHHelper(MeshObject, BVH, Depth);
        }

        public void Dispose()
        {
            foreach (var r in _roots)
            {
                r.GeometryRef?.Dispose();
            }
        }
    }

    public class MeshBVHVisualizer : MeshBVHHelper
    {
        public MeshBVHVisualizer(Object3D mesh = null, MeshBVH bvh = null, int depth = 10) : base(mesh, bvh, depth)
        {
            Trace.TraceWarning("MeshBVHVisualizer deprecated. Use MeshBVHHelper instead.");
        }
    }
}
