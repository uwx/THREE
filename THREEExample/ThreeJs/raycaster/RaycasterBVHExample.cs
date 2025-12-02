using ImGuiNET;
using OpenTK.Windowing.Common.Input;
using System;
using System.Collections.Generic;
using System.Text;
using THREE;
using THREE.Objects;
using THREEExample;
using static System.Runtime.InteropServices.JavaScript.JSType;
namespace THREEExample.ThreeJs.Loader
{
    [Example("Raycast BVH", ExampleCategory.ThreeJs, "RayCaster")]
    public class RaycasterBVHExample : Example
    {
        Mesh mesh;
        MeshBVHHelper helper;
        MeshBVH bvh;
        InstancedMesh sphereInstance;
        LineSegments lineSegments;

        Raycaster _raycaster;
        Vector3 _position;
        Quaternion _quaternion;
        Vector3 _scale;
        Matrix4 _matrix;
        Vector3 _axis;
        int MAX_RAYS = 3000;
        THREE.Color RAY_COLOR = THREE.Color.Hex(0x444444);

        struct Parameters {
            public bool useBVH = true;
            public bool firstHitOnly = true;
            public int Count = 150;
            public bool displayHelper = false;
            public int helperDepth = 10;
            public Parameters() { }
        }
        Parameters _params = new Parameters();

        public RaycasterBVHExample() : base() { }
        public override void InitCamera()
        {
            camera.Fov = 50.0f;
            camera.Aspect = this.glControl.AspectRatio;
            camera.Near = 1.0f;
            camera.Far = 100.0f;
            camera.Position.Z = 10;
        }

        public override void InitLighting()
        {
            base.InitLighting();
            var ambientLight = new AmbientLight(THREE.Color.Hex(0x888888));
            var directionalLight = new DirectionalLight() { Color = THREE.Color.Hex(0x888888) };

            scene.Add(ambientLight);
            scene.Add(directionalLight);
        }
        public override void Init()
        {
            base.Init();
            scene.Background = THREE.Color.Hex(0xeeeeee);
            var lineGeometry = new THREE.BufferGeometry();
            lineGeometry.SetAttribute("position", new THREE.BufferAttribute<float>(new float[_params.Count * 2 * 3], 3));
            lineSegments = new THREE.LineSegments(lineGeometry, new THREE.LineBasicMaterial() {
                Color = RAY_COLOR,
                Transparent = true,
                Opacity = 0.25f,
                DepthWrite = false
            });

            sphereInstance = new THREE.InstancedMesh(
                new THREE.SphereBufferGeometry(1),
                new THREE.MeshBasicMaterial() { Color = RAY_COLOR },
                    2 * MAX_RAYS
                );
            sphereInstance.InstanceMatrix.SetUsage(Constants.DynamicDrawUsage);
            sphereInstance.InstanceCount = 0;

            scene.Add(lineSegments);
            scene.Add(sphereInstance);

            // initialize ray update helpers
            _raycaster = new Raycaster();
            _position = new Vector3();
            _quaternion = new Quaternion();
            _scale = new Vector3(1,1,1);
            _matrix = new Matrix4();
            _axis = new Vector3();

            BuildScene();
            InitRays();
            
            AddGuiControlsAction = () =>
            {
                ImGui.Text("BVH Controls");
                if (ImGui.SliderInt("count", ref _params.Count, 1, MAX_RAYS))
                {
                    var lineGeometry = lineSegments.Geometry as BufferGeometry;
                    lineGeometry.SetAttribute("position", new THREE.BufferAttribute<float>(new float[_params.Count * 2 * 3], 3));
                   (lineGeometry.GetAttribute<float>("position") as BufferAttribute<float>).NeedsUpdate = true;
                }
                if(ImGui.Checkbox("use BVH", ref _params.useBVH))
                {
                    var geometry = mesh.Geometry as BufferGeometry;
                    if(_params.useBVH)
                    {
                        bvh = geometry.UserData["BoundsTree"] as MeshBVH;
                    }
                    else
                    {
                        bvh = null;
                    }
                }

                ImGui.Text("Helper Settings");
                
                if (ImGui.Checkbox("Display helper",ref _params.displayHelper))
                {
                    if(_params.displayHelper)
                    {
                        helper.UpdateHelper();
                        scene.Add(helper);
                    }
                    else
                    {
                        scene.Remove(helper);   
                    }
                }
                if(ImGui.SliderInt("Helper Depth", ref _params.helperDepth, 1, 20))
                {
                    if(helper != null)
                    {
                        helper.Depth = _params.helperDepth;
                        helper.UpdateHelper();
                    }
                }
                
            };

        }
        public override void Render()
        {
            if (helper!=null)
            {

                helper.Visible = _params.displayHelper;

            }

            if (mesh!=null)
            {

                mesh.Rotation.Y += 0.002f;
                mesh.UpdateMatrixWorld();

            }

            UpdateRays();
            base.Render();
        }
        public void BuildScene()
        {
            AssimpLoader loader = new AssimpLoader();
            var group = loader.Load(@"../../../../assets/models/fbx/stanford-bunny.fbx");
            var childMesh = (Mesh)group.Children[0].Children[0];
            mesh = childMesh; // assign loaded mesh
            var geometry = childMesh.Geometry as BufferGeometry;
            geometry.Translate(0, 0.5f / 0.0075f, 0);
            bvh = geometry.ComputeBoundsTree();
            if (!_params.useBVH) bvh = null;
            mesh.Scale.SetScalar(0.0075f);
            // ensure both sides are raycastable
            if (mesh.Material != null) mesh.Material.Side = Constants.DoubleSide;
            scene.Add(mesh);
            helper = new MeshBVHHelper(mesh);
            helper.Color.Set(0xE91E63);
            helper.UpdateHelper();
            scene.Add(helper);
           
            if (_params.displayHelper) scene.Add(helper);
        }

        public void InitRays()
        {
            if (sphereInstance == null) return;
            var position = new Vector3();
            var matrix = new Matrix4();
            //int total = MAX_RAYS * 2;
            int total = MAX_RAYS * 2;
            for (int i = 0; i < total; i++)
            {
                float phi = (float)(MathUtils.random.NextDouble() * Math.PI);
                float theta = (float)(MathUtils.random.NextDouble() * Math.PI * 2.0);
                position.SetFromSphericalCoords(1f, phi, theta).MultiplyScalar(3.75f);
                matrix.MakeTranslation(position.X, position.Y, position.Z);
                sphereInstance.SetMatrixAt(i, matrix);
            }
            sphereInstance.InstanceCount = total;
        }

        public void UpdateRays()
        {
            if (mesh == null || sphereInstance == null || lineSegments == null) return;
            int rayCount = Math.Clamp(_params.Count, 0, MAX_RAYS);
            int lineNum = 0;
           
            var bg = lineSegments.Geometry as BufferGeometry;
            if (bg == null) return;
            var posAttr = bg.Attributes["position"] as BufferAttribute<float>;

            // compute world-space center to aim rays toward
            if (mesh.Geometry.BoundingSphere == null) mesh.Geometry.ComputeBoundingSphere();
            var worldCenter = mesh.Geometry.BoundingSphere.Center.Clone().ApplyMatrix4(mesh.MatrixWorld);

            for (int i = 0; i < rayCount; i++)
            {
                int originIndex = i * 2;
                sphereInstance.GetMatrixAt(originIndex, _matrix);
                _matrix.Decompose(_position, _quaternion, _scale);
                double offset = 1e-4 * Environment.TickCount;
                _axis.Set(
                    (float)Math.Sin(i * 100 + offset),
                    (float)Math.Cos(-i * 10 + offset),
                    (float)Math.Sin(i * 1 + offset)
                ).Normalize();
                _position.ApplyAxisAngle(_axis, 0.001f);

                _scale.SetScalar(0.02f);
                _matrix.Compose(_position, _quaternion, _scale);
                sphereInstance.SetMatrixAt(originIndex, _matrix);

                //configure ray in world space, aim at mesh center
                _raycaster.ray.origin.Copy(_position);
                _raycaster.ray.direction.Copy(worldCenter).Sub(_position).Normalize();

                List<Intersection> hits;                
                bool usedBVH = _params.useBVH && bvh!=null;
                if (usedBVH)
                {
                    var inv = new Matrix4().Copy(mesh.MatrixWorld).Invert();
                    var localRay = new Ray(_raycaster.ray.origin.Clone(), _raycaster.ray.direction.Clone());
                    localRay.ApplyMatrix4(inv);

                    // scale near/far for non-uniform scaling
                    var worldScale = new Vector3().SetFromMatrixScale(mesh.MatrixWorld);
                    var dirScaled = localRay.direction.Clone().Multiply(worldScale);
                    double scaleFactor = dirScaled.Length();
                    double near = _raycaster.near / scaleFactor;
                    double far = _raycaster.far / scaleFactor;

                    hits = bvh.Raycast(localRay, mesh.Material, near, far);
                    hits.Sort((a, b) => a.distance.CompareTo(b.distance));
                }
                else
                {
                    hits = _raycaster.IntersectObject(mesh);
                }

                int hitIndex = originIndex + 1;
                if (hits.Count > 0)
                {
                    var hit = hits[0];
                    var point = hit.point.Clone();
                    if (usedBVH) point.ApplyMatrix4(mesh.MatrixWorld);
                    _scale.SetScalar(0.01f);
                    _matrix.Compose(point, _quaternion, _scale);
                    sphereInstance.SetMatrixAt(hitIndex, _matrix);
                    posAttr.SetXYZ(lineNum++, _position.X, _position.Y, _position.Z);
                    posAttr.SetXYZ(lineNum++, point.X, point.Y, point.Z);
                }
                else
                {
                    _scale.SetScalar(0.01f);
                    _matrix.Compose(_position, _quaternion, _scale);
                    sphereInstance.SetMatrixAt(hitIndex, _matrix);
                    posAttr.SetXYZ(lineNum++, _position.X, _position.Y, _position.Z);
                    posAttr.SetXYZ(lineNum++, 0, 0, 0);
                }
            }

            sphereInstance.InstanceCount = rayCount * 2;
            sphereInstance.InstanceMatrix.NeedsUpdate = true;

            bg.SetDrawRange(0, lineNum);
            posAttr.NeedsUpdate = true;
        }
    }
}

